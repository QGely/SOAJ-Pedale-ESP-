/*
 * ============================================================================
 *  SOAJ - Pédale d'effet guitare ESP32
 *  PedaleEsclave.ino  —  ESP32 n°N : Esclave (traitement audio local)
 * ============================================================================
 *
 *  Rôle :
 *    - Reçoit UNIQUEMENT des paramètres (gain, clip, tone, volume, ON/OFF)
 *      du maître via ESP-NOW. Aucun audio ne transite par radio.
 *    - Lit la guitare sur l'ADC (GPIO34, ADC1 : compatible WiFi/ESP-NOW).
 *    - Traite le signal localement, en flottant, échantillon par échantillon.
 *    - Sort le signal traité sur le DAC interne (GPIO25), centré sur 128.
 *
 *  Chaîne de traitement (dans l'ordre) :
 *    1.  Lecture ADC 12 bits sur GPIO34.
 *    2.  Suivi et suppression de l'offset DC (moyenne glissante lente).
 *    3.  Filtre passe-haut ~40 Hz (retire résidu DC et basses parasites).
 *    4.  Filtre passe-bas ~5 kHz AVANT saturation (réduit le bruit ADC).
 *    5.  Noise gate PROGRESSIF (suiveur d'enveloppe + gain lissé :
 *        pas de coupure brutale, silence total sans guitare).
 *    6.  Étage de saturation = émulation du circuit ADTL082 (schéma LTspice) :
 *        gain x51..x501 (pot DRIVE R5+R6), shelf C5/R7 (graves non amplifiés),
 *        passe-bas C4 en contre-réaction.
 *    7.  Écrêtage aux rails ±12 V (tanh) PUIS diodes d'écrêtage (paramètre D :
 *        silicium ±0,6 V, LED ±1,7 V, germanium ±0,3 V) : crêtes aplaties.
 *    8.  Tone = pot R8+R9 + C7 : passe-bas variable APRÈS saturation.
 *    9.  Lissage de TOUS les paramètres reçus (pas de craquement).
 *    10. Volume (pot R10+R11) x amplitude DAC maximale OUTPUT_LEVEL.
 *    11. Écriture DAC 8 bits centrée sur 128 (+ mise en forme du bruit
 *        de quantification pour rester propre à bas volume).
 *
 *  IMPORTANT — matériel :
 *    L'entrée GPIO34 ne supporte NI tension négative NI plus de 3,3 V.
 *    La guitare doit passer par un condensateur de liaison + pont diviseur
 *    de polarisation à 1,65 V (voir README.md, section câblage).
 *
 *  Carte : ELEGOO ESP32 (NodeMCU-like, CP2102) — Arduino IDE, "ESP32 Dev Module".
 * ============================================================================
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <math.h>

// Lecture ADC rapide (~10 µs) via le pilote bas niveau.
// Si votre version du core ESP32 refuse de compiler cet include,
// commentez la ligne USE_LEGACY_ADC : le code basculera sur analogRead().
#define USE_LEGACY_ADC
#ifdef USE_LEGACY_ADC
  #include <driver/adc.h>
#endif

// ---------------------------------------------------------------------------
// Brochage et communication
// ---------------------------------------------------------------------------
#define PIN_GUITAR_IN   34            // ADC1_CH6 — compatible WiFi/ESP-NOW
#define PIN_AUDIO_OUT   25            // DAC1
#define WIFI_CHANNEL    1             // doit être IDENTIQUE sur le maître

// ---------------------------------------------------------------------------
// Réglages audio globaux
// ---------------------------------------------------------------------------
#define SAMPLE_RATE_HZ      20000     // 20 kHz : suffisant pour une guitare saturée
#define SAMPLE_PERIOD_US    (1000000UL / SAMPLE_RATE_HZ)

// Amplification d'entrée logicielle, AVANT l'étage de saturation.
// 2.0 = équivalent d'un micro un peu plus chaud : garantit un écrêtage franc
// des crêtes avec un simple bobinage (~100 mV). Mettre 1.0 pour être
// strictement fidèle au circuit, 3.0-4.0 si le signal reste trop propre
// (vérifiez la crête d'entrée au vu-mètre).
#define INPUT_GAIN          2.0f

// --- Émulation du circuit de saturation ADTL082 (schéma LTspice) -----------
// Ampli op non-inverseur qui sature sur ses rails ±12 V :
//   R7 1 kΩ + C5 220 nF (jambe de gain)  -> le plein gain ne s'applique
//     qu'au-dessus de ~723 Hz : graves propres, médiums/aigus boostés.
//   R6 50 kΩ + R5 450 kΩ (potentiomètre DRIVE = paramètre G, 0..1)
//     -> gain x51 (G=0, quasi clean) à x501 (G=1, fuzz).
//   C4 100 pF en contre-réaction -> passe-bas 3,2 à 32 kHz selon le drive.
//   Écrêtage aux rails ±12 V -> crêtes aplaties (la vraie saturation).
// En unités normalisées (±1 = ±1,65 V côté ADC), les rails valent ±7,27.
#define AMP_R7_OHMS         1000.0f
#define AMP_R6_OHMS         50e3f
#define AMP_R5_OHMS         450e3f
#define AMP_SHELF_HZ        723.0f     // zéro posé par C5/R7
#define AMP_C4_FARADS       100e-12f
#define RAIL_NORM           7.27f      // ±12 V exprimés en unités ADC (±1,65 V)
#define BYPASS_GAIN         8.0f       // rattrapage de niveau quand l'effet est coupé

// --- Diodes d'écrêtage (paramètre D) ----------------------------------------
// Deuxième étage de saturation, bien plus agressif que les rails : une paire
// de diodes tête-bêche vers la masse écrase le signal à ±Vf (tension de seuil).
// Le signal sort de l'ampli op jusqu'à ±12 V et vient s'écraser sur ±0,6 V :
// c'est ~20x plus de saturation que les rails seuls.
// Seuils exprimés en unités normalisées (±1 = ±1,65 V) :
#define VDIODE_SILICIUM     0.364f     // 2 x 1N4148, ±0,6 V  -> saturation forte
#define VDIODE_LED          1.030f     // 2 x LED,    ±1,7 V  -> crunch plus ouvert
#define VDIODE_GERMANIUM    0.182f     // 2 x germanium, ±0,3 V -> fuzz compressé

// --- Tone : potentiomètre R8+R9 (10 kΩ) + C7 (22 nF) -----------------------
// Passe-bas variable : T=1 -> brillant (~14 kHz), T=0.5 -> ~1,4 kHz (comme le
// schéma avec R8=R9=5k), T=0 -> sombre (~760 Hz).
#define TONE_C7_FARADS      22e-9f
#define TONE_R_MIN_OHMS     500.0f
#define TONE_R_MAX_OHMS     9500.0f

// Amplitude DAC maximale (fraction de la pleine échelle) quand volume = 1.0.
// Avec le volume par défaut (V=0.5), la sortie fait ~±28 pas de DAC (~±0,37 V) :
// audible mais modéré — le niveau final se règle sur l'ampli.
// NB : en dessous de ~0.10, la sortie tombe sous ±3 pas du DAC 8 bits
// et devient inaudible/quantifiée : ne descendez pas trop.
#define OUTPUT_LEVEL        0.45f

// Vu-mètre de diagnostic : 1 = affiche chaque seconde l'état de la chaîne
// (crête d'entrée, enveloppe, gate, crête DAC) sur le port série.
// L'affichage bloque l'audio ~5 ms par seconde : mettre 0 pour jouer proprement.
#define DEBUG_METER         1

// Bornes de sécurité des paramètres (identiques côté maître)
// G, T et V sont maintenant des POSITIONS DE POTENTIOMÈTRE, de 0.0 à 1.0 :
//   G = drive (R5+R6), T = tone (R8+R9), V = volume (R10+R11)
#define GAIN_MIN            0.0f
#define GAIN_MAX            1.0f
#define CLIP_MIN            0.50f     // C réduit virtuellement les rails ±12 V
#define CLIP_MAX            0.95f
#define VOLUME_MAX          1.0f

// Filtres fixes
#define HPF_FREQ_HZ         40.0f     // passe-haut d'entrée (garde le mi grave 82 Hz)
#define LPF_PRE_FREQ_HZ     5000.0f   // passe-bas avant saturation

// Noise gate — placé AVANT le gros gain, sinon le souffle serait amplifié x500.
// Amplitudes normalisées (pleine échelle ADC = 1.0). Une guitare directe fait
// ~0.06 en crête, le bruit ADC ~0.001.
#define GATE_LOW            0.003f    // en dessous : fermé (silence)
#define GATE_HIGH           0.009f    // au-dessus : complètement ouvert
#define ENV_ATTACK          0.05f     // suiveur d'enveloppe : montée rapide
#define ENV_RELEASE         0.0005f   // descente lente (~100 ms)
#define GATE_OPEN_COEF      0.010f    // ouverture du gate ~5 ms
#define GATE_CLOSE_COEF     0.0008f   // fermeture douce ~60 ms (garde le sustain)

// Lissage des paramètres reçus (~60 ms) : aucun craquement au changement
#define PARAM_SMOOTH        0.0008f

// Suivi de l'offset DC (très lent, ~100 ms)
#define DC_TRACK_COEF       0.0005f

// ---------------------------------------------------------------------------
// Paquet de paramètres (DOIT rester identique dans PedaleMaitre.ino)
// ---------------------------------------------------------------------------
#define PARAMS_MAGIC 0x534F414AUL     // "SOAJ"

typedef struct __attribute__((packed)) {
  uint32_t magic;
  float    gain;
  float    clip;
  float    tone;
  float    volume;
  uint8_t  effectOn;
  uint8_t  diode;     // 0 = sans (rails seuls), 1 = silicium, 2 = LED, 3 = germanium
} PedalParams;

// Cibles reçues par radio (écrites par le callback ESP-NOW, lues par l'audio)
// Défauts = potentiomètres à mi-course, comme sur le schéma (R5=R6 -> drive
// à 0.5 serait 275k ; on part à 0.5 ; R8=R9 -> tone 0.5 ; R10=R11 -> vol 0.5)
static volatile float tgtGain   = 0.5f;   // position du pot DRIVE (0..1)
static volatile float tgtClip   = 0.85f;  // seuil d'écrêtage (0.5..0.95)
static volatile float tgtTone   = 0.5f;   // position du pot TONE (0..1)
static volatile float tgtVolume = 0.5f;   // position du pot VOLUME (0..1)
static volatile float tgtEffect = 1.0f;   // 1 = effet, 0 = bypass (fondu doux)
static volatile float tgtVd     = VDIODE_SILICIUM;  // seuil des diodes (D=1 par défaut)

// ---------------------------------------------------------------------------
// État du traitement audio
// ---------------------------------------------------------------------------
static float dcOffset  = 2048.0f;  // milieu de l'ADC 12 bits
static float hpPrevX   = 0.0f, hpPrevY = 0.0f;   // passe-haut d'entrée
static float lpPre     = 0.0f;                    // passe-bas pré-saturation
static float shPrevX   = 0.0f, shPrevY = 0.0f;   // passe-haut du shelf de gain (C5/R7)
static float lpC4      = 0.0f;                    // passe-bas C4 (contre-réaction)
static float lpPost    = 0.0f;                    // passe-bas de tonalité (C7)
static float envelope  = 0.0f;                    // suiveur d'enveloppe
static float gateGain  = 0.0f;                    // gain du gate (0..1)
static float quantErr  = 0.0f;                    // mise en forme du bruit DAC

// Paramètres lissés (valeurs effectives utilisées par l'audio)
static float smGain    = 0.5f;
static float smClip    = 0.85f;
static float smTone    = 0.5f;
static float smVolume  = 0.0f;     // démarre à 0 : montée en douceur, pas de "pop"
static float smEffect  = 1.0f;
static float smVd      = VDIODE_SILICIUM;  // seuil de diode lissé (change sans "pop")

// Coefficients calculés au setup()
static float hpfA        = 0.0f;
static float lpfPreAlpha = 0.0f;
static float shelfA      = 0.0f;   // passe-haut 723 Hz du shelf de gain
#define DT_SEC (1.0f / (float)SAMPLE_RATE_HZ)

static uint32_t nextSampleUs = 0;

// ---------------------------------------------------------------------------
// Utilitaires
// ---------------------------------------------------------------------------
static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// Coefficient d'un passe-bas 1 pôle : y += alpha * (x - y)
static float lowpassAlpha(float freqHz) {
  const float dt = 1.0f / (float)SAMPLE_RATE_HZ;
  const float rc = 1.0f / (2.0f * (float)M_PI * freqHz);
  return dt / (rc + dt);
}

// Coefficient d'un passe-haut 1 pôle : y = a * (yPrev + x - xPrev)
static float highpassA(float freqHz) {
  const float dt = 1.0f / (float)SAMPLE_RATE_HZ;
  const float rc = 1.0f / (2.0f * (float)M_PI * freqHz);
  return rc / (rc + dt);
}

static inline int readGuitarAdc() {
#ifdef USE_LEGACY_ADC
  return adc1_get_raw(ADC1_CHANNEL_6);   // GPIO34
#else
  return analogRead(PIN_GUITAR_IN);
#endif
}

// ---------------------------------------------------------------------------
// Réception ESP-NOW (tourne sur le cœur WiFi ; l'audio tourne sur l'autre cœur)
// ---------------------------------------------------------------------------
static void applyParams(const PedalParams *p) {
  tgtGain   = clampf(p->gain,   GAIN_MIN, GAIN_MAX);
  tgtClip   = clampf(p->clip,   CLIP_MIN, CLIP_MAX);
  tgtTone   = clampf(p->tone,   0.0f, 1.0f);
  tgtVolume = clampf(p->volume, 0.0f, VOLUME_MAX);
  tgtEffect = p->effectOn ? 1.0f : 0.0f;
  switch (p->diode) {
    case 0:  tgtVd = RAIL_NORM;         break;   // sans diodes : rails seuls
    case 2:  tgtVd = VDIODE_LED;        break;
    case 3:  tgtVd = VDIODE_GERMANIUM;  break;
    default: tgtVd = VDIODE_SILICIUM;   break;   // 1 (et valeurs inconnues)
  }
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
static void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  (void)info;
#else
static void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  (void)mac;
#endif
  if (len != (int)sizeof(PedalParams)) return;
  PedalParams p;
  memcpy(&p, data, sizeof(p));
  if (p.magic != PARAMS_MAGIC) return;   // paquet étranger -> ignoré
  applyParams(&p);
}

// ---------------------------------------------------------------------------
// Vu-mètre de diagnostic (port série, une ligne par seconde)
// ---------------------------------------------------------------------------
#if DEBUG_METER
static float    mPeakIn  = 0.0f;   // crête d'entrée (pas d'ADC, avant boost)
static int      mPeakOut = 0;      // crête de sortie (pas de DAC autour de 128)
static uint32_t mCount   = 0;

static inline void meterTick(float raw) {
  const float dev = fabsf(raw - dcOffset);
  if (dev > mPeakIn) mPeakIn = dev;

  if (++mCount >= (uint32_t)SAMPLE_RATE_HZ) {   // une fois par seconde
    if (mPeakIn < 6.0f) {
      Serial.printf("[Metre] entree crete: %.0f pas ADC — PAS DE SIGNAL GUITARE "
                    "(verifiez jack, condensateur C1 et pont diviseur)\n", mPeakIn);
    } else {
      Serial.printf("[Metre] entree: %.0f pas ADC | enveloppe: %.3f | gate: %s (%.2f) | "
                    "sortie DAC: +/-%d pas | G=%.2f V=%.3f E=%.0f Vd=%.2f\n",
                    mPeakIn, envelope,
                    (gateGain > 0.5f) ? "OUVERT" : "ferme", gateGain,
                    mPeakOut, smGain, smVolume, smEffect, smVd);
    }
    mPeakIn  = 0.0f;
    mPeakOut = 0;
    mCount   = 0;
  }
}
#endif

// ---------------------------------------------------------------------------
// Traitement d'UN échantillon audio
// ---------------------------------------------------------------------------
static inline void processSample() {
  // ---- 9. Lissage des paramètres (chaque échantillon, très progressif) ----
  smGain   += PARAM_SMOOTH * (tgtGain   - smGain);
  smClip   += PARAM_SMOOTH * (tgtClip   - smClip);
  smTone   += PARAM_SMOOTH * (tgtTone   - smTone);
  smVolume += PARAM_SMOOTH * (tgtVolume - smVolume);
  smEffect += PARAM_SMOOTH * (tgtEffect - smEffect);
  smVd     += PARAM_SMOOTH * (tgtVd     - smVd);

  // ---- 1. Lecture ADC --------------------------------------------------
  const int raw = readGuitarAdc();

  // ---- 2. Suppression de l'offset DC (suivi lent) ----------------------
  dcOffset += DC_TRACK_COEF * ((float)raw - dcOffset);
  // Normalise vers ±1 puis applique le boost d'entrée logiciel
  float x = ((float)raw - dcOffset) * (INPUT_GAIN / 2048.0f);

#if DEBUG_METER
  meterTick((float)raw);
#endif

  // ---- 3. Passe-haut ~80 Hz --------------------------------------------
  const float hp = hpfA * (hpPrevY + x - hpPrevX);
  hpPrevX = x;
  hpPrevY = hp;

  // ---- 4. Passe-bas ~5 kHz avant saturation (anti-bruit ADC) ------------
  lpPre += lpfPreAlpha * (hp - lpPre);
  float sig = lpPre;

  // ---- 5. Noise gate progressif -----------------------------------------
  const float mag = fabsf(sig);
  envelope += (mag > envelope ? ENV_ATTACK : ENV_RELEASE) * (mag - envelope);

  float gateTarget;
  if      (envelope <= GATE_LOW)  gateTarget = 0.0f;
  else if (envelope >= GATE_HIGH) gateTarget = 1.0f;
  else    gateTarget = (envelope - GATE_LOW) / (GATE_HIGH - GATE_LOW);

  gateGain += (gateTarget > gateGain ? GATE_OPEN_COEF : GATE_CLOSE_COEF)
              * (gateTarget - gateGain);
  sig *= gateGain;

  // Gate complètement fermé : sortie strictement silencieuse (128 pile).
  if (gateGain < 0.001f) {
    quantErr = 0.0f;
    lpPost  *= 0.999f;   // vide doucement le filtre de tonalité
    dacWrite(PIN_AUDIO_OUT, 128);
    return;
  }

  // ---- 6. Étage ADTL082 : gain x51..x501 avec shelf (C5/R7) ---------------
  // Le pot DRIVE (G) fixe Rf = R6 + G*R5 ; gain haut-médium = 1 + Rf/R7.
  // Le réseau C5/R7 fait que seules les fréquences > ~723 Hz reçoivent le
  // plein gain : on l'émule en n'amplifiant que la partie passe-haut du signal.
  const float rfOhms  = AMP_R6_OHMS + smGain * AMP_R5_OHMS;
  const float ampGain = 1.0f + rfOhms / AMP_R7_OHMS;

  const float sh = shelfA * (shPrevY + sig - shPrevX);   // passe-haut 723 Hz
  shPrevX = sig;
  shPrevY = sh;
  float amp = sig + (ampGain - 1.0f) * sh;

  // Passe-bas C4 en contre-réaction : 3,2 kHz (drive max) à 32 kHz (drive min)
  const float rc4 = rfOhms * AMP_C4_FARADS;
  lpC4 += (DT_SEC / (rc4 + DT_SEC)) * (amp - lpC4);
  amp = lpC4;

  // ---- 7a. Écrêtage aux rails ±12 V de l'ampli op --------------------------
  // tanh donne le même aplatissement qu'un ampli op poussé aux rails, sans
  // les harmoniques d'aliasing d'un écrêtage numérique dur.
  float sat = RAIL_NORM * tanhf(amp / RAIL_NORM);

  // ---- 7b. Diodes d'écrêtage (paramètre D) ---------------------------------
  // Le signal (jusqu'à ±12 V) vient s'écraser sur le seuil des diodes
  // (±0,6 V en silicium) : c'est là que se fait le gros de la saturation.
  // Le paramètre C module le seuil effectif (plus bas = écrase plus tôt).
  // En mode D=0 le "seuil" est RAIL_NORM : on retrouve les rails seuls.
  const float vdEff = smVd * smClip;
  sat = vdEff * tanhf(sat / vdEff);
  sat *= (1.0f / vdEff);              // renormalise : écrêtage plein -> ±1
                                      // (le volume perçu ne dépend pas de D/C)

  // ---- 8. Tone : pot R8+R9 + C7, passe-bas variable après saturation ------
  const float rTone = TONE_R_MIN_OHMS + (1.0f - smTone) * (TONE_R_MAX_OHMS - TONE_R_MIN_OHMS);
  const float rcT   = rTone * TONE_C7_FARADS;
  lpPost += (DT_SEC / (rcT + DT_SEC)) * (sat - lpPost);

  // ---- Bypass en fondu : mélange signal propre <-> signal saturé ----------
  // (le signal propre est remonté pour rester comparable au signal d'effet)
  const float dry = sig * BYPASS_GAIN;
  const float y   = dry + (lpPost - dry) * smEffect;

  // ---- 10. Volume (pot R10+R11) + niveau de sortie global -----------------
  // OUTPUT_LEVEL fixe l'amplitude DAC maximale à V=1. À V=0.5 : ~±28 pas.
  const float outF = y * smVolume * OUTPUT_LEVEL;

  // ---- 11. DAC 8 bits centré sur 128 + mise en forme du bruit -------------
  // L'erreur de quantification est réinjectée sur l'échantillon suivant :
  // à très bas volume, le son reste net au lieu de "grésiller".
  const float desired = 128.0f + outF * 127.0f + quantErr;
  int dacVal = (int)lroundf(desired);
  if (dacVal < 0)   dacVal = 0;
  if (dacVal > 255) dacVal = 255;
  quantErr = desired - (float)dacVal;

#if DEBUG_METER
  const int outDev = (dacVal > 128) ? (dacVal - 128) : (128 - dacVal);
  if (outDev > mPeakOut) mPeakOut = outDev;
#endif

  dacWrite(PIN_AUDIO_OUT, (uint8_t)dacVal);
}

// ---------------------------------------------------------------------------
// Initialisation
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== SOAJ Pédale — ESCLAVE (audio) ===");

  // --- ADC ---
#ifdef USE_LEGACY_ADC
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);  // 0 .. ~3,3 V
#else
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_GUITAR_IN, ADC_11db);
#endif

  // --- DAC : sortie au repos = 128 (milieu, aucun son) ---
  dacWrite(PIN_AUDIO_OUT, 128);

  // --- Coefficients de filtres fixes ---
  hpfA        = highpassA(HPF_FREQ_HZ);
  lpfPreAlpha = lowpassAlpha(LPF_PRE_FREQ_HZ);
  shelfA      = highpassA(AMP_SHELF_HZ);

  // --- WiFi / ESP-NOW (réception uniquement) ---
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] Échec init — redémarrage dans 3 s");
    delay(3000);
    ESP.restart();
  }
  esp_now_register_recv_cb(onDataRecv);

  Serial.print("[WiFi] MAC esclave : ");
  Serial.println(WiFi.macAddress());
  Serial.printf("Audio : %d Hz, entrée GPIO%d, sortie DAC GPIO%d\n",
                SAMPLE_RATE_HZ, PIN_GUITAR_IN, PIN_AUDIO_OUT);

  // Laisse l'offset DC se stabiliser avant de sortir du son :
  // ~200 ms de lectures sans sortie audio (évite le "plop" au démarrage).
  for (int i = 0; i < 4000; i++) {
    const int raw = readGuitarAdc();
    dcOffset += 0.01f * ((float)raw - dcOffset);
    delayMicroseconds(50);
  }
  Serial.printf("Offset DC mesuré : %.0f (attendu ~2048 si polarisation a 1,65 V)\n", dcOffset);
  if (dcOffset < 1200.0f || dcOffset > 2900.0f) {
    Serial.println("ATTENTION : offset DC anormal — vérifiez le pont diviseur");
    Serial.println("            de polarisation sur GPIO34 (voir README.md).");
  }

  nextSampleUs = micros();
}

// ---------------------------------------------------------------------------
// Boucle audio : cadence fixe pilotée par micros()
// (loop() tourne sur le cœur 1 ; le WiFi/ESP-NOW tourne sur le cœur 0,
//  donc la réception radio ne perturbe pas la cadence audio)
// ---------------------------------------------------------------------------
void loop() {
  const uint32_t now = micros();
  if ((int32_t)(now - nextSampleUs) < 0) return;   // pas encore l'heure

  nextSampleUs += SAMPLE_PERIOD_US;
  // Si on a pris trop de retard (>1 ms), on resynchronise au lieu de rattraper
  if ((int32_t)(now - nextSampleUs) > 1000) nextSampleUs = now + SAMPLE_PERIOD_US;

  processSample();
}
