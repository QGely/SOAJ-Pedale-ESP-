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
 *    3.  Filtre passe-haut ~80 Hz (retire résidu DC et basses parasites).
 *    4.  Filtre passe-bas ~5 kHz AVANT saturation (réduit le bruit ADC).
 *    5.  Noise gate PROGRESSIF (suiveur d'enveloppe + gain lissé :
 *        pas de coupure brutale, silence total sans guitare).
 *    6.  Gain modéré (borné à GAIN_MAX).
 *    7.  Saturation DOUCE (soft clipping à genou tanh, pas de hard clip).
 *    8.  Filtre passe-bas de tonalité APRÈS saturation (calme les aigus).
 *    9.  Lissage de TOUS les paramètres reçus (pas de craquement).
 *    10. Volume très réduit + facteur global OUTPUT_LEVEL.
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

// Niveau de sortie global : réduit fortement l'amplitude DAC.
// Montez-le (jusqu'à ~0.5) si vous ajoutez un pont diviseur matériel
// en sortie du DAC — le son sera moins quantifié (plus propre).
#define OUTPUT_LEVEL        0.15f

// Bornes de sécurité des paramètres (identiques côté maître)
#define GAIN_MIN            0.5f
#define GAIN_MAX            6.0f
#define CLIP_MIN            0.50f
#define CLIP_MAX            0.95f
#define VOLUME_MAX          0.25f     // plafond logiciel absolu

// Filtres fixes
#define HPF_FREQ_HZ         80.0f     // passe-haut d'entrée
#define LPF_PRE_FREQ_HZ     5000.0f   // passe-bas avant saturation
#define TONE_FREQ_MIN_HZ    700.0f    // tone = 0.0 -> très sombre
#define TONE_FREQ_MAX_HZ    4500.0f   // tone = 1.0 -> brillant

// Noise gate (amplitudes normalisées, pleine échelle = 1.0)
#define GATE_LOW            0.008f    // en dessous : fermé (silence)
#define GATE_HIGH           0.020f    // au-dessus : complètement ouvert
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
} PedalParams;

// Cibles reçues par radio (écrites par le callback ESP-NOW, lues par l'audio)
static volatile float tgtGain   = 2.5f;
static volatile float tgtClip   = 0.85f;
static volatile float tgtTone   = 0.35f;
static volatile float tgtVolume = 0.12f;
static volatile float tgtEffect = 1.0f;   // 1 = effet, 0 = bypass (fondu doux)

// ---------------------------------------------------------------------------
// État du traitement audio
// ---------------------------------------------------------------------------
static float dcOffset  = 2048.0f;  // milieu de l'ADC 12 bits
static float hpPrevX   = 0.0f, hpPrevY = 0.0f;   // passe-haut
static float lpPre     = 0.0f;                    // passe-bas pré-saturation
static float lpPost    = 0.0f;                    // passe-bas de tonalité
static float envelope  = 0.0f;                    // suiveur d'enveloppe
static float gateGain  = 0.0f;                    // gain du gate (0..1)
static float quantErr  = 0.0f;                    // mise en forme du bruit DAC

// Paramètres lissés (valeurs effectives utilisées par l'audio)
static float smGain    = 2.5f;
static float smClip    = 0.85f;
static float smTone    = 0.35f;
static float smVolume  = 0.0f;     // démarre à 0 : montée en douceur, pas de "pop"
static float smEffect  = 1.0f;

// Coefficients calculés au setup()
static float hpfA        = 0.0f;
static float lpfPreAlpha = 0.0f;

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
// Traitement d'UN échantillon audio
// ---------------------------------------------------------------------------
static inline void processSample() {
  // ---- 9. Lissage des paramètres (chaque échantillon, très progressif) ----
  smGain   += PARAM_SMOOTH * (tgtGain   - smGain);
  smClip   += PARAM_SMOOTH * (tgtClip   - smClip);
  smTone   += PARAM_SMOOTH * (tgtTone   - smTone);
  smVolume += PARAM_SMOOTH * (tgtVolume - smVolume);
  smEffect += PARAM_SMOOTH * (tgtEffect - smEffect);

  // ---- 1. Lecture ADC --------------------------------------------------
  const int raw = readGuitarAdc();

  // ---- 2. Suppression de l'offset DC (suivi lent) ----------------------
  dcOffset += DC_TRACK_COEF * ((float)raw - dcOffset);
  float x = ((float)raw - dcOffset) * (1.0f / 2048.0f);   // normalise vers ±1

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

  // ---- 6. Gain modéré ----------------------------------------------------
  const float driven = sig * smGain;

  // ---- 7. Saturation douce (genou tanh au-delà du seuil "clip") ----------
  // En dessous du seuil : signal intact. Au-dessus : compression progressive
  // vers ±1. Jamais d'écrêtage dur, donc pas d'aigus stridents.
  const float th   = smClip;
  const float knee = 1.0f - th;
  float sat;
  if      (driven >  th) sat =  th + knee * tanhf((driven - th) / knee);
  else if (driven < -th) sat = -th - knee * tanhf((-driven - th) / knee);
  else                   sat = driven;

  // ---- 8. Passe-bas de tonalité après saturation --------------------------
  const float toneFreq  = TONE_FREQ_MIN_HZ + smTone * (TONE_FREQ_MAX_HZ - TONE_FREQ_MIN_HZ);
  const float toneAlpha = lowpassAlpha(toneFreq);
  lpPost += toneAlpha * (sat - lpPost);

  // ---- Bypass en fondu : mélange signal propre <-> signal saturé ----------
  const float y = sig + (lpPost - sig) * smEffect;

  // ---- 10. Volume très réduit + niveau de sortie global -------------------
  const float outF = y * smVolume * OUTPUT_LEVEL;

  // ---- 11. DAC 8 bits centré sur 128 + mise en forme du bruit -------------
  // L'erreur de quantification est réinjectée sur l'échantillon suivant :
  // à très bas volume, le son reste net au lieu de "grésiller".
  const float desired = 128.0f + outF * 127.0f + quantErr;
  int dacVal = (int)lroundf(desired);
  if (dacVal < 0)   dacVal = 0;
  if (dacVal > 255) dacVal = 255;
  quantErr = desired - (float)dacVal;

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
