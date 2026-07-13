/*
 * ============================================================================
 *  SOAJ - Pédale d'effet guitare ESP32
 *  PedaleEsclave.ino  —  ESP32 n°N : Esclave (traitement audio local)
 * ============================================================================
 *
 *  Rôle :
 *    - Reçoit UNIQUEMENT des paramètres (gain, tone, volume, ON/OFF)
 *      du maître via ESP-NOW. Aucun audio ne transite par radio.
 *    - Lit la guitare sur l'ADC (GPIO34, ADC1 : compatible WiFi/ESP-NOW).
 *    - Applique la fonction de transfert H(s,g,t,v) du circuit TL082
 *      (réponse LINÉAIRE pure, pas de saturation numérique ajoutée).
 *    - Sort le signal traité sur le DAC interne (GPIO25), centré sur 128.
 *
 *  Formule implémentée (voir README) :
 *    H(s,g,t,v) = Hin(s) × Hampli(s,g) × Hsortie(s,t,v)
 *
 *    Hin(s)      = 0.47·s / (1 + 0.4747·s)
 *                  (liaison d'entrée R4/C3/R3 — passe-haut ~0,335 Hz)
 *
 *    Hampli(s,g) = 1 + (0.11·g·s) / ((1 + 5e-5·g·s)(1 + 2.2e-4·s))
 *                  (ampli non-inverseur : R5 = g·500k, C4 100 pF, R7 1k, C5 220 nF)
 *
 *    Hsortie(s,t,v) = 0.1·v·s / (1 + (0.11242 − 0.00022·t)·s
 *                                  + (2.42e-5·t − 2.2e-6·t²)·s²)
 *                  (tone R8/R9 10k + C7 22 nF, liaison C6 1 µF, volume R10/R11 100k)
 *
 *  Chaque étage est converti en filtre numérique IIR (biquad) par
 *  transformation bilinéaire  s = 2·fs·(1 − z⁻¹)/(1 + z⁻¹)  à 20 kHz.
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
#define SAMPLE_RATE_HZ      20000     // 20 kHz
#define SAMPLE_PERIOD_US    (1000000UL / SAMPLE_RATE_HZ)
#define DT_SEC              (1.0f / (float)SAMPLE_RATE_HZ)

// Amplification d'entrée logicielle (1.0 = strictement fidèle au circuit)
#define INPUT_GAIN          2.0f

// Niveau de sortie global (1.0 = fidèle à la formule ; baisser si ça sature
// trop fort le DAC — la formule peut donner un gain de plusieurs centaines)
#define OUTPUT_LEVEL        1.0f

// Bypass : rattrapage de niveau quand l'effet est coupé
#define BYPASS_GAIN         8.0f

// Course du potentiomètre DRIVE :
//   1 = course logarithmique (pot "audio") : le bouton est progressif,
//       g réel = (e^(p·ln501) − 1)/500 -> p=0.5 donne g≈0.043 (gain ~×20)
//   0 = course linéaire, strictement fidèle au schéma (R5 = p·500k) —
//       attention : dès p=0.1 le gain dépasse ×50, tout le haut de la
//       course sature le DAC de la même façon
#define DRIVE_TAPER_LOG     1

// Lissage des paramètres (~60 ms) : aucun craquement au changement
#define PARAM_SMOOTH        0.0008f

// Recalcul des coefficients de filtres tous les N échantillons (~3 ms) :
// assez fréquent pour suivre le lissage, assez rare pour tenir les 50 µs
// du cycle audio (le recalcul contient des divisions coûteuses)
#define COEF_UPDATE_SAMPLES 64

// Suivi de l'offset DC (très lent, ~100 ms)
#define DC_TRACK_COEF       0.0005f

// Vu-mètre de diagnostic (1 = affiche chaque seconde sur port série)
// ATTENTION : bloque l'audio ~10 ms par seconde -> craquement périodique.
// ACTIVÉ pendant la mise au point — remettre à 0 quand tout fonctionne.
#define DEBUG_METER         1

// Bip de test au démarrage : 2 bips de 440 Hz envoyés DIRECTEMENT au DAC,
// sans passer par l'ADC ni par la formule. Si vous n'entendez PAS les bips,
// le problème est entre le GPIO25 et l'ampli (câblage, condensateur de
// liaison, volume ampli), pas dans le traitement. Mettre à 0 pour désactiver.
#define STARTUP_TEST_TONE   1

// Bornes de sécurité des paramètres
#define GAIN_MIN            0.0f
#define GAIN_MAX            1.0f
#define VOLUME_MAX          1.0f

// ---------------------------------------------------------------------------
// Paquet de paramètres (DOIT rester identique dans PedaleMaitre.ino)
// ---------------------------------------------------------------------------
#define PARAMS_MAGIC 0x534F414AUL     // "SOAJ"

typedef struct __attribute__((packed)) {
  uint32_t magic;
  float    gain;
  float    clip;      // ignoré en mode linéaire (gardé pour compatibilité)
  float    tone;
  float    volume;
  uint8_t  effectOn;
  uint8_t  diode;     // ignoré en mode linéaire (gardé pour compatibilité)
} PedalParams;

// Cibles reçues par radio
static volatile float tgtGain   = 0.5f;
static volatile float tgtTone   = 0.5f;
static volatile float tgtVolume = 0.5f;
static volatile float tgtEffect = 1.0f;

// ---------------------------------------------------------------------------
// Biquad IIR : y[n] = b0·x[n] + b1·x[n-1] + b2·x[n-2] − a1·y[n-1] − a2·y[n-2]
// ---------------------------------------------------------------------------
typedef struct {
  float b0, b1, b2;   // coefficients du numérateur
  float a1, a2;       // coefficients du dénominateur (a0 normalisé à 1)
  float x1, x2;       // x[n-1], x[n-2]
  float y1, y2;       // y[n-1], y[n-2]
} Biquad;

static inline float biquadRun(Biquad *f, float x) {
  const float y = f->b0 * x + f->b1 * f->x1 + f->b2 * f->x2
                            - f->a1 * f->y1 - f->a2 * f->y2;
  f->x2 = f->x1;  f->x1 = x;
  f->y2 = f->y1;  f->y1 = y;
  return y;
}

// Transformation bilinéaire GÉNÉRIQUE :
//   H(s) = (n0 + n1·s + n2·s²) / (d0 + d1·s + d2·s²)
// devient un biquad en z avec s = K·(1 − z⁻¹)/(1 + z⁻¹), K = 2·fs.
// Ne touche PAS à l'état (x1..y2) : on peut recalculer les coefficients
// en cours de route sans "pop".
static void bilinear2(Biquad *f,
                      float n0, float n1, float n2,
                      float d0, float d1, float d2) {
  const float K  = 2.0f * (float)SAMPLE_RATE_HZ;
  const float K2 = K * K;

  const float B0 = n0 + n1 * K + n2 * K2;
  const float B1 = 2.0f * (n0 - n2 * K2);
  const float B2 = n0 - n1 * K + n2 * K2;

  const float A0 = d0 + d1 * K + d2 * K2;
  const float A1 = 2.0f * (d0 - d2 * K2);
  const float A2 = d0 - d1 * K + d2 * K2;

  const float inv = 1.0f / A0;
  f->b0 = B0 * inv;  f->b1 = B1 * inv;  f->b2 = B2 * inv;
  f->a1 = A1 * inv;  f->a2 = A2 * inv;
}

// --- Étage 1 : Hin(s) = 0.47·s / (1 + 0.4747·s) ----------------------------
static void calcHin(Biquad *f) {
  bilinear2(f, 0.0f, 0.47f, 0.0f,
               1.0f, 0.4747f, 0.0f);
}

// --- Étage 2 : Hampli(s,g) = 1 + 0.11·g·s / ((1+5e-5·g·s)(1+2.2e-4·s)) -----
// Mise au même dénominateur :
//   num = 1 + (τ1+τ2+0.11·g)·s + τ1·τ2·s²
//   den = 1 + (τ1+τ2)·s        + τ1·τ2·s²
// avec τ1 = R5·C4 = 5e-5·g  et  τ2 = R7·C5 = 2.2e-4.
static void calcAmpli(Biquad *f, float g) {
  const float t1 = 5e-5f * g;
  const float t2 = 2.2e-4f;
  const float tg = 0.11f * g;
  bilinear2(f, 1.0f, t1 + t2 + tg, t1 * t2,
               1.0f, t1 + t2,      t1 * t2);
}

// --- Étage 3 : Hsortie(s,t,v) -----------------------------------------------
//   num = 0.1·v·s
//   den = 1 + (0.11242 − 0.00022·t)·s + (2.42e-5·t − 2.2e-6·t²)·s²
static void calcSortie(Biquad *f, float t, float v) {
  const float d1 = 0.11242f - 0.00022f * t;
  const float d2 = 2.42e-5f * t - 2.2e-6f * t * t;
  bilinear2(f, 0.0f, 0.1f * v, 0.0f,
               1.0f, d1,       d2);
}

// Course du pot DRIVE : position du bouton p (0..1) -> variable g de la formule
static inline float driveTaper(float p) {
#if DRIVE_TAPER_LOG
  return (expf(p * 6.2166f) - 1.0f) / 500.0f;   // ln(501) = 6.2166
#else
  return p;
#endif
}

// ---------------------------------------------------------------------------
// État du traitement audio
// ---------------------------------------------------------------------------
static float dcOffset = 2048.0f;
static Biquad fHin    = {0};
static Biquad fAmpli  = {0};
static Biquad fSortie = {0};
static float  quantErr = 0.0f;       // mise en forme du bruit DAC

// Paramètres lissés (valeurs effectives utilisées par l'audio)
static float smGain    = 0.5f;
static float smTone    = 0.5f;
static float smVolume  = 0.0f;       // démarre à 0 : montée douce, pas de "pop"
static float smEffect  = 1.0f;

static uint16_t coefCountdown = 0;   // recalcul périodique des coefficients
static uint32_t nextSampleUs  = 0;

// ---------------------------------------------------------------------------
// Utilitaires
// ---------------------------------------------------------------------------
static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline int readAdcOnce() {
#ifdef USE_LEGACY_ADC
  return adc1_get_raw(ADC1_CHANNEL_6);
#else
  return analogRead(PIN_GUITAR_IN);
#endif
}

// Lecture guitare = MÉDIANE de 3 lectures ADC (élimine les pics parasites WiFi)
static inline int readGuitarAdc() {
  const int a = readAdcOnce();
  const int b = readAdcOnce();
  const int c = readAdcOnce();
  const int lo = (a < b) ? a : b;
  const int hi = (a > b) ? a : b;
  return (c > hi) ? hi : ((c < lo) ? lo : c);
}

// ---------------------------------------------------------------------------
// Réception ESP-NOW
// ---------------------------------------------------------------------------
static void applyParams(const PedalParams *p) {
  tgtGain   = clampf(p->gain,   GAIN_MIN, GAIN_MAX);
  tgtTone   = clampf(p->tone,   0.0f, 1.0f);
  tgtVolume = clampf(p->volume, 0.0f, VOLUME_MAX);
  tgtEffect = p->effectOn ? 1.0f : 0.0f;
  // p->clip et p->diode sont ignorés : la chaîne est purement linéaire
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
  if (p.magic != PARAMS_MAGIC) return;
  applyParams(&p);
}

// ---------------------------------------------------------------------------
// Vu-mètre de diagnostic
// ---------------------------------------------------------------------------
#if DEBUG_METER
static float    mPeakIn  = 0.0f;
static int      mPeakOut = 0;
static uint32_t mCount   = 0;

static inline void meterTick(float raw) {
  const float dev = fabsf(raw - dcOffset);
  if (dev > mPeakIn) mPeakIn = dev;

  if (++mCount >= (uint32_t)SAMPLE_RATE_HZ) {
    if (mPeakIn < 6.0f) {
      Serial.printf("[Metre] entree crete: %.0f pas ADC — PAS DE SIGNAL GUITARE "
                    "(verifiez jack, condensateur de liaison et pont diviseur)\n",
                    mPeakIn);
    } else if (mPeakOut < 2) {
      const float g = driveTaper(smGain);
      Serial.printf("[Metre] entree: %.0f pas ADC mais sortie DAC ~0 — signal "
                    "PERDU dans le traitement (G=%.2f g=%.3f T=%.2f V=%.3f E=%.0f)\n",
                    mPeakIn, smGain, g, smTone, smVolume, smEffect);
    } else {
      const float g = driveTaper(smGain);
      Serial.printf("[Metre] entree: %.0f pas ADC | sortie DAC: +/-%d pas | "
                    "G=%.2f (g=%.3f, gain max ~x%.0f) T=%.2f V=%.3f E=%.0f\n",
                    mPeakIn, mPeakOut, smGain, g, 1.0f + 500.0f * g,
                    smTone, smVolume, smEffect);
    }
    mPeakIn  = 0.0f;
    mPeakOut = 0;
    mCount   = 0;
  }
}
#endif

// ---------------------------------------------------------------------------
// Bip de test : sinus envoyé directement au DAC (ne passe ni par l'ADC,
// ni par la formule). Sert à valider le chemin GPIO25 -> ampli.
// ---------------------------------------------------------------------------
#if STARTUP_TEST_TONE
static void playTestTone(float freqHz, float durSec, float amp) {
  const uint32_t n = (uint32_t)(durSec * (float)SAMPLE_RATE_HZ);
  const float    w = 2.0f * (float)M_PI * freqHz / (float)SAMPLE_RATE_HZ;
  for (uint32_t i = 0; i < n; i++) {
    const float s = amp * sinf(w * (float)i);
    dacWrite(PIN_AUDIO_OUT, (uint8_t)lroundf(128.0f + s * 127.0f));
    delayMicroseconds(SAMPLE_PERIOD_US - 12);   // ~12 µs déjà consommés
  }
  dacWrite(PIN_AUDIO_OUT, 128);
}
#endif

// ---------------------------------------------------------------------------
// Traitement d'UN échantillon : y = Hin × Hampli × Hsortie appliqué à x
// ---------------------------------------------------------------------------
static inline void processSample() {
  // Lissage des paramètres (progression douce, pas de craquement)
  smGain   += PARAM_SMOOTH * (tgtGain   - smGain);
  smTone   += PARAM_SMOOTH * (tgtTone   - smTone);
  smVolume += PARAM_SMOOTH * (tgtVolume - smVolume);
  smEffect += PARAM_SMOOTH * (tgtEffect - smEffect);

  // Recalcul périodique des coefficients (g, t, v changent lentement grâce
  // au lissage ; recalculer chaque échantillon ne tiendrait pas dans les 50 µs)
  if (coefCountdown == 0) {
    coefCountdown = COEF_UPDATE_SAMPLES;
    calcAmpli(&fAmpli, driveTaper(smGain));
    calcSortie(&fSortie, smTone, smVolume);
  }
  coefCountdown--;

  // --- Lecture ADC + suppression de l'offset DC ---
  const int raw = readGuitarAdc();
  dcOffset += DC_TRACK_COEF * ((float)raw - dcOffset);
  const float x = ((float)raw - dcOffset) * (INPUT_GAIN / 2048.0f);

#if DEBUG_METER
  meterTick((float)raw);
#endif

  // --- La formule : trois étages en cascade ---
  float y = biquadRun(&fHin,    x);   // Hin(s)
  y       = biquadRun(&fAmpli,  y);   // Hampli(s,g)
  y       = biquadRun(&fSortie, y);   // Hsortie(s,t,v)

  // --- Bypass en fondu : mélange signal direct <-> signal traité ---
  const float dry = x * BYPASS_GAIN;
  y = dry + (y - dry) * smEffect;

  // --- DAC 8 bits centré sur 128 + mise en forme du bruit ---
  const float desired = 128.0f + y * OUTPUT_LEVEL * 127.0f + quantErr;
  int dacVal = (int)lroundf(desired);
  if (dacVal < 0)   dacVal = 0;
  if (dacVal > 255) dacVal = 255;
  quantErr = desired - (float)dacVal;
  // En cas d'écrêtage DAC, ne pas accumuler une erreur géante
  if (quantErr >  1.0f) quantErr =  1.0f;
  if (quantErr < -1.0f) quantErr = -1.0f;

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
  Serial.println("\n=== SOAJ Pédale — ESCLAVE v3-diagnostic (formule H(s,g,t,v), linéaire) ===");

  // --- ADC ---
#ifdef USE_LEGACY_ADC
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
#else
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_GUITAR_IN, ADC_11db);
#endif

  // --- DAC : sortie au repos = 128 (milieu, aucun son) ---
  dacWrite(PIN_AUDIO_OUT, 128);

  // --- Coefficients initiaux des trois étages ---
  calcHin(&fHin);
  calcAmpli(&fAmpli, driveTaper(smGain));
  calcSortie(&fSortie, smTone, smVolume);

  // --- WiFi / ESP-NOW ---
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
  Serial.printf("Audio : %d Hz — H(s,g,t,v) = Hin x Hampli x Hsortie (lineaire)\n",
                SAMPLE_RATE_HZ);

  // Stabilisation de l'offset DC avant de sortir du son (évite le "plop")
  for (int i = 0; i < 4000; i++) {
    const int raw = readGuitarAdc();
    dcOffset += 0.01f * ((float)raw - dcOffset);
    delayMicroseconds(50);
  }
  Serial.printf("Offset DC mesuré : %.0f (attendu ~2048)\n", dcOffset);
  if (dcOffset < 1200.0f || dcOffset > 2900.0f) {
    Serial.println("ATTENTION : offset DC anormal — vérifiez le pont diviseur");
  }

#if STARTUP_TEST_TONE
  // Deux bips de 440 Hz directement sur le DAC : si vous ne les entendez pas
  // dans l'ampli, le problème est le câblage de sortie, pas le traitement.
  Serial.println("[Test] BIP 440 Hz x2 sur le DAC (test du chemin de sortie)...");
  playTestTone(440.0f, 0.35f, 0.6f);
  delay(150);
  playTestTone(440.0f, 0.35f, 0.6f);
  Serial.println("[Test] Bips terminés. Si rien entendu : vérifier GPIO25 -> ampli.");
#endif

  nextSampleUs = micros();
}

// ---------------------------------------------------------------------------
// Boucle audio : cadence fixe 20 kHz
// ---------------------------------------------------------------------------
void loop() {
  const uint32_t now = micros();
  if ((int32_t)(now - nextSampleUs) < 0) return;

  nextSampleUs += SAMPLE_PERIOD_US;
  if ((int32_t)(now - nextSampleUs) > 1000) nextSampleUs = now + SAMPLE_PERIOD_US;

  processSample();
}
