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
 *    - Traite le signal localement en appliquant H(s,g,t,v) mathématiquement.
 *    - Sort le signal traité sur le DAC interne (GPIO25), centré sur 128.
 *
 *  Chaîne de traitement (formule mathématique pure) :
 *    H(s,g,t,v) = Hin(s) × Hampli(s,g) × Hsortie(s,t,v)
 *
 *    Hin(s)      : filtre passe-haut d'entrée ~0.335 Hz (quasi no-op)
 *    Hampli(s,g) : amplificateur réglable, gain exponentiel x2 → x500
 *    Hsortie(s,t,v) : filtre passe-bas de tonalité + volume
 *
 *  Réponse linéaire uniquement, sans saturation non-linéaire.
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

// Amplification d'entrée logicielle
#define INPUT_GAIN          2.0f

// Gain exponentiel du drive : G ∈ [0,1] → gain ∈ [DRIVE_GAIN_MIN, DRIVE_GAIN_MAX]
#define DRIVE_GAIN_MIN      2.0f       // G=0 : quasi clean
#define DRIVE_GAIN_MAX      500.0f     // G=1 : heavy metal

// Volume maximal avant saturation DAC
#define OUTPUT_LEVEL        0.45f

// Lissage des paramètres (~60 ms) : aucun craquement au changement
#define PARAM_SMOOTH        0.0008f

// Suivi de l'offset DC (très lent, ~100 ms)
#define DC_TRACK_COEF       0.0005f

// Vu-mètre de diagnostic (1 = affiche chaque seconde sur port série)
// ATTENTION : bloque l'audio ~10 ms par seconde → craquement périodique.
// Laisser à 0 pour JOUER.
#define DEBUG_METER         0

// Bornes de sécurité des paramètres
#define GAIN_MIN            0.0f
#define GAIN_MAX            1.0f
#define CLIP_MIN            0.50f     // (non utilisé en mode linéaire)
#define CLIP_MAX            0.95f
#define VOLUME_MAX          1.0f

// Coefficients constants pour les formules mathématiques
// Hin(s) : filtre passe-haut d'entrée
#define HIN_NUM_COEF        0.47f     // numérateur : 0.47*s
#define HIN_DEN_COEF        0.4747f   // dénominateur : 1 + 0.4747*s

// Hampli(s,g) : amplificateur avec gain exponentiel
// Hin à ignorer ou traiter comme no-op (fc ~0.335 Hz)
// Hampli : 1 + (0.11*g*s) / ((1 + 5e-5*g*s)(1 + 2.2e-4*s))

// Hsortie(s,t,v) : filtre passe-bas de tonalité
// Tonalité : R = 500 + (1-t)*9000 Ω, C = 22 nF
// Volume : gain = v

#define TONE_C_FARADS       22e-9f
#define TONE_R_MIN_OHMS     500.0f
#define TONE_R_MAX_OHMS     9500.0f

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
  uint8_t  diode;
} PedalParams;

// Cibles reçues par radio
static volatile float tgtGain   = 0.5f;
static volatile float tgtClip   = 0.85f;
static volatile float tgtTone   = 0.5f;
static volatile float tgtVolume = 0.5f;
static volatile float tgtEffect = 1.0f;

// ---------------------------------------------------------------------------
// Filtres IIR 2ème ordre générique
// ---------------------------------------------------------------------------
struct IIR_Filter2 {
  float b[3], a[2];      // coefficients : y[n] = sum(b*x) - sum(a*y_prev)
  float x[3], y[2];      // état (x[0] courant, x[1] t-1, x[2] t-2, idem y)
};

// Applique un filtre IIR 2ème ordre
static inline float applyIIR2(IIR_Filter2 *f, float xn) {
  float yn = f->b[0]*xn + f->b[1]*f->x[1] + f->b[2]*f->x[2]
           - f->a[0]*f->y[0] - f->a[1]*f->y[1];
  f->x[2] = f->x[1];
  f->x[1] = xn;
  f->y[1] = f->y[0];
  f->y[0] = yn;
  return yn;
}

// ---------------------------------------------------------------------------
// État du traitement audio
// ---------------------------------------------------------------------------
static float dcOffset   = 2048.0f;
static IIR_Filter2 fHin     = {0};      // filtre d'entrée (passe-haut)
static IIR_Filter2 fAmpli   = {0};      // amplificateur avec filtrage
static IIR_Filter2 fTone    = {0};      // filtre de tonalité

// Paramètres lissés (valeurs effectives utilisées par l'audio)
static float smGain    = 0.5f;
static float smClip    = 0.85f;
static float smTone    = 0.5f;
static float smVolume  = 0.0f;
static float smEffect  = 1.0f;

// Coefficients pré-calculés
static float driveLogSpan = 0.0f;
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
// Calcul des coefficients IIR via transformation bilinéaire
// ---------------------------------------------------------------------------

// Filtre passe-haut 1er ordre Hin(s) = 0.47*s / (1 + 0.4747*s)
// Fréquence de coupure fc ≈ 0.335 Hz (quasi no-op en audio)
static void calcIIR_Hin(IIR_Filter2 *f) {
  const float fs = (float)SAMPLE_RATE_HZ;
  const float a = 2.0f * fs;

  // Coefficients bilinéaires pour passe-haut 1er ordre
  const float rc = 1.0f / (HIN_DEN_COEF * a);
  const float denom = 1.0f + 1.0f / (HIN_DEN_COEF * a);

  f->b[0] = HIN_NUM_COEF * a / denom;
  f->b[1] = -f->b[0];
  f->b[2] = 0.0f;

  f->a[0] = (1.0f / HIN_DEN_COEF - a) / denom;
  f->a[1] = 0.0f;
}

// Filtre amplificateur Hampli avec gain exponentiel dépendant de g
// Hampli(s,g) ≈ gain_exponentiel(g) avec filtrage RC
// Simplifié : gain direct + filtre passe-bas lent pour éviter aliasing
static void calcIIR_Ampli(IIR_Filter2 *f, float g) {
  const float fs = (float)SAMPLE_RATE_HZ;

  // Gain exponentiel : x2 (clean) → x500 (metal)
  const float gain = DRIVE_GAIN_MIN * expf(g * driveLogSpan);

  // Passe-bas pour éviter aliasing du gain : environ 3-4 kHz selon le gain
  // Plus le gain est grand, plus on filtre haut (pour conserver l'attaque)
  const float fc_lpf = 2000.0f + 3000.0f * (1.0f - g);  // ~2-5 kHz
  const float wc = 2.0f * (float)M_PI * fc_lpf;

  // Coefficient bilinéaire 1er ordre : passe-bas
  const float a = 2.0f * fs;
  const float alpha = wc / (a + wc);

  // Gain en cascade avec passe-bas
  f->b[0] = gain * alpha;
  f->b[1] = gain * alpha;
  f->b[2] = 0.0f;

  f->a[0] = (a - wc) / (a + wc);
  f->a[1] = 0.0f;
}

// Filtre tonalité Hsortie : passe-bas 1er ordre avec volume
// R = 500 + (1-t)*9000, C = 22 nF
static void calcIIR_Tone(IIR_Filter2 *f, float t, float v) {
  const float fs = (float)SAMPLE_RATE_HZ;

  // Résistance variable selon tone
  const float rTone = TONE_R_MIN_OHMS + (1.0f - t) * (TONE_R_MAX_OHMS - TONE_R_MIN_OHMS);

  // Passe-bas RC
  const float rc = rTone * TONE_C_FARADS;
  const float alpha = DT_SEC / (rc + DT_SEC);

  // Gain volume
  const float outLevel = v * OUTPUT_LEVEL;

  f->b[0] = alpha * outLevel;
  f->b[1] = alpha * outLevel;
  f->b[2] = 0.0f;

  f->a[0] = 1.0f - alpha;
  f->a[1] = 0.0f;
}

// ---------------------------------------------------------------------------
// Réception ESP-NOW
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
    Serial.printf("[Metre] entree: %.0f pas ADC | sortie DAC: +/-%d pas | "
                  "G=%.2f (x%.0f) T=%.2f V=%.3f E=%.0f\n",
                  mPeakIn, mPeakOut,
                  smGain, DRIVE_GAIN_MIN * expf(smGain * driveLogSpan),
                  smTone, smVolume, smEffect);
    mPeakIn  = 0.0f;
    mPeakOut = 0;
    mCount   = 0;
  }
}
#endif

// ---------------------------------------------------------------------------
// Traitement d'UN échantillon audio : H(s,g,t,v) = Hin × Hampli × Hsortie
// ---------------------------------------------------------------------------
static inline void processSample() {
  // Lissage des paramètres (progression douce, pas de craquement)
  smGain   += PARAM_SMOOTH * (tgtGain   - smGain);
  smClip   += PARAM_SMOOTH * (tgtClip   - smClip);
  smTone   += PARAM_SMOOTH * (tgtTone   - smTone);
  smVolume += PARAM_SMOOTH * (tgtVolume - smVolume);
  smEffect += PARAM_SMOOTH * (tgtEffect - smEffect);

  // Recalcule les coefficients des filtres qui dépendent de g et t
  calcIIR_Ampli(&fAmpli, smGain);
  calcIIR_Tone(&fTone, smTone, smVolume);

  // --- 1. Lecture ADC ---
  const int raw = readGuitarAdc();

  // --- 2. Suppression offset DC + normalisation ---
  dcOffset += DC_TRACK_COEF * ((float)raw - dcOffset);
  float x = ((float)raw - dcOffset) * (INPUT_GAIN / 2048.0f);

#if DEBUG_METER
  meterTick((float)raw);
#endif

  // --- 3. Chaîne de filtres H(s,g,t,v) ---
  // Hin(s) : filtre passe-haut d'entrée (presque no-op, fc ~0.335 Hz)
  // Mais on l'applique pour fidélité mathématique complète
  float y = applyIIR2(&fHin, x);

  // Hampli(s,g) : amplificateur avec gain exponentiel
  // (déjà intégré le filtrage pour éviter aliasing)
  y = applyIIR2(&fAmpli, y);

  // Hsortie(s,t,v) : filtre passe-bas de tonalité + volume
  y = applyIIR2(&fTone, y);

  // --- 4. Bypass optionnel ---
  const float dry = x * BYPASS_GAIN;
  y = dry + (y - dry) * smEffect;

  // --- 5. DAC 8 bits centré sur 128 ---
  const float desired = 128.0f + y * 127.0f;
  int dacVal = (int)lroundf(desired);
  if (dacVal < 0)   dacVal = 0;
  if (dacVal > 255) dacVal = 255;

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
  Serial.println("\n=== SOAJ Pédale — ESCLAVE (formule mathématique pure) ===");

  // --- ADC ---
#ifdef USE_LEGACY_ADC
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
#else
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_GUITAR_IN, ADC_11db);
#endif

  // --- DAC ---
  dacWrite(PIN_AUDIO_OUT, 128);

  // --- Coefficients pré-calculés ---
  driveLogSpan = logf(DRIVE_GAIN_MAX / DRIVE_GAIN_MIN);

  // Initialisation des filtres
  calcIIR_Hin(&fHin);
  calcIIR_Ampli(&fAmpli, 0.5f);
  calcIIR_Tone(&fTone, 0.5f, 0.5f);

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
  Serial.printf("Audio : %d Hz, H(s,g,t,v) = Hin × Hampli × Hsortie (réponse linéaire)\n",
                SAMPLE_RATE_HZ);

  // Stabilisation offset DC
  for (int i = 0; i < 4000; i++) {
    const int raw = readGuitarAdc();
    dcOffset += 0.01f * ((float)raw - dcOffset);
    delayMicroseconds(50);
  }
  Serial.printf("Offset DC mesuré : %.0f (attendu ~2048)\n", dcOffset);
  if (dcOffset < 1200.0f || dcOffset > 2900.0f) {
    Serial.println("ATTENTION : offset DC anormal — vérifiez le pont diviseur");
  }

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
