/*
 * ============================================================================
 *  SOAJ - Pédale d'effet guitare ESP32
 *  PedaleEsclavePsyche.ino  —  ESP32 esclave PSYCHE (distorsion + réverbe)
 * ============================================================================
 *
 *  Pédale psychédélique : distorsion dosable de ZÉRO (aucune modification)
 *  à PRONONCÉE, suivie d'une RÉVERBÉRATION spacieuse à queue modulée.
 *
 *  Chaîne :
 *    x -> gate -> [ Drive ×g -> distorsion S(u,d) ] -> RÉVERBE -> tone -> vol
 *
 *  Réverbération = Schroeder/Freeverb réduit, calculé échantillon par
 *  échantillon à 20 kHz :
 *    - 4 filtres en PEIGNE amortis en parallèle (delays 25,3 / 27,0 / 29,0 /
 *      30,8 ms) : la masse diffuse de la réverbe. Le feedback règle la durée
 *      (pot Decay), l'amortissement interne la brillance (pot Bright).
 *    - 2 filtres PASSE-TOUT en série (12,6 / 10,0 ms) : densifient les
 *      premières réflexions.
 *    - WARBLE : la queue de réverbe est modulée en amplitude par deux LFO
 *      lents désaccordés (0,6 Hz + 4,7 Hz) -> la pulsation psychédélique.
 *
 *  Le paquet radio est LE MÊME que la pédale SATU : le maître pilote les
 *  deux sans modification. Les jauges y prennent ce sens :
 *    Drive  (gain)  : pré-gain ×1 -> ×60 (course log)
 *    Dist   (dist)  : 0 = AUCUNE distorsion -> 1 = prononcée
 *    Octave (oct)   : REVERB — mix 0 = sec -> 1 = cathédrale
 *    Low    (low)   : DECAY — durée de la queue (0 = courte, 1 = quasi infinie)
 *    Mid    (mid)   : WARBLE — profondeur de la modulation psychédélique
 *    High   (high)  : BRIGHT — brillance de la queue de réverbe
 *    Tone   (tone)  : passe-bas final (0 = sombre, 1 = brillant)
 *    Volume (volume): niveau de sortie
 *
 *  IMPORTANT — matériel : identique à la pédale SATU (GPIO34 polarisé à
 *  1,65 V en entrée, DAC GPIO25 en sortie — voir README.md).
 *
 *  Carte : ELEGOO ESP32 (NodeMCU-like, CP2102) — Arduino IDE, "ESP32 Dev Module".
 * ============================================================================
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <math.h>

#define USE_LEGACY_ADC
#ifdef USE_LEGACY_ADC
  #include <driver/adc.h>
#endif

// ---------------------------------------------------------------------------
// Brochage et communication
// ---------------------------------------------------------------------------
#define PIN_GUITAR_IN   34
#define PIN_AUDIO_OUT   25
#define WIFI_CHANNEL    1             // doit être IDENTIQUE sur le maître

// ---------------------------------------------------------------------------
// Réglages audio globaux
// ---------------------------------------------------------------------------
#define SAMPLE_RATE_HZ      20000
#define SAMPLE_PERIOD_US    (1000000UL / SAMPLE_RATE_HZ)
#define DT_SEC              (1.0f / (float)SAMPLE_RATE_HZ)

#define INPUT_GAIN          2.0f
#define OUTPUT_LEVEL        0.9f
#define BYPASS_GAIN         8.0f

// --- Drive : pré-gain log ×1 (p=0) -> ×60 (p=1) -----------------------------
#define DRIVE_LOG_SPAN      4.094f    // ln(60)

// --- Distorsion S(u,d) : 0 = aucune -> 1 = prononcée ------------------------
// Seuil Vsat = 1.5 · 10^(−1.6·d) : ±1.5 (rien) -> ±0.038 (prononcée).
// tanh légèrement asymétrique (harmoniques paires, chaleur lampe), pas
// d'écrêtage dur : le caractère reste rond, psychédélique, jamais criard.
#define DIST_V0             1.5f
#define DIST_LOGSPAN        1.6f
#define DIST_ASYM           0.22f
#define DIST_MIX_RAMP       4.0f      // fondu clean->disto sur le 1er quart
static const float DIST_BIAS_OUT = 0.21651f;   // tanh(0.22)
// Passe-bas anti-souffle AVANT la distorsion : le souffle des convertisseurs
// amplifié par le drive puis compressé par la disto = nappe de bruit 6-8 kHz
#define PRE_LPF_HZ          4000.0f

// --- Réverbération (Schroeder/Freeverb réduit, tailles à 20 kHz) ------------
#define COMB1_LEN           506       // 25,3 ms
#define COMB2_LEN           539       // 27,0 ms
#define COMB3_LEN           579       // 29,0 ms
#define COMB4_LEN           615       // 30,8 ms
#define ALLP1_LEN           252       // 12,6 ms
#define ALLP2_LEN           200       // 10,0 ms
#define REVERB_INPUT_GAIN   0.40f     // niveau d'injection dans les peignes
#define REVERB_WET_GAIN     0.60f     // niveau de la queue en sortie
#define FB_MIN              0.70f     // Decay 0 : queue courte (~0,3 s)
#define FB_MAX              0.98f     // Decay 1 : queue très longue (~4 s)
#define DAMP_MIN            0.15f     // Bright 1 : queue brillante
#define DAMP_MAX            0.70f     // Bright 0 : queue sombre

// --- Warble : deux LFO désaccordés qui modulent la queue --------------------
#define WARBLE_HZ_SLOW      0.6f
#define WARBLE_HZ_FAST      4.7f
#define WARBLE_DEPTH_MAX    0.45f     // profondeur d'AM à Warble = 1

// --- Tone final : passe-bas 1 pôle, 760 Hz (t=0) -> 14,5 kHz (t=1) ----------
#define TONE_C_FARADS       22e-9f
#define TONE_R_MIN_OHMS     500.0f
#define TONE_R_MAX_OHMS     9500.0f

// --- Noise gate (mêmes principes que la pédale SATU) -------------------------
// Appliqué à l'ENTRÉE seulement : la queue de réverbe continue de sonner
// naturellement après la fermeture (c'est voulu — l'écho meurt tout seul).
#define GATE_LOW            0.002f
#define GATE_HIGH           0.006f
#define GATE_DRIVE_SCALE    1.5f
#define GATE_DIST_SCALE     1.0f
#define ENV_ATTACK          0.01f
#define ENV_RELEASE         0.002f
#define GATE_OPEN_COEF      0.010f
#define GATE_CLOSE_COEF     0.004f
#define GATE_SNAP           0.005f

#define PARAM_SMOOTH        0.0008f
#define COEF_UPDATE_SAMPLES 64
#define DC_TRACK_COEF       0.0005f
#define DEBUG_METER         0

// ---------------------------------------------------------------------------
// Paquet de paramètres (IDENTIQUE au maître et à la pédale SATU)
// ---------------------------------------------------------------------------
#define PARAMS_MAGIC 0x534F414AUL     // "SOAJ"

typedef struct __attribute__((packed)) {
  uint32_t magic;
  float    gain;      // Drive
  float    dist;      // Dist
  float    oct;       // -> REVERB (mix)
  float    low;       // -> DECAY
  float    mid;       // -> WARBLE
  float    high;      // -> BRIGHT
  float    tone;      // Tone
  float    volume;    // Volume
  uint8_t  effectOn;
} PedalParams;

static volatile float tgtGain   = 0.4f;
static volatile float tgtDist   = 0.3f;
static volatile float tgtReverb = 0.5f;   // oct
static volatile float tgtDecay  = 0.5f;   // low
static volatile float tgtWarble = 0.3f;   // mid
static volatile float tgtBright = 0.5f;   // high
static volatile float tgtTone   = 0.6f;
static volatile float tgtVolume = 0.5f;
static volatile float tgtEffect = 1.0f;
// Mode TEST DIRECT (effectOn = 2) : ADC -> DAC, aucun traitement (diagnostic)
static volatile bool  directMode = false;

// ---------------------------------------------------------------------------
// État du traitement
// ---------------------------------------------------------------------------
static float dcOffset  = 2048.0f;
static float envelope  = 0.0f;
static float gateGain  = 0.0f;
// Passe-haut 300 Hz pour la mesure d'enveloppe : rejette la ronflette secteur
// ET le grondement de frottement de la main posée sur les cordes (100-300 Hz)
#define GATE_DETECT_HPF_HZ  300.0f
static float envHpX = 0.0f, envHpY = 0.0f, envHpA = 0.0f;
static float lpTone    = 0.0f;      // passe-bas de tone (1 pôle)
static float lpPre     = 0.0f;      // passe-bas anti-souffle pré-disto (1 pôle)
static float lpPreAlpha = 0.5f;     // coefficient (calculé au setup)
static float quantErr  = 0.0f;      // erreur du quantificateur sigma-delta
static float dacTarget = 128.0f;    // valeur DAC visée (tenue entre échantillons)

// Paramètres lissés
static float smGain   = 0.4f, smDist  = 0.3f, smReverb = 0.5f, smDecay = 0.5f;
static float smWarble = 0.3f, smBright = 0.5f, smTone  = 0.6f, smVolume = 0.0f;
static float smEffect = 1.0f;

// Dérivés (recalculés toutes les ~3 ms — powf/expf coûteux)
static float driveGain  = 1.0f;
static float distVsat   = 1.0f, distMix = 0.0f, distBias = 0.0f;
static float combFb     = 0.84f, damp = 0.4f;
static float warbleDepth = 0.0f;
static float toneAlpha  = 0.5f;

// Buffers de réverbération (float statiques : ~8,5 Ko de RAM)
static float comb1[COMB1_LEN], comb2[COMB2_LEN], comb3[COMB3_LEN], comb4[COMB4_LEN];
static float allp1[ALLP1_LEN], allp2[ALLP2_LEN];
static int   ic1 = 0, ic2 = 0, ic3 = 0, ic4 = 0, ia1 = 0, ia2 = 0;
static float filt1 = 0.0f, filt2 = 0.0f, filt3 = 0.0f, filt4 = 0.0f;   // amortissement

// LFO du warble (accumulateurs de phase)
static float ph1 = 0.0f, ph2 = 0.0f;
static const float PH1_INC = 2.0f * (float)M_PI * WARBLE_HZ_SLOW / (float)SAMPLE_RATE_HZ;
static const float PH2_INC = 2.0f * (float)M_PI * WARBLE_HZ_FAST / (float)SAMPLE_RATE_HZ;

static uint16_t coefCountdown = 0;
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

// Médiane de 3 lectures : élimine les pics parasites WiFi de l'ADC
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
  tgtGain   = clampf(p->gain,   0.0f, 1.0f);
  tgtDist   = clampf(p->dist,   0.0f, 1.0f);
  tgtReverb = clampf(p->oct,    0.0f, 1.0f);
  tgtDecay  = clampf(p->low,    0.0f, 1.0f);
  tgtWarble = clampf(p->mid,    0.0f, 1.0f);
  tgtBright = clampf(p->high,   0.0f, 1.0f);
  tgtTone   = clampf(p->tone,   0.0f, 1.0f);
  tgtVolume = clampf(p->volume, 0.0f, 1.0f);
  tgtEffect = (p->effectOn == 1) ? 1.0f : 0.0f;
  directMode = (p->effectOn == 2);
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
    Serial.printf("[Metre] entree: %.0f pas | gate=%.2f | DAC +/-%d | G=%.2f D=%.2f "
                  "Rv=%.2f Dc=%.2f Wb=%.2f Br=%.2f T=%.2f V=%.2f\n",
                  mPeakIn, gateGain, mPeakOut, smGain, smDist, smReverb,
                  smDecay, smWarble, smBright, smTone, smVolume);
    mPeakIn = 0.0f; mPeakOut = 0; mCount = 0;
  }
}
#endif

// ---------------------------------------------------------------------------
// Pas de SIGMA-DELTA du DAC (~40 kHz effectifs, voir loop) : repousse le
// bruit de quantification 8 bits vers l'ultrasonique au lieu de la bande
// audible 6-9 kHz. Mesuré : -6,7 dB de bruit dans la bande 5-9 kHz.
// ---------------------------------------------------------------------------
// Dither triangulaire ±0,5 LSB : casse les cycles limites du quantificateur
// (sifflements purs 4-9 kHz quand la sortie est quasi constante)
static uint32_t ditherSeed = 0x5678EF01u;
static inline float ditherTpdf() {
  ditherSeed ^= ditherSeed << 13;
  ditherSeed ^= ditherSeed >> 17;
  ditherSeed ^= ditherSeed << 5;
  const int a = (int)(ditherSeed & 0xFFu);
  const int b = (int)((ditherSeed >> 8) & 0xFFu);
  return (float)(a - b) * (0.5f / 255.0f);
}

static inline void dacStep() {
  const float dth     = ditherTpdf();
  const float desired = dacTarget + quantErr + dth;
  int v = (int)lroundf(desired);
  if (v < 0)   v = 0;
  if (v > 255) v = 255;
  quantErr = (desired - dth) - (float)v;   // le dither reste hors de la boucle
  if (quantErr >  1.0f) quantErr =  1.0f;
  if (quantErr < -1.0f) quantErr = -1.0f;
#if DEBUG_METER
  const int outDev = (v > 128) ? (v - 128) : (128 - v);
  if (outDev > mPeakOut) mPeakOut = outDev;
#endif
  dacWrite(PIN_AUDIO_OUT, (uint8_t)v);
}

// ---------------------------------------------------------------------------
// Traitement d'UN échantillon
// ---------------------------------------------------------------------------
static inline void processSample() {
  // --- MODE TEST DIRECT : ADC -> DAC, strictement rien d'autre ---
  if (directMode) {
    const int raw = readGuitarAdc();
    dacWrite(PIN_AUDIO_OUT, (uint8_t)(raw >> 4));
    return;
  }

  // --- Lissage des paramètres ---
  smGain   += PARAM_SMOOTH * (tgtGain   - smGain);
  smDist   += PARAM_SMOOTH * (tgtDist   - smDist);
  smReverb += PARAM_SMOOTH * (tgtReverb - smReverb);
  smDecay  += PARAM_SMOOTH * (tgtDecay  - smDecay);
  smWarble += PARAM_SMOOTH * (tgtWarble - smWarble);
  smBright += PARAM_SMOOTH * (tgtBright - smBright);
  smTone   += PARAM_SMOOTH * (tgtTone   - smTone);
  smVolume += PARAM_SMOOTH * (tgtVolume - smVolume);
  smEffect += PARAM_SMOOTH * (tgtEffect - smEffect);

  // --- Dérivés coûteux, toutes les ~3 ms ---
  if (coefCountdown == 0) {
    coefCountdown = COEF_UPDATE_SAMPLES;
    driveGain = expf(smGain * DRIVE_LOG_SPAN);                  // ×1 -> ×60
    distVsat  = DIST_V0 * powf(10.0f, -DIST_LOGSPAN * smDist);  // ±1.5 -> ±0.038
    distMix   = (smDist < 0.01f) ? 0.0f
              : (smDist * DIST_MIX_RAMP > 1.0f ? 1.0f : smDist * DIST_MIX_RAMP);
    distBias  = DIST_ASYM * distVsat;
    combFb    = FB_MIN + (FB_MAX - FB_MIN) * smDecay;
    damp      = DAMP_MAX - (DAMP_MAX - DAMP_MIN) * smBright;
    warbleDepth = WARBLE_DEPTH_MAX * smWarble;
    const float rTone = TONE_R_MIN_OHMS + (1.0f - smTone) * (TONE_R_MAX_OHMS - TONE_R_MIN_OHMS);
    const float rc    = rTone * TONE_C_FARADS;
    toneAlpha = DT_SEC / (rc + DT_SEC);
  }
  coefCountdown--;

  // --- Lecture ADC + offset DC ---
  const int raw = readGuitarAdc();
  dcOffset += DC_TRACK_COEF * ((float)raw - dcOffset);
  const float x = ((float)raw - dcOffset) * (INPUT_GAIN / 2048.0f);

#if DEBUG_METER
  meterTick((float)raw);
#endif

  // --- Noise gate à l'entrée (la queue de réverbe, elle, sonne librement) ---
  // Mesure d'enveloppe passe-haut 120 Hz : la ronflette secteur ne tient
  // plus le gate ouvert
  const float eh = envHpA * (envHpY + x - envHpX);
  envHpX = x;
  envHpY = eh;
  const float mag = fabsf(eh);
  envelope += (mag > envelope ? ENV_ATTACK : ENV_RELEASE) * (mag - envelope);
  const float gateScale = 1.0f + GATE_DRIVE_SCALE * smGain + GATE_DIST_SCALE * smDist;
  const float gLow  = GATE_LOW  * gateScale;
  const float gHigh = GATE_HIGH * gateScale;
  float gateTarget;
  if      (envelope <= gLow)  gateTarget = 0.0f;
  else if (envelope >= gHigh) gateTarget = 1.0f;
  else {
    const float lin = (envelope - gLow) / (gHigh - gLow);
    gateTarget = lin * lin;                    // pente d'expandeur
  }
  gateGain += (gateTarget > gateGain ? GATE_OPEN_COEF : GATE_CLOSE_COEF)
              * (gateTarget - gateGain);
  if (gateGain < GATE_SNAP && gateTarget <= 0.0f) gateGain = 0.0f;

  const float xg = x * gateGain;

  // --- Anti-souffle : passe-bas 4 kHz avant le gain/la distorsion ---
  lpPre += lpPreAlpha * (xg - lpPre);

  // --- Distorsion S(u,d) : 0 = signal inchangé -> 1 = prononcée ---
  const float pre = lpPre * driveGain;
  float y = pre;
  if (distMix > 0.0f) {
    const float sat = tanhf((pre + distBias) / distVsat) - DIST_BIAS_OUT;
    y = pre + (sat - pre) * distMix;
  }

  // --- RÉVERBE : 4 peignes amortis en parallèle + 2 passe-tout en série ---
  const float rin = y * REVERB_INPUT_GAIN;
  float acc;
  {
    float o;
    o = comb1[ic1]; filt1 = o * (1.0f - damp) + filt1 * damp;
    comb1[ic1] = rin + filt1 * combFb; if (++ic1 >= COMB1_LEN) ic1 = 0; acc  = o;
    o = comb2[ic2]; filt2 = o * (1.0f - damp) + filt2 * damp;
    comb2[ic2] = rin + filt2 * combFb; if (++ic2 >= COMB2_LEN) ic2 = 0; acc += o;
    o = comb3[ic3]; filt3 = o * (1.0f - damp) + filt3 * damp;
    comb3[ic3] = rin + filt3 * combFb; if (++ic3 >= COMB3_LEN) ic3 = 0; acc += o;
    o = comb4[ic4]; filt4 = o * (1.0f - damp) + filt4 * damp;
    comb4[ic4] = rin + filt4 * combFb; if (++ic4 >= COMB4_LEN) ic4 = 0; acc += o;
    acc *= 0.25f;
    // Passe-tout de diffusion (Schroeder, g = 0.5)
    float bo;
    bo = allp1[ia1]; allp1[ia1] = acc + bo * 0.5f; if (++ia1 >= ALLP1_LEN) ia1 = 0;
    acc = bo - acc;
    bo = allp2[ia2]; allp2[ia2] = acc + bo * 0.5f; if (++ia2 >= ALLP2_LEN) ia2 = 0;
    acc = bo - acc;
  }
  float wet = acc * REVERB_WET_GAIN;

  // --- WARBLE : pulsation psychédélique de la queue (2 LFO désaccordés) ---
  ph1 += PH1_INC; if (ph1 > 2.0f * (float)M_PI) ph1 -= 2.0f * (float)M_PI;
  ph2 += PH2_INC; if (ph2 > 2.0f * (float)M_PI) ph2 -= 2.0f * (float)M_PI;
  if (warbleDepth > 0.001f) {
    wet *= 1.0f + warbleDepth * (0.7f * sinf(ph1) + 0.3f * sinf(ph2));
  }

  // --- Mix dry/wet (équilibre de puissance) ---
  y = y * (1.0f - 0.5f * smReverb) + wet * smReverb;

  // --- Tone : passe-bas 1 pôle 760 Hz -> 14,5 kHz ---
  lpTone += toneAlpha * (y - lpTone);
  y = lpTone;

  // --- Bypass en fondu ---
  const float dry = x * BYPASS_GAIN;
  y = y * smVolume * OUTPUT_LEVEL * 2.0f;      // volume (V=0.5 -> x0.9)
  y = dry + (y - dry) * smEffect;

  // --- Cible DAC : la sortie physique est tenue par le sigma-delta (~40 kHz) ---
  dacTarget = 128.0f + y * 127.0f;
  dacStep();
}

// ---------------------------------------------------------------------------
// Initialisation
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== SOAJ Pédale — ESCLAVE PSYCHE (distorsion + réverbe) ===");

#ifdef USE_LEGACY_ADC
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
#else
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_GUITAR_IN, ADC_11db);
#endif

  dacWrite(PIN_AUDIO_OUT, 128);

  // Passe-haut de la mesure d'enveloppe du gate (anti-ronflette + anti-frottement)
  {
    const float rc = 1.0f / (2.0f * (float)M_PI * GATE_DETECT_HPF_HZ);
    envHpA = rc / (rc + DT_SEC);
  }
  // Passe-bas anti-souffle 4 kHz pré-distorsion
  {
    const float rc = 1.0f / (2.0f * (float)M_PI * PRE_LPF_HZ);
    lpPreAlpha = DT_SEC / (rc + DT_SEC);
  }

  // Buffers de réverbe à zéro (silence initial)
  memset(comb1, 0, sizeof(comb1)); memset(comb2, 0, sizeof(comb2));
  memset(comb3, 0, sizeof(comb3)); memset(comb4, 0, sizeof(comb4));
  memset(allp1, 0, sizeof(allp1)); memset(allp2, 0, sizeof(allp2));

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
  Serial.printf("Audio : %d Hz — gate -> drive -> disto -> reverbe warble -> tone\n",
                SAMPLE_RATE_HZ);
  Serial.println("Jauges du maitre : Octave=Reverb, Low=Decay, Mid=Warble, High=Bright");

  // Stabilisation de l'offset DC (évite le "plop" au démarrage)
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
// Boucle audio : échantillons à 20 kHz + pas sigma-delta intermédiaires
// (le DAC est re-quantifié toutes les >=25 µs, soit ~40 kHz effectifs)
// ---------------------------------------------------------------------------
#define SD_STEP_US 25
static uint32_t lastStepUs = 0;

void loop() {
  const uint32_t now = micros();

  if ((int32_t)(now - nextSampleUs) >= 0) {
    nextSampleUs += SAMPLE_PERIOD_US;
    if ((int32_t)(now - nextSampleUs) > 1000) nextSampleUs = now + SAMPLE_PERIOD_US;
    processSample();                       // calcule dacTarget + 1er pas
    lastStepUs = micros();
  } else if (!directMode && (uint32_t)(now - lastStepUs) >= SD_STEP_US) {
    dacStep();                             // pas sigma-delta intermédiaire
    lastStepUs = now;                      // (coupé en mode TEST DIRECT)
  }
}
