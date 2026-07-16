/*
 * ============================================================================
 *  SOAJ - Pédale d'effet guitare ESP32
 *  PedaleNam.ino  —  PROJET AUTONOME : 1 seul ESP32 + captures TONE3000 (NAM)
 * ============================================================================
 *
 *  Téléphone ──(WiFi + page web)──> ESP32 unique ──> Ampli
 *   │  http://192.168.4.1                ▲
 *   │                            Guitare (jack, ADC GPIO34)
 *   └── fichier .nam téléchargé sur tone3000.com
 *
 *  Le principe :
 *    1. Sur le téléphone, télécharger une capture .nam depuis tone3000.com
 *       (Neural Amp Modeler — réseau de neurones, bien trop lourd pour
 *       tourner en temps réel sur un ESP32).
 *    2. Se connecter au WiFi de la pédale, ouvrir la page, IMPORTER le .nam :
 *       le JavaScript de la page EXÉCUTE le réseau NAM sur le téléphone
 *       (WaveNet et LSTM gérés, quelques dizaines de secondes, une seule
 *       fois), mesure sa courbe d'écrêtage et sa réponse en fréquence, et
 *       n'envoie à l'ESP32 que le PROFIL extrait (~2,5 Ko).
 *    3. L'ESP32 grave le profil en flash (NVS) et le joue en temps réel à
 *       20 kHz : drive log -> passe-haut -> passe-bas -> COURBE D'ÉCRÊTAGE
 *       mesurée (257 points interpolés) -> médiums -> passe-bas de voicing
 *       -> tone -> volume. Gate, anti-parasites et sigma-delta identiques
 *       aux autres pédales du dépôt.
 *
 *  L'approximation garde le caractère statique de la capture (courbe de
 *  saturation, équilibre spectral) mais pas sa dynamique fine — c'est la
 *  limite d'un ESP32 seul, discutée dans le README.
 *
 *  Répartition des cœurs (un seul ESP32 fait radio ET audio) :
 *    - cœur 1 : tâche audio temps réel (boucle serrée, WDT désactivé)
 *    - cœur 0 : WiFi/portail captif/serveur web (tâche à basse priorité,
 *      delay(2) à chaque tour pour laisser vivre la pile WiFi)
 *  AUCUN audio ne transite par WiFi : uniquement paramètres et profil.
 *
 *  IMPORTANT — matériel : identique aux autres pédales (GPIO34 polarisé à
 *  1,65 V par pont diviseur + condensateur de liaison, DAC GPIO25 filtré —
 *  voir README.md). L'import d'un profil écrit en flash : bref craquement
 *  possible à cet instant, c'est normal (une seule fois par import).
 *
 *  Carte : ELEGOO ESP32 (NodeMCU-like, CP2102) — Arduino IDE, "ESP32 Dev Module".
 * ============================================================================
 */

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <math.h>

// Lecture ADC rapide (~10 µs) via le pilote bas niveau.
#define USE_LEGACY_ADC
#ifdef USE_LEGACY_ADC
  #include <driver/adc.h>
#endif

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
#define AP_SSID         "SOAJ-NAM"
#define AP_PASSWORD     "soaj1234"    // min. 8 caractères ; "" = réseau ouvert
#define WIFI_CHANNEL    1

#define PIN_GUITAR_IN   34            // ADC1_CH6 — compatible WiFi
#define PIN_AUDIO_OUT   25            // DAC1

#define SAMPLE_RATE_HZ      20000
#define SAMPLE_PERIOD_US    (1000000UL / SAMPLE_RATE_HZ)
#define DT_SEC              (1.0f / (float)SAMPLE_RATE_HZ)

#define INPUT_GAIN          2.0f      // 3-4 si signal faible (micro simple)
#define OUTPUT_LEVEL        1.0f
#define BYPASS_GAIN         8.0f
#define CLEAN_GAIN          16.0f     // Intensité = 0 : signal brut rehaussé

// Liaison d'entrée : passe-haut 40 Hz (retire le continu résiduel)
#define IN_HPF_HZ           40.0f

// Tone final : passe-bas 1 pôle, 760 Hz (t=0) -> 14,5 kHz (t=1)
#define TONE_C_FARADS       22e-9f
#define TONE_R_MIN_OHMS     500.0f
#define TONE_R_MAX_OHMS     9500.0f

// Noise gate (mêmes principes éprouvés que les autres pédales)
#define GATE_LOW            0.002f
#define GATE_HIGH           0.006f
#define GATE_DRIVE_SCALE    1.5f
#define ENV_ATTACK          0.01f
#define ENV_RELEASE         0.002f
#define GATE_OPEN_COEF      0.010f
#define GATE_CLOSE_COEF     0.004f
#define GATE_SNAP           0.005f

#define PARAM_SMOOTH        0.0008f   // lissage ~60 ms, aucun craquement
#define COEF_UPDATE_SAMPLES 64
#define DC_TRACK_COEF       0.0005f
#define PROF_FADE_STEP      0.0015f   // fondu anti-clic au changement de profil
#define DEBUG_METER         0

// ---------------------------------------------------------------------------
// Profil de pédale : ce que le téléphone extrait du .nam et nous envoie
// (mêmes champs que PedaleEsclaveSatu/profils_pedales.h)
// ---------------------------------------------------------------------------
#define LUT_TAILLE 257        // points de la courbe d'écrêtage
#define LUT_PLAGE  4.0f       // la courbe couvre u dans [-LUT_PLAGE, +LUT_PLAGE]

typedef struct {
  char  nom[24];              // nom de la capture (affiché sur la page)
  float preHpfHz;             // passe-haut avant écrêtage (resserrage du grave)
  float preLpfHz;             // passe-bas avant écrêtage (anti-souffle)
  float gainMin, gainMax;     // plage du pot DRIVE (course logarithmique)
  float postMidHz;            // correction de médiums APRÈS écrêtage
  float postMidQ;
  float postMidDb;            //   (0 dB = neutre)
  float postLpfHz;            // passe-bas de sortie (voicing)
  float sortieGain;           // rattrapage de niveau
  float courbe[LUT_TAILLE];   // courbe d'écrêtage mesurée sur le réseau NAM
} ProfilNam;                  // ~1,1 Ko — tient dans un blob NVS

static ProfilNam profilActif;             // joué par la tâche audio
static ProfilNam profilRecu;              // rempli par le serveur web
static volatile bool profilEnAttente = false;   // vrai -> l'audio l'échange

// Cibles pilotées par la page web
static volatile float tgtDrive  = 0.5f;   // g : position du pot drive (0..1)
static volatile float tgtMix    = 1.0f;   // d : intensité (0 = clean pur)
static volatile float tgtTone   = 0.6f;   // t
static volatile float tgtVolume = 0.5f;   // v
static volatile float tgtEffect = 1.0f;   // e : 0 = bypass, 1 = effet

static Preferences prefs;
static WebServer   server(80);
static DNSServer   dnsServer;

// ---------------------------------------------------------------------------
// Biquad IIR + transformation bilinéaire (identique aux autres pédales)
// ---------------------------------------------------------------------------
typedef struct {
  float b0, b1, b2;
  float a1, a2;
  float x1, x2, y1, y2;
} Biquad;

static inline float biquadRun(Biquad *f, float x) {
  const float y = f->b0 * x + f->b1 * f->x1 + f->b2 * f->x2
                            - f->a1 * f->y1 - f->a2 * f->y2;
  f->x2 = f->x1;  f->x1 = x;
  f->y2 = f->y1;  f->y1 = y;
  return y;
}

static void resetBiquad(Biquad *f) {
  f->x1 = f->x2 = f->y1 = f->y2 = 0.0f;
}

// 2e ordre : H(s) = (n0 + n1·s + n2·s²) / (d0 + d1·s + d2·s²)
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

// 1er ordre : H(s) = (n0 + n1·s)/(d0 + d1·s) — pas de mode parasite en z=−1
static void bilinear1(Biquad *f, float n0, float n1, float d0, float d1) {
  const float K   = 2.0f * (float)SAMPLE_RATE_HZ;
  const float A0  = d0 + d1 * K;
  const float inv = 1.0f / A0;
  f->b0 = (n0 + n1 * K) * inv;
  f->b1 = (n0 - n1 * K) * inv;
  f->b2 = 0.0f;
  f->a1 = (d0 - d1 * K) * inv;
  f->a2 = 0.0f;
}

// Cloche (RBJ Audio EQ Cookbook) pour la correction de médiums du profil
static void calcPeak(Biquad *f, float f0, float Q, float dB) {
  const float A  = powf(10.0f, dB / 40.0f);
  const float w0 = 2.0f * (float)M_PI * f0 / (float)SAMPLE_RATE_HZ;
  const float cw = cosf(w0), sw = sinf(w0);
  const float alpha = sw / (2.0f * Q);
  const float b0 = 1.0f + alpha * A;
  const float b1 = -2.0f * cw;
  const float b2 = 1.0f - alpha * A;
  const float a0 = 1.0f + alpha / A;
  const float a1 = -2.0f * cw;
  const float a2 = 1.0f - alpha / A;
  const float inv = 1.0f / a0;
  f->b0 = b0 * inv;  f->b1 = b1 * inv;  f->b2 = b2 * inv;
  f->a1 = a1 * inv;  f->a2 = a2 * inv;
}

// ---------------------------------------------------------------------------
// État du traitement audio
// ---------------------------------------------------------------------------
static float  dcOffset  = 2048.0f;
static float  envelope  = 0.0f;
static float  gateGain  = 0.0f;
static float  envHpX = 0.0f, envHpY = 0.0f, envHpA = 0.0f;  // HP 120 Hz mesure
static Biquad fInHp   = {0};   // liaison d'entrée 40 Hz
static Biquad fPreHp  = {0};   // profil : passe-haut avant écrêtage
static Biquad fPreLp  = {0};   // profil : passe-bas avant écrêtage
static Biquad fMid    = {0};   // profil : médiums post-écrêtage
static Biquad fPostLp = {0};   // profil : passe-bas de voicing
static float  lpTone     = 0.0f;
static float  toneAlpha  = 0.5f;
static float  quantErr   = 0.0f;
static float  dacTarget  = 128.0f;
static float  profFade   = 1.0f;   // fondu de changement de profil
static float  driveGain  = 1.0f;   // pot drive dans gainMin..gainMax (log)
static float  satMix     = 1.0f;   // fondu clean <-> profil (pot Intensité)

// Paramètres lissés
static float smDrive  = 0.5f;
static float smMix    = 1.0f;
static float smTone   = 0.6f;
static float smVolume = 0.0f;      // démarre à 0 : montée douce, pas de "pop"
static float smEffect = 1.0f;

static uint16_t coefCountdown = 0;

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

// Lecture de la courbe d'écrêtage avec interpolation linéaire.
// Hors de ±LUT_PLAGE la courbe est prolongée à plat (saturation écrasée).
static inline float lireCourbe(const float *lut, float u) {
  const float pos = (u + LUT_PLAGE) * ((float)(LUT_TAILLE - 1) / (2.0f * LUT_PLAGE));
  if (pos <= 0.0f) return lut[0];
  if (pos >= (float)(LUT_TAILLE - 1)) return lut[LUT_TAILLE - 1];
  const int   i  = (int)pos;
  const float fr = pos - (float)i;
  return lut[i] + fr * (lut[i + 1] - lut[i]);
}

// ---------------------------------------------------------------------------
// Profil : bornage, filtres, profil neutre de démarrage, sauvegarde NVS
// ---------------------------------------------------------------------------
static void bornerProfil(ProfilNam *p) {
  p->nom[sizeof(p->nom) - 1] = '\0';
  p->preHpfHz   = clampf(p->preHpfHz,   20.0f, 2000.0f);
  p->preLpfHz   = clampf(p->preLpfHz,  1000.0f, 9000.0f);
  p->gainMin    = clampf(p->gainMin,     0.5f, 100.0f);
  p->gainMax    = clampf(p->gainMax, p->gainMin, 1000.0f);
  p->postMidHz  = clampf(p->postMidHz, 100.0f, 4000.0f);
  p->postMidQ   = clampf(p->postMidQ,    0.3f, 4.0f);
  p->postMidDb  = clampf(p->postMidDb, -12.0f, 12.0f);
  p->postLpfHz  = clampf(p->postLpfHz, 1000.0f, 9000.0f);
  p->sortieGain = clampf(p->sortieGain,  0.05f, 1.5f);
  for (int i = 0; i < LUT_TAILLE; i++)
    p->courbe[i] = clampf(p->courbe[i], -1.5f, 1.5f);
}

// Recalcule les filtres depuis profilActif et remet leurs états à zéro
// (appelé à la pointe basse du fondu : l'échange est inaudible)
static void chargerFiltresProfil() {
  const ProfilNam *p = &profilActif;
  const float tauH = 1.0f / (2.0f * (float)M_PI * p->preHpfHz);
  bilinear1(&fPreHp, 0.0f, tauH, 1.0f, tauH);
  const float wLp = 2.0f * (float)M_PI * p->preLpfHz;
  bilinear2(&fPreLp, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f / (0.707f * wLp), 1.0f / (wLp * wLp));
  calcPeak(&fMid, p->postMidHz, p->postMidQ, p->postMidDb);
  const float wOut = 2.0f * (float)M_PI * p->postLpfHz;
  bilinear2(&fPostLp, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f / (0.707f * wOut), 1.0f / (wOut * wOut));
  resetBiquad(&fPreHp);
  resetBiquad(&fPreLp);
  resetBiquad(&fMid);
  resetBiquad(&fPostLp);
}

// Profil de démarrage si rien en flash : tanh doux, voicing neutre
static void profilNeutre(ProfilNam *p) {
  memset(p, 0, sizeof(*p));
  strncpy(p->nom, "NEUTRE (tanh)", sizeof(p->nom) - 1);
  p->preHpfHz   = 40.0f;
  p->preLpfHz   = 4000.0f;
  p->gainMin    = 2.0f;
  p->gainMax    = 60.0f;
  p->postMidHz  = 700.0f;
  p->postMidQ   = 1.0f;
  p->postMidDb  = 0.0f;
  p->postLpfHz  = 5500.0f;
  p->sortieGain = 1.0f;
  for (int i = 0; i < LUT_TAILLE; i++) {
    const float u = -LUT_PLAGE + (2.0f * LUT_PLAGE * (float)i) / (float)(LUT_TAILLE - 1);
    p->courbe[i] = 0.95f * tanhf(u);
  }
}

// NVS : le profil survit à l'extinction (écrit UNE fois par import)
static void sauverProfilNVS(const ProfilNam *p) {
  prefs.putBytes("profil", p, sizeof(*p));
}

static bool chargerProfilNVS(ProfilNam *p) {
  if (prefs.getBytesLength("profil") != sizeof(*p)) return false;
  prefs.getBytes("profil", p, sizeof(*p));
  bornerProfil(p);
  return true;
}

// ---------------------------------------------------------------------------
// Analyse du JSON envoyé par la page (/api/profil). Format produit par notre
// propre JavaScript — analyseur minimal clé par clé, sans bibliothèque.
// ---------------------------------------------------------------------------
static bool jsonFloat(const char *s, const char *cle, float *out) {
  const char *p = strstr(s, cle);
  if (!p) return false;
  p = strchr(p + strlen(cle), ':');
  if (!p) return false;
  *out = strtof(p + 1, NULL);
  return true;
}

static bool parseProfilJson(const char *s, ProfilNam *p) {
  memset(p, 0, sizeof(*p));

  // nom : première chaîne après "nom":
  const char *n = strstr(s, "\"nom\"");
  if (n) {
    n = strchr(n + 5, ':');
    if (n) n = strchr(n, '"');
    if (n) {
      n++;
      size_t i = 0;
      while (*n && *n != '"' && i < sizeof(p->nom) - 1) p->nom[i++] = *n++;
      p->nom[i] = '\0';
    }
  }
  if (p->nom[0] == '\0') strncpy(p->nom, "SANS NOM", sizeof(p->nom) - 1);

  bool ok = true;
  ok &= jsonFloat(s, "\"preHpfHz\"",   &p->preHpfHz);
  ok &= jsonFloat(s, "\"preLpfHz\"",   &p->preLpfHz);
  ok &= jsonFloat(s, "\"gainMin\"",    &p->gainMin);
  ok &= jsonFloat(s, "\"gainMax\"",    &p->gainMax);
  ok &= jsonFloat(s, "\"postMidHz\"",  &p->postMidHz);
  ok &= jsonFloat(s, "\"postMidQ\"",   &p->postMidQ);
  ok &= jsonFloat(s, "\"postMidDb\"",  &p->postMidDb);
  ok &= jsonFloat(s, "\"postLpfHz\"",  &p->postLpfHz);
  ok &= jsonFloat(s, "\"sortieGain\"", &p->sortieGain);
  if (!ok) return false;

  // lut : exactement LUT_TAILLE nombres
  const char *l = strstr(s, "\"lut\"");
  if (!l) return false;
  l = strchr(l, '[');
  if (!l) return false;
  l++;
  for (int i = 0; i < LUT_TAILLE; i++) {
    char *fin = NULL;
    p->courbe[i] = strtof(l, &fin);
    if (fin == l) return false;      // pas un nombre -> tableau trop court
    l = fin;
    while (*l == ',' || *l == ' ' || *l == '\n' || *l == '\r') l++;
  }
  bornerProfil(p);
  return true;
}

// ---------------------------------------------------------------------------
// Sigma-delta du DAC + dither TPDF (identique aux autres pédales : repousse
// le bruit de quantification 8 bits vers l'ultrasonique, casse les cycles
// limites qui sifflent)
// ---------------------------------------------------------------------------
static uint32_t ditherSeed = 0x9ABC1234u;
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
  quantErr = (desired - dth) - (float)v;
  if (quantErr >  1.0f) quantErr =  1.0f;
  if (quantErr < -1.0f) quantErr = -1.0f;
  dacWrite(PIN_AUDIO_OUT, (uint8_t)v);
}

// ---------------------------------------------------------------------------
// Traitement d'UN échantillon (tâche audio, cœur 1)
// ---------------------------------------------------------------------------
static inline void processSample() {
  // --- Lissage des paramètres ---
  smDrive  += PARAM_SMOOTH * (tgtDrive  - smDrive);
  smMix    += PARAM_SMOOTH * (tgtMix    - smMix);
  smTone   += PARAM_SMOOTH * (tgtTone   - smTone);
  smVolume += PARAM_SMOOTH * (tgtVolume - smVolume);
  smEffect += PARAM_SMOOTH * (tgtEffect - smEffect);

  // --- Dérivés coûteux, toutes les ~3 ms ---
  if (coefCountdown == 0) {
    coefCountdown = COEF_UPDATE_SAMPLES;
    driveGain = profilActif.gainMin
                * powf(profilActif.gainMax / profilActif.gainMin, smDrive);
    satMix = (smMix < 0.01f) ? 0.0f : ((smMix * 4.0f > 1.0f) ? 1.0f : smMix * 4.0f);
    const float rTone = TONE_R_MIN_OHMS + (1.0f - smTone) * (TONE_R_MAX_OHMS - TONE_R_MIN_OHMS);
    const float rc    = rTone * TONE_C_FARADS;
    toneAlpha = DT_SEC / (rc + DT_SEC);
  }
  coefCountdown--;

  // --- Nouveau profil reçu : fondu descendant, échange, fondu montant ---
  if (profilEnAttente) {
    profFade -= PROF_FADE_STEP;
    if (profFade <= 0.0f) {
      profFade = 0.0f;
      memcpy(&profilActif, &profilRecu, sizeof(profilActif));
      chargerFiltresProfil();
      driveGain = profilActif.gainMin
                  * powf(profilActif.gainMax / profilActif.gainMin, smDrive);
      profilEnAttente = false;
    }
  } else if (profFade < 1.0f) {
    profFade += PROF_FADE_STEP;
    if (profFade > 1.0f) profFade = 1.0f;
  }

  // --- Lecture ADC + suppression de l'offset DC ---
  const int raw = readGuitarAdc();
  dcOffset += DC_TRACK_COEF * ((float)raw - dcOffset);
  const float x = ((float)raw - dcOffset) * (INPUT_GAIN / 2048.0f);

  // --- Noise gate : enveloppe mesurée derrière un passe-haut 120 Hz
  //     (la ronflette secteur ne tient pas le gate ouvert) ---
  const float eh = envHpA * (envHpY + x - envHpX);
  envHpX = x;
  envHpY = eh;
  const float mag = fabsf(eh);
  envelope += (mag > envelope ? ENV_ATTACK : ENV_RELEASE) * (mag - envelope);
  const float gateScale = 1.0f + GATE_DRIVE_SCALE * smDrive;
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
  if (gateGain < GATE_SNAP && gateTarget <= 0.0f) {
    gateGain = 0.0f;
    quantErr = 0.0f;
  }

  // --- Chaîne du profil : liaison -> drive -> filtres -> courbe -> voicing ---
  float y = biquadRun(&fInHp, x) * gateGain;   // liaison 40 Hz + gate
  y *= driveGain;                              // pot DRIVE (gainMin..gainMax, log)
  y = biquadRun(&fPreHp, y);                   // resserrage du grave du profil
  y = biquadRun(&fPreLp, y);                   // anti-souffle pré-écrêtage
  if (satMix > 0.0f) {
    const float w = lireCourbe(profilActif.courbe, y) * profilActif.sortieGain;
    y += (w - y) * satMix;                     // courbe d'écrêtage mesurée
  }
  y = biquadRun(&fMid, y);                     // médiums (bosse/creux du profil)
  y = biquadRun(&fPostLp, y);                  // passe-bas de voicing

  // --- Tone + volume ---
  lpTone += toneAlpha * (y - lpTone);
  y = lpTone * smVolume * 2.0f * OUTPUT_LEVEL;

  // --- Gate en sortie (atténuation au carré) + fondu de profil ---
  y *= gateGain * profFade;

  // --- Intensité = 0 : clean pur (signal brut rehaussé, dosé par Volume) ---
  const float clean = x * CLEAN_GAIN * smVolume;
  y = clean + (y - clean) * satMix;

  // --- Bypass en fondu ---
  const float dry = x * BYPASS_GAIN;
  y = dry + (y - dry) * smEffect;

  dacTarget = 128.0f + y * OUTPUT_LEVEL * 127.0f;
  dacStep();
}

// ---------------------------------------------------------------------------
// Page web (embarquée en flash) : jauges + import de capture .nam.
// Le JavaScript contient un MOTEUR NAM complet (WaveNet + LSTM, disposition
// des poids conforme à NeuralAmpModelerCore) : le réseau tourne sur le
// TÉLÉPHONE pour la calibration, jamais sur l'ESP32.
// ---------------------------------------------------------------------------
static const char INDEX_HTML[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html lang="fr"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>SOAJ NAM</title>
<style>
:root{--bg:#141210;--panel:#211d17;--line:#373025;--amber:#ffa62b;--amber2:#ff7b1c;
--txt:#f3e9d8;--mut:#968b7a;--ok:#7dd069;--bad:#d05555}
*{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent}
html,body{height:100%}
body{background:radial-gradient(1200px 600px at 50% -100px,#26211a,var(--bg));
color:var(--txt);font-family:-apple-system,'Segoe UI',Roboto,Helvetica,Arial,sans-serif;
display:flex;justify-content:center;padding:18px 14px 40px}
.pedal{width:100%;max-width:430px}
header{display:flex;align-items:center;justify-content:space-between;margin-bottom:16px}
h1{font-size:24px;letter-spacing:5px;font-weight:800}
h1 b{color:var(--amber)}
.sub{font-size:11px;color:var(--mut);letter-spacing:2px;margin-top:2px}
.link{display:flex;align-items:center;gap:7px;font-size:12px;color:var(--mut)}
.dot{width:10px;height:10px;border-radius:50%;background:var(--bad);transition:.3s}
.dot.on{background:var(--ok);box-shadow:0 0 10px var(--ok)}
.card{background:var(--panel);border:1px solid var(--line);border-radius:16px;
padding:16px 16px 12px;margin-bottom:12px;box-shadow:0 4px 18px #0006}
.ctl{margin-bottom:14px}
.row{display:flex;justify-content:space-between;align-items:baseline;margin-bottom:4px}
.row label{font-size:12px;letter-spacing:3px;color:var(--mut);text-transform:uppercase;font-weight:600}
.row output{font-size:17px;font-weight:800;color:var(--amber);font-variant-numeric:tabular-nums}
.hint{font-size:10px;color:#6d6455;letter-spacing:1px;margin-top:2px}
input[type=range]{width:100%;height:38px;appearance:none;-webkit-appearance:none;background:transparent;cursor:pointer}
input[type=range]::-webkit-slider-runnable-track{height:10px;border-radius:5px;
background:linear-gradient(90deg,var(--amber2),var(--amber) var(--p,50%),#3a332a var(--p,50%));
box-shadow:inset 0 1px 3px #0008}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:28px;height:28px;border-radius:50%;
background:linear-gradient(#f4ead9,#cfc2ab);border:3px solid var(--amber2);margin-top:-9px;
box-shadow:0 2px 8px #000b}
input[type=range]::-moz-range-track{height:10px;border-radius:5px;background:#3a332a}
input[type=range]::-moz-range-progress{height:10px;border-radius:5px;background:linear-gradient(90deg,var(--amber2),var(--amber))}
input[type=range]::-moz-range-thumb{width:24px;height:24px;border-radius:50%;
background:#e8dcc7;border:3px solid var(--amber2);box-shadow:0 2px 8px #000b}
.pname{font-size:16px;font-weight:800;color:var(--amber);margin:4px 0 8px;letter-spacing:1px}
button.b{width:100%;padding:12px;border-radius:12px;border:1px solid var(--line);
background:#1a1712;color:var(--txt);font-size:13px;font-weight:700;letter-spacing:1px;cursor:pointer}
button.b:disabled{opacity:.4}
button.b.go{background:linear-gradient(#3a2c14,#2c210f);color:var(--amber);border-color:var(--amber2)}
input[type=file]{width:100%;color:var(--mut);font-size:12px;margin:8px 0}
.bar{height:8px;border-radius:4px;background:#3a332a;margin:10px 0 4px;overflow:hidden}
.bar div{height:100%;width:0%;background:linear-gradient(90deg,var(--amber2),var(--amber));transition:width .2s}
#msg{font-size:11px;color:var(--mut);min-height:14px}
.fswrap{display:flex;flex-direction:column;align-items:center;padding:18px 0 14px}
#fsw{width:96px;height:96px;border-radius:50%;cursor:pointer;border:5px solid #4a4133;
background:radial-gradient(circle at 35% 30%,#5c5346,#2c2721 70%);
box-shadow:0 6px 20px #000c,inset 0 2px 6px #fff2;transition:.2s}
#fsw.on{border-color:var(--amber2);background:radial-gradient(circle at 35% 30%,#ffcf7e,var(--amber2) 75%);
box-shadow:0 0 34px #ff9a2b88,0 6px 20px #000c}
#fswtxt{margin-top:12px;font-size:13px;letter-spacing:4px;font-weight:800;color:var(--mut)}
#fsw.on+#fswtxt{color:var(--amber)}
footer{text-align:center;font-size:10px;color:#5c5546;letter-spacing:1px;margin-top:6px}
</style></head><body>
<div class="pedal">
<header>
  <div><h1>S<b>O</b>AJ</h1><div class="sub">CAPTURE NAM / TONE3000 &middot; 1 ESP32</div></div>
  <div class="link"><span id="lktxt">liaison</span><span class="dot" id="dot"></span></div>
</header>

<div class="card">
  <div class="row"><label>Capture charg&eacute;e</label></div>
  <div class="pname" id="pnom">&mdash;</div>
  <div class="hint">t&eacute;l&eacute;chargez un fichier .nam sur tone3000.com AVANT de vous connecter
  &agrave; la p&eacute;dale, puis importez-le ici : le t&eacute;l&eacute;phone ex&eacute;cute le r&eacute;seau NAM,
  en extrait le profil et l'envoie &agrave; la p&eacute;dale (grav&eacute; en flash).</div>
  <input type="file" id="fnam" accept=".nam,application/json">
  <button class="b go" id="analyser" disabled>ANALYSER ET ENVOYER</button>
  <div class="bar"><div id="prog"></div></div>
  <div id="msg"></div>
</div>

<div class="card">
  <div class="ctl"><div class="row"><label for="g">Drive</label><output id="og">0.50</output></div>
    <input type="range" id="g" min="0" max="1" step="0.01" value="0.5">
    <div class="hint">pousse la courbe de la capture (course logarithmique)</div></div>
  <div class="ctl"><div class="row"><label for="d">Intensit&eacute;</label><output id="od">1.00</output></div>
    <input type="range" id="d" min="0" max="1" step="0.01" value="1">
    <div class="hint">0 = CLEAN PUR (signal brut) &rarr; 1 = caract&egrave;re complet de la capture</div></div>
  <div class="ctl"><div class="row"><label for="t">Tone</label><output id="ot">0.60</output></div>
    <input type="range" id="t" min="0" max="1" step="0.01" value="0.6">
    <div class="hint">0 = sombre, 1 = brillant</div></div>
  <div class="ctl"><div class="row"><label for="v">Volume</label><output id="ov">0.50</output></div>
    <input type="range" id="v" min="0" max="1" step="0.01" value="0.5">
    <div class="hint">niveau de sortie</div></div>
</div>

<div class="card fswrap">
  <button id="fsw" aria-label="effet"></button>
  <div id="fswtxt">BYPASS</div>
</div>
<footer>http://192.168.4.1 &middot; http://soaj-nam.local &mdash; SOAJ NAM ESP32<br>
page inaccessible ? couper les donn&eacute;es mobiles et rester sur ce WiFi</footer>
</div>

<script>
// ===========================================================================
// UI : jauges + liaison (m&ecirc;me logique que les autres p&eacute;dales SOAJ)
// ===========================================================================
var KEYS=['g','d','t','v'];
var st={g:0.5,d:1,t:0.6,v:0.5,e:1};
var timer=null,lastEdit=0;
function $(id){return document.getElementById(id)}
function paint(){
  KEYS.forEach(function(k){
    var el=$(k);el.value=st[k];
    el.style.setProperty('--p',(100*st[k])+'%');
    $('o'+k).textContent=Number(st[k]).toFixed(2);
  });
  $('fsw').className=(st.e==1)?'on':'';
  $('fswtxt').textContent=(st.e==1)?'EFFET ON':'BYPASS';
  if(st.nom)$('pnom').textContent=st.nom;
}
function link(ok){
  $('dot').className='dot'+(ok?' on':'');
  $('lktxt').textContent=ok?'connecté':'hors ligne';
}
function push(){
  clearTimeout(timer);
  timer=setTimeout(function(){
    var q=KEYS.map(function(k){return k+'='+st[k]}).join('&');
    fetch('/api/set?'+q+'&e='+st.e)
      .then(function(r){link(r.ok)}).catch(function(){link(false)});
  },120);
}
KEYS.forEach(function(k){
  $(k).addEventListener('input',function(){
    st[k]=+this.value;lastEdit=Date.now();paint();push();
  });
});
$('fsw').addEventListener('click',function(){
  st.e=st.e==1?0:1;lastEdit=Date.now();paint();push();
});
function poll(){
  fetch('/api/get').then(function(r){return r.json()}).then(function(j){
    link(true);
    if(Date.now()-lastEdit>2000){st=j;paint();}
  }).catch(function(){link(false)});
}
poll();setInterval(poll,3000);

// ===========================================================================
// MOTEUR NAM : ex&eacute;cute le r&eacute;seau du fichier .nam, &eacute;chantillon par
// &eacute;chantillon. Disposition des poids conforme &agrave; NeuralAmpModelerCore
// (WaveNet : rechannel, [conv dilat&eacute;e, mixin, 1x1]xN, head_rechannel,
// head_scale ; LSTM : [W 4h x (e+h) ligne par ligne, b, h0, c0]xN, t&ecirc;te).
// ===========================================================================
function sigmoid(v){return 1/(1+Math.exp(-v));}
function actFn(nom){
  if(nom==='Tanh')return Math.tanh;
  if(nom==='ReLU')return function(v){return v>0?v:0};
  if(nom==='Sigmoid')return sigmoid;
  if(nom==='Hardtanh')return function(v){return v<-1?-1:(v>1?1:v)};
  if(nom==='LeakyReLU')return function(v){return v>0?v:0.01*v};
  throw 'Activation non geree : '+nom;
}
function creerModele(nam){
  var W=nam.weights,pos=0;
  function lire(n){var a=new Float32Array(n);for(var i=0;i<n;i++)a[i]=W[pos++];return a;}
  var sr=nam.sample_rate||48000;

  if(nam.architecture==='LSTM'){
    var c=nam.config,H=c.hidden_size,cells=[];
    for(var l=0;l<c.num_layers;l++){
      var ins=(l===0)?c.input_size:H;
      cells.push({ins:ins,W:lire(4*H*(ins+H)),b:lire(4*H),h:lire(H),c:lire(H),
                  xh:new Float32Array(ins+H),ifgo:new Float32Array(4*H)});
    }
    var headW=lire(H),headB=lire(1)[0];
    return {sr:sr,prechauffe:Math.floor(0.5*sr),reste:W.length-pos,
      traiter:function(x){
        for(var l=0;l<cells.length;l++){
          var cl=cells[l],ins=cl.ins,cols=ins+H,i,j;
          for(i=0;i<ins;i++)cl.xh[i]=(l===0)?x:cells[l-1].h[i];
          for(i=0;i<H;i++)cl.xh[ins+i]=cl.h[i];
          for(var r=0;r<4*H;r++){
            var s=cl.b[r],ro=r*cols;
            for(j=0;j<cols;j++)s+=cl.W[ro+j]*cl.xh[j];
            cl.ifgo[r]=s;
          }
          for(i=0;i<H;i++){
            cl.c[i]=sigmoid(cl.ifgo[H+i])*cl.c[i]
                   +sigmoid(cl.ifgo[i])*Math.tanh(cl.ifgo[2*H+i]);
            cl.h[i]=sigmoid(cl.ifgo[3*H+i])*Math.tanh(cl.c[i]);
          }
        }
        var hf=cells[cells.length-1].h,s2=headB;
        for(var k=0;k<H;k++)s2+=headW[k]*hf[k];
        return s2;
      }};
  }

  if(nam.architecture!=='WaveNet')throw 'Architecture non geree : '+nam.architecture;
  var arrs=[];
  nam.config.layers.forEach(function(lc){
    var C=lc.channels,K=lc.kernel_size,gated=lc.gated;
    var A={C:C,inSize:lc.input_size,headSize:lc.head_size,gated:gated,
           act:actFn(lc.activation),rech:lire(C*lc.input_size),layers:[]};
    lc.dilations.forEach(function(d){
      var out=gated?2*C:C;
      // Conv1D : l'export aplatit pour i(sortie) pour j(entree) pour k(tap)
      var convW=new Float32Array(out*C*K);
      for(var i=0;i<out;i++)for(var j=0;j<C;j++)for(var k=0;k<K;k++)
        convW[(k*out+i)*C+j]=W[pos++];
      var bl=(K-1)*d+1;
      A.layers.push({d:d,K:K,out:out,convW:convW,convB:lire(out),
                     mix:lire(out*lc.condition_size),oneW:lire(C*C),oneB:lire(C),
                     hist:new Float32Array(bl*C),bl:bl,pos:0,z:new Float32Array(out)});
    });
    A.headRechW=lire(lc.head_size*C);
    A.headRechB=lc.head_bias?lire(lc.head_size):null;
    arrs.push(A);
  });
  var headScale=W[pos++];
  var rf=1;
  arrs.forEach(function(A){A.layers.forEach(function(L){rf+=(L.K-1)*L.d;});});
  return {sr:sr,prechauffe:rf,reste:W.length-pos,
    traiter:function(x){
      var vecIn=[x],headCarry=null,a,l,i,j,k;
      for(a=0;a<arrs.length;a++){
        var A=arrs[a],C=A.C;
        var cur=new Float32Array(C);
        for(i=0;i<C;i++){var s=0;for(j=0;j<A.inSize;j++)s+=A.rech[i*A.inSize+j]*vecIn[j];cur[i]=s;}
        var headAcc=new Float32Array(C);
        if(a>0)for(i=0;i<C;i++)headAcc[i]=headCarry[i];
        for(l=0;l<A.layers.length;l++){
          var L=A.layers[l],p=L.pos,out=L.out,z=L.z;
          for(i=0;i<C;i++)L.hist[p*C+i]=cur[i];
          for(i=0;i<out;i++)z[i]=L.convB[i]+L.mix[i]*x;
          for(k=0;k<L.K;k++){
            var q=p-(L.K-1-k)*L.d;q%=L.bl;if(q<0)q+=L.bl;
            var hb=q*C,wb=k*out;
            for(i=0;i<out;i++){
              var t=0,ro=(wb+i)*C;
              for(j=0;j<C;j++)t+=L.convW[ro+j]*L.hist[hb+j];
              z[i]+=t;
            }
          }
          if(A.gated){for(i=0;i<C;i++)z[i]=A.act(z[i])*sigmoid(z[C+i]);}
          else{for(i=0;i<out;i++)z[i]=A.act(z[i]);}
          var nxt=new Float32Array(C);
          for(i=0;i<C;i++){
            headAcc[i]+=z[i];
            var u=L.oneB[i],r2=i*C;
            for(j=0;j<C;j++)u+=L.oneW[r2+j]*z[j];
            nxt[i]=cur[i]+u;
          }
          L.pos=(p+1)%L.bl;
          cur=nxt;
        }
        var hs=A.headSize,hc=new Float32Array(hs);
        for(i=0;i<hs;i++){
          var v=A.headRechB?A.headRechB[i]:0,r3=i*C;
          for(j=0;j<C;j++)v+=A.headRechW[r3+j]*headAcc[j];
          hc[i]=v;
        }
        headCarry=hc;vecIn=cur;
      }
      return headScale*headCarry[0];
    }};
}

// ===========================================================================
// CALIBRATION : mesure la courbe d'&eacute;cr&ecirc;tage (sinus 220 Hz en rampe,
// cr&ecirc;tes + et − s&eacute;par&eacute;es = asym&eacute;trie) et la r&eacute;ponse lin&eacute;aire
// (sinus par paliers + Goertzel) -> profil pour l'ESP32.
// ===========================================================================
var LUT_T=257,LUT_P=4.0;
function goertzel(buf,debut,n,f,sr){
  var w=2*Math.PI*f/sr,cw=Math.cos(w),c=2*cw,s0,s1=0,s2=0;
  for(var i=0;i<n;i++){s0=buf[debut+i]+c*s1-s2;s2=s1;s1=s0;}
  var re=s1-s2*cw,im=s2*Math.sin(w);
  return Math.sqrt(re*re+im*im);
}
function interpDemi(entrees,sorties,u){
  // interpolation lineaire sur paires triees ; plat au-dela, 0 en-deca
  var n=entrees.length;
  if(n===0)return 0;
  if(u<=entrees[0])return sorties[0]*(u/Math.max(entrees[0],1e-9));
  if(u>=entrees[n-1])return sorties[n-1];
  var lo=0,hi=n-1;
  while(hi-lo>1){var m=(lo+hi)>>1;if(entrees[m]<=u)lo=m;else hi=m;}
  var fr=(u-entrees[lo])/(entrees[hi]-entrees[lo]);
  return sorties[lo]+fr*(sorties[hi]-sorties[lo]);
}
function calibrer(modele,nom,progression,fini,erreur){
  var sr=modele.sr;
  var nRampe=Math.floor(1.5*sr),per=Math.round(sr/220);
  var freqs=[],nF=16,f0=40,f1=Math.min(10000,0.45*sr);
  for(var i=0;i<nF;i++)freqs.push(f0*Math.pow(f1/f0,i/(nF-1)));
  var nParF=3584,nGo=2048;
  var total=modele.prechauffe+nRampe+nF*nParF,fait=0;
  var rIn=new Float32Array(nRampe),rOut=new Float32Array(nRampe);
  var sIn=new Float32Array(nParF),sOut=new Float32Array(nParF);
  var dbs=[];
  var phase=0,idx=0,iF=0;   // phase 0 : prechauffe, 1 : rampe, 2 : sinus

  function pompe(){
    try{
      var bloc=8192;
      while(bloc-->0){
        if(phase===0){
          modele.traiter(0);
          if(++idx>=modele.prechauffe){phase=1;idx=0;}
        }else if(phase===1){
          var amp=0.95*idx/nRampe;
          var xin=amp*Math.sin(2*Math.PI*220*idx/sr);
          rIn[idx]=xin;rOut[idx]=modele.traiter(xin);
          if(++idx>=nRampe){phase=2;idx=0;}
        }else{
          var f=freqs[iF];
          var xs=0.02*Math.sin(2*Math.PI*f*idx/sr);
          sIn[idx]=xs;sOut[idx]=modele.traiter(xs);
          if(++idx>=nParF){
            var d0=nParF-nGo;
            var gi=goertzel(sIn,d0,nGo,f,sr),go=goertzel(sOut,d0,nGo,f,sr);
            dbs.push(20*Math.log10(Math.max(go,1e-9)/Math.max(gi,1e-9)));
            idx=0;
            if(++iF>=nF){terminer();return;}
          }
        }
        fait++;
      }
      progression(Math.floor(100*fait/total));
      setTimeout(pompe,0);
    }catch(e){erreur(''+e);}
  }

  function terminer(){
    // --- courbe statique : cretes +/− par periode (robuste au dephasage) ---
    var eP=[],sP=[],eN=[],sN=[],nper=Math.floor(nRampe/per);
    for(var i2=3;i2<nper;i2++){
      var d=i2*per,f2=d+per,mIn=-9,MIn=9,mo=0,MO=-9,mO=9,j2;
      var moy=0;
      for(j2=d;j2<f2;j2++)moy+=rOut[j2];
      moy/=per;
      var maxI=0,minI=0,maxO=-1e9,minO=1e9;
      for(j2=d;j2<f2;j2++){
        if(rIn[j2]>maxI)maxI=rIn[j2];
        if(rIn[j2]<minI)minI=rIn[j2];
        if(rOut[j2]>maxO)maxO=rOut[j2];
        if(rOut[j2]<minO)minO=rOut[j2];
      }
      eP.push(maxI);sP.push(maxO-moy);
      eN.push(-minI);sN.push(moy-minO);
    }
    var echelle=(LUT_P/2)/0.95;
    var lut=new Array(LUT_T),crete=0;
    for(var i3=0;i3<LUT_T;i3++){
      var u=-LUT_P+2*LUT_P*i3/(LUT_T-1),v;
      if(u>=0)v=interpDemi(eP.map(function(a){return a*echelle}),sP,u);
      else v=-interpDemi(eN.map(function(a){return a*echelle}),sN,-u);
      lut[i3]=v;
      if(Math.abs(v)>crete)crete=Math.abs(v);
    }
    if(crete<1e-6){erreur('sortie du modele muette');return;}
    for(var i4=0;i4<LUT_T;i4++)lut[i4]=Math.round(lut[i4]*0.95/crete*1e5)/1e5;
    var sortieGain=Math.min(Math.max(crete/0.95,0.05),1.5);

    // --- reponse lineaire : reference, coins -3 dB, bosse de mediums ---
    var refs=[],i5;
    for(i5=0;i5<nF;i5++)if(freqs[i5]>=400&&freqs[i5]<=1200)refs.push(dbs[i5]);
    if(refs.length===0)refs=dbs.slice();
    refs.sort(function(a,b){return a-b});
    var ref=refs[Math.floor(refs.length/2)];
    var bas=40,haut=8000;
    for(i5=0;i5<nF;i5++)if(dbs[i5]>=ref-3){bas=Math.max(freqs[i5],40);break;}
    for(i5=nF-1;i5>=0;i5--)if(dbs[i5]>=ref-3){haut=Math.min(freqs[i5],8000);break;}
    var midDb=0,midHz=700,zLo=Math.max(300,2*bas),zHi=Math.min(2000,haut/2);
    for(i5=0;i5<nF;i5++){
      if(freqs[i5]<zLo||freqs[i5]>zHi)continue;
      var ec=dbs[i5]-ref;
      if(Math.abs(ec)>=1.5&&Math.abs(ec)>Math.abs(midDb)){
        midDb=Math.max(-12,Math.min(12,ec));midHz=freqs[i5];
      }
    }
    fini({nom:nom.substring(0,23),
          preHpfHz:Math.round(bas),preLpfHz:Math.round(Math.min(haut,5000)),
          gainMin:2,gainMax:30,
          postMidHz:Math.round(midHz),postMidQ:0.8,postMidDb:Math.round(midDb*10)/10,
          postLpfHz:Math.round(Math.max(haut,2000)),
          sortieGain:Math.round(sortieGain*100)/100,lut:lut});
  }
  pompe();
}

// ===========================================================================
// Import : lecture du .nam -> moteur -> calibration -> POST /api/profil
// ===========================================================================
var fichierNam=null;
$('fnam').addEventListener('change',function(){
  fichierNam=this.files[0]||null;
  $('analyser').disabled=!fichierNam;
  $('msg').textContent=fichierNam?('fichier : '+fichierNam.name):'';
});
$('analyser').addEventListener('click',function(){
  if(!fichierNam)return;
  var btn=$('analyser');btn.disabled=true;
  $('msg').textContent='lecture du fichier...';
  fichierNam.text().then(function(txt){
    var nam=JSON.parse(txt);
    var modele=creerModele(nam);
    if(modele.reste!==0)throw 'fichier .nam invalide ('+modele.reste+' poids inattendus)';
    var nom=(nam.metadata&&nam.metadata.name)?nam.metadata.name
            :fichierNam.name.replace(/\.nam$/i,'');
    $('msg').textContent='analyse du reseau ('+nam.architecture+', '
                         +nam.weights.length+' poids)...';
    calibrer(modele,nom,
      function(pct){$('prog').style.width=pct+'%';},
      function(profil){
        $('msg').textContent='envoi du profil a la pedale...';
        fetch('/api/profil',{method:'POST',body:JSON.stringify(profil)})
          .then(function(r){if(!r.ok)throw 'HTTP '+r.status;return r.json();})
          .then(function(j){
            $('prog').style.width='100%';
            $('msg').textContent='profil "'+j.nom+'" charge et grave en flash.';
            st.nom=j.nom;paint();btn.disabled=false;
          })
          .catch(function(e){$('msg').textContent='echec envoi : '+e;btn.disabled=false;});
      },
      function(e){$('msg').textContent='echec analyse : '+e;btn.disabled=false;});
  }).catch(function(e){$('msg').textContent='fichier illisible : '+e;btn.disabled=false;});
});
</script>
</body></html>
)rawliteral";

// ---------------------------------------------------------------------------
// Serveur web
// ---------------------------------------------------------------------------
static void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

static void handleApiGet() {
  char buf[160];
  snprintf(buf, sizeof(buf),
           "{\"g\":%.2f,\"d\":%.2f,\"t\":%.2f,\"v\":%.2f,\"e\":%d,\"nom\":\"%s\"}",
           (double)tgtDrive, (double)tgtMix, (double)tgtTone, (double)tgtVolume,
           (tgtEffect > 0.5f) ? 1 : 0, profilActif.nom);
  server.send(200, "application/json", buf);
}

static void handleApiSet() {
  if (server.hasArg("g")) tgtDrive  = clampf(server.arg("g").toFloat(), 0.0f, 1.0f);
  if (server.hasArg("d")) tgtMix    = clampf(server.arg("d").toFloat(), 0.0f, 1.0f);
  if (server.hasArg("t")) tgtTone   = clampf(server.arg("t").toFloat(), 0.0f, 1.0f);
  if (server.hasArg("v")) tgtVolume = clampf(server.arg("v").toFloat(), 0.0f, 1.0f);
  if (server.hasArg("e")) tgtEffect = (server.arg("e").toInt() >= 1) ? 1.0f : 0.0f;
  handleApiGet();
}

// POST /api/profil : profil extrait du .nam par le téléphone
static void handleApiProfil() {
  const String &corps = server.arg("plain");
  ProfilNam p;
  if (!parseProfilJson(corps.c_str(), &p)) {
    server.send(400, "application/json", "{\"erreur\":\"profil invalide\"}");
    return;
  }
  memcpy(&profilRecu, &p, sizeof(p));
  profilEnAttente = true;              // la tâche audio l'échange en fondu
  sauverProfilNVS(&p);                 // survivra à l'extinction (écriture
                                       // flash : bref craquement possible)
  Serial.printf("[Profil] \"%s\" reçu : HPF %.0f Hz, LPF %.0f Hz, "
                "mid %+.1f dB @ %.0f Hz, sortie x%.2f\n",
                p.nom, p.preHpfHz, p.preLpfHz, p.postMidDb, p.postMidHz,
                p.sortieGain);
  char buf[64];
  snprintf(buf, sizeof(buf), "{\"nom\":\"%s\"}", p.nom);
  server.send(200, "application/json", buf);
}

// Toute URL inconnue redirige vers la page (portail captif). "no-store" :
// certains téléphones mettent en cache la réponse de leur sonde de
// connectivité et ne re-testent plus — on l'interdit.
static void handleNotFound() {
  server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
  server.sendHeader("Cache-Control", "no-store");
  server.send(302, "text/plain", "");
}

// Sondes de détection de portail captif : à la connexion, chaque OS teste
// une URL précise pour savoir s'il y a Internet. On répond NOUS-MÊMES une
// redirection vers la page : le téléphone ouvre alors sa fenêtre
// « connexion au réseau » — AUCUN accès Internet n'est nécessaire.
// (onNotFound les couvrirait déjà ; les déclarer rend l'intention explicite
// et garantit le comportement.)
static void routesPortailCaptif() {
  const char *sondes[] = {
    "/generate_204", "/gen_204",                      // Android
    "/hotspot-detect.html", "/library/test/success.html",  // iOS / macOS
    "/connecttest.txt", "/ncsi.txt", "/redirect",     // Windows
    "/canonical.html", "/success.txt",                // Firefox
  };
  for (size_t i = 0; i < sizeof(sondes) / sizeof(sondes[0]); i++)
    server.on(sondes[i], handleNotFound);
}

// ---------------------------------------------------------------------------
// Tâches : audio serré sur le cœur 1, web/DNS sur le cœur 0
// ---------------------------------------------------------------------------
#define SD_STEP_US 25

static void tacheAudio(void *arg) {
  (void)arg;
  uint32_t nextSampleUs = micros();
  uint32_t lastStepUs   = 0;
  for (;;) {
    const uint32_t now = micros();
    if ((int32_t)(now - nextSampleUs) >= 0) {
      nextSampleUs += SAMPLE_PERIOD_US;
      if ((int32_t)(now - nextSampleUs) > 1000) nextSampleUs = now + SAMPLE_PERIOD_US;
      processSample();
      lastStepUs = micros();
    } else if ((uint32_t)(now - lastStepUs) >= SD_STEP_US) {
      dacStep();                       // pas sigma-delta intermédiaire (~40 kHz)
      lastStepUs = now;
    }
  }
}

static void tacheWeb(void *arg) {
  (void)arg;
  for (;;) {
    dnsServer.processNextRequest();
    server.handleClient();
    delay(2);                          // laisse respirer la pile WiFi
  }
}

// ---------------------------------------------------------------------------
// Initialisation
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== SOAJ NAM — 1 ESP32, captures TONE3000 via le téléphone ===");

  // --- ADC / DAC ---
#ifdef USE_LEGACY_ADC
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
#else
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_GUITAR_IN, ADC_11db);
#endif
  dacWrite(PIN_AUDIO_OUT, 128);

  // --- Profil : depuis la flash, sinon neutre ---
  prefs.begin("soajnam", false);
  if (chargerProfilNVS(&profilActif)) {
    Serial.printf("[Profil] restauré depuis la flash : \"%s\"\n", profilActif.nom);
  } else {
    profilNeutre(&profilActif);
    Serial.println("[Profil] aucun en flash : profil NEUTRE (tanh) chargé");
  }
  chargerFiltresProfil();
  driveGain = profilActif.gainMin
              * powf(profilActif.gainMax / profilActif.gainMin, smDrive);

  // --- Filtres fixes ---
  {
    const float tauIn = 1.0f / (2.0f * (float)M_PI * IN_HPF_HZ);
    bilinear1(&fInHp, 0.0f, tauIn, 1.0f, tauIn);
    const float rc = 1.0f / (2.0f * (float)M_PI * 120.0f);
    envHpA = rc / (rc + DT_SEC);
  }

  // --- WiFi : point d'accès + portail captif + serveur ---
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD, WIFI_CHANNEL);
  Serial.printf("[WiFi] Point d'accès \"%s\" (mdp \"%s\") — http://", AP_SSID, AP_PASSWORD);
  Serial.println(WiFi.softAPIP());

  // DNS captif : TOUTE résolution renvoie l'IP de la pédale, y compris les
  // requêtes malformées (NoError + TTL court : certains téléphones gardent
  // sinon un échec en cache et concluent « pas d'Internet, pas de portail »)
  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.setTTL(30);
  dnsServer.start(53, "*", WiFi.softAPIP());

  // mDNS : accès de secours SANS DNS captif ni Internet — http://soaj-nam.local
  // (utile si le téléphone route les IP inconnues vers la 4G/5G)
  if (MDNS.begin("soaj-nam")) MDNS.addService("http", "tcp", 80);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/get", HTTP_GET, handleApiGet);
  server.on("/api/set", HTTP_GET, handleApiSet);
  server.on("/api/profil", HTTP_POST, handleApiProfil);
  routesPortailCaptif();                 // sondes Android/iOS/Windows/Firefox
  server.onNotFound(handleNotFound);
  server.begin();

  Serial.println("[Web] Si la page ne s'ouvre pas toute seule :");
  Serial.println("      1. COUPER les données mobiles (4G/5G) le temps du réglage");
  Serial.println("      2. Rester connecté au WiFi même si « pas d'Internet »");
  Serial.println("      3. Taper http://192.168.4.1 (avec http:// !) ou http://soaj-nam.local");

  // --- Stabilisation de l'offset DC avant de sortir du son ---
  for (int i = 0; i < 4000; i++) {
    const int raw = readGuitarAdc();
    dcOffset += 0.01f * ((float)raw - dcOffset);
    delayMicroseconds(50);
  }
  Serial.printf("Offset DC mesuré : %.0f (attendu ~2048)\n", dcOffset);
  if (dcOffset < 1200.0f || dcOffset > 2900.0f) {
    Serial.println("ATTENTION : offset DC anormal — vérifiez le pont diviseur");
  }

  // --- Tâches : audio (cœur 1, boucle serrée), web (cœur 0) ---
  disableCore1WDT();       // la boucle audio ne rend jamais la main
  xTaskCreatePinnedToCore(tacheAudio, "audio", 8192, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(tacheWeb,   "web",   8192, NULL, 1, NULL, 0);

  Serial.printf("Audio : %d Hz — drive -> filtres du profil -> courbe NAM -> tone -> volume\n",
                SAMPLE_RATE_HZ);
}

void loop() {
  vTaskDelay(portMAX_DELAY);           // tout vit dans les deux tâches
}
