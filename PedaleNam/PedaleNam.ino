/*
 * ============================================================================
 *  SOAJ - Pédale d'effet guitare ESP32
 *  PedaleNam.ino  —  PROJET AUTONOME : 1 ESP32 + catalogue TONE3000 intégré
 * ============================================================================
 *
 *                     Internet (4G du téléphone)
 *                          ▲
 *                          │ partage de connexion (hotspot)
 *   TÉLÉPHONE ─────────────┤
 *     navigateur ──────────┼──> page web DANS l'ESP32 (http://soaj-nam.local)
 *       │  « CHOISIR SUR TONE3000 » → catalogue officiel tone3000.com
 *       ▼                  │
 *   ESP32 (station WiFi) ──┘   + point d'accès SOAJ-NAM (secours/config)
 *       │   ① relaie l'API TONE3000 (OAuth PKCE, HTTPS)
 *       │   ② télécharge la capture .nam choisie (LittleFS)
 *       │   ③ la page l'analyse sur le TÉLÉPHONE (moteur NAM en JS)
 *       │   ④ le profil (~2,5 Ko) est gravé dans un des 8 SLOTS (NVS)
 *       ▼
 *   Guitare -> GPIO34 (ADC) … GPIO25 (DAC) -> Ampli  (audio 100 % local)
 *
 *  L'expérience visée (« plug and play ») :
 *    - Une fois : entrer le nom/mot de passe du partage de connexion dans la
 *      page + coller sa clé API TONE3000 (tone3000.com -> Réglages -> API
 *      Keys -> clé publiable t3k_pub_…).
 *    - Ensuite : hotspot ON -> pédale ON -> page -> « CHOISIR SUR TONE3000 »
 *      -> le VRAI catalogue s'ouvre (le téléphone a Internet) -> choisir une
 *      pédale -> retour automatique -> la pédale télécharge la capture,
 *      le téléphone l'analyse (moteur NAM JavaScript, WaveNet + LSTM),
 *      le profil est gravé dans le slot choisi. Les 8 slots se rappellent
 *      ensuite SANS hotspot ni Internet.
 *
 *  Pourquoi cette répartition :
 *    - un réseau de neurones NAM ne peut pas tourner en temps réel sur un
 *      ESP32 -> l'analyse (une seule fois, non temps réel) se fait sur le
 *      téléphone, qui est fait pour ça ;
 *    - l'ESP32 relaie les appels API (jetons OAuth gardés dans la pédale,
 *      aucun souci de CORS, n'importe quel téléphone peut piloter) ;
 *    - l'audio reste local et n'est JAMAIS interrompu (cœur 1 dédié).
 *
 *  Cartes/outils : ELEGOO ESP32 (NodeMCU-like) — Arduino IDE, carte
 *  « ESP32 Dev Module », schéma de partition par défaut (l'appli + LittleFS
 *  pour le fichier .nam téléchargé). Câblage : voir README (pont diviseur
 *  1,65 V sur GPIO34, filtre RC sur GPIO25).
 * ============================================================================
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <LittleFS.h>
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

// --- TONE3000 ---------------------------------------------------------------
#define T3K_HOTE            "https://www.tone3000.com"
// 1 = vérifier le certificat (racine ISRG/Let's Encrypt embarquée plus bas).
// Si TONE3000 change un jour d'autorité de certification, passer à 0 le
// temps d'une mise à jour (connexion toujours chiffrée, mais non vérifiée).
#define T3K_TLS_STRICT      1
#define FICHIER_NAM         "/capture.nam"   // capture téléchargée (LittleFS)

// --- Slots de profils mémorisés ----------------------------------------------
#define NB_SLOTS 8

// ---------------------------------------------------------------------------
// Profil de pédale : ce que le téléphone extrait du .nam et nous envoie
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

static uint8_t slotActif = 0;
static char    nomsSlots[NB_SLOTS][24];   // cache des noms ("" = slot vide)

// Cibles pilotées par la page web
static volatile float tgtDrive  = 0.5f;   // g : position du pot drive (0..1)
static volatile float tgtMix    = 1.0f;   // d : intensité (0 = clean pur)
static volatile float tgtTone   = 0.6f;   // t
static volatile float tgtVolume = 0.5f;   // v
static volatile float tgtEffect = 1.0f;   // e : 0 = bypass, 1 = effet

static Preferences prefs;
static WebServer   server(80);
static DNSServer   dnsServer;

// Identifiants du partage de connexion (mémorisés en flash)
static char hsSsid[33] = "";
static char hsMdp[65]  = "";

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

// Médiane de 3 lectures : élimine les pics parasites radio de l'ADC
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
// Profil : bornage, filtres, profil neutre, slots NVS
// ---------------------------------------------------------------------------
static void bornerProfil(ProfilNam *p) {
  p->nom[sizeof(p->nom) - 1] = '\0';
  // Le nom part dans du JSON construit à la main : on neutralise " et antislash
  for (size_t i = 0; p->nom[i]; i++)
    if (p->nom[i] == '"' || p->nom[i] == '\\') p->nom[i] = '\'';
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

// --- Slots NVS : 8 profils mémorisés, survivent à l'extinction ---
static void cleSlot(int n, char *buf, size_t taille) {
  snprintf(buf, taille, "prof%d", n);
}

static bool chargerSlotNVS(int n, ProfilNam *p) {
  char cle[12];
  cleSlot(n, cle, sizeof(cle));
  if (prefs.getBytesLength(cle) != sizeof(*p)) return false;
  prefs.getBytes(cle, p, sizeof(*p));
  bornerProfil(p);
  return true;
}

static void sauverSlotNVS(int n, const ProfilNam *p) {
  char cle[12];
  cleSlot(n, cle, sizeof(cle));
  prefs.putBytes(cle, p, sizeof(*p));
  strncpy(nomsSlots[n], p->nom, sizeof(nomsSlots[n]) - 1);
  nomsSlots[n][sizeof(nomsSlots[n]) - 1] = '\0';
}

static void chargerNomsSlots() {
  ProfilNam tmp;
  for (int n = 0; n < NB_SLOTS; n++) {
    nomsSlots[n][0] = '\0';
    if (chargerSlotNVS(n, &tmp)) {
      strncpy(nomsSlots[n], tmp.nom, sizeof(nomsSlots[n]) - 1);
      nomsSlots[n][sizeof(nomsSlots[n]) - 1] = '\0';
    }
  }
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

// Extrait une valeur chaîne : "cle":"valeur"
static bool jsonChaine(const char *s, const char *cle, char *out, size_t taille) {
  const char *p = strstr(s, cle);
  if (!p) return false;
  p = strchr(p + strlen(cle), ':');
  if (!p) return false;
  p = strchr(p, '"');
  if (!p) return false;
  p++;
  size_t i = 0;
  while (*p && *p != '"' && i < taille - 1) out[i++] = *p++;
  out[i] = '\0';
  return true;
}

static bool parseProfilJson(const char *s, ProfilNam *p) {
  memset(p, 0, sizeof(*p));

  if (!jsonChaine(s, "\"nom\"", p->nom, sizeof(p->nom)) || p->nom[0] == '\0')
    strncpy(p->nom, "SANS NOM", sizeof(p->nom) - 1);

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
// Sigma-delta du DAC + dither TPDF (identique aux autres pédales)
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

  // --- Noise gate : enveloppe mesurée derrière un passe-haut 120 Hz ---
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
// Client TONE3000 : l'ESP32 relaie l'API officielle (HTTPS + OAuth PKCE).
// Les jetons vivent dans la pédale (NVS) : n'importe quel téléphone pilote,
// et la page n'a aucun souci de CORS (elle ne parle qu'à l'ESP32).
// ---------------------------------------------------------------------------
// Racine ISRG Root X1 (Let's Encrypt — utilisée par tone3000.com), valable
// jusqu'en 2035. Source : paquet certifi (bundle Mozilla).
static const char ISRG_ROOT_X1[] PROGMEM =
  "-----BEGIN CERTIFICATE-----\n"
  "MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw\n"
  "TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh\n"
  "cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4\n"
  "WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu\n"
  "ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY\n"
  "MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc\n"
  "h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+\n"
  "0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U\n"
  "A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW\n"
  "T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH\n"
  "B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC\n"
  "B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv\n"
  "KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn\n"
  "OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn\n"
  "jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw\n"
  "qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI\n"
  "rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV\n"
  "HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq\n"
  "hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL\n"
  "ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ\n"
  "3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK\n"
  "NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5\n"
  "ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur\n"
  "TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC\n"
  "jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc\n"
  "oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq\n"
  "4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA\n"
  "mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d\n"
  "emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=\n"
  "-----END CERTIFICATE-----\n";

// Encodage pour corps x-www-form-urlencoded (suffisant pour nos valeurs)
static String urlEncode(const String &s) {
  String out;
  const char *hex = "0123456789ABCDEF";
  for (size_t i = 0; i < s.length(); i++) {
    const char c = s.c_str()[i];
    if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
        (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.' || c == '~') {
      out += c;
    } else {
      out += '%';
      out += hex[(c >> 4) & 0xF];
      out += hex[c & 0xF];
    }
  }
  return out;
}

// Requête HTTPS vers tone3000.com. corps vide = GET, sinon POST formulaire.
// Retourne le code HTTP (<= 0 = échec réseau/TLS) et remplit `reponse`.
static int t3kRequete(const String &url, const String &corps,
                      const String &bearer, String &reponse) {
  WiFiClientSecure tls;
#if T3K_TLS_STRICT
  tls.setCACert(ISRG_ROOT_X1);
#else
  tls.setInsecure();                 // chiffré mais non vérifié (dépannage)
#endif
  HTTPClient http;
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  http.setTimeout(20000);
  if (!http.begin(tls, url)) return -1;
  if (bearer.length()) http.addHeader("Authorization", String("Bearer ") + bearer);
  int code;
  if (corps.length()) {
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");
    code = http.POST(corps);
  } else {
    code = http.GET();
  }
  if (code > 0) reponse = http.getString();
  http.end();
  return code;
}

// Échange (ou rafraîchit) les jetons OAuth et les grave en NVS
static bool t3kStockerJetons(const String &reponse) {
  char acc[512], ref[512];
  if (!jsonChaine(reponse.c_str(), "\"access_token\"", acc, sizeof(acc))) return false;
  if (!jsonChaine(reponse.c_str(), "\"refresh_token\"", ref, sizeof(ref))) return false;
  prefs.putString("t3kAcc", acc);
  prefs.putString("t3kRef", ref);
  return true;
}

static bool t3kRafraichir() {
  const String ref = prefs.getString("t3kRef", "");
  const String cle = prefs.getString("t3kCle", "");
  if (!ref.length() || !cle.length()) return false;
  String rep;
  const String corps = String("grant_type=refresh_token&refresh_token=") + urlEncode(ref)
                     + "&client_id=" + urlEncode(cle);
  const int code = t3kRequete(String(T3K_HOTE) + "/api/v1/oauth/token", corps, "", rep);
  return (code >= 200 && code < 300) && t3kStockerJetons(rep);
}

// GET authentifié avec re-tentative après rafraîchissement du jeton
static int t3kGetAuth(const String &url, String &reponse) {
  String acc = prefs.getString("t3kAcc", "");
  if (!acc.length()) return 401;
  int code = t3kRequete(url, "", acc, reponse);
  if (code == 401 && t3kRafraichir()) {
    acc = prefs.getString("t3kAcc", "");
    code = t3kRequete(url, "", acc, reponse);
  }
  return code;
}

// ---------------------------------------------------------------------------
// Page web (embarquée en flash). Le JavaScript contient :
//  - le MOTEUR NAM (WaveNet + LSTM, conforme à NeuralAmpModelerCore) qui
//    tourne sur le TÉLÉPHONE pour analyser la capture ;
//  - le flux OAuth TONE3000 (PKCE, SHA-256 en pur JS : pas de crypto.subtle
//    sur une page http) — le bouton ouvre le VRAI catalogue tone3000.com.
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
background:#1a1712;color:var(--txt);font-size:13px;font-weight:700;letter-spacing:1px;cursor:pointer;margin-top:6px}
button.b:disabled{opacity:.4}
button.b.go{background:linear-gradient(#3a2c14,#2c210f);color:var(--amber);border-color:var(--amber2)}
input[type=file],input[type=text],input[type=password],select{width:100%;color:var(--txt);
background:#1a1712;border:1px solid var(--line);border-radius:10px;padding:10px;font-size:13px;margin:6px 0}
input[type=file]{color:var(--mut);font-size:12px}
.bar{height:8px;border-radius:4px;background:#3a332a;margin:10px 0 4px;overflow:hidden}
.bar div{height:100%;width:0%;background:linear-gradient(90deg,var(--amber2),var(--amber));transition:width .2s}
.msg{font-size:11px;color:var(--mut);min-height:14px}
.grid{display:grid;grid-template-columns:repeat(4,1fr);gap:8px;margin:8px 0 4px}
.grid button{padding:10px 4px;border-radius:12px;border:1px solid var(--line);
background:#1a1712;color:var(--mut);font-size:11px;font-weight:700;cursor:pointer}
.grid button small{display:block;font-size:9px;font-weight:400;margin-top:3px}
.grid button.sel{background:linear-gradient(#3a2c14,#2c210f);color:var(--amber);
border-color:var(--amber2);box-shadow:0 0 12px #ff7b1c44}
.fswrap{display:flex;flex-direction:column;align-items:center;padding:18px 0 14px}
#fsw{width:96px;height:96px;border-radius:50%;cursor:pointer;border:5px solid #4a4133;
background:radial-gradient(circle at 35% 30%,#5c5346,#2c2721 70%);
box-shadow:0 6px 20px #000c,inset 0 2px 6px #fff2;transition:.2s}
#fsw.on{border-color:var(--amber2);background:radial-gradient(circle at 35% 30%,#ffcf7e,var(--amber2) 75%);
box-shadow:0 0 34px #ff9a2b88,0 6px 20px #000c}
#fswtxt{margin-top:12px;font-size:13px;letter-spacing:4px;font-weight:800;color:var(--mut)}
#fsw.on+#fswtxt{color:var(--amber)}
details{margin-top:8px}
summary{font-size:11px;color:var(--mut);letter-spacing:2px;cursor:pointer;text-transform:uppercase}
footer{text-align:center;font-size:10px;color:#5c5546;letter-spacing:1px;margin-top:6px}
</style></head><body>
<div class="pedal">
<header>
  <div><h1>S<b>O</b>AJ</h1><div class="sub">TONE3000 INT&Eacute;GR&Eacute; &middot; 1 ESP32</div></div>
  <div class="link"><span id="lktxt">liaison</span><span class="dot" id="dot"></span></div>
</header>

<div class="card">
  <div class="row"><label>P&eacute;dale active</label></div>
  <div class="pname" id="pnom">&mdash;</div>
  <div class="grid" id="slots"></div>
  <div class="hint">8 emplacements m&eacute;moris&eacute;s dans la p&eacute;dale &mdash; un appui pour changer (fondu, sans Internet)</div>
</div>

<div class="card">
  <div class="row"><label>TONE3000</label><span class="msg" id="etatnet"></span></div>
  <div style="display:flex;gap:8px;align-items:center">
    <select id="slotsel" style="flex:0 0 110px"></select>
    <button class="b go" id="t3kgo" style="flex:1;margin-top:0">CHOISIR SUR TONE3000</button>
  </div>
  <div class="bar"><div id="tprog"></div></div>
  <div class="msg" id="tmsg"></div>
  <details id="conf">
    <summary>Configuration (1<sup>re</sup> fois)</summary>
    <div class="hint">1. Sur le t&eacute;l&eacute;phone : activer le PARTAGE DE CONNEXION, puis renseigner :</div>
    <input type="text" id="hsssid" placeholder="nom du partage de connexion (SSID)">
    <input type="password" id="hsmdp" placeholder="mot de passe du partage">
    <button class="b" id="hsok">ENREGISTRER LE R&Eacute;SEAU</button>
    <div class="hint">2. Cl&eacute; API : tone3000.com &rarr; R&eacute;glages &rarr; API Keys &rarr; cr&eacute;er une cl&eacute;
    publiable t3k_pub&hellip; (et y autoriser l'adresse de redirection affich&eacute;e ci-dessous)</div>
    <input type="text" id="t3kcle" placeholder="t3k_pub_...">
    <button class="b" id="cleok">ENREGISTRER LA CL&Eacute;</button>
    <div class="hint" id="rediruri"></div>
  </details>
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

<div class="card">
  <details>
    <summary>Import manuel d'un fichier .nam (secours)</summary>
    <div class="hint">marche sans compte TONE3000 : fichier .nam d&eacute;j&agrave; t&eacute;l&eacute;charg&eacute; sur le t&eacute;l&eacute;phone</div>
    <input type="file" id="fnam" accept=".nam,application/json">
    <button class="b" id="analyser" disabled>ANALYSER ET ENVOYER</button>
    <div class="bar"><div id="prog"></div></div>
    <div class="msg" id="msg"></div>
  </details>
</div>
<footer>http://192.168.4.1 &middot; http://soaj-nam.local &mdash; SOAJ NAM ESP32</footer>
</div>

<script>
// ===========================================================================
// UI : jauges + liaison + slots
// ===========================================================================
var KEYS=['g','d','t','v'];
var st={g:0.5,d:1,t:0.6,v:0.5,e:1,slot:0,slots:[],sta:0,cle:''};
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
  $('etatnet').textContent=st.sta?'en ligne via le téléphone':'hors ligne (partage de connexion ?)';
  $('etatnet').style.color=st.sta?'#7dd069':'#d05555';
  $('t3kgo').disabled=!(st.sta&&st.cle);
  peindreSlots();
}
function peindreSlots(){
  var g=$('slots');g.innerHTML='';
  var sel=$('slotsel');var selVal=sel.value;sel.innerHTML='';
  for(var i=0;i<8;i++){
    var b=document.createElement('button');
    b.className=(i==st.slot)?'sel':'';
    b.innerHTML=(i+1)+'<small>'+((st.slots[i]||'&mdash;'))+'</small>';
    (function(n){b.addEventListener('click',function(){
      fetch('/api/slot?n='+n).then(function(){poll();});
    });})(i);
    g.appendChild(b);
    var o=document.createElement('option');
    o.value=i;o.textContent='slot '+(i+1);
    sel.appendChild(o);
  }
  if(selVal!=='')sel.value=selVal;
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
    else{st.slots=j.slots;st.slot=j.slot;st.sta=j.sta;st.cle=j.cle;st.nom=j.nom;peindreSlots();}
  }).catch(function(){link(false)});
}
poll();setInterval(poll,3000);

// ===========================================================================
// Configuration : partage de connexion + clé API
// ===========================================================================
$('hsok').addEventListener('click',function(){
  fetch('/api/hotspot?ssid='+encodeURIComponent($('hsssid').value)
        +'&mdp='+encodeURIComponent($('hsmdp').value))
    .then(function(){$('tmsg').textContent='réseau enregistré, connexion en cours (~15 s)...';});
});
$('cleok').addEventListener('click',function(){
  fetch('/api/t3kcle?cle='+encodeURIComponent($('t3kcle').value.trim()))
    .then(function(){$('tmsg').textContent='clé enregistrée.';poll();});
});
$('rediruri').textContent='adresse de redirection à autoriser pour la clé : '+location.origin+'/';

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
// CALIBRATION : courbe d'&eacute;cr&ecirc;tage (sinus 220 Hz en rampe, cr&ecirc;tes
// + et − s&eacute;par&eacute;es) + r&eacute;ponse lin&eacute;aire (paliers + Goertzel) -> profil.
// ===========================================================================
var LUT_T=257,LUT_P=4.0;
function goertzel(buf,debut,n,f,sr){
  var w=2*Math.PI*f/sr,cw=Math.cos(w),c=2*cw,s0,s1=0,s2=0;
  for(var i=0;i<n;i++){s0=buf[debut+i]+c*s1-s2;s2=s1;s1=s0;}
  var re=s1-s2*cw,im=s2*Math.sin(w);
  return Math.sqrt(re*re+im*im);
}
function interpDemi(entrees,sorties,u){
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
  var phase=0,idx=0,iF=0;

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
    var eP=[],sP=[],eN=[],sN=[],nper=Math.floor(nRampe/per);
    for(var i2=3;i2<nper;i2++){
      var d=i2*per,f2=d+per,j2,moy=0;
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

// Analyse un objet .nam (déjà parsé) puis grave le profil dans un slot
function analyserEtEnvoyer(nam,nom,slot,progression,message,fini){
  var modele=creerModele(nam);
  if(modele.reste!==0)throw 'fichier .nam invalide ('+modele.reste+' poids inattendus)';
  message('analyse du reseau ('+nam.architecture+', '+nam.weights.length+' poids)...');
  calibrer(modele,nom,progression,function(profil){
    message('envoi du profil a la pedale...');
    fetch('/api/profil?slot='+slot,{method:'POST',body:JSON.stringify(profil)})
      .then(function(r){if(!r.ok)throw 'HTTP '+r.status;return r.json();})
      .then(function(j){progression(100);fini(j.nom);poll();})
      .catch(function(e){message('echec envoi : '+e);});
  },function(e){message('echec analyse : '+e);});
}

// ===========================================================================
// TONE3000 : PKCE + flux « select » — le bouton ouvre le VRAI catalogue.
// SHA-256 en pur JavaScript : crypto.subtle n'existe pas sur une page http.
// ===========================================================================
function sha256Octets(txt){
  var K=[0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,
         0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,
         0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,
         0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,
         0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,
         0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,
         0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,
         0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2];
  var H=[0x6a09e667,0xbb67ae85,0x3c6ef372,0xa54ff53a,0x510e527f,0x9b05688c,0x1f83d9ab,0x5be0cd19];
  var oct=[];
  for(var i=0;i<txt.length;i++)oct.push(txt.charCodeAt(i)&0xff);
  var bits=oct.length*8;
  oct.push(0x80);
  while(oct.length%64!==56)oct.push(0);
  for(i=7;i>=0;i--)oct.push((i>=4)?0:((bits>>>(i*8))&0xff));
  function rr(x,n){return (x>>>n)|(x<<(32-n));}
  var w=new Array(64);
  for(var b0=0;b0<oct.length;b0+=64){
    for(i=0;i<16;i++)w[i]=(oct[b0+4*i]<<24)|(oct[b0+4*i+1]<<16)|(oct[b0+4*i+2]<<8)|oct[b0+4*i+3];
    for(i=16;i<64;i++){
      var s0=rr(w[i-15],7)^rr(w[i-15],18)^(w[i-15]>>>3);
      var s1=rr(w[i-2],17)^rr(w[i-2],19)^(w[i-2]>>>10);
      w[i]=(w[i-16]+s0+w[i-7]+s1)|0;
    }
    var a=H[0],b=H[1],c=H[2],d=H[3],e=H[4],f=H[5],g=H[6],h=H[7];
    for(i=0;i<64;i++){
      var S1=rr(e,6)^rr(e,11)^rr(e,25);
      var ch=(e&f)^(~e&g);
      var t1=(h+S1+ch+K[i]+w[i])|0;
      var S0=rr(a,2)^rr(a,13)^rr(a,22);
      var mj=(a&b)^(a&c)^(b&c);
      var t2=(S0+mj)|0;
      h=g;g=f;f=e;e=(d+t1)|0;d=c;c=b;b=a;a=(t1+t2)|0;
    }
    H[0]=(H[0]+a)|0;H[1]=(H[1]+b)|0;H[2]=(H[2]+c)|0;H[3]=(H[3]+d)|0;
    H[4]=(H[4]+e)|0;H[5]=(H[5]+f)|0;H[6]=(H[6]+g)|0;H[7]=(H[7]+h)|0;
  }
  var out=new Uint8Array(32);
  for(i=0;i<8;i++){out[4*i]=(H[i]>>>24)&255;out[4*i+1]=(H[i]>>>16)&255;
                   out[4*i+2]=(H[i]>>>8)&255;out[4*i+3]=H[i]&255;}
  return out;
}
function b64url(octets){
  var s='';
  for(var i=0;i<octets.length;i++)s+=String.fromCharCode(octets[i]);
  return btoa(s).replace(/\+/g,'-').replace(/\//g,'_').replace(/=/g,'');
}
function tmsg(t){$('tmsg').textContent=t;}
function tprog(p){$('tprog').style.width=p+'%';}

var T3K='https://www.tone3000.com';
$('t3kgo').addEventListener('click',function(){
  var v=b64url(crypto.getRandomValues(new Uint8Array(32)));
  var s=b64url(crypto.getRandomValues(new Uint8Array(16)));
  localStorage.setItem('t3k_v',v);
  localStorage.setItem('t3k_s',s);
  localStorage.setItem('t3k_slot',$('slotsel').value||'0');
  var r=location.origin+'/';
  location.href=T3K+'/api/v1/oauth/authorize?client_id='+encodeURIComponent(st.cle)
    +'&redirect_uri='+encodeURIComponent(r)+'&response_type=code'
    +'&code_challenge='+b64url(sha256Octets(v))+'&code_challenge_method=S256'
    +'&state='+s+'&prompt=select_tone&format=nam';
});

// Retour du catalogue : ?code=...&state=...&tone_id=...
function t3kRetour(){
  var q=new URLSearchParams(location.search);
  if(!q.get('code')&&!q.get('error'))return;
  var v=localStorage.getItem('t3k_v'),s=localStorage.getItem('t3k_s');
  var slot=+(localStorage.getItem('t3k_slot')||'0');
  var toneId=q.get('tone_id');
  var code=q.get('code'),etat=q.get('state'),err=q.get('error');
  history.replaceState(null,'','/');
  if(err){tmsg('refuse par TONE3000 : '+err);return;}
  if(etat!==s){tmsg('etat PKCE invalide, reessayez');return;}
  tmsg('echange du code aupres de la pedale...');tprog(2);
  fetch('/t3k/token?code='+encodeURIComponent(code)+'&verifier='+encodeURIComponent(v)
        +'&redirect='+encodeURIComponent(location.origin+'/'),{method:'POST'})
  .then(function(r){return r.json();})
  .then(function(j){
    if(!j.ok)throw (j.erreur||'echec du jeton');
    if(!toneId)throw 'aucune pedale selectionnee';
    tmsg('liste des modeles...');tprog(5);
    return fetch('/t3k/modeles?tone_id='+encodeURIComponent(toneId));
  })
  .then(function(r){return r.json();})
  .then(function(j){
    var liste=j.data||[];
    if(!liste.length)throw 'aucun modele pour cette pedale';
    var m=liste[0];
    for(var i=0;i<liste.length;i++)
      if((liste[i].size||'')==='standard'){m=liste[i];break;}
    tmsg('la pedale telecharge "'+m.name+'"...');tprog(8);
    return fetch('/t3k/telecharger?url='+encodeURIComponent(m.model_url),{method:'POST'})
      .then(function(r){return r.json();})
      .then(function(t){
        if(!t.ok)throw (t.erreur||'echec du telechargement');
        tmsg('lecture de la capture ('+Math.round(t.taille/1024)+' Ko)...');tprog(10);
        return fetch('/t3k/fichier').then(function(r){return r.json();})
          .then(function(nam){
            var nom=(nam.metadata&&nam.metadata.name)?nam.metadata.name:m.name;
            analyserEtEnvoyer(nam,nom,slot,
              function(p){tprog(10+Math.floor(p*0.9));},tmsg,
              function(n){tmsg('"'+n+'" grave dans le slot '+(slot+1)+' — pret a jouer !');});
          });
      });
  })
  .catch(function(e){tmsg('echec : '+e);tprog(0);});
}
t3kRetour();

// ===========================================================================
// Import manuel (secours) : fichier .nam local -> analyse -> slot choisi
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
    var nom=(nam.metadata&&nam.metadata.name)?nam.metadata.name
            :fichierNam.name.replace(/\.nam$/i,'');
    var slot=+($('slotsel').value||'0');
    analyserEtEnvoyer(nam,nom,slot,
      function(p){$('prog').style.width=p+'%';},
      function(t){$('msg').textContent=t;},
      function(n){$('msg').textContent='"'+n+'" grave dans le slot '+(slot+1)+'.';btn.disabled=false;});
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
  String rep = "{\"g\":" + String(tgtDrive, 2) + ",\"d\":" + String(tgtMix, 2)
             + ",\"t\":" + String(tgtTone, 2) + ",\"v\":" + String(tgtVolume, 2)
             + ",\"e\":" + String((tgtEffect > 0.5f) ? 1 : 0)
             + ",\"nom\":\"" + profilActif.nom + "\""
             + ",\"slot\":" + String((int)slotActif)
             + ",\"sta\":" + String((WiFi.status() == WL_CONNECTED) ? 1 : 0)
             + ",\"cle\":\"" + prefs.getString("t3kCle", "") + "\",\"slots\":[";
  for (int n = 0; n < NB_SLOTS; n++) {
    rep += "\"";
    rep += nomsSlots[n];
    rep += (n < NB_SLOTS - 1) ? "\"," : "\"";
  }
  rep += "]}";
  server.send(200, "application/json", rep);
}

static void handleApiSet() {
  if (server.hasArg("g")) tgtDrive  = clampf(server.arg("g").toFloat(), 0.0f, 1.0f);
  if (server.hasArg("d")) tgtMix    = clampf(server.arg("d").toFloat(), 0.0f, 1.0f);
  if (server.hasArg("t")) tgtTone   = clampf(server.arg("t").toFloat(), 0.0f, 1.0f);
  if (server.hasArg("v")) tgtVolume = clampf(server.arg("v").toFloat(), 0.0f, 1.0f);
  if (server.hasArg("e")) tgtEffect = (server.arg("e").toInt() >= 1) ? 1.0f : 0.0f;
  handleApiGet();
}

// POST /api/profil?slot=n : profil analysé par le téléphone -> slot NVS + actif
static void handleApiProfil() {
  int n = server.hasArg("slot") ? server.arg("slot").toInt() : (int)slotActif;
  if (n < 0) n = 0;
  if (n >= NB_SLOTS) n = NB_SLOTS - 1;
  const String &corps = server.arg("plain");
  ProfilNam p;
  if (!parseProfilJson(corps.c_str(), &p)) {
    server.send(400, "application/json", "{\"erreur\":\"profil invalide\"}");
    return;
  }
  sauverSlotNVS(n, &p);                // écriture flash : bref craquement possible
  slotActif = (uint8_t)n;
  prefs.putUChar("slotActif", slotActif);
  memcpy(&profilRecu, &p, sizeof(p));
  profilEnAttente = true;              // la tâche audio l'échange en fondu
  Serial.printf("[Profil] \"%s\" grave dans le slot %d et active\n", p.nom, n + 1);
  server.send(200, "application/json", String("{\"nom\":\"") + p.nom + "\"}");
}

// GET /api/slot?n= : activer un profil mémorisé (fondu anti-clic)
static void handleApiSlot() {
  const int n = server.arg("n").toInt();
  ProfilNam p;
  if (n < 0 || n >= NB_SLOTS || !chargerSlotNVS(n, &p)) {
    server.send(404, "application/json", "{\"erreur\":\"slot vide\"}");
    return;
  }
  slotActif = (uint8_t)n;
  prefs.putUChar("slotActif", slotActif);
  memcpy(&profilRecu, &p, sizeof(p));
  profilEnAttente = true;
  Serial.printf("[Profil] slot %d (\"%s\") active\n", n + 1, p.nom);
  server.send(200, "application/json", String("{\"nom\":\"") + p.nom + "\"}");
}

// GET /api/hotspot?ssid=&mdp= : identifiants du partage de connexion
static void handleApiHotspot() {
  strncpy(hsSsid, server.arg("ssid").c_str(), sizeof(hsSsid) - 1);
  strncpy(hsMdp,  server.arg("mdp").c_str(),  sizeof(hsMdp) - 1);
  hsSsid[sizeof(hsSsid) - 1] = '\0';
  hsMdp[sizeof(hsMdp) - 1]   = '\0';
  prefs.putString("hsSsid", hsSsid);
  prefs.putString("hsMdp", hsMdp);
  if (hsSsid[0]) {
    Serial.printf("[WiFi] Connexion au partage \"%s\"...\n", hsSsid);
    WiFi.begin(hsSsid, hsMdp);
  }
  server.send(200, "application/json", "{\"ok\":1}");
}

// GET /api/t3kcle?cle= : clé API publiable TONE3000
static void handleApiT3kCle() {
  prefs.putString("t3kCle", server.arg("cle"));
  server.send(200, "application/json", "{\"ok\":1}");
}

// POST /t3k/token?code=&verifier=&redirect= : échange OAuth (PKCE) — les
// jetons restent dans la pédale
static void handleT3kToken() {
  const String cle = prefs.getString("t3kCle", "");
  if (!cle.length()) {
    server.send(400, "application/json", "{\"ok\":0,\"erreur\":\"cle API absente\"}");
    return;
  }
  const String corps = String("grant_type=authorization_code")
                     + "&code=" + urlEncode(server.arg("code"))
                     + "&code_verifier=" + urlEncode(server.arg("verifier"))
                     + "&redirect_uri=" + urlEncode(server.arg("redirect"))
                     + "&client_id=" + urlEncode(cle);
  String rep;
  const int code = t3kRequete(String(T3K_HOTE) + "/api/v1/oauth/token", corps, "", rep);
  if (code < 200 || code >= 300 || !t3kStockerJetons(rep)) {
    Serial.printf("[T3K] Echec echange jeton (HTTP %d)\n", code);
    server.send(502, "application/json",
                String("{\"ok\":0,\"erreur\":\"HTTP ") + code + "\"}");
    return;
  }
  Serial.println("[T3K] Compte TONE3000 connecte (jetons en flash)");
  server.send(200, "application/json", "{\"ok\":1}");
}

// GET /t3k/modeles?tone_id= : relaie la liste des modèles d'une pédale
static void handleT3kModeles() {
  String rep;
  const String url = String(T3K_HOTE) + "/api/v1/models?tone_id="
                   + urlEncode(server.arg("tone_id"));
  const int code = t3kGetAuth(url, rep);
  if (code < 200 || code >= 300) {
    server.send(502, "application/json",
                String("{\"erreur\":\"HTTP ") + code + "\"}");
    return;
  }
  server.send(200, "application/json", rep);
}

// POST /t3k/telecharger?url= : télécharge la capture .nam dans LittleFS
static void handleT3kTelecharger() {
  const String url = server.arg("url");
  if (!url.startsWith(String(T3K_HOTE) + "/")) {   // uniquement tone3000.com
    server.send(400, "application/json", "{\"ok\":0,\"erreur\":\"url refusee\"}");
    return;
  }
  String acc = prefs.getString("t3kAcc", "");
  if (!acc.length()) {
    server.send(401, "application/json", "{\"ok\":0,\"erreur\":\"pas de jeton\"}");
    return;
  }

  WiFiClientSecure tls;
#if T3K_TLS_STRICT
  tls.setCACert(ISRG_ROOT_X1);
#else
  tls.setInsecure();
#endif
  HTTPClient http;
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  http.setTimeout(30000);
  if (!http.begin(tls, url)) {
    server.send(502, "application/json", "{\"ok\":0,\"erreur\":\"connexion\"}");
    return;
  }
  http.addHeader("Authorization", String("Bearer ") + acc);
  int code = http.GET();
  if (code == 401) {                    // jeton expiré -> rafraîchir et retenter
    http.end();
    if (t3kRafraichir()) {
      acc = prefs.getString("t3kAcc", "");
      http.begin(tls, url);
      http.addHeader("Authorization", String("Bearer ") + acc);
      code = http.GET();
    }
  }
  if (code < 200 || code >= 300) {
    http.end();
    Serial.printf("[T3K] Echec telechargement (HTTP %d)\n", code);
    server.send(502, "application/json",
                String("{\"ok\":0,\"erreur\":\"HTTP ") + code + "\"}");
    return;
  }

  LittleFS.remove(FICHIER_NAM);
  File f = LittleFS.open(FICHIER_NAM, "w");
  if (!f) {
    http.end();
    server.send(500, "application/json", "{\"ok\":0,\"erreur\":\"flash pleine\"}");
    return;
  }
  http.writeToStream(&f);
  const size_t taille = f.size();
  f.close();
  http.end();
  Serial.printf("[T3K] Capture telechargee : %u octets\n", (unsigned)taille);
  server.send(200, "application/json",
              String("{\"ok\":1,\"taille\":") + taille + "}");
}

// GET /t3k/fichier : sert la capture téléchargée à la page (même origine)
static void handleT3kFichier() {
  File f = LittleFS.open(FICHIER_NAM, "r");
  if (!f) {
    server.send(404, "application/json", "{\"erreur\":\"aucune capture\"}");
    return;
  }
  server.streamFile(f, "application/json");
  f.close();
}

// Toute URL inconnue redirige vers la page (portail captif pour le mode AP)
static void handleNotFound() {
  server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
  server.sendHeader("Cache-Control", "no-store");
  server.send(302, "text/plain", "");
}

// Sondes de détection de portail captif (utiles quand le téléphone est
// connecté au point d'accès SOAJ-NAM plutôt qu'au partage de connexion)
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
// Tâches : audio serré sur le cœur 1, web/DNS/WiFi sur le cœur 0
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
  uint32_t staDernierEssai = 0;
  bool     staAnnonce      = false;
  for (;;) {
    dnsServer.processNextRequest();
    server.handleClient();

    // Reconnexion au partage de connexion (toutes les 15 s tant que perdu)
    if (hsSsid[0]) {
      const bool ok = (WiFi.status() == WL_CONNECTED);
      if (ok && !staAnnonce) {
        Serial.print("[WiFi] Relie au partage de connexion — IP : ");
        Serial.println(WiFi.localIP());
        staAnnonce = true;
      } else if (!ok) {
        staAnnonce = false;
        if (millis() - staDernierEssai > 15000) {
          staDernierEssai = millis();
          WiFi.begin(hsSsid, hsMdp);
        }
      }
    }
    delay(2);                          // laisse respirer la pile WiFi
  }
}

// ---------------------------------------------------------------------------
// Initialisation
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== SOAJ NAM — 1 ESP32, catalogue TONE3000 integre ===");

  // --- ADC / DAC ---
#ifdef USE_LEGACY_ADC
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
#else
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_GUITAR_IN, ADC_11db);
#endif
  dacWrite(PIN_AUDIO_OUT, 128);

  // --- Flash : NVS (profils, réglages) + LittleFS (capture téléchargée) ---
  prefs.begin("soajnam", false);
  if (!LittleFS.begin(true)) Serial.println("[Flash] LittleFS indisponible !");

  // Migration depuis l'ancienne version (un seul profil sous "profil")
  if (prefs.getBytesLength("profil") == sizeof(ProfilNam)
      && prefs.getBytesLength("prof0") != sizeof(ProfilNam)) {
    ProfilNam ancien;
    prefs.getBytes("profil", &ancien, sizeof(ancien));
    bornerProfil(&ancien);
    sauverSlotNVS(0, &ancien);
    Serial.println("[Profil] ancien profil migre vers le slot 1");
  }

  chargerNomsSlots();
  slotActif = prefs.getUChar("slotActif", 0);
  if (slotActif >= NB_SLOTS) slotActif = 0;
  if (chargerSlotNVS(slotActif, &profilActif)) {
    Serial.printf("[Profil] slot %d restaure : \"%s\"\n", slotActif + 1, profilActif.nom);
  } else {
    profilNeutre(&profilActif);
    Serial.println("[Profil] slot vide : profil NEUTRE (tanh) charge");
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

  // --- WiFi : point d'accès (config/secours) + station (partage de connexion) ---
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASSWORD, WIFI_CHANNEL);
  Serial.printf("[WiFi] Point d'acces \"%s\" (mdp \"%s\") — http://", AP_SSID, AP_PASSWORD);
  Serial.println(WiFi.softAPIP());

  prefs.getString("hsSsid", hsSsid, sizeof(hsSsid));
  prefs.getString("hsMdp",  hsMdp,  sizeof(hsMdp));
  if (hsSsid[0]) {
    Serial.printf("[WiFi] Connexion au partage \"%s\"...\n", hsSsid);
    WiFi.begin(hsSsid, hsMdp);
  } else {
    Serial.println("[WiFi] Aucun partage memorise : le renseigner sur la page (Configuration)");
  }

  // DNS captif (mode AP) : toute résolution renvoie l'IP de la pédale
  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.setTTL(30);
  dnsServer.start(53, "*", WiFi.softAPIP());

  // mDNS : http://soaj-nam.local sur le partage de connexion ET sur l'AP
  if (MDNS.begin("soaj-nam")) MDNS.addService("http", "tcp", 80);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/get", HTTP_GET, handleApiGet);
  server.on("/api/set", HTTP_GET, handleApiSet);
  server.on("/api/profil", HTTP_POST, handleApiProfil);
  server.on("/api/slot", HTTP_GET, handleApiSlot);
  server.on("/api/hotspot", HTTP_GET, handleApiHotspot);
  server.on("/api/t3kcle", HTTP_GET, handleApiT3kCle);
  server.on("/t3k/token", HTTP_POST, handleT3kToken);
  server.on("/t3k/modeles", HTTP_GET, handleT3kModeles);
  server.on("/t3k/telecharger", HTTP_POST, handleT3kTelecharger);
  server.on("/t3k/fichier", HTTP_GET, handleT3kFichier);
  routesPortailCaptif();
  server.onNotFound(handleNotFound);
  server.begin();

  Serial.println("[Web] Page : http://soaj-nam.local (partage de connexion)");
  Serial.println("      ou http://192.168.4.1 (WiFi SOAJ-NAM, donnees mobiles COUPEES)");

  // --- Stabilisation de l'offset DC avant de sortir du son ---
  for (int i = 0; i < 4000; i++) {
    const int raw = readGuitarAdc();
    dcOffset += 0.01f * ((float)raw - dcOffset);
    delayMicroseconds(50);
  }
  Serial.printf("Offset DC mesure : %.0f (attendu ~2048)\n", dcOffset);
  if (dcOffset < 1200.0f || dcOffset > 2900.0f) {
    Serial.println("ATTENTION : offset DC anormal — verifiez le pont diviseur");
  }

  // --- Tâches : audio (cœur 1, boucle serrée), web (cœur 0) ---
  disableCore1WDT();       // la boucle audio ne rend jamais la main
  xTaskCreatePinnedToCore(tacheAudio, "audio", 8192, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(tacheWeb,   "web",  12288, NULL, 1, NULL, 0);

  Serial.printf("Audio : %d Hz — drive -> filtres du profil -> courbe NAM -> tone -> volume\n",
                SAMPLE_RATE_HZ);
}

void loop() {
  vTaskDelay(portMAX_DELAY);           // tout vit dans les deux tâches
}
