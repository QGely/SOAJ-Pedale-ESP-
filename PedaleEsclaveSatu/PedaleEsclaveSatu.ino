/*
 * ============================================================================
 *  SOAJ - Pédale d'effet guitare ESP32
 *  PedaleEsclaveSatu.ino  —  ESP32 esclave SATU (saturation/fuzz/EQ/octave)
 * ============================================================================
 *
 *  Rôle :
 *    - Reçoit UNIQUEMENT des paramètres du maître via ESP-NOW.
 *    - Lit la guitare sur l'ADC (GPIO34), traite, sort sur le DAC (GPIO25).
 *
 *  Formule complète (8 potentiomètres g, d, o, b, m, h, t, v) :
 *
 *    y = Hsortie(s,t,v) · EQh(s,h) · EQm(s,m) · EQb(s,b) · S( O( Hampli(s,g) · Hin(s) · x , o ) , d )
 *
 *  + OCTAVE O(u,o) — étage fOXX Tone Machine (redressement double alternance) :
 *    O(u,o) = u + o·( 2·HPF₁₀Hz(|u|) − u )
 *    |u| ne contient que les harmoniques pairs -> fondamentale remplacée
 *    par l'octave supérieure (|sin wt| = 2/π − 4/π·Σ cos(2k·wt)/(4k²−1)).
 *
 *  + CLEAN PUR : à Dist = 0, le signal ne traverse AUCUN étage — sortie =
 *    x · CLEAN_GAIN · Volume, c'est tout. Le pot Dist fond continûment de ce
 *    chemin brut (0) vers la chaîne traitée complète (~0.25 et au-delà).
 *
 *  + NOISE GATE (indispensable en high-gain : sans lui, guitare à volume 0,
 *    le bruit ADC est normalisé vers ±1 par le fuzz -> souffle constant).
 *    Enveloppe mesurée sur l'entrée ; gain du gate appliqué AVANT l'étage de
 *    gain ET APRÈS la saturation (atténuation au carré pendant la fermeture,
 *    la disto ne peut pas "remonter" le fondu). Seuils proportionnels à
 *    (1 + 2·g + 2·d) : plus de drive/fuzz = fermeture plus précoce.
 *
 *  Étages linéaires (transformation bilinéaire s = 2·fs·(1−z⁻¹)/(1+z⁻¹)) :
 *    Hin(s)      = 0.47·s / (1 + 0.4747·s)                (liaison d'entrée)
 *    Hampli(s,g) = 1 + 0.11·g·s / ((1+5e-5·g·s)(1+2.2e-4·s))   (ampli TL082)
 *    EQb(s,b)    = low-shelf  100 Hz, −12..+12 dB (b : 0.5 = plat)
 *    EQm(s,m)    = peak       700 Hz, Q=0.9, −12..+12 dB (m : 0.5 = plat)
 *    EQh(s,h)    = high-shelf 3,2 kHz, −12..+12 dB (h : 0.5 = plat)
 *    Hsortie(s,t,v) = 0.1·v·s / (1 + (0.11242−0.00022·t)·s
 *                                  + (2.42e-5·t − 2.2e-6·t²)·s²)  (tone+volume)
 *
 *  Étage NON-LINÉAIRE S(u,d) — voicing fuzz asymétrique (type Muse) :
 *    d = 0 : S(u,0) = u                     (strictement linéaire, 0 distorsion)
 *    d > 0 : Vsat = 2·10^(−2.3·d)   (seuil de ±2.0 à ±0.01, log)
 *            soft = tanh( (u + 0.28·Vsat) / Vsat ) − tanh(0.28)   (asymétrique)
 *            hard = clip( soft · (1 + 2.2·d), −1, +1 )            (bords carrés)
 *            S(u,d) = u + ( hard − u ) · w(d),  w = min(1, 4·d)
 *    L'asymétrie crée les harmoniques PAIRES (son "lampe"), l'écrêtage dur
 *    final donne les bords carrés du fuzz — le caractère Bellamy/Fuzz Factory.
 *
 *  IMPORTANT — matériel :
 *    L'entrée GPIO34 ne supporte NI tension négative NI plus de 3,3 V.
 *    (condensateur de liaison + pont diviseur 1,65 V, voir README.md)
 *
 *  NB : le paquet ESP-NOW a changé (7 paramètres) — re-téléverser le MAÎTRE
 *  en même temps que ce fichier, sinon les réglages seront ignorés.
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

// Niveau de sortie global
#define OUTPUT_LEVEL        1.0f

// Bypass : rattrapage de niveau quand l'effet est coupé
#define BYPASS_GAIN         8.0f

// CLEAN pur : à Dist = 0, le signal ne traverse AUCUN étage (ni filtre, ni
// EQ, ni tone) — seulement ce rehaussement, dosé par le pot Volume :
//   y = x · CLEAN_GAIN · V   (V=0.5 -> x8, même niveau que le bypass)
// Le pot Dist fait le fondu continu entre ce chemin pur (0) et la chaîne
// complète traitée (à partir de ~0.25).
#define CLEAN_GAIN          16.0f

// Course du potentiomètre DRIVE (1 = logarithmique, 0 = linéaire fidèle)
#define DRIVE_TAPER_LOG     1

// --- Octave O(u,o) — étage fOXX Tone Machine ---------------------------------
// Dans la fOXX, Q2 monté en déphaseur (collecteur 4K7 / émetteur 4K7) sort
// deux copies du signal en opposition de phase ; deux diodes germanium n'en
// gardent chacune qu'une demi-alternance ; leur somme = REDRESSEMENT DOUBLE
// ALTERNANCE : u -> |u|. Comme |sin(wt)| ne contient QUE les harmoniques
// pairs (2w, 4w...), la fondamentale disparaît : c'est l'octave supérieure.
// La liaison 10 µF retire la composante continue du signal redressé.
//   O(u,o) = u + o·( OCT_GAIN·HPF(|u|) − u )   (o=0 : inchangé, o=1 : plein octaver)
#define OCT_HPF_HZ          10.0f     // liaison 10 µF : retire le DC de |u|
#define OCT_GAIN            2.0f      // rattrape le niveau perdu au redressement

// --- Saturation S(u,d) — voicing HIGH-GAIN MODERNE (serré, sans fizz) -------
// Trois ingrédients (montage des amplis metal modernes) :
//   0. AVANT l'écrêtage : passe-haut de resserrage TIGHT_HPF_HZ — retire le
//      bas qui "baveait" dans la disto, l'attaque devient nette et percutante.
//   1. Écrêtage doux ASYMÉTRIQUE : tanh décalé de SAT_ASYM·Vsat -> harmoniques
//      PAIRES, caractère lampe.
//   2. Deuxième étage tanh (au lieu de l'ancien écrêtage dur tranché) :
//      re-amplifié ×(1 + 2.2·d) puis tanh -> aussi agressif, mais SANS angle
//      dur. L'angle net de l'ancien clip créait de l'aliasing (fréquences
//      inharmoniques dissonantes) : c'était le "je-ne-sais-quoi" déplaisant
//      sur les notes seules.
//   3. APRÈS l'égaliseur : passe-bas CAB_LPF_HZ ordre 2 = simulation de
//      haut-parleur guitare (un vrai HP coupe ~5,5 kHz ; sans lui, tout le
//      grésil numérique au-dessus partait dans l'ampli).
// Seuil : Vsat = SAT_V0 · 10^(−SAT_LOGSPAN·d)
//   d=0 -> ±2.0 (aucun écrêtage audible), d=1 -> ±0.01 (extrême, rapport 200)
#define SAT_V0              2.0f
#define SAT_LOGSPAN         2.3f
#define SAT_ASYM            0.28f     // décalage d'asymétrie (fractions de Vsat)
#define SAT_HARD            2.2f      // poussée du 2e étage tanh à d max
#define SAT_MIX_RAMP        4.0f      // fondu linéaire->saturé (continuité à d=0)
#define TIGHT_HPF_HZ        140.0f    // resserrage pré-écrêtage — RÉFÉRENCE à
                                      // Low=0.5 ; le pot Low le fait glisser
                                      // de 280 Hz (Low=0, serré) à 70 Hz
                                      // (Low=1, fuzz gras type Psycho)
#define PRE_LPF_HZ          4000.0f   // passe-bas ANTI-SOUFFLE avant la
                                      // saturation : le drive amplifie x20-100
                                      // le souffle des convertisseurs, la
                                      // saturation le compresse vers le
                                      // plafond -> nappe de bruit 6-8 kHz.
                                      // On coupe AVANT le gain ; la disto
                                      // recrée les harmoniques au-dessus.
                                      // (rôle du condensateur C4 du circuit
                                      // TL082 réel, neutralisé par la course
                                      // log du pot drive)
#define CAB_LPF_HZ          5500.0f   // simulation haut-parleur (post-EQ)
#define CAB_Q               0.707f    // Butterworth

// --- Égaliseur 3 bandes (formules RBJ "Audio EQ Cookbook") ------------------
#define EQ_LOW_HZ           100.0f    // low-shelf
#define EQ_MID_HZ           700.0f    // peak
#define EQ_MID_Q            0.9f
#define EQ_HIGH_HZ          3200.0f   // high-shelf
#define EQ_RANGE_DB         12.0f     // course des pots : −12 dB .. +12 dB

// --- Noise gate --------------------------------------------------------------
// Sans gate, guitare à volume 0 : le bruit de fond de l'ADC (~1-2 pas) est
// normalisé vers ±1 par le fuzz -> souffle constant. Le gate mesure
// l'enveloppe du signal d'ENTRÉE et applique son gain DEUX fois : avant
// l'étage de gain ET après la saturation (sinon la disto recomprime le fondu
// de fermeture et on entend le bruit "redescendre").
// Amplitudes normalisées : une guitare directe fait ~0.12 en crête (avec
// INPUT_GAIN 2), le bruit ADC ~0.001-0.003.
#define GATE_LOW            0.002f    // en dessous : fermé
#define GATE_HIGH           0.006f    // au-dessus : complètement ouvert
#define GATE_DRIVE_SCALE    1.5f      // seuils x(1 + 1.5·G ...)
#define GATE_DIST_SCALE     1.0f      //        ...  + 1·D)
                                      // Volontairement plus doux qu'avant : la
                                      // corde de MI AIGU (la plus faible) doit
                                      // sonner même jouée doucement à fort gain
#define ENV_ATTACK          0.01f     // montée d'enveloppe ~2,5 ms (un parasite
                                      // d'UN échantillon n'ouvre pas le gate)
#define ENV_RELEASE         0.002f    // descente RAPIDE (~25 ms) : l'enveloppe
                                      // suit la vraie fin de note au lieu de
                                      // traîner (la traîne = la "courbe qui
                                      // descend" audible après chaque note)
#define GATE_OPEN_COEF      0.010f    // ouverture ~5 ms
#define GATE_CLOSE_COEF     0.004f    // fermeture FRANCHE ~12 ms : pas de fondu
                                      // audible, la note s'arrête c'est tout
#define GATE_SNAP           0.005f    // en dessous : gain collé à 0 strict
                                      // (sortie DAC exactement au repos)

// --- Gate INTELLIGENT : sustain qui chante (solo) vs arrêt net (rythmique) --
// Deux enveloppes : la rapide (ci-dessus) et une LENTE (~170 ms de mémoire).
// Cordes ÉTOUFFÉES : la rapide s'effondre sous 35 % de la lente en ~30 ms
//   -> fermeture franche (comportement rythmique inchangé).
// Note TENUE qui décroît naturellement : les deux descendent ensemble
//   -> seuils abaissés (x0.6) et fermeture douce : la note chante jusqu'au
//   bout, comme le sustain du solo de Psycho.
#define ENV_RELEASE_SLOW    0.0003f   // enveloppe lente (~170 ms)
#define ABRUPT_RATIO        0.35f     // rapide < 35 % de la lente = étouffé
#define SUSTAIN_THRESH      0.6f      // seuils x0.6 pendant un sustain naturel
#define GATE_CLOSE_SOFT     0.0008f   // fermeture douce ~60 ms en fin de sustain

// Lissage des paramètres (~60 ms) : aucun craquement au changement
#define PARAM_SMOOTH        0.0008f

// Recalcul des coefficients tous les N échantillons (~3 ms)
#define COEF_UPDATE_SAMPLES 64

// Suivi de l'offset DC (très lent, ~100 ms)
#define DC_TRACK_COEF       0.0005f

// Vu-mètre de diagnostic (1 = une ligne/seconde sur le port série ;
// bloque l'audio ~10 ms -> craquement périodique. 0 pour JOUER.)
#define DEBUG_METER         0

// Bip de test 440 Hz au démarrage (diagnostic du chemin DAC -> ampli)
#define STARTUP_TEST_TONE   0

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
  float    gain;      // g : drive (0..1)
  float    dist;      // d : saturation (0 = aucune .. 1 = extrême)
  float    oct;       // o : octave fOXX (0 = off .. 1 = plein octaver)
  float    low;       // b : graves  (0..1, 0.5 = plat, ±12 dB)
  float    mid;       // m : médiums (0..1, 0.5 = plat, ±12 dB)
  float    high;      // h : aigus   (0..1, 0.5 = plat, ±12 dB)
  float    tone;      // t : tonalité (0..1)
  float    volume;    // v : volume (0..1)
  uint8_t  effectOn;  // 0 = bypass, 1 = effet actif
} PedalParams;

// Cibles reçues par radio (défauts si le maître n'est pas encore là)
static volatile float tgtGain   = 0.5f;
static volatile float tgtDist   = 0.3f;   // léger crunch par défaut
static volatile float tgtOct    = 0.0f;   // octave coupée par défaut
static volatile float tgtLow    = 0.5f;   // EQ plat
static volatile float tgtMid    = 0.5f;
static volatile float tgtHigh   = 0.5f;
static volatile float tgtTone   = 0.5f;
static volatile float tgtVolume = 0.5f;
static volatile float tgtEffect = 1.0f;
// Mode TEST DIRECT (effectOn = 2) : ADC copié tel quel vers le DAC,
// strictement AUCUN traitement — sert à diagnostiquer si un bruit vient
// des convertisseurs/du câblage ou du traitement logiciel.
static volatile bool  directMode = false;

// ---------------------------------------------------------------------------
// Biquad IIR : y[n] = b0·x[n] + b1·x[n-1] + b2·x[n-2] − a1·y[n-1] − a2·y[n-2]
// ---------------------------------------------------------------------------
typedef struct {
  float b0, b1, b2;   // numérateur
  float a1, a2;       // dénominateur (a0 normalisé à 1)
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
// Ne touche pas à l'état : coefficients modifiables en cours de route.
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

// Transformation bilinéaire du 1er ORDRE : H(s) = (n0 + n1·s)/(d0 + d1·s).
// Un 1er ordre passé dans bilinear2 (avec n2=d2=0) crée une paire pôle/zéro
// EXACTEMENT sur le cercle unité en z=−1 : annulation parfaite en arithmétique
// exacte, mais fragile en float32 sur de longues durées. Cette version pose
// un vrai filtre du 1er ordre : b2 = a2 = 0, aucun mode parasite.
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

// --- Étage 1 : Hin(s) = 0.47·s / (1 + 0.4747·s) ----------------------------
static void calcHin(Biquad *f) {
  bilinear1(f, 0.0f, 0.47f, 1.0f, 0.4747f);
}

// --- Étage 2 : Hampli(s,g) = 1 + 0.11·g·s / ((1+5e-5·g·s)(1+2.2e-4·s)) -----
static void calcAmpli(Biquad *f, float g) {
  const float t1 = 5e-5f * g;
  const float t2 = 2.2e-4f;
  const float tg = 0.11f * g;
  bilinear2(f, 1.0f, t1 + t2 + tg, t1 * t2,
               1.0f, t1 + t2,      t1 * t2);
}

// --- Étage 6 : Hsortie(s,t,v) -----------------------------------------------
static void calcSortie(Biquad *f, float t, float v) {
  // t borné à 0.02 : à t=0 exactement, d2 s'annule et le biquad dégénère en
  // 1er ordre avec un pôle sur le cercle unité (cf. bilinear1). Inaudible.
  if (t < 0.02f) t = 0.02f;
  const float d1 = 0.11242f - 0.00022f * t;
  const float d2 = 2.42e-5f * t - 2.2e-6f * t * t;
  bilinear2(f, 0.0f, 0.1f * v, 0.0f,
               1.0f, d1,       d2);
}

// --- Étages 4-5 : égaliseur 3 bandes (RBJ Audio EQ Cookbook) ----------------
// Le paramètre p (0..1) devient un gain en dB : (p − 0.5) · 2 · EQ_RANGE_DB.

static void calcLowShelf(Biquad *f, float f0, float dB) {
  const float A  = powf(10.0f, dB / 40.0f);
  const float w0 = 2.0f * (float)M_PI * f0 / (float)SAMPLE_RATE_HZ;
  const float cw = cosf(w0), sw = sinf(w0);
  const float alpha = sw * 0.5f * sqrtf(2.0f);          // pente S = 1
  const float k  = 2.0f * sqrtf(A) * alpha;

  const float b0 =        A * ((A + 1.0f) - (A - 1.0f) * cw + k);
  const float b1 = 2.0f * A * ((A - 1.0f) - (A + 1.0f) * cw);
  const float b2 =        A * ((A + 1.0f) - (A - 1.0f) * cw - k);
  const float a0 =             (A + 1.0f) + (A - 1.0f) * cw + k;
  const float a1 =    -2.0f * ((A - 1.0f) + (A + 1.0f) * cw);
  const float a2 =             (A + 1.0f) + (A - 1.0f) * cw - k;

  const float inv = 1.0f / a0;
  f->b0 = b0 * inv;  f->b1 = b1 * inv;  f->b2 = b2 * inv;
  f->a1 = a1 * inv;  f->a2 = a2 * inv;
}

static void calcHighShelf(Biquad *f, float f0, float dB) {
  const float A  = powf(10.0f, dB / 40.0f);
  const float w0 = 2.0f * (float)M_PI * f0 / (float)SAMPLE_RATE_HZ;
  const float cw = cosf(w0), sw = sinf(w0);
  const float alpha = sw * 0.5f * sqrtf(2.0f);          // pente S = 1
  const float k  = 2.0f * sqrtf(A) * alpha;

  const float b0 =         A * ((A + 1.0f) + (A - 1.0f) * cw + k);
  const float b1 = -2.0f * A * ((A - 1.0f) + (A + 1.0f) * cw);
  const float b2 =         A * ((A + 1.0f) + (A - 1.0f) * cw - k);
  const float a0 =              (A + 1.0f) - (A - 1.0f) * cw + k;
  const float a1 =      2.0f * ((A - 1.0f) - (A + 1.0f) * cw);
  const float a2 =              (A + 1.0f) - (A - 1.0f) * cw - k;

  const float inv = 1.0f / a0;
  f->b0 = b0 * inv;  f->b1 = b1 * inv;  f->b2 = b2 * inv;
  f->a1 = a1 * inv;  f->a2 = a2 * inv;
}

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

// Course du pot DRIVE : position du bouton p (0..1) -> variable g de la formule
static inline float driveTaper(float p) {
#if DRIVE_TAPER_LOG
  return (expf(p * 6.2166f) - 1.0f) / 500.0f;   // ln(501) = 6.2166
#else
  return p;
#endif
}

// Position de pot EQ (0..1) -> gain en dB (−EQ_RANGE_DB .. +EQ_RANGE_DB)
static inline float eqDb(float p) {
  return (p - 0.5f) * 2.0f * EQ_RANGE_DB;
}

// Resserrage pré-écrêtage LIÉ AU POT LOW : le pot façonne le grave AVANT et
// APRÈS la disto. Low = 0.5 -> 140 Hz (référence), Low = 1 -> 70 Hz (fuzz
// GRAS pleine bande, le son Psycho/Bellamy), Low = 0 -> 280 Hz (très serré,
// metal chirurgical). fc = 140 · 2^((0.5−low)·2)
static void calcTight(Biquad *f, float lowPos) {
  const float fc  = TIGHT_HPF_HZ * powf(2.0f, (0.5f - lowPos) * 2.0f);
  const float tau = 1.0f / (2.0f * (float)M_PI * fc);
  bilinear1(f, 0.0f, tau, 1.0f, tau);
}

// ---------------------------------------------------------------------------
// État du traitement audio
// ---------------------------------------------------------------------------
static float dcOffset = 2048.0f;
static Biquad fHin    = {0};
static Biquad fAmpli  = {0};
static Biquad fTight  = {0};   // passe-haut de resserrage pré-écrêtage (140 Hz)
static Biquad fPreLp  = {0};   // passe-bas anti-souffle pré-saturation (4 kHz)
static Biquad fLow    = {0};
static Biquad fMid    = {0};
static Biquad fHigh   = {0};
static Biquad fCab    = {0};   // passe-bas "haut-parleur" post-EQ (5,5 kHz)
static Biquad fSortie = {0};
static float  quantErr  = 0.0f;      // erreur du quantificateur sigma-delta
static float  dacTarget = 128.0f;    // valeur DAC visée (float, tenue entre
                                     // deux échantillons pour le sigma-delta)
static float  envelope = 0.0f;       // suiveur d'enveloppe RAPIDE du gate
static float  envSlow  = 0.0f;       // suiveur LENT (mémoire de la note ~170 ms)
static float  gateGain = 0.0f;       // gain du gate (0..1)
// Passe-haut 300 Hz pour la MESURE d'enveloppe uniquement : rejette à la fois
// la ronflette secteur (50/100/150 Hz) ET le grondement de frottement de la
// main posée sur les cordes (mesuré : énergie concentrée 100-300 Hz). Une
// vraie note ouvre par ses harmoniques (mi grave 82 Hz : 328/410/492 Hz...)
#define GATE_DETECT_HPF_HZ  300.0f
static float  envHpX = 0.0f, envHpY = 0.0f, envHpA = 0.0f;

// Étage octave fOXX : passe-haut 1 pôle sur le signal redressé |u|
// (émule la liaison 10 µF qui retire la composante continue 2/π)
static float  octPrevX = 0.0f, octPrevY = 0.0f;
static float  octA     = 0.0f;       // coefficient du passe-haut (calculé au setup)

// Paramètres lissés
static float smGain    = 0.5f;
static float smDist    = 0.3f;
static float smOct     = 0.0f;
static float smLow     = 0.5f;
static float smMid     = 0.5f;
static float smHigh    = 0.5f;
static float smTone    = 0.5f;
static float smVolume  = 0.0f;       // démarre à 0 : montée douce, pas de "pop"
static float smEffect  = 1.0f;

// Coefficients de saturation (recalculés toutes les ~3 ms, pas à chaque
// échantillon : powf est coûteux)
static float satVsat = 1.0f;
static float satMix  = 0.0f;
static float satBias = 0.0f;   // décalage d'asymétrie = SAT_ASYM · Vsat
static float satHard = 1.0f;   // gain vers l'écrêtage dur = 1 + SAT_HARD · d
// Compensation du décalage en sortie : tanh(SAT_ASYM), constant
static const float SAT_BIAS_OUT = 0.27290f;   // tanh(0.28)

// Dernières valeurs EQ pour lesquelles les biquads ont été calculés
// (on ne refait le calcul trigonométrique que si le pot a vraiment bougé)
static float eqLowCalc = -1.0f, eqMidCalc = -1.0f, eqHighCalc = -1.0f;

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
  tgtDist   = clampf(p->dist,   0.0f, 1.0f);
  tgtOct    = clampf(p->oct,    0.0f, 1.0f);
  tgtLow    = clampf(p->low,    0.0f, 1.0f);
  tgtMid    = clampf(p->mid,    0.0f, 1.0f);
  tgtHigh   = clampf(p->high,   0.0f, 1.0f);
  tgtTone   = clampf(p->tone,   0.0f, 1.0f);
  tgtVolume = clampf(p->volume, 0.0f, VOLUME_MAX);
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
  if (len != (int)sizeof(PedalParams)) return;   // ancien maître -> ignoré
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
    } else {
      Serial.printf("[Metre] entree: %.0f pas ADC | env=%.4f gate=%s(%.2f) | "
                    "sortie DAC: +/-%d pas | G=%.2f D=%.2f (Vsat=%.3f) O=%.2f "
                    "B/M/H=%.2f/%.2f/%.2f T=%.2f V=%.2f E=%.0f\n",
                    mPeakIn, envelope, (gateGain > 0.5f) ? "OUVERT" : "ferme",
                    gateGain, mPeakOut, smGain, smDist, satVsat, smOct,
                    smLow, smMid, smHigh, smTone, smVolume, smEffect);
    }
    mPeakIn  = 0.0f;
    mPeakOut = 0;
    mCount   = 0;
  }
}
#endif

// ---------------------------------------------------------------------------
// Bip de test : sinus envoyé directement au DAC (diagnostic sortie)
// ---------------------------------------------------------------------------
#if STARTUP_TEST_TONE
static void playTestTone(float freqHz, float durSec, float amp) {
  const uint32_t n = (uint32_t)(durSec * (float)SAMPLE_RATE_HZ);
  const float    w = 2.0f * (float)M_PI * freqHz / (float)SAMPLE_RATE_HZ;
  for (uint32_t i = 0; i < n; i++) {
    const float s = amp * sinf(w * (float)i);
    dacWrite(PIN_AUDIO_OUT, (uint8_t)lroundf(128.0f + s * 127.0f));
    delayMicroseconds(SAMPLE_PERIOD_US - 12);
  }
  dacWrite(PIN_AUDIO_OUT, 128);
}
#endif

// ---------------------------------------------------------------------------
// Pas de SIGMA-DELTA du DAC : quantifie dacTarget en réinjectant l'erreur.
// Appelé au moins 2x par période d'échantillon (~40 kHz et plus, voir loop) :
// le bruit de quantification du DAC 8 bits est repoussé vers l'ultrasonique
// (12-20 kHz, que ni l'ampli ni l'oreille ne reproduisent) au lieu de
// s'empiler en bande audible 6-9 kHz — c'était le "scratch" continu.
// Mesuré en simulation : -6,7 dB de bruit dans la bande 5-9 kHz.
// ---------------------------------------------------------------------------
// Générateur de DITHER triangulaire ±0,5 LSB (xorshift32, ~0,1 µs).
// Sans dither, la boucle d'erreur entre en CYCLE LIMITE dès que la sortie
// est quasi constante : elle émet un SIFFLEMENT PUR dont la fréquence dépend
// du niveau (mesuré : raies à 4-9 kHz jusqu'à -14 dB LSB) — c'était le "son
// persistant". Le dither casse le cycle : -8 à -15 dB sur la raie, l'énergie
// devient un souffle diffus non tonal.
static uint32_t ditherSeed = 0x1234ABCDu;
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
  // Pas de gain, pas de filtre, pas de gate, pas de sigma-delta : la
  // guitare telle que les convertisseurs la voient (12 bits -> 8 bits, 1:1).
  if (directMode) {
    const int raw = readGuitarAdc();
    dacWrite(PIN_AUDIO_OUT, (uint8_t)(raw >> 4));
    return;
  }

  // Lissage des paramètres (progression douce, pas de craquement)
  smGain   += PARAM_SMOOTH * (tgtGain   - smGain);
  smDist   += PARAM_SMOOTH * (tgtDist   - smDist);
  smOct    += PARAM_SMOOTH * (tgtOct    - smOct);
  smLow    += PARAM_SMOOTH * (tgtLow    - smLow);
  smMid    += PARAM_SMOOTH * (tgtMid    - smMid);
  smHigh   += PARAM_SMOOTH * (tgtHigh   - smHigh);
  smTone   += PARAM_SMOOTH * (tgtTone   - smTone);
  smVolume += PARAM_SMOOTH * (tgtVolume - smVolume);
  smEffect += PARAM_SMOOTH * (tgtEffect - smEffect);

  // Recalcul périodique des coefficients (~3 ms). Les biquads d'EQ, plus
  // coûteux (cos/sin/pow), ne sont refaits que si leur pot a bougé.
  if (coefCountdown == 0) {
    coefCountdown = COEF_UPDATE_SAMPLES;

    calcAmpli(&fAmpli, driveTaper(smGain));
    calcSortie(&fSortie, smTone, smVolume);

    satVsat = SAT_V0 * powf(10.0f, -SAT_LOGSPAN * smDist);
    satMix  = (smDist < 0.01f) ? 0.0f
            : (smDist * SAT_MIX_RAMP > 1.0f ? 1.0f : smDist * SAT_MIX_RAMP);
    satBias = SAT_ASYM * satVsat;
    satHard = 1.0f + SAT_HARD * smDist;

    if (fabsf(smLow - eqLowCalc) > 0.002f) {
      calcLowShelf(&fLow, EQ_LOW_HZ, eqDb(smLow));
      calcTight(&fTight, smLow);       // le pot Low pilote aussi le resserrage
      eqLowCalc = smLow;
    }
    if (fabsf(smMid - eqMidCalc) > 0.002f) {
      calcPeak(&fMid, EQ_MID_HZ, EQ_MID_Q, eqDb(smMid));
      eqMidCalc = smMid;
    }
    if (fabsf(smHigh - eqHighCalc) > 0.002f) {
      calcHighShelf(&fHigh, EQ_HIGH_HZ, eqDb(smHigh));
      eqHighCalc = smHigh;
    }
  }
  coefCountdown--;

  // --- Lecture ADC + suppression de l'offset DC ---
  const int raw = readGuitarAdc();
  dcOffset += DC_TRACK_COEF * ((float)raw - dcOffset);
  const float x = ((float)raw - dcOffset) * (INPUT_GAIN / 2048.0f);

#if DEBUG_METER
  meterTick((float)raw);
#endif

  // --- Noise gate INTELLIGENT : enveloppes rapide + lente sur l'ENTRÉE ---
  // La mesure passe d'abord par un passe-haut 300 Hz : ni le ronflement
  // secteur ni le GRONDEMENT DE FROTTEMENT de la main posée sur les cordes
  // (énergie 100-300 Hz) n'ouvrent le gate ; une vraie note ouvre par ses
  // harmoniques, toujours présentes au-dessus de 300 Hz.
  const float eh = envHpA * (envHpY + x - envHpX);
  envHpX = x;
  envHpY = eh;
  const float mag = fabsf(eh);
  envelope += (mag > envelope ? ENV_ATTACK : ENV_RELEASE)      * (mag - envelope);
  envSlow  += (mag > envSlow  ? ENV_ATTACK : ENV_RELEASE_SLOW) * (mag - envSlow);

  // Cordes étouffées (chute brutale) ou note tenue (décroissance naturelle) ?
  const bool abrupt = (envelope < ABRUPT_RATIO * envSlow);

  const float baseScale = 1.0f + GATE_DRIVE_SCALE * smGain
                               + GATE_DIST_SCALE  * smDist;
  // Les seuils "sustain" (x0.6) ne s'appliquent que si une VRAIE note a
  // précédé (mémoire lente au-dessus du seuil haut) : le frottement de la
  // main, qui varie lentement lui aussi, ne peut plus en profiter.
  const bool  wasNote   = (envSlow > GATE_HIGH * baseScale);
  const float gateScale = baseScale
                          * ((!abrupt && wasNote) ? SUSTAIN_THRESH : 1.0f);
  const float gLow  = GATE_LOW  * gateScale;
  const float gHigh = GATE_HIGH * gateScale;

  float gateTarget;
  if      (envelope <= gLow)  gateTarget = 0.0f;
  else if (envelope >= gHigh) gateTarget = 1.0f;
  else {
    // Pente d'EXPANDEUR (au carré) : dès que la note faiblit, l'atténuation
    // accélère — la fin de note s'arrête net au lieu de descendre en traînant
    const float lin = (envelope - gLow) / (gHigh - gLow);
    gateTarget = lin * lin;
  }

  // Fermeture : franche sur un arrêt net, douce sur une fin de sustain
  const float closeCoef = abrupt ? GATE_CLOSE_COEF : GATE_CLOSE_SOFT;
  gateGain += (gateTarget > gateGain ? GATE_OPEN_COEF : closeCoef)
              * (gateTarget - gateGain);
  // Verrou : sous GATE_SNAP le gain est collé à 0 STRICT -> aucune note en
  // cours = aucune sortie du tout (le DAC reste exactement au repos, même
  // le dither de quantification est remis à zéro)
  if (gateGain < GATE_SNAP && gateTarget <= 0.0f) {
    gateGain = 0.0f;
    quantErr = 0.0f;
  }

  // --- 1-2. Étages linéaires d'entrée (gate appliqué AVANT le gain) ---
  float y = biquadRun(&fHin,   x) * gateGain;   // Hin(s)
  y       = biquadRun(&fAmpli, y);              // Hampli(s,g)

  // --- 2a. Resserrage pré-écrêtage : passe-haut 140 Hz sur la voie saturée
  //     (le bas "bave" dans la disto ; l'EQ Low peut le remettre APRÈS) ---
  y = biquadRun(&fTight, y);

  // --- 2a-bis. Anti-souffle : passe-bas 4 kHz AVANT la saturation ---
  // Le souffle des convertisseurs amplifié par le drive puis compressé par
  // le fuzz = la nappe de bruit continue 6-8 kHz. On le coupe ici ; la
  // saturation recrée de toute façon les harmoniques au-dessus de 4 kHz.
  y = biquadRun(&fPreLp, y);

  // --- 2b. Octave O(u,o) — redressement double alternance (fOXX) ---
  // |u| ne contient que les harmoniques PAIRS -> la fondamentale disparaît,
  // remplacée par l'octave supérieure. Le passe-haut émule la liaison 10 µF.
  if (smOct > 0.005f) {
    const float rect   = fabsf(y);
    const float rectAc = octA * (octPrevY + rect - octPrevX);
    octPrevX = rect;
    octPrevY = rectAc;
    y += smOct * (OCT_GAIN * rectAc - y);       // fondu direct <-> octave
  }

  // --- 3. Saturation S(u,d) : double tanh en cascade (agressif SANS fizz) ---
  if (satMix > 0.0f) {
    // Étage 1 : écrêtage doux ASYMÉTRIQUE (harmoniques paires, "lampe")
    const float soft = tanhf((y + satBias) / satVsat) - SAT_BIAS_OUT;
    // Étage 2 : re-amplifié puis tanh — aussi dense qu'un clip dur, mais sans
    // angle net donc quasi pas d'aliasing (l'ancien clip dur créait des
    // fréquences inharmoniques dissonantes sur les notes seules)
    const float hard = tanhf(soft * satHard);
    y += (hard - y) * satMix;               // fondu linéaire <-> saturé
  }

  // --- 4-5. Égaliseur 3 bandes ---
  y = biquadRun(&fLow,  y);           // low-shelf  100 Hz
  y = biquadRun(&fMid,  y);           // peak       700 Hz
  y = biquadRun(&fHigh, y);           // high-shelf 3,2 kHz

  // --- 5b. Simulation haut-parleur : passe-bas 5,5 kHz ordre 2 ---
  // Un HP guitare ne reproduit rien au-dessus de ~5,5 kHz : ce filtre retire
  // le grésil numérique aigu qui rendait les notes seules désagréables.
  y = biquadRun(&fCab, y);

  // --- 6. Tone + volume ---
  y = biquadRun(&fSortie, y);         // Hsortie(s,t,v)

  // --- Gate appliqué AUSSI en sortie : la saturation recomprime le fondu
  //     d'entrée, sans cette 2e application on entendrait le bruit
  //     "redescendre" à chaque fin de note (atténuation au carré) ---
  y *= gateGain;

  // --- CLEAN pur : à Dist = 0 le signal ne subit AUCUNE modification,
  //     seulement le rehaussement CLEAN_GAIN dosé par le pot Volume.
  //     satMix (0 -> 1 sur le premier quart du pot Dist) fait le fondu
  //     continu entre ce chemin brut et la chaîne traitée complète. ---
  const float clean = x * CLEAN_GAIN * smVolume;
  y = clean + (y - clean) * satMix;

  // --- Bypass en fondu : mélange signal direct <-> signal traité ---
  const float dry = x * BYPASS_GAIN;
  y = dry + (y - dry) * smEffect;

  // --- Cible DAC : la sortie physique est tenue par le sigma-delta (~40 kHz) ---
  dacTarget = 128.0f + y * OUTPUT_LEVEL * 127.0f;
  dacStep();
}

// ---------------------------------------------------------------------------
// Initialisation
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== SOAJ Pédale — ESCLAVE SATU (saturation + EQ + octave fOXX) ===");

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

  // --- Coefficients initiaux de tous les étages ---
  // Passe-haut de l'étage octave : y = a·(yPrev + x − xPrev), a = rc/(rc+dt)
  {
    const float rc = 1.0f / (2.0f * (float)M_PI * OCT_HPF_HZ);
    octA = rc / (rc + DT_SEC);
  }
  calcHin(&fHin);
  calcAmpli(&fAmpli, driveTaper(smGain));
  calcSortie(&fSortie, smTone, smVolume);
  // Resserrage pré-écrêtage (fréquence pilotée par le pot Low)
  calcTight(&fTight, smLow);
  // Passe-haut de la mesure d'enveloppe du gate (anti-ronflette + anti-frottement)
  {
    const float rc = 1.0f / (2.0f * (float)M_PI * GATE_DETECT_HPF_HZ);
    envHpA = rc / (rc + DT_SEC);
  }
  // Simulation haut-parleur : passe-bas Butterworth H(s) = 1/(1 + s/(Q·w0) + s²/w0²)
  {
    const float w0 = 2.0f * (float)M_PI * CAB_LPF_HZ;
    bilinear2(&fCab, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f / (CAB_Q * w0), 1.0f / (w0 * w0));
  }
  // Anti-souffle pré-saturation : passe-bas Butterworth 4 kHz
  {
    const float w0 = 2.0f * (float)M_PI * PRE_LPF_HZ;
    bilinear2(&fPreLp, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f / (CAB_Q * w0), 1.0f / (w0 * w0));
  }
  calcLowShelf(&fLow, EQ_LOW_HZ, eqDb(smLow));    eqLowCalc  = smLow;
  calcPeak(&fMid, EQ_MID_HZ, EQ_MID_Q, eqDb(smMid)); eqMidCalc = smMid;
  calcHighShelf(&fHigh, EQ_HIGH_HZ, eqDb(smHigh)); eqHighCalc = smHigh;
  satVsat = SAT_V0 * powf(10.0f, -SAT_LOGSPAN * smDist);
  satMix  = (smDist * SAT_MIX_RAMP > 1.0f) ? 1.0f : smDist * SAT_MIX_RAMP;
  satBias = SAT_ASYM * satVsat;
  satHard = 1.0f + SAT_HARD * smDist;

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
  Serial.printf("Audio : %d Hz — Hin x Hampli -> S(u,d) -> EQ 3 bandes -> Hsortie\n",
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
  Serial.println("[Test] BIP 440 Hz x2 sur le DAC (test du chemin de sortie)...");
  playTestTone(440.0f, 0.35f, 0.6f);
  delay(150);
  playTestTone(440.0f, 0.35f, 0.6f);
  Serial.println("[Test] Bips terminés. Si rien entendu : vérifier GPIO25 -> ampli.");
#endif

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
