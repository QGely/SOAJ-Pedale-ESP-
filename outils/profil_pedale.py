#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
============================================================================
 SOAJ - Pédale d'effet guitare ESP32
 profil_pedale.py — convertit une capture TONE3000 (NAM) en "profil" SOAJ
============================================================================

Un modèle NAM (fichier .nam de tone3000.com) est un réseau de neurones bien
trop lourd pour l'ESP32. Ce script en extrait une approximation légère — un
PROFIL — que l'esclave SATU sait exécuter en temps réel :

    profil = courbe d'écrêtage statique (257 points, mesurée)
           + passe-haut d'entrée      (resserrage du grave)
           + passe-bas pré-écrêtage   (anti-souffle)
           + correction de médiums    (bosse TS9 / creux Big Muff...)
           + passe-bas de sortie      (voicing / haut-parleur)
           + gain de rattrapage

C'est le modèle "Hammerstein" classique : filtre -> non-linéarité -> filtre.
Pour une pédale de saturation (essentiellement une non-linéarité entourée de
filtres), l'approximation garde l'essentiel du caractère ; elle perd le
comportement dynamique fin du réseau (la capture NAM reste la référence).

UTILISATION — 3 étapes, aucun code à écrire :

  1. Générer le signal de test (WAV mono) :
         python3 profil_pedale.py --test-wav test_sec.wav

  2. Faire passer test_sec.wav dans la capture TONE3000 :
     ouvrir le plugin Neural Amp Modeler (gratuit, neuralampmodeler.com)
     dans un DAW (Reaper, Ardour...), charger le fichier .nam téléchargé
     depuis tone3000.com, traiter le WAV et exporter le résultat
     (MÊME fréquence d'échantillonnage, MÊME durée, mono, sans normaliser).

  3. Mesurer et générer le code C du profil :
         python3 profil_pedale.py --analyser test_sec.wav test_traite.wav --nom MAPEDALE

     Copier le bloc imprimé dans PedaleEsclaveSatu/profils_pedales.h
     (et incrémenter NB_PROFILS + ajouter l'entrée dans PROFILS[]).

Le script sait aussi régénérer l'en-tête complet livré avec le dépôt
(3 profils de référence analytiques TS9 / RAT / Big Muff) :
         python3 profil_pedale.py --entete ../PedaleEsclaveSatu/profils_pedales.h

Dépendance : numpy (pip install numpy). Les WAV sont lus/écrits en PCM
16 bits mono via le module standard `wave`.
============================================================================
"""

import argparse
import sys
import wave

import numpy as np

# ---------------------------------------------------------------------------
# Constantes partagées avec le firmware (PedaleEsclaveSatu/profils_pedales.h)
# ---------------------------------------------------------------------------
LUT_TAILLE = 257     # points de la courbe d'écrêtage
LUT_PLAGE = 4.0      # la courbe couvre u dans [-LUT_PLAGE, +LUT_PLAGE]

# --- Signal de test (étape 1) ----------------------------------------------
TEST_SR = 48000      # fréquence d'échantillonnage du WAV de test
RAMPE_HZ = 220.0     # sinus dont l'amplitude monte : mesure la courbe statique
RAMPE_SEC = 8.0
RAMPE_AMP_MAX = 0.95
SILENCE_SEC = 0.5
SWEEP_SEC = 6.0      # balayage log petit signal : mesure la réponse linéaire
SWEEP_F0 = 30.0
SWEEP_F1 = 15000.0
SWEEP_AMP = 0.03     # assez faible pour rester sous le seuil d'écrêtage


# ---------------------------------------------------------------------------
# Courbes analytiques de référence (profils livrés avec le dépôt)
# ---------------------------------------------------------------------------
def courbe_ts9(u):
    """Ibanez TS9 : diodes en contre-réaction -> genou DOUX, légère asymétrie
    (le caractère 'lampe' de l'overdrive). Les médiums bossus viennent des
    filtres du profil, pas de la courbe."""
    up, un = 1.0, 0.85           # seuil un peu plus bas côté négatif
    return np.where(u >= 0.0, up * np.tanh(u / up), un * np.tanh(u / un))


def courbe_rat(u):
    """ProCo RAT : diodes vers la masse APRÈS le gros gain du LM308 ->
    genou DUR, presque carré (|x|^6 arrondit à peine le coude)."""
    return u / (1.0 + np.abs(u) ** 6) ** (1.0 / 6.0)


def courbe_muff(u):
    """Big Muff Pi : DEUX étages d'écrêtage doux en cascade -> compression
    énorme, bords ronds, sustain infini."""
    e1 = np.tanh(1.6 * u)
    return 0.95 * np.tanh(2.2 * e1) / np.tanh(2.2)


# Paramètres de voicing de chaque référence :
#   (fonction, preHpfHz, preLpfHz, gainMin, gainMax,
#    postMidHz, postMidQ, postMidDb, postLpfHz, sortieGain, commentaire)
REFERENCES = {
    "ts9": (courbe_ts9, 720.0, 5000.0, 8.0, 120.0,
            700.0, 0.80, 3.5, 3500.0, 1.00,
            "Ibanez TS9 Tube Screamer — overdrive doux, medium bossu"),
    "rat": (courbe_rat, 120.0, 4000.0, 12.0, 700.0,
            700.0, 1.00, 0.0, 2200.0, 0.90,
            "ProCo RAT — distorsion mordante, ecretage dur"),
    "muff": (courbe_muff, 90.0, 3500.0, 20.0, 600.0,
             750.0, 0.60, -8.0, 3000.0, 0.90,
             "EHX Big Muff Pi — fuzz compresse, mediums creuses"),
}


# ---------------------------------------------------------------------------
# WAV mono PCM 16 bits (module standard, aucune dépendance audio)
# ---------------------------------------------------------------------------
def ecrire_wav(chemin, signal, sr):
    donnees = np.clip(signal, -1.0, 1.0)
    pcm = (donnees * 32767.0).astype(np.int16)
    with wave.open(chemin, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(sr)
        w.writeframes(pcm.tobytes())


def lire_wav(chemin):
    with wave.open(chemin, "rb") as w:
        nch, sw, sr, n = (w.getnchannels(), w.getsampwidth(),
                          w.getframerate(), w.getnframes())
        brut = w.readframes(n)
    if sw == 2:
        x = np.frombuffer(brut, dtype=np.int16).astype(np.float64) / 32768.0
    elif sw == 4:
        x = np.frombuffer(brut, dtype=np.int32).astype(np.float64) / 2147483648.0
    else:
        sys.exit(f"{chemin} : PCM {8*sw} bits non géré (exporter en 16 ou 32 bits)")
    if nch > 1:
        x = x.reshape(-1, nch).mean(axis=1)   # mono = moyenne des canaux
    return x, sr


# ---------------------------------------------------------------------------
# Étape 1 : signal de test
# ---------------------------------------------------------------------------
def generer_test(sr=TEST_SR):
    t_rampe = np.arange(int(RAMPE_SEC * sr)) / sr
    amp = RAMPE_AMP_MAX * t_rampe / RAMPE_SEC              # rampe 0 -> max
    rampe = amp * np.sin(2.0 * np.pi * RAMPE_HZ * t_rampe)

    silence = np.zeros(int(SILENCE_SEC * sr))

    # Balayage sinus logarithmique (Farina) 30 Hz -> 15 kHz, petit signal
    t_sw = np.arange(int(SWEEP_SEC * sr)) / sr
    k = np.log(SWEEP_F1 / SWEEP_F0)
    phase = 2.0 * np.pi * SWEEP_F0 * SWEEP_SEC / k * (np.exp(t_sw * k / SWEEP_SEC) - 1.0)
    sweep = SWEEP_AMP * np.sin(phase)

    return np.concatenate([rampe, silence, sweep, silence])


# ---------------------------------------------------------------------------
# Étape 3 : analyse sec/traité -> paramètres du profil
# ---------------------------------------------------------------------------
def aligner(sec, traite):
    """Compense la latence du plugin par corrélation croisée (2 premières s)."""
    n = min(len(sec), len(traite), 2 * TEST_SR)
    corr = np.correlate(traite[:n], sec[:n], mode="full")
    dec = int(np.argmax(np.abs(corr))) - (n - 1)
    if dec > 0:
        traite = traite[dec:]
    elif dec < 0:
        sec = sec[-dec:]
    n = min(len(sec), len(traite))
    return sec[:n], traite[:n], dec


def mesurer_courbe(sec, traite, sr):
    """Courbe statique par crêtes synchrones : sur chaque période du sinus en
    rampe, la crête d'entrée A donne la crête de sortie f(A) (méthode robuste
    au déphasage des filtres). Crêtes + et − mesurées séparément -> asymétrie."""
    n_rampe = int(RAMPE_SEC * sr)
    x, y = sec[:n_rampe], traite[:n_rampe]
    par_periode = int(round(sr / RAMPE_HZ))
    nper = n_rampe // par_periode

    e_pos, s_pos, e_neg, s_neg = [], [], [], []
    for i in range(2, nper - 1):           # on saute la 1re période (transitoire)
        d, f = i * par_periode, (i + 1) * par_periode
        e_pos.append(x[d:f].max());  s_pos.append(y[d:f].max())
        e_neg.append(-x[d:f].min()); s_neg.append(-y[d:f].min())
    e_pos, s_pos = np.array(e_pos), np.array(s_pos)
    e_neg, s_neg = np.array(e_neg), np.array(s_neg)

    # Axe d'entrée de la LUT : la pleine échelle du test (RAMPE_AMP_MAX)
    # est placée à LUT_PLAGE/2 -> la moitié haute de la table reste couverte
    # par prolongement plat (la saturation y est de toute façon écrasée).
    echelle = (LUT_PLAGE / 2.0) / RAMPE_AMP_MAX
    u = np.linspace(-LUT_PLAGE, LUT_PLAGE, LUT_TAILLE)

    def interp_demi(entrees, sorties, uu):
        ordre = np.argsort(entrees)
        return np.interp(uu, entrees[ordre] * echelle, sorties[ordre],
                         left=0.0, right=float(sorties[ordre][-1]))

    lut = np.where(u >= 0.0,
                   interp_demi(e_pos, s_pos, np.maximum(u, 0.0)),
                   -interp_demi(e_neg, s_neg, np.maximum(-u, 0.0)))
    # Normalisation : crête à ~0.95 (le firmware rattrape avec sortieGain)
    crete = np.max(np.abs(lut))
    gain_norm = 0.95 / crete if crete > 1e-9 else 1.0
    return lut * gain_norm, 1.0 / gain_norm


def mesurer_reponse(sec, traite, sr):
    """Réponse linéaire petit signal (section balayage) -> coins HPF/LPF et
    bosse/creux de médiums, par simple rapport de spectres lissé."""
    d = int((RAMPE_SEC + SILENCE_SEC) * sr)
    f = d + int(SWEEP_SEC * sr)
    x, y = sec[d:f], traite[d:f]

    X, Y = np.fft.rfft(x), np.fft.rfft(y)
    freqs = np.fft.rfftfreq(len(x), 1.0 / sr)
    rep = np.abs(Y) / np.maximum(np.abs(X), 1e-9)

    # Lissage en bandes log (1/6 d'octave)
    centres = SWEEP_F0 * 2.0 ** (np.arange(0, np.log2(SWEEP_F1 / SWEEP_F0), 1.0 / 6.0))
    niveaux = np.array([
        np.median(rep[(freqs >= c / 2 ** (1 / 12)) & (freqs < c * 2 ** (1 / 12))])
        for c in centres])
    db = 20.0 * np.log10(np.maximum(niveaux, 1e-9))

    ref_db = np.median(db[(centres >= 400.0) & (centres <= 1200.0)])

    def coin_bas():
        for c, v in zip(centres, db):
            if v >= ref_db - 3.0:
                return max(c, 40.0)
        return 40.0

    def coin_haut():
        for c, v in zip(centres[::-1], db[::-1]):
            if v >= ref_db - 3.0:
                return min(c, 8000.0)
        return 8000.0

    bas, haut = coin_bas(), coin_haut()

    # Bosse/creux de médiums : cherché ENTRE les coins (à une octave de marge),
    # sinon les pentes des passe-haut/passe-bas seraient prises pour des bosses
    zone_mid = (centres >= max(300.0, 2.0 * bas)) & (centres <= min(2000.0, haut / 2.0))
    mid_db, mid_hz = 0.0, 700.0
    if np.any(zone_mid):
        ecart = db[zone_mid] - ref_db
        i_max = int(np.argmax(np.abs(ecart)))
        if abs(ecart[i_max]) >= 1.5:         # en dessous : pas de vraie bosse
            mid_db = float(np.clip(ecart[i_max], -12.0, 12.0))
            mid_hz = float(centres[zone_mid][i_max])

    return bas, haut, mid_hz, mid_db


def analyser(chemin_sec, chemin_traite, nom):
    sec, sr1 = lire_wav(chemin_sec)
    traite, sr2 = lire_wav(chemin_traite)
    if sr1 != sr2:
        sys.exit(f"Fréquences différentes ({sr1} vs {sr2} Hz) : ré-exporter au même taux")
    sec, traite, dec = aligner(sec, traite)
    print(f"// Latence plugin compensée : {dec} échantillons", file=sys.stderr)

    lut, rattrapage = mesurer_courbe(sec, traite, sr1)
    hpf, lpf, mid_hz, mid_db = mesurer_reponse(sec, traite, sr1)

    # Le niveau de la capture est déjà "dans" la courbe : plage de drive
    # modérée par défaut (à ajuster à l'oreille dans PROFILS[]).
    imprimer_profil(nom, lut, hpf, min(lpf, 5000.0), 4.0, 40.0,
                    mid_hz, 0.8, mid_db, max(lpf, 2000.0), min(rattrapage, 1.2),
                    f"mesure depuis capture NAM ({chemin_traite})")


# ---------------------------------------------------------------------------
# Génération du code C
# ---------------------------------------------------------------------------
def format_lut(nom_c, lut):
    lignes = [f"static const float COURBE_{nom_c}[LUT_TAILLE] = {{"]
    for i in range(0, LUT_TAILLE, 8):
        vals = ", ".join(f"{v:+.5f}f" for v in lut[i:i + 8])
        lignes.append(f"  {vals},")
    lignes[-1] = lignes[-1].rstrip(",") + " };"
    return "\n".join(lignes)


def format_entree_profils(nom, nom_c, hpf, lpf, gmin, gmax,
                          mid_hz, mid_q, mid_db, post_lpf, sgain):
    return (f'  {{ "{nom}", {hpf:.0f}.0f, {lpf:.0f}.0f, {gmin:.1f}f, {gmax:.1f}f, '
            f'{mid_hz:.0f}.0f, {mid_q:.2f}f, {mid_db:.1f}f, {post_lpf:.0f}.0f, '
            f'{sgain:.2f}f, COURBE_{nom_c} }},')


def imprimer_profil(nom, lut, hpf, lpf, gmin, gmax,
                    mid_hz, mid_q, mid_db, post_lpf, sgain, origine):
    nom_c = "".join(ch if ch.isalnum() else "_" for ch in nom.upper())
    print(f"// ---- Profil {nom} — {origine} ----")
    print(format_lut(nom_c, lut))
    print("// À ajouter dans PROFILS[] (et incrémenter NB_PROFILS) :")
    print(format_entree_profils(nom, nom_c, hpf, lpf, gmin, gmax,
                                mid_hz, mid_q, mid_db, post_lpf, sgain))


ENTETE_DEBUT = """\
/*
 * ============================================================================
 *  SOAJ - Pédale d'effet guitare ESP32
 *  profils_pedales.h — profils de saturation pour PedaleEsclaveSatu.ino
 * ============================================================================
 *
 *  FICHIER GÉNÉRÉ par outils/profil_pedale.py — ne pas éditer les tables à
 *  la main (relancer :  python3 outils/profil_pedale.py --entete <ce fichier>).
 *
 *  Un profil = approximation légère d'une pédale (capture TONE3000/NAM ou
 *  modèle analytique) exécutable à 20 kHz sur ESP32 :
 *    passe-haut -> passe-bas -> COURBE D'ÉCRÊTAGE (LUT) -> médiums -> passe-bas
 *  Le profil 0 est le circuit SOAJ d'origine (chaîne existante, pas de LUT).
 *
 *  Pour convertir votre propre capture tone3000.com : voir le mode d'emploi
 *  en tête de outils/profil_pedale.py (3 étapes, résultat à coller ici).
 * ============================================================================
 */
#pragma once

#define LUT_TAILLE 257        // points de la courbe d'écrêtage
#define LUT_PLAGE  4.0f       // la courbe couvre u dans [-LUT_PLAGE, +LUT_PLAGE]
#define NB_PROFILS 4          // 0 = circuit SOAJ + les profils ci-dessous

typedef struct {
  const char  *nom;           // affiché au moniteur série
  float preHpfHz;             // passe-haut avant écrêtage (resserrage du grave)
  float preLpfHz;             // passe-bas avant écrêtage (anti-souffle)
  float gainMin, gainMax;     // plage du pot DRIVE (course logarithmique)
  float postMidHz;            // correction de médiums APRÈS écrêtage :
  float postMidQ;             //   bosse TS9 (+dB) ou creux Big Muff (−dB)
  float postMidDb;            //   0 dB = neutre
  float postLpfHz;            // passe-bas de sortie (voicing)
  float sortieGain;           // rattrapage de niveau
  const float *courbe;        // LUT_TAILLE points sur ±LUT_PLAGE (NULL = profil 0)
} ProfilPedale;
"""


def generer_entete(chemin):
    u = np.linspace(-LUT_PLAGE, LUT_PLAGE, LUT_TAILLE)
    blocs, entrees = [], [
        '  { "SOAJ", 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.00f, 0.0f, 0.0f, '
        "0.00f, NULL },   // circuit d'origine (chaîne existante)"
    ]
    for cle, (fct, hpf, lpf, gmin, gmax, mhz, mq, mdb, plpf, sg, com) in REFERENCES.items():
        nom = cle.upper()
        blocs.append(f"// ---- {nom} : {com} ----\n" + format_lut(nom, fct(u)))
        entrees.append(format_entree_profils(nom, nom, hpf, lpf, gmin, gmax,
                                             mhz, mq, mdb, plpf, sg) + f"  // {com}")
    texte = (ENTETE_DEBUT + "\n" + "\n\n".join(blocs) + "\n\n"
             + "static const ProfilPedale PROFILS[NB_PROFILS] = {\n"
             + "\n".join(entrees) + "\n};\n")
    with open(chemin, "w", encoding="utf-8") as f:
        f.write(texte)
    print(f"Écrit : {chemin} ({len(texte)} octets, {len(REFERENCES)} profils + circuit)")


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description="Convertit une capture TONE3000 (NAM) en profil SOAJ pour ESP32.")
    ap.add_argument("--test-wav", metavar="SORTIE.wav",
                    help="étape 1 : génère le signal de test à traiter par le plugin NAM")
    ap.add_argument("--sr", type=int, default=TEST_SR,
                    help=f"fréquence du WAV de test (défaut {TEST_SR})")
    ap.add_argument("--analyser", nargs=2, metavar=("SEC.wav", "TRAITE.wav"),
                    help="étape 3 : mesure le profil depuis le couple sec/traité")
    ap.add_argument("--nom", default="MAPEDALE",
                    help="nom du profil généré (avec --analyser)")
    ap.add_argument("--ref", choices=sorted(REFERENCES),
                    help="imprime un profil analytique de référence (ts9, rat, muff)")
    ap.add_argument("--entete", metavar="FICHIER.h",
                    help="régénère l'en-tête complet livré avec le dépôt")
    args = ap.parse_args()

    if args.test_wav:
        ecrire_wav(args.test_wav, generer_test(args.sr), args.sr)
        print(f"Écrit : {args.test_wav} ({args.sr} Hz, "
              f"{RAMPE_SEC + 2 * SILENCE_SEC + SWEEP_SEC:.0f} s). "
              "À traiter par le plugin NAM SANS normalisation, puis --analyser.")
    elif args.analyser:
        analyser(args.analyser[0], args.analyser[1], args.nom)
    elif args.ref:
        fct, hpf, lpf, gmin, gmax, mhz, mq, mdb, plpf, sg, com = REFERENCES[args.ref]
        u = np.linspace(-LUT_PLAGE, LUT_PLAGE, LUT_TAILLE)
        imprimer_profil(args.ref.upper(), fct(u), hpf, lpf, gmin, gmax,
                        mhz, mq, mdb, plpf, sg, com)
    elif args.entete:
        generer_entete(args.entete)
    else:
        ap.print_help()


if __name__ == "__main__":
    main()
