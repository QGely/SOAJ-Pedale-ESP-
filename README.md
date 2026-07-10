# SOAJ — Pédale d'effet guitare (saturation) sur ESP32

Pédale de saturation numérique contrôlée sans fil :

```
Téléphone ──(Bluetooth BLE)──> ESP32 MAÎTRE ──(ESP-NOW / WiFi)──> ESP32 ESCLAVE ──> Ampli
                                                                       ▲
                                                              Guitare (jack, ADC)
```

- **Seuls les paramètres** (gain, clip, tone, volume, ON/OFF) transitent par radio : **jamais l'audio**.
- L'audio est traité **localement** dans l'esclave, échantillon par échantillon (20 kHz), en flottant.
- Sans guitare en entrée : **silence total** (noise gate progressif + DAC au repos à 128).

> Note : le lien maître → esclave utilise **ESP-NOW (WiFi)** et non le BLE, conformément au
> schéma d'architecture (cellule WiFi entre pédales). C'est aussi pour cela que l'entrée
> guitare est sur **GPIO34 (ADC1)** : l'ADC2 est inutilisable quand le WiFi est actif.

## Contenu du dépôt

| Dossier | Carte | Rôle |
|---|---|---|
| `PedaleMaitre/` | ESP32 n°1 | Reçoit les réglages du téléphone (BLE), les diffuse en ESP-NOW |
| `PedaleEsclave/` | ESP32 n°N | Reçoit les paramètres, traite l'audio guitare, sortie DAC |

Compiler avec l'Arduino IDE, carte **« ESP32 Dev Module »** (cartes ELEGOO ESP32 CP2102).
Aucune bibliothèque externe : tout est inclus dans le core ESP32 (BLE, WiFi, ESP-NOW).

---

## Câblage de l'esclave — À LIRE : c'est la principale cause de grésillement

### Entrée guitare → GPIO34

L'ADC de l'ESP32 **ne supporte ni tension négative, ni plus de 3,3 V**. Le signal guitare
est alternatif (±100 mV environ) : il faut le **recentrer à 1,65 V** avec un condensateur
de liaison et un pont diviseur. Sans ce montage, la moitié négative du signal est écrasée
→ distorsion horrible + bruit.

```
                        3,3 V
                          │
                         R1 100 kΩ
   Guitare                │
   (pointe) ──── C1 ──────┼──────────── GPIO34
              100 nF      │
              à 1 µF     R2 100 kΩ
                          │
   Guitare               GND
   (manchon) ──────────── GND (commun avec l'ESP32)
```

- C1 : condensateur film ou céramique 100 nF à 1 µF (bloque le continu, laisse passer l'audio).
- R1/R2 : 100 kΩ chacun → polarisation à **1,65 V**, l'ADC lit ~2048 au repos.
- Au démarrage, l'esclave mesure l'offset et **prévient sur le port série** si la
  polarisation est absente ou anormale.
- Optionnel mais recommandé : un petit préampli (ex. MCP6002 alimenté en 3,3 V, gain ×5 à ×10)
  entre la guitare et C1. Le signal guitare brut est faible (~100 mV) : plus il est fort à
  l'entrée de l'ADC, moins on amplifie le bruit de quantification en logiciel.

### Sortie DAC GPIO25 → ampli

```
   GPIO25 ──── R3 1 kΩ ────┬──── C3 10 µF (+ vers GPIO25) ──── vers l'ampli (pointe)
                           │
                          C2 10 nF
                           │
                          GND ─────────────────────────────── vers l'ampli (manchon)
```

- R3 + C2 : filtre passe-bas ~16 kHz qui lisse les marches du DAC 8 bits.
- C3 : condensateur de liaison, supprime l'offset continu de 1,65 V avant l'ampli.
- Si le son reste trop fort ou « granuleux » : ajoutez un pont diviseur (ex. 10 kΩ / 1 kΩ)
  après C3 et **montez `OUTPUT_LEVEL`** dans le code (voir plus bas) — mieux vaut atténuer
  en matériel qu'en logiciel, le DAC 8 bits garde alors toute sa résolution.

### Alimentation et masse (anti-bruit)

- **Masse commune unique** : guitare, ESP32 et ampli sur la même masse, câbles courts.
- Évitez d'alimenter l'ESP32 par le port USB d'un PC pendant le jeu (boucle de masse
  + bruit d'alim) : préférez une batterie ou un bloc secteur 5 V propre.
- Éloignez les fils audio de l'antenne WiFi de la carte.

---

## Chaîne de traitement audio (esclave)

Tout est fait pour un son **propre, doux et silencieux au repos** :

1. Lecture ADC 12 bits sur GPIO34 à 20 kHz.
2. Suivi et suppression de l'**offset DC** (moyenne glissante lente).
3. **Passe-haut ~40 Hz** : retire le résidu continu et les basses parasites
   (garde le mi grave de la guitare, 82 Hz).
4. **Passe-bas ~5 kHz avant saturation** : réduit le bruit ADC avant amplification.
5. **Noise gate progressif** : suiveur d'enveloppe (attaque rapide, relâchement lent),
   ouverture/fermeture en fondu — pas de coupure brutale, pas de souffle sans guitare.
   Gate fermé = sortie strictement à 128 (silence absolu). Placé AVANT le gros gain,
   sinon le souffle serait amplifié ×500.
6. **Étage de saturation = émulation du circuit ADTL082** (schéma LTspice) :
   - potentiomètre **DRIVE** (R5 450k + R6 50k) = paramètre `G` : gain ×51 (G=0)
     à ×501 (G=1) ;
   - réseau C5 220n / R7 1k : le plein gain ne s'applique qu'au-dessus de ~723 Hz
     (graves propres, médiums/aigus qui saturent — le caractère du circuit) ;
   - C4 100p en contre-réaction : passe-bas 3,2 à 32 kHz selon le drive.
7. **Écrêtage aux rails ±12 V** (`tanh`, sans aliasing numérique) : le signal amplifié
   ×51..×501 vient s'aplatir sur les rails — c'est ça, la saturation. Le paramètre `C`
   abaisse virtuellement les rails (plus bas = écrase plus tôt).
8. **Tone = potentiomètre R8+R9 (10k) + C7 (22n)** = paramètre `T` : passe-bas
   variable ~760 Hz (T=0, sombre) à ~14 kHz (T=1, brillant) ; T=0.5 ≈ 1,45 kHz
   comme le schéma (R8=R9=5k).
9. **Lissage de tous les paramètres** (~60 ms) : aucun craquement quand on change une
   valeur ou qu'on active/désactive l'effet (le bypass est un fondu enchaîné).
10. **Volume** (pot R10+R11, 0 à 1) × amplitude DAC maximale `OUTPUT_LEVEL`.
    Au volume par défaut (0,5), la sortie fait ~±28 pas de DAC (~±0,37 V) : audible
    mais modéré — le niveau final se règle sur l'ampli.
11. Sortie **DAC centrée sur 128**, avec mise en forme du bruit de quantification
    (le son reste net même à bas volume).

### Réglages de départ (déjà dans le code)

`G`, `T` et `V` sont des **positions de potentiomètre, de 0.0 à 1.0**, comme sur le
circuit : G = drive (R5+R6), T = tone (R8+R9), V = volume (R10+R11). Les valeurs par
défaut correspondent aux potentiomètres à mi-course, comme sur le schéma.

| Paramètre | Valeur par défaut | Plage autorisée |
|---|---|---|
| drive (G) | 0.5 (gain ×276) | 0.0 – 1.0 (×51 à ×501) |
| clip (C) | 0.85 (rails presque pleins) | 0.50 – 0.95 |
| tone (T) | 0.5 (≈1,45 kHz) | 0.0 – 1.0 |
| volume (V) | 0.5 | 0.0 – 1.0 |
| effet (E) | 1 (ON) | 0 / 1 |
| `INPUT_GAIN` | 2.0 (micro simple bobinage) | constante — 1.0 = fidèle au circuit, 3–4 si signal faible |
| `OUTPUT_LEVEL` | 0.45 (amplitude DAC max à V=1) | constante — ne pas descendre sous ~0.10 (inaudible) |
| `DEBUG_METER` | 1 (vu-mètre série ON) | constante — mettre 0 pour jouer sans micro-coupures |

> **Vérifié par simulation** (vrai code compilé hors carte, corde de La pincée
> injectée dans l'ADC) : les crêtes de sortie plafonnent à ±24-25 pas de DAC pendant
> que l'énergie (RMS) double quand on monte le niveau d'entrée ou le drive — c'est
> l'aplatissement des crêtes attendu d'un ampli op poussé à ses rails. Sans guitare :
> DAC constant à 128, silence absolu.

### Vu-mètre de diagnostic (port série, 115200 baud)

Avec `DEBUG_METER 1`, l'esclave affiche chaque seconde l'état de toute la chaîne :

```
[Metre] entree: 118 pas ADC | enveloppe: 0.142 | gate: OUVERT (0.97) | sortie DAC: +/-26 pas | G=0.50 V=0.500 E=1
[Metre] entree crete: 3 pas ADC — PAS DE SIGNAL GUITARE (verifiez jack, condensateur C1 et pont diviseur)
```

Lecture rapide : `entree` < 6 pas = problème de câblage d'entrée ; `gate: ferme` en
jouant = seuils de gate trop hauts ; `sortie DAC: +/-0` = volume à zéro ou effet coupé.

---

## Protocole de contrôle

### Téléphone → Maître (BLE)

Le maître s'annonce sous le nom **`SOAJ-Pedale`** avec un service type *Nordic UART* :

- Service : `6e400001-b5a3-f393-e0a9-e50e24dcca9e`
- Écriture (téléphone → pédale) : `6e400002-…`
- Notification (pédale → téléphone, état courant) : `6e400003-…`

Compatible avec **nRF Connect**, **Serial Bluetooth Terminal** (mode BLE) ou votre propre
appli iOS/Android/Harmony.

Commandes texte (UTF-8), une ou plusieurs à la fois, séparées par `;`.
Chaque valeur est une position de potentiomètre entre 0.0 et 1.0 :

```
G:0.5        → drive (pot R5+R6 : gain ×51 à ×501)
C:0.85       → hauteur des rails d'écrêtage (0.5 à 0.95)
T:0.5        → tone (pot R8+R9 : 0 = sombre, 1 = brillant)
V:0.5        → volume (pot R10+R11)
E:1  /  E:0  → effet ON / bypass
G:0.8;T:0.4;V:0.6;E:1    → tout en une commande
```

Toute valeur hors plage est automatiquement ramenée dans les bornes (côté maître **et**
côté esclave). Après chaque commande, le maître notifie l'état complet au téléphone.

### Maître → Esclave (ESP-NOW)

Paquet binaire de 21 octets (`magic "SOAJ"` + 4 floats + 1 octet ON/OFF), diffusé en
broadcast sur le **canal WiFi 1**, renvoyé toutes les 300 ms (un esclave allumé après le
maître récupère donc les réglages en moins d'une seconde). Les paquets sans le bon
`magic` sont ignorés. Plusieurs esclaves peuvent écouter le même broadcast.

---

## Pas de son du tout ? Checklist dans l'ordre

1. **Ouvrez le moniteur série de l'esclave** (115200 baud) : le vu-mètre dit tout.
2. `entree crete: < 6 pas — PAS DE SIGNAL GUITARE` → le problème est AVANT l'ESP32 :
   jack mal branché, C1 absent/coupé, pont diviseur R1/R2 absent (le message
   « offset DC anormal » au démarrage le confirme), volume de la guitare à zéro.
3. `entree` correct mais `gate: ferme` en jouant → montez le volume de la guitare ou
   baissez `GATE_LOW`/`GATE_HIGH`.
4. `gate: OUVERT` mais `sortie DAC: +/-0` → volume logiciel à zéro : envoyez `V:0.5`
   depuis le téléphone (ou vérifiez que le maître est allumé : sans maître, l'esclave
   garde ses valeurs par défaut, qui sont audibles).
5. `sortie DAC: +/-20` ou plus mais rien à l'ampli → le problème est APRÈS l'ESP32 :
   câble GPIO25→ampli, condensateur de liaison C3 (indispensable : sans lui certains
   amplis bloquent), masse commune, canal/volume de l'ampli, ampli sur l'entrée guitare
   (pas l'entrée AUX).
6. Testez l'ampli et le câble seuls : touchez la pointe du jack côté ampli avec le
   doigt → un « bzzz » doit sortir. Sinon, le souci est côté ampli/câble.

## Dépannage du bruit / grésillement

| Symptôme | Cause probable | Solution |
|---|---|---|
| Grésillement continu, même sans jouer | Pas de pont diviseur sur GPIO34 (le message série « offset DC anormal » apparaît) | Câbler C1 + R1 + R2 comme ci-dessus |
| Souffle amplifié | Gain trop haut pour un signal d'entrée trop faible | Baisser `G` ou `INPUT_GAIN`, ou monter les seuils du gate |
| Son « granuleux » à bas volume | Quantification 8 bits du DAC à trop faible amplitude | Diviseur résistif en sortie + augmenter `OUTPUT_LEVEL` |
| Aigus stridents | Tone trop haut | baissez `T` (0.2 à 0.4) |
| Craquements en changeant un réglage | (déjà traité : lissage ~60 ms) | Vérifier que vous utilisez bien ce code |
| Le gate coupe les notes tenues | Seuils trop hauts | Baisser `GATE_LOW` / `GATE_HIGH` dans `PedaleEsclave.ino` |
| Le gate laisse passer du souffle | Seuils trop bas | Monter `GATE_LOW` / `GATE_HIGH` |

## Règles à ne jamais enfreindre

- Jamais de tension négative ni > 3,3 V sur GPIO34 (toujours passer par C1 + pont diviseur).
- Ne pas déplacer l'entrée guitare sur un pin ADC2 : l'ADC2 ne fonctionne pas avec le WiFi.
- Ne jamais envoyer d'audio par BLE/WiFi : uniquement les paramètres.
