# SOAJ — Pédale d'effet guitare (saturation) sur ESP32

Pédale de saturation numérique contrôlée sans fil :

```
Téléphone ──(WiFi + page web)──> ESP32 MAÎTRE ──(ESP-NOW)──> ESP32 ESCLAVE ──> Ampli
             http://192.168.4.1                                    ▲
                                                          Guitare (jack, ADC)
```

- Le maître crée son **propre réseau WiFi** (`SOAJ-Pedale`) et sert une **page web avec
  des jauges** : aucun routeur, aucune appli à installer.
- **Seuls les paramètres** (gain, clip, tone, volume, ON/OFF, diodes) transitent par
  radio : **jamais l'audio**.
- L'audio est traité **localement** dans l'esclave, échantillon par échantillon (20 kHz), en flottant.
- Sans guitare en entrée : **silence total** (noise gate progressif + DAC au repos à 128).

> Note : le lien maître → esclave utilise **ESP-NOW** sur le même canal WiFi que le
> point d'accès. C'est aussi pour cela que l'entrée guitare est sur **GPIO34 (ADC1)** :
> l'ADC2 est inutilisable quand le WiFi est actif.

## Contenu du dépôt

| Dossier | Carte | Rôle |
|---|---|---|
| `PedaleMaitre/` | ESP32 n°1 | Point d'accès WiFi + page web de réglage, diffuse en ESP-NOW |
| `PedaleEsclave/` | ESP32 n°N | Reçoit les paramètres, traite l'audio guitare, sortie DAC |
| `PedaleNam/` | **1 seul ESP32** | **Projet à part** : pédale autonome qui charge des captures TONE3000 (.nam) importées depuis le téléphone — voir la section dédiée en bas |

Compiler avec l'Arduino IDE, carte **« ESP32 Dev Module »** (cartes ELEGOO ESP32 CP2102).
Aucune bibliothèque externe : tout est inclus dans le core ESP32 (WiFi, WebServer,
DNSServer, ESP-NOW).

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

1. Lecture ADC 12 bits sur GPIO34 à 20 kHz — **médiane de 3 lectures** : l'ADC de
   l'ESP32 crache des pics isolés quand le WiFi émet ; amplifiés ×500 ils faisaient
   un gros grésillement. La médiane les élimine sans adoucir le son.
2. Suivi et suppression de l'**offset DC** (moyenne glissante lente).
3. **Passe-haut ~40 Hz** : retire le résidu continu et les basses parasites
   (garde le mi grave de la guitare, 82 Hz).
4. **Passe-bas ~4 kHz avant saturation** : coupe le souffle ADC avant le gros gain
   (les harmoniques brillantes sont recréées par la distorsion elle-même).
5. **Noise gate progressif** : suiveur d'enveloppe, ouverture/fermeture en fondu —
   pas de coupure brutale, pas de souffle sans guitare. Gate fermé = sortie
   strictement à 128 (silence absolu). Placé AVANT le gros gain, et ses **seuils
   montent avec le drive** (×1 à drive 0, ×4 à drive max), comme le gate d'un
   ampli high-gain.
6. **Étage de saturation = émulation du circuit ADTL082** (schéma LTspice) :
   - potentiomètre **DRIVE** = paramètre `G`, course **exponentielle** comme un
     pot audio : **×2 (G=0, quasi clean) → ×30 (G=0.5, crunch) → ×500 (G=1,
     heavy metal)** — toute la course du bouton est utile ;
   - réseau C5 220n / R7 1k : le plein gain ne s'applique qu'au-dessus de ~723 Hz
     (graves propres, médiums/aigus qui saturent — le caractère du circuit) ;
   - C4 100p en contre-réaction : passe-bas qui descend à 3,2 kHz à drive max
     (adoucit le grésillement aigu).
7. **Écrêtage à deux étages** (`tanh`, sans aliasing numérique) :
   - d'abord les **rails ±12 V** de l'ampli op ;
   - puis les **diodes d'écrêtage** (paramètre `D`) : le signal amplifié jusqu'à
     ±12 V vient s'écraser sur le seuil des diodes — c'est là que se fait le gros
     de la saturation (~20× plus agressif que les rails seuls en silicium).

   | `D` | Diodes | Seuil | Caractère |
   |---|---|---|---|
   | 0 | aucune | rails ±12 V | boost léger, presque clean |
   | **1** | silicium (2×1N4148) | ±0,6 V | **saturation forte (défaut)** |
   | 2 | LED | ±1,7 V | crunch plus ouvert |
   | 3 | germanium | ±0,3 V | fuzz très compressé |

   Le paramètre `C` règle la **dureté du genou** d'écrêtage : 0.95 = genou doux et
   rond (tanh), 0.5 = genou dur presque carré → nettement plus d'harmoniques et
   d'agressivité. Le volume perçu ne change pas quand on change `D` (renormalisation).
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
| drive (G) | 0.5 (gain ×30, crunch) | 0.0 – 1.0 (×2 clean à ×500 heavy metal) |
| clip (C) | 0.85 (plutôt doux) | 0.50 (dur/agressif) – 0.95 (doux/rond) |
| tone (T) | 0.5 (≈1,45 kHz) | 0.0 – 1.0 |
| volume (V) | 0.5 | 0.0 – 1.0 |
| effet (E) | 1 (ON) | 0 / 1 |
| diodes (D) | 1 (silicium) | 0 = sans, 1 = silicium, 2 = LED, 3 = germanium |
| `INPUT_GAIN` | 2.0 (micro simple bobinage) | constante — 1.0 = fidèle au circuit, 3–4 si signal faible |
| `OUTPUT_LEVEL` | 0.45 (amplitude DAC max à V=1) | constante — ne pas descendre sous ~0.10 (inaudible) |
| `DEBUG_METER` | **0 (OFF pour jouer)** | constante — mettre 1 SEULEMENT pour diagnostiquer : chaque affichage coupe l'audio ~10 ms → un craquement par seconde |

> **Vérifié par simulation** (vrai code compilé hors carte, corde de La pincée +
> parasites ADC type WiFi injectés) : sans guitare, DAC constant à 128 **même avec
> les parasites** (médiane + gate). Balayage du drive : facteur de crête 2,7 (G=0,
> quasi sinus) → 1,16 (G=1, quasi carré = heavy metal) pendant que l'énergie RMS
> est multipliée par 7. Clip : 1,57 (doux) → 1,19 (dur) à réglages égaux.

### Vu-mètre de diagnostic (port série, 115200 baud)

Le vu-mètre est **désactivé par défaut** (chaque affichage coupe l'audio ~10 ms →
un craquement par seconde). Pour diagnostiquer : mettre `DEBUG_METER 1` en tête de
`PedaleEsclave.ino`, re-téléverser, et l'esclave affiche chaque seconde l'état de
toute la chaîne (remettre 0 pour jouer) :

```
[Metre] entree: 118 pas ADC | enveloppe: 0.142 | gate: OUVERT (0.97) | sortie DAC: +/-26 pas | G=0.50 V=0.500 E=1
[Metre] entree crete: 3 pas ADC — PAS DE SIGNAL GUITARE (verifiez jack, condensateur C1 et pont diviseur)
```

Lecture rapide : `entree` < 6 pas = problème de câblage d'entrée ; `gate: ferme` en
jouant = seuils de gate trop hauts ; `sortie DAC: +/-0` = volume à zéro ou effet coupé.

---

## Profils de pédales (captures TONE3000 → ESP32)

La pédale SATU embarque des **profils de saturation** sélectionnables depuis le
téléphone : le profil 0 est le circuit SOAJ d'origine, les suivants remplacent
l'étage de saturation par l'approximation d'une pédale réelle :

| `p` | Profil | Caractère |
|---|---|---|
| 0 | **SOAJ** (défaut) | le circuit d'origine décrit ci-dessus |
| 1 | TS9 | Ibanez Tube Screamer — overdrive doux, médiums bossus |
| 2 | RAT | ProCo RAT — distorsion mordante, écrêtage dur |
| 3 | MUFF | Big Muff Pi — fuzz compressé, médiums creusés |

- Sélection : boutons « Saturation — profil de pédale » sur la page web,
  `p=` dans l'API HTTP, ou `P:1` au port série du maître.
- Dans un profil : **Drive** parcourt la plage de gain de la pédale émulée,
  **Dist** dose l'intensité (0 = clean pur, comme d'habitude). Gate, octave,
  EQ 3 bandes, tone et volume restent actifs.
- Le changement de profil se fait **en fondu (~35 ms)** : aucun clic.

### Convertir votre propre capture tone3000.com

Un modèle NAM complet (réseau de neurones 48 kHz) ne peut pas tourner en temps
réel sur un ESP32 : le script `outils/profil_pedale.py` en extrait une
**approximation légère** (courbe d'écrêtage mesurée en 257 points + filtres de
voicing) exécutable à 20 kHz. En 3 étapes sur PC :

```
python3 outils/profil_pedale.py --test-wav test_sec.wav
# passer test_sec.wav dans le plugin NAM (gratuit) avec la capture .nam
# téléchargée depuis tone3000.com, exporter test_traite.wav SANS normaliser
python3 outils/profil_pedale.py --analyser test_sec.wav test_traite.wav --nom MAPEDALE
```

Le bloc C imprimé se colle dans `PedaleEsclaveSatu/profils_pedales.h`
(incrémenter `NB_PROFILS`, ajouter l'entrée dans `PROFILS[]`, ajouter un
bouton dans la page du maître), puis re-téléverser esclave et maître.
L'approximation garde le caractère statique de la capture (courbe d'écrêtage,
voicing) mais pas son comportement dynamique fin — la capture NAM d'origine
reste la référence.

> Rappel : le téléphone connecté à `SOAJ-Pedale` n'a pas Internet — la
> conversion se fait à l'avance sur PC, les profils sont compilés dans
> l'esclave. Rien d'autre que les paramètres ne transite par radio.

---

## Contrôle depuis le téléphone (WiFi + page web)

### Connexion — 3 étapes, aucune appli à installer

1. Alimentez le maître. Il crée le réseau WiFi **`SOAJ-Pedale`**
   (mot de passe : **`soaj1234`**, modifiable en tête de `PedaleMaitre.ino`).
2. Sur le téléphone : **Réglages → WiFi → SOAJ-Pedale**.
3. La page de contrôle **s'ouvre toute seule** (portail captif). Sinon, ouvrez un
   navigateur sur **`http://192.168.4.1`**.

> Le téléphone peut afficher « Pas de connexion Internet » : c'est normal, la pédale
> n'est pas un routeur. Restez connecté quand même (iOS : « Utiliser sans Internet »).

### La page de contrôle

- **4 jauges** : Drive (×2 clean → ×500 heavy metal), Clip (dureté), Tone (sombre →
  brillant), Volume — chaque geste est envoyé à la pédale en ~120 ms.
- **4 boutons diodes** : sans / silicium / LED / germanium.
- **Footswitch** ON / bypass.
- **Voyant de liaison** en haut à droite : vert = pédale jointe, rouge = hors ligne.
  La page relit l'état réel toutes les 3 s (plusieurs téléphones peuvent être
  connectés en même temps, ils restent synchronisés).

### API HTTP (pour votre propre appli ou des scripts)

```
GET http://192.168.4.1/api/get
    → {"g":0.50,"c":0.85,"t":0.50,"v":0.50,"e":1,"d":1}

GET http://192.168.4.1/api/set?g=0.8&t=0.4&v=0.6&e=1&d=1
    → chaque argument est optionnel ; répond avec l'état complet
```

| Clé | Rôle | Plage |
|---|---|---|
| `g` | drive (exponentiel : ×2 clean → ×500 heavy metal) | 0.0 – 1.0 |
| `c` | dureté de l'écrêtage (plus bas = plus dur/agressif) | 0.5 – 0.95 |
| `t` | tone (pot R8+R9 : 0 = sombre, 1 = brillant) | 0.0 – 1.0 |
| `v` | volume (pot R10+R11) | 0.0 – 1.0 |
| `e` | effet ON / bypass | 0 / 1 |
| `d` | diodes : 0 = sans, 1 = silicium, 2 = LED, 3 = germanium | 0 – 3 |

Toute valeur hors plage est automatiquement ramenée dans les bornes (côté maître **et**
côté esclave).

### Dépannage / secours : le port série du maître

Le moniteur série du maître (115200 baud) accepte les mêmes commandes qu'avant :

```
G:0.8;C:0.85;T:0.4;V:0.6;E:1;D:1
```

(taper la ligne puis Entrée — pratique pour tester sans téléphone).

### Maître → Esclave (ESP-NOW)

Paquet binaire de 22 octets (`magic "SOAJ"` + 4 floats + 1 octet ON/OFF + 1 octet diodes),
diffusé en
broadcast sur le **canal WiFi 1**, renvoyé toutes les 300 ms (un esclave allumé après le
maître récupère donc les réglages en moins d'une seconde). Les paquets sans le bon
`magic` sont ignorés. Plusieurs esclaves peuvent écouter le même broadcast.

---

## Pas de son du tout ? Checklist dans l'ordre

1. Mettez `DEBUG_METER 1` dans `PedaleEsclave.ino`, re-téléversez, puis **ouvrez le
   moniteur série de l'esclave** (115200 baud) : le vu-mètre dit tout.
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
- Ne jamais envoyer d'audio par WiFi : uniquement les paramètres.

---

## PedaleNam — projet à part : 1 ESP32 + captures TONE3000 importées du téléphone

`PedaleNam/` est une pédale **autonome et indépendante** du duo maître/esclave :
un **seul ESP32** fait le point d'accès WiFi, la page web ET l'audio
(mêmes câblages d'entrée/sortie que les autres pédales).

```
Téléphone ──(WiFi + page web)──> ESP32 unique ──> Ampli
 │  http://192.168.4.1                 ▲
 │                             Guitare (jack, ADC GPIO34)
 └── fichier .nam téléchargé sur tone3000.com
```

### Le principe

Un fichier `.nam` (Neural Amp Modeler, le format de tone3000.com) est un
réseau de neurones : bien trop lourd pour tourner en temps réel sur un ESP32.
L'astuce : **c'est le téléphone qui l'exécute**, pas l'ESP32.

1. Sur le téléphone (avec Internet), télécharger une capture `.nam` de pédale
   sur [tone3000.com](https://www.tone3000.com).
2. Se connecter au WiFi **`SOAJ-NAM`** (mdp `soaj1234`), la page s'ouvre
   (portail captif).
3. **Importer le fichier .nam** : le JavaScript de la page exécute le réseau
   (WaveNet et LSTM gérés, disposition des poids conforme à
   NeuralAmpModelerCore), mesure sa courbe d'écrêtage (257 points, crêtes
   positives et négatives séparées = asymétrie préservée) et sa réponse en
   fréquence (sinus par paliers + Goertzel), puis n'envoie à l'ESP32 que ce
   **profil** de ~2,5 Ko. L'analyse prend de quelques secondes à ~1 minute
   selon le téléphone et la taille du modèle — une seule fois par capture.
4. L'ESP32 grave le profil en flash (il **survit à l'extinction**) et le joue
   en temps réel à 20 kHz : drive log → passe-haut → passe-bas → courbe
   d'écrêtage interpolée → médiums → passe-bas de voicing → tone → volume,
   avec le même gate, les mêmes anti-parasites et le même DAC sigma-delta
   que les autres pédales du dépôt. Changement de profil **en fondu**, sans clic.

### Réglages sur la page

| Jauge | Rôle |
|---|---|
| Drive | pousse la courbe de la capture (course logarithmique) |
| Intensité | 0 = clean pur (signal brut) → 1 = caractère complet de la capture |
| Tone | passe-bas final, 0 = sombre → 1 = brillant |
| Volume | niveau de sortie |

API : `GET /api/get`, `GET /api/set?g=&d=&t=&v=&e=`,
`POST /api/profil` (JSON du profil — c'est ce qu'envoie la page).

### La page ne s'ouvre pas ? (« ce réseau n'a pas d'accès Internet »)

C'est le téléphone qui bloque, pas la pédale — **aucune connexion Internet
n'est nécessaire**, il faut juste l'empêcher de contourner le WiFi local :

1. **Couper les données mobiles (4G/5G)** le temps du réglage : sinon le
   téléphone envoie les requêtes par le réseau mobile et `192.168.4.1`
   n'aboutit jamais.
2. À l'avertissement « ce réseau n'a pas d'accès Internet », répondre
   **« Rester connecté »** (Android) ou **« Utiliser sans Internet »** (iOS).
3. Si la fenêtre ne s'ouvre pas toute seule, taper **`http://192.168.4.1`**
   dans le navigateur — **avec le préfixe `http://`** : tapé nu, Chrome force
   le HTTPS et affiche « pas de connexion ». `http://soaj-nam.local` marche
   aussi (mDNS, pratique sur iPhone et PC).
4. Pour l'import du `.nam`, préférez un vrai navigateur (Safari/Chrome) à la
   petite fenêtre de connexion du portail : ouvrez `http://192.168.4.1`
   à la main.

Le croquis répond de lui-même aux sondes de détection de portail
d'Android, iOS, Windows et Firefox : sur la plupart des téléphones la
fenêtre s'ouvre seule une fois les données mobiles coupées.

### Limites honnêtes

Le profil garde le **caractère statique** de la capture (courbe de saturation,
asymétrie, équilibre spectral) mais pas sa **dynamique fine** (le réseau
complet réagit à l'enveloppe du jeu). Ça marche très bien pour les pédales de
saturation/disto/fuzz ; pour un ampli complet capturé avec son grain dynamique,
la capture NAM d'origine sur un vrai moteur NAM (PC, Daisy Seed…) reste la
référence. Sans surprise, la qualité est aussi bornée par les convertisseurs
de l'ESP32 (ADC 12 bits, DAC 8 bits + sigma-delta).

> Vérifié par simulation : croquis compilé hors carte et alimenté avec le
> profil réellement produit par le moteur JavaScript sur les modèles d'exemple
> officiels de NeuralAmpModelerCore (comptage des poids exact, WaveNet et
> LSTM) — silence strict au repos, échange de profil sans saut, aucun NaN.
