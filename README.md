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
| `PedaleNam/` | **1 seul ESP32** | **Projet à part** : pédale autonome avec le catalogue TONE3000 intégré à sa page (choix, téléchargement et mémorisation de 8 pédales) — voir la section dédiée en bas |

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

## PedaleNam — projet à part : 1 ESP32 avec le catalogue TONE3000 intégré

`PedaleNam/` est une pédale **autonome et indépendante** du duo maître/esclave :
un **seul ESP32** (mêmes câblages d'entrée/sortie que les autres pédales), et
le **catalogue tone3000.com directement dans la page de la pédale**.

```
                Internet (4G du téléphone)
                     ▲
                     │ partage de connexion (hotspot)
Téléphone ───────────┤
  navigateur ────────┼──> page web DANS l'ESP32 (http://soaj-nam.local)
    « CHOISIR SUR    │
      TONE3000 »     │
                     ▼
                 ESP32 : relaie l'API TONE3000, télécharge la capture .nam,
                 la page l'analyse sur le téléphone, le profil est gravé
                 dans un des 8 SLOTS mémorisés
                     │
     Guitare ──> GPIO34 (ADC) … GPIO25 (DAC) ──> Ampli   (audio 100 % local)
```

### Pourquoi le partage de connexion

Le téléphone **garde son Internet** (plus de « ce réseau n'a pas d'accès
Internet », plus de portail captif) et en fait profiter la pédale, qui se sert
elle-même dans le catalogue via l'**API officielle TONE3000** (OAuth PKCE —
leurs flux prévoient explicitement le matériel embarqué). L'ESP32 n'est
**jamais** connecté à autre chose que le téléphone.

### Installation (une seule fois)

1. Créer une **clé API** sur son compte tone3000.com (Réglages → API Keys →
   clé publiable `t3k_pub_…`) et y autoriser l'adresse de redirection que la
   page affiche (section Configuration).
2. Activer le **partage de connexion** du téléphone, ouvrir la page de la
   pédale, renseigner le nom/mot de passe du partage + coller la clé API.
   (La toute première fois, la page s'ouvre par le WiFi propre de la pédale
   `SOAJ-NAM` / mdp `soaj1234` — couper les données mobiles pour cet accès-là,
   ou taper `http://192.168.4.1` avec le préfixe `http://`.)

### Utilisation (plug and play)

1. Partage de connexion ON → pédale ON (elle rejoint toute seule) →
   `http://soaj-nam.local`.
2. **CHOISIR SUR TONE3000** : le vrai catalogue s'ouvre (le téléphone a
   Internet), on navigue, on choisit une pédale, retour automatique à la page.
3. La pédale **télécharge la capture elle-même**, le téléphone **l'analyse**
   (moteur NAM en JavaScript — WaveNet et LSTM, conforme à
   NeuralAmpModelerCore, de quelques secondes à ~1 min), et le profil
   (~2,5 Ko) est **gravé dans le slot choisi**. Prêt à jouer.
4. Les **8 slots** se rappellent d'un appui, **sans Internet ni hotspot**
   (fondu anti-clic, mémorisés pour toujours).

L'import manuel d'un fichier `.nam` déjà téléchargé reste disponible en bas
de page (secours — marche sans compte TONE3000).

### Réglages sur la page

| Jauge | Rôle |
|---|---|
| Drive | pousse la courbe de la capture (course logarithmique) |
| Intensité | 0 = clean pur (signal brut) → 1 = caractère complet de la capture |
| Tone | passe-bas final, 0 = sombre → 1 = brillant |
| Volume | niveau de sortie |

API locale : `GET /api/get`, `GET /api/set?g=&d=&t=&v=&e=`,
`GET /api/slot?n=`, `POST /api/profil?slot=n` (JSON du profil),
`GET /api/hotspot?ssid=&mdp=`, `GET /api/t3kcle?cle=`,
et le relais TONE3000 : `POST /t3k/token`, `GET /t3k/modeles?tone_id=`,
`POST /t3k/telecharger?url=`, `GET /t3k/fichier`.

### La page ne s'ouvre pas en mode WiFi SOAJ-NAM ? (première configuration)

C'est le téléphone qui bloque, pas la pédale — **aucune connexion Internet
n'est nécessaire** pour ce mode :

1. **Couper les données mobiles (4G/5G)** le temps du réglage.
2. À l'avertissement « ce réseau n'a pas d'accès Internet », répondre
   **« Rester connecté »** (Android) ou **« Utiliser sans Internet »** (iOS).
3. Taper **`http://192.168.4.1`** — **avec le préfixe `http://`** (tapé nu,
   Chrome force le HTTPS). `http://soaj-nam.local` marche aussi.

(Une fois le partage de connexion configuré, ce problème disparaît : le
téléphone garde Internet et la page se sert normalement.)

### Limites honnêtes

- Le profil garde le **caractère statique** de la capture (courbe de
  saturation, asymétrie, équilibre spectral) mais pas sa **dynamique fine** :
  très bien pour les pédales de saturation/disto/fuzz, moins fidèle pour un
  ampli complet au grain très dynamique — la capture NAM d'origine sur un
  vrai moteur NAM reste la référence.
- Qualité bornée par les convertisseurs de l'ESP32 (ADC 12 bits, DAC 8 bits
  + sigma-delta).
- Le HTTPS vérifie le certificat de tone3000.com avec la racine embarquée
  (ISRG/Let's Encrypt, valable 2035) ; en cas de changement d'autorité chez
  eux, passer `T3K_TLS_STRICT` à 0 le temps d'une mise à jour.
- L'écriture flash d'un profil peut faire un bref craquement (une fois par
  import).

> Vérifié par simulation : croquis compilé hors carte (audio, slots NVS,
> analyse du JSON, bascule en fondu), moteur JavaScript validé sur les modèles
> d'exemple officiels de NeuralAmpModelerCore (comptage des poids exact,
> WaveNet et LSTM), SHA-256 du flux OAuth vérifié contre l'implémentation de
> référence de node. Le flux réseau réel (OAuth, téléchargement) reste à
> valider sur le matériel avec une vraie clé API.
