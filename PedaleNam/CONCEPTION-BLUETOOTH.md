 > **⚠️ Piste remplacée** : la conception retenue est désormais le
> « plug and play » par partage de connexion — voir
> **CONCEPTION-PLUG-AND-PLAY.md**. Le hotspot du téléphone apporte les mêmes
> bénéfices que le BLE (le téléphone garde Internet, plus de portail captif)
> tout en donnant Internet à l'ESP32, qui télécharge et analyse lui-même les
> captures via l'API officielle TONE3000. Ce document est conservé comme
> trace de réflexion.

# SOAJ NAM — Conception globale de la version BLUETOOTH

> Schéma de pensée pour faire évoluer `PedaleNam` : le téléphone pilote la
> pédale en **Bluetooth (BLE)** au lieu du WiFi. Document de conception —
> le code viendra ensuite.

## Pourquoi le Bluetooth change tout

Avec le WiFi, le téléphone doit *quitter* son réseau habituel pour rejoindre
celui de la pédale : il perd Internet, râle (« pas d'accès Internet »), tente
le portail captif, route parfois tout vers la 4G… C'est la source de tous les
ennuis rencontrés.

En Bluetooth, le téléphone **garde son Internet** (4G ou WiFi de la maison) :

- il peut naviguer sur **tone3000.com en direct** pendant qu'il est relié à
  la pédale — choisir une capture, la télécharger, l'analyser, l'envoyer,
  l'essayer, en choisir une autre… sans jamais changer de réseau ;
- plus de portail captif, plus de DNS, plus de « rester connecté ? » ;
- l'ESP32 n'a **toujours pas besoin d'Internet** : il ne reçoit que le petit
  profil (~2,5 Ko), comme aujourd'hui.

## Vue d'ensemble

```
        Internet (4G/WiFi maison — le téléphone le GARDE)
            │
   ┌────────┴─────────────────────────────────────────────┐
   │ TÉLÉPHONE                                            │
   │  1. tone3000.com : choisir/télécharger la capture    │
   │     .nam de la pédale voulue                         │
   │  2. Page d'import (moteur NAM en JavaScript,         │
   │     DÉJÀ ÉCRIT) : exécute le réseau, en extrait      │
   │     le profil (courbe 257 pts + filtres, ~2,5 Ko)    │
   │  3. Web Bluetooth : connexion à « SOAJ-NAM »         │
   └────────┬─────────────────────────────────────────────┘
            │  BLE (GATT) : profils, réglages, sélection
            │  — JAMAIS l'audio, JAMAIS le .nam complet
   ┌────────┴─────────────────────────────────────────────┐
   │ ESP32 (1 seul, AUCUN Internet)                       │
   │  - service BLE : reçoit profils + réglages           │
   │  - flash NVS : PLUSIEURS profils mémorisés (slots)   │
   │  - cœur 1 : audio temps réel 20 kHz (chaîne actuelle │
   │    inchangée : gate, courbe interpolée, voicing,     │
   │    tone, volume, DAC sigma-delta)                    │
   └────────┬─────────────────────────────────────────────┘
            │
   Guitare ─┴─> GPIO34 (ADC) … GPIO25 (DAC) ─> Ampli
   (câblage inchangé : pont diviseur 1,65 V + filtre de sortie,
    voir README — le schéma de câblage joint n'est pas parvenu,
    on suppose celui du dépôt)
```

## Les trois usages, vus du musicien

1. **Choisir une pédale** (à la maison, une fois) : tone3000.com → télécharger
   le `.nam` → page d'import → « Analyser » (quelques secondes à ~1 min) →
   « Envoyer vers le slot 3 » → la pédale grave le profil en flash.
2. **Sélectionner une pédale** (n'importe quand) : la page liste les slots
   mémorisés (« 1: TS9, 2: RAT, 3: Big Muff… ») → un appui → la pédale bascule
   en fondu (~35 ms, mécanisme déjà en place). Marche même sans Internet.
3. **Jouer** : jauges Drive / Intensité / Tone / Volume + footswitch en direct
   par BLE (quelques dizaines de ms de latence de commande — l'audio, lui,
   reste local et instantané).

## Côté ESP32 — ce qui change, ce qui ne change pas

| Bloc | Sort |
|---|---|
| Chaîne audio 20 kHz (cœur 1) | **inchangée** (gate, courbe, voicing, sigma-delta) |
| Import de profil + fondu anti-clic | **inchangé** (même struct `ProfilNam`) |
| NVS | étendu : **8 slots** (~9 Ko en tout) + n° du slot actif |
| WiFi + portail captif + page embarquée | **remplacés** par un service BLE (cœur 0) |
| Bibliothèques | BLE du core ESP32 (`BLEDevice.h`) — toujours **aucune dépendance externe** |

### Service GATT (esquisse)

Un service « SOAJ-NAM » avec 4 caractéristiques :

| Caractéristique | Sens | Rôle |
|---|---|---|
| `REGLAGES` | écriture sans réponse + notification | 5 octets : drive, intensité, tone, volume (0-255) + effet ON/OFF — les jauges en direct |
| `PROFIL_DONNEES` | écriture | le profil en morceaux : `[offset 2 octets][données ≤ 500 octets]` (MTU BLE ≈ 512) |
| `PROFIL_CTRL` | écriture | `DEBUT(taille, slot)` / `FIN(CRC32)` / `CHOISIR(slot)` / `EFFACER(slot)` |
| `ETAT` | lecture + notification | slot actif, nom du profil, liste des slots occupés |

Un profil de 2,5 Ko = ~6 paquets BLE : transfert en moins d'une seconde.
Le CRC32 en fin de transfert protège d'un envoi tronqué : sans CRC valide,
rien n'est gravé ni joué.

### Points de vigilance techniques

- **Radio + audio sur la même puce** : comme le WiFi aujourd'hui — la médiane
  de 3 lectures ADC reste (elle a été conçue pour ça). Le BLE est même plus
  doux que les balises AP du WiFi.
- **RAM** : la pile BLE Bluedroid prend ~80 Ko ; on récupère ce que le WiFi
  libère. À surveiller à la compilation, pas bloquant.
- **BLE seulement, pas de Bluetooth « classique » (SPP)** : l'iPhone ne parle
  pas SPP, et le BLE suffit largement pour 2,5 Ko.

## Côté téléphone — où vit la page ?

Le moteur NAM JavaScript existant (WaveNet + LSTM, validé sur les modèles
officiels) est **réutilisé tel quel**. Seul le transport change : Web
Bluetooth au lieu de `fetch()`. Trois options :

| Option | Android | iPhone | Effort | Verdict |
|---|---|---|---|---|
| **A. Page hébergée** (GitHub Pages, gratuite, HTTPS) avec Web Bluetooth | Chrome ✔ natif | Safari ✘ → navigateur gratuit **Bluefy** ✔ | faible (la page existe déjà à 90 %) | **recommandé** |
| B. Application native | ✔ | ✔ | élevé (2 applis à développer/installer) | plus tard, si besoin |
| C. Garder le WiFi corrigé (portail durci, `http://soaj-nam.local`) | ✔ | ✔ | déjà fait | **conservé en secours** dans le dépôt |

Note sur l'option A : la page est *hébergée* sur Internet, mais c'est le
**téléphone** qui y accède (il a Internet) — l'ESP32, lui, n'y touche jamais.
Une fois chargée, la page fonctionne même hors ligne pour la sélection de
slots. Le Web Bluetooth exige une page HTTPS : GitHub Pages convient, et le
dépôt peut l'héberger directement (`docs/` ou branche `gh-pages`).

Contrainte assumée : sur iPhone, Safari ne gère pas le Web Bluetooth — il
faudra l'appli gratuite **Bluefy** (un navigateur avec Web Bluetooth). Sur
Android et PC, Chrome suffit.

## Jalons de réalisation

1. **Firmware BLE** : service GATT + slots NVS + sélection en fondu
   (remplace WiFi/portail dans `PedaleNam`, ou nouveau dossier `PedaleNamBle/`
   pour garder les deux versions).
2. **Page hébergée** : reprendre la page actuelle, remplacer la couche
   `fetch()` par Web Bluetooth, ajouter la gestion des slots.
3. **Banc d'essai** : les tests hors carte existants (moteur JS sous node,
   croquis compilé sur PC) s'étendent au protocole de transfert (chunks + CRC).
4. **Test réel** : latence des jauges, solidité du transfert, bruit radio.

## Questions ouvertes (à trancher avant de coder)

- **Téléphone cible : Android, iPhone, ou les deux ?** (décide si on
  documente Bluefy ou si Chrome suffit)
- Remplacer le WiFi dans `PedaleNam` ou créer `PedaleNamBle/` à côté ?
  (proposition : un dossier à part, le WiFi corrigé reste une solution
  de secours qui marche sans rien installer)
- Nombre de slots (proposition : 8) et faut-il des noms modifiables ?
