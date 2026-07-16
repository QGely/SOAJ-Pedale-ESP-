> **✅ Mis en œuvre** dans `PedaleNam.ino` avec un ajustement : l'analyse de la
> capture reste faite par le **téléphone** (moteur NAM JavaScript déjà validé,
> quelques secondes au lieu de ~1-3 min sur ESP32) — l'ESP32 s'occupe de
> l'OAuth, du téléchargement, des 8 slots et de l'audio. Voir le README.

# SOAJ NAM — Conception « PLUG AND PLAY » : l'ESP32 télécharge tout seul depuis TONE3000

> Schéma de pensée global. L'idée : **le partage de connexion du téléphone**
> donne Internet à l'ESP32, qui se sert alors **lui-même** dans le catalogue
> TONE3000 grâce à leur **API officielle**, télécharge la capture `.nam`,
> **l'analyse lui-même** et la joue. Le téléphone ne sert plus qu'à choisir.
> Remplace la piste Bluetooth (CONCEPTION-BLUETOOTH.md) : le hotspot apporte
> les mêmes bénéfices SANS développer de transport BLE ni page hébergée.

## Le déclic : le partage de connexion règle tout

Jusqu'ici le téléphone devait rejoindre le WiFi de la pédale → il perdait
Internet → portail captif, « pas de connexion », etc. En inversant les rôles :

- le téléphone **active son partage de connexion** (hotspot) — 1 geste ;
- la pédale **rejoint le hotspot** (identifiants mémorisés, automatique) ;
- le téléphone **garde son Internet** (4G), la pédale **en profite aussi** ;
- le navigateur du téléphone atteint la page de la pédale **sans râler**
  (pour lui, c'est un simple appareil du réseau local).

Plus de portail captif, plus de données mobiles à couper, ET la pédale a
Internet — sans jamais dépendre d'un routeur ou d'une box.

## Vue d'ensemble

```
                    Internet (4G du téléphone)
                          ▲
                          │ partage de connexion (hotspot)
   ┌──────────────────────┴──────────────────────────────────┐
   │ TÉLÉPHONE = point d'accès + télécommande                │
   │   navigateur → http://soaj-nam.local (page de la pédale)│
   │   - champ de recherche TONE3000 (« big muff », etc.)    │
   │   - vignettes/résultats, bouton « CHARGER »             │
   │   - jauges Drive/Intensité/Tone/Volume + slots          │
   └──────────────────────┬──────────────────────────────────┘
                          │ WiFi local (hotspot)
   ┌──────────────────────┴──────────────────────────────────┐
   │ ESP32 (station WiFi sur le hotspot)                     │
   │  ① API TONE3000 (officielle, HTTPS + OAuth PKCE) :      │
   │     recherche → GET /api/v1/tones/search                │
   │     modèles   → GET /api/v1/models?tone_id=…            │
   │     fichier   → GET model_url (jeton Bearer)            │
   │  ② stocke le .nam téléchargé en flash (LittleFS)        │
   │  ③ L'ANALYSE LUI-MÊME : moteur WaveNet/LSTM en C++,     │
   │     PAS en temps réel (~1 à 3 min, cœur 0, pendant que  │
   │     l'audio continue de jouer le profil courant) :      │
   │     courbe d'écrêtage 257 pts + réponse en fréquence    │
   │  ④ grave le profil (~2,5 Ko) dans un SLOT NVS (×8)      │
   │  ⑤ cœur 1 : audio temps réel 20 kHz (chaîne inchangée)  │
   └──────────────────────┬──────────────────────────────────┘
                          │
      Guitare ────────────┴─> GPIO34 (ADC) … GPIO25 (DAC) ──> Ampli
      (câblage inchangé : pont diviseur 1,65 V + filtre de sortie)
```

Une fois un profil gravé, **le hotspot n'est plus nécessaire** : la pédale
joue seule, et le changement de slot marche depuis la page (ou plus tard un
footswitch physique).

## L'expérience utilisateur visée

### Une seule fois (installation)

1. Créer une **clé API** sur son compte tone3000.com (Réglages → API Keys →
   `t3k_pub_…`) et la renseigner dans la pédale (page de configuration).
2. Brancher la pédale : elle ouvre son propre WiFi `SOAJ-NAM` (comme
   aujourd'hui) → page → entrer le **nom et mot de passe du partage de
   connexion** du téléphone (mémorisés en flash).
3. « **Connecter mon compte TONE3000** » : la page ouvre la fenêtre
   d'autorisation TONE3000 (le téléphone a Internet), TONE3000 renvoie le
   code à la pédale sur le réseau local (flux OAuth « LAN-relay » prévu par
   leur API pour le matériel embarqué, sécurisé par PKCE). Jetons en flash,
   rafraîchis automatiquement.

### Ensuite, à chaque fois (plug and play)

1. Activer le partage de connexion → allumer la pédale (elle rejoint seule).
2. Ouvrir `http://soaj-nam.local` → **chercher** (« tube screamer ») →
   résultats → **CHARGER dans le slot 2**.
3. La pédale télécharge, analyse (~1-3 min, barre de progression, l'audio
   continue), grave, bascule en fondu. C'est prêt — et mémorisé pour toujours.

## Répartition du travail dans l'ESP32

| Tâche | Cœur | Quand |
|---|---|---|
| Audio 20 kHz (chaîne actuelle inchangée) | 1 | toujours, jamais interrompu |
| WiFi + page web + API TONE3000 | 0 | à la demande |
| Téléchargement .nam → LittleFS | 0 | quelques secondes |
| **Calibration** (moteur NAM C++ non temps réel) | 0, basse priorité | ~1-3 min par capture |

Le moteur de calibration C++ est la transposition directe du moteur
JavaScript déjà écrit et validé (disposition des poids vérifiée sur les
modèles d'exemple officiels de NeuralAmpModelerCore) : mêmes mesures
(rampe 220 Hz → courbe, sinus par paliers + Goertzel → filtres).

## Contraintes assumées (les écrire avant de coder)

- **Taille des modèles** : les poids doivent tenir en RAM pour l'analyse
  (~25 000 poids max ≈ 100 Ko — couvre les architectures NAM standard, lite,
  feather, nano, et les LSTM courants). Les captures géantes sont refusées
  avec un message clair. Le fichier `.nam` complet (jusqu'à ~1 Mo) passe par
  LittleFS, pas par la RAM.
- **HTTPS** : certificat racine de tone3000.com embarqué dans le firmware
  (à re-vérifier s'il change ; prévoir un garde-fou).
- **API** : 100 requêtes/minute (largement assez), clé publiable requise,
  compte TONE3000 nécessaire (l'autorisation OAuth est LEUR mécanisme
  officiel — on ne contourne rien).
- **mDNS** : `http://soaj-nam.local` marche sûrement sur iPhone/PC, selon la
  version sur Android ; en secours, l'IP est visible dans « appareils
  connectés » du partage de connexion, et le WiFi propre `SOAJ-NAM` reste
  disponible pour la configuration.
- **Toujours l'approximation statique** : courbe + voicing, pas la dynamique
  fine du réseau (limite ESP32 inchangée, voir README).
- **Batterie** : le hotspot consomme — mais il ne sert qu'au chargement.

## Ce qui est réutilisé tel quel

- Chaîne audio 20 kHz, gate, courbe interpolée, fondu anti-clic, sigma-delta.
- La struct `ProfilNam` et la NVS (étendue à 8 slots).
- La page web actuelle (jauges) — enrichie : recherche, slots, configuration.
- L'import manuel d'un `.nam` par la page (moteur JS) **reste en secours** :
  il marche sans compte TONE3000 et sans hotspot.

## Jalons de réalisation

1. **Mode station + configuration** : rejoindre le hotspot (NVS), AP `SOAJ-NAM`
   conservé en secours/configuration, mDNS.
2. **Moteur de calibration C++** (portage du moteur JS validé) + banc d'essai
   hors carte (mêmes tests que le moteur JS : comptage des poids, profils).
3. **Client TONE3000** : OAuth PKCE LAN-relay, recherche, téléchargement
   LittleFS, analyse JSON en flux (le tableau `weights` lu au fil de l'eau).
4. **Page enrichie** : recherche/résultats/slots/progression.
5. **Test réel** de bout en bout sur le matériel.

## Questions ouvertes

- Nombre de slots : 8 proposés — assez ?
- Faut-il un bouton physique pour changer de slot sans téléphone (GPIO libre) ?
- Garder l'import manuel `.nam` visible sur la page, ou le ranger dans un
  onglet « avancé » ?
