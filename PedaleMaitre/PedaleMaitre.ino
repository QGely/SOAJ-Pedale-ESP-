/*
 * ============================================================================
 *  SOAJ - Pédale d'effet guitare ESP32
 *  PedaleMaitre.ino  —  ESP32 n°1 : Maître général
 * ============================================================================
 *
 *  Rôle :
 *    Téléphone --(Bluetooth BLE)--> ESP32 Maître --(ESP-NOW / WiFi)--> Esclave
 *
 *    - Reçoit les réglages du téléphone en BLE (service type "Nordic UART").
 *    - Valide et borne les paramètres (gain, clip, tone, volume, ON/OFF).
 *    - Les diffuse à l'esclave en ESP-NOW (broadcast, canal WiFi fixe).
 *    - Renvoie périodiquement les paramètres (si l'esclave démarre après).
 *
 *  AUCUN audio ne transite ici : uniquement quelques octets de paramètres.
 *
 *  Carte : ELEGOO ESP32 (NodeMCU-like, CP2102) — Arduino IDE, carte "ESP32 Dev Module".
 * ============================================================================
 */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
#define BLE_DEVICE_NAME   "SOAJ-Pedale"

// Service BLE style Nordic UART (compatible avec les applis "nRF Connect",
// "Serial Bluetooth Terminal" (mode BLE), etc.)
#define SERVICE_UUID      "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHAR_RX_UUID      "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  // téléphone -> pédale (WRITE)
#define CHAR_TX_UUID      "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  // pédale -> téléphone (NOTIFY)

#define WIFI_CHANNEL      1          // doit être IDENTIQUE sur l'esclave
#define RESEND_PERIOD_MS  300        // renvoi périodique des paramètres

// Bornes de sécurité (identiques côté esclave — défense en profondeur)
// G, T et V sont des positions de potentiomètre (0.0 à 1.0), comme sur le
// circuit : G = drive (R5+R6), T = tone (R8+R9), V = volume (R10+R11).
#define GAIN_MIN    0.0f
#define GAIN_MAX    1.0f
#define CLIP_MIN    0.50f
#define CLIP_MAX    0.95f
#define TONE_MIN    0.0f
#define TONE_MAX    1.0f
#define VOLUME_MIN  0.0f
#define VOLUME_MAX  1.0f

// ---------------------------------------------------------------------------
// Paquet de paramètres (DOIT rester identique dans PedaleEsclave.ino)
// ---------------------------------------------------------------------------
#define PARAMS_MAGIC 0x534F414AUL    // "SOAJ" — rejette les paquets étrangers

typedef struct __attribute__((packed)) {
  uint32_t magic;
  float    gain;      // 0.0 .. 1.0  position du pot DRIVE (gain ampli x51..x501)
  float    clip;      // 0.50 .. 0.95 seuil d'écrêtage effectif (module les diodes)
  float    tone;      // 0.0 .. 1.0  position du pot TONE (0 = sombre, 1 = brillant)
  float    volume;    // 0.0 .. 1.0  position du pot VOLUME
  uint8_t  effectOn;  // 0 = bypass, 1 = effet actif
  uint8_t  diode;     // 0 = sans (rails ±12 V seuls), 1 = silicium ±0,6 V,
                      // 2 = LED ±1,7 V, 3 = germanium ±0,3 V
} PedalParams;

// Valeurs de départ : potentiomètres à mi-course, diodes silicium
static PedalParams params = {
  PARAMS_MAGIC,
  0.5f,    // drive
  0.85f,   // clip
  0.5f,    // tone
  0.5f,    // volume
  1,       // effet ON
  1        // diodes silicium
};

static const uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static BLECharacteristic *pTxChar = nullptr;
static volatile bool paramsDirty  = true;   // envoyer dès que possible
static uint32_t lastSendMs        = 0;

// ---------------------------------------------------------------------------
// Utilitaires
// ---------------------------------------------------------------------------
static float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static void sendParamsEspNow() {
  esp_err_t err = esp_now_send(BROADCAST_MAC, (const uint8_t *)&params, sizeof(params));
  if (err != ESP_OK) {
    Serial.printf("[ESP-NOW] Erreur d'envoi : %d\n", (int)err);
  }
}

// Renvoie l'état courant au téléphone (notification BLE)
static void notifyStatus() {
  if (pTxChar == nullptr) return;
  char buf[80];
  snprintf(buf, sizeof(buf), "G:%.2f;C:%.2f;T:%.2f;V:%.3f;E:%d;D:%d",
           params.gain, params.clip, params.tone, params.volume, params.effectOn, params.diode);
  pTxChar->setValue((uint8_t *)buf, strlen(buf));
  pTxChar->notify();
}

// ---------------------------------------------------------------------------
// Analyse des commandes du téléphone
// ---------------------------------------------------------------------------
// Format texte simple, insensible à la casse, séparateurs ';' ',' ou espace.
// G, T, V = positions de potentiomètre 0.0 à 1.0 :
//   G:0.5   -> drive (pot R5+R6 : gain x51 à x501)
//   C:0.85  -> seuil d'écrêtage (0.5 à 0.95, plus bas = écrase plus tôt)
//   T:0.5   -> tone (pot R8+R9 : 0 = sombre, 1 = brillant)
//   V:0.5   -> volume (pot R10+R11)
//   E:1     -> effet ON / E:0 -> bypass
//   D:1     -> diodes : 0 = sans, 1 = silicium, 2 = LED, 3 = germanium
// Exemple complet : "G:0.8;C:0.85;T:0.4;V:0.6;E:1;D:1"
// ---------------------------------------------------------------------------
static void parseCommand(const String &cmd) {
  int i = 0;
  const int n = cmd.length();
  bool changed = false;

  while (i < n) {
    char c = cmd.charAt(i);
    if (c == ';' || c == ',' || c == ' ' || c == '\r' || c == '\n') { i++; continue; }

    char key = toupper(c);
    i++;
    // saute ':' ou '='
    while (i < n && (cmd.charAt(i) == ':' || cmd.charAt(i) == '=' || cmd.charAt(i) == ' ')) i++;

    // lit le nombre
    int start = i;
    while (i < n && (isDigit(cmd.charAt(i)) || cmd.charAt(i) == '.' || cmd.charAt(i) == '-')) i++;
    if (start == i) continue;  // pas de valeur -> ignore
    float v = cmd.substring(start, i).toFloat();

    switch (key) {
      case 'G': params.gain   = clampf(v, GAIN_MIN, GAIN_MAX);       changed = true; break;
      case 'C': params.clip   = clampf(v, CLIP_MIN, CLIP_MAX);       changed = true; break;
      case 'T': params.tone   = clampf(v, TONE_MIN, TONE_MAX);       changed = true; break;
      case 'V': params.volume = clampf(v, VOLUME_MIN, VOLUME_MAX);   changed = true; break;
      case 'E': params.effectOn = (v >= 0.5f) ? 1 : 0;               changed = true; break;
      case 'D': params.diode  = (uint8_t)clampf(v, 0.0f, 3.0f);      changed = true; break;
      default:  break;  // clé inconnue -> ignore
    }
  }

  if (changed) {
    paramsDirty = true;
    Serial.printf("[BLE] Nouveaux paramètres : G=%.2f C=%.2f T=%.2f V=%.3f E=%d D=%d\n",
                  params.gain, params.clip, params.tone, params.volume, params.effectOn, params.diode);
  }
}

// ---------------------------------------------------------------------------
// Callbacks BLE
// ---------------------------------------------------------------------------
class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) override {
    String value = pChar->getValue().c_str();
    if (value.length() > 0) {
      parseCommand(value);
    }
  }
};

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *) override {
    Serial.println("[BLE] Téléphone connecté");
    paramsDirty = true;  // pousse l'état courant vers le téléphone
  }
  void onDisconnect(BLEServer *pServer) override {
    Serial.println("[BLE] Téléphone déconnecté — relance de la publicité");
    pServer->getAdvertising()->start();
  }
};

// ---------------------------------------------------------------------------
// Initialisation
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== SOAJ Pédale — MAÎTRE ===");

  // --- WiFi / ESP-NOW ---
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] Échec init — redémarrage dans 3 s");
    delay(3000);
    ESP.restart();
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST_MAC, 6);
  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("[ESP-NOW] Échec ajout du pair broadcast");
  }

  Serial.print("[WiFi] MAC maître : ");
  Serial.println(WiFi.macAddress());

  // --- BLE ---
  BLEDevice::init(BLE_DEVICE_NAME);
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  BLECharacteristic *pRxChar = pService->createCharacteristic(
      CHAR_RX_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  pRxChar->setCallbacks(new RxCallbacks());

  pTxChar = pService->createCharacteristic(
      CHAR_TX_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pTxChar->addDescriptor(new BLE2902());

  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();

  Serial.printf("[BLE] Publicité démarrée : \"%s\"\n", BLE_DEVICE_NAME);
  Serial.println("Format : G:0.5;C:0.85;T:0.5;V:0.5;E:1;D:1 (pots 0..1 ; D: 0=sans 1=silicium 2=LED 3=germanium)");
}

// ---------------------------------------------------------------------------
// Boucle principale : envoi immédiat sur changement + renvoi périodique
// ---------------------------------------------------------------------------
void loop() {
  uint32_t now = millis();

  if (paramsDirty || (now - lastSendMs) >= RESEND_PERIOD_MS) {
    sendParamsEspNow();
    if (paramsDirty) {
      notifyStatus();
      paramsDirty = false;
    }
    lastSendMs = now;
  }

  delay(10);
}
