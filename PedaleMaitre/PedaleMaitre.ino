/*
 * ============================================================================
 *  SOAJ - Pédale d'effet guitare ESP32
 *  PedaleMaitre.ino  —  ESP32 n°1 : Maître général (point d'accès WiFi + web)
 * ============================================================================
 *
 *  Rôle :
 *    Téléphone --(WiFi, page web)--> ESP32 Maître --(ESP-NOW)--> Esclave(s)
 *
 *    - Crée un réseau WiFi "SOAJ-Pedale" (point d'accès, aucun routeur requis).
 *    - Sert une page web avec des jauges : drive, clip, tone, volume,
 *      ON/OFF et choix des diodes d'écrêtage.
 *    - Valide et borne les paramètres, puis les diffuse à l'esclave en
 *      ESP-NOW (broadcast, même canal WiFi que le point d'accès).
 *    - Renvoie périodiquement les paramètres (si l'esclave démarre après).
 *    - Bonus : accepte aussi les commandes texte sur le port série
 *      (ex : "G:0.8;T:0.4;V:0.6;D:1").
 *
 *  Utilisation :
 *    1. Sur le téléphone : réglages WiFi -> réseau "SOAJ-Pedale",
 *       mot de passe "soaj1234".
 *    2. La page de contrôle s'ouvre toute seule (portail captif) ;
 *       sinon, ouvrir un navigateur sur http://192.168.4.1
 *
 *  AUCUN audio ne transite ici : uniquement quelques octets de paramètres.
 *
 *  Carte : ELEGOO ESP32 (NodeMCU-like, CP2102) — Arduino IDE, carte "ESP32 Dev Module".
 * ============================================================================
 */

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
#define AP_SSID           "SOAJ-Pedale"
#define AP_PASSWORD       "soaj1234"   // min. 8 caractères ; "" = réseau ouvert
#define WIFI_CHANNEL      1            // doit être IDENTIQUE sur l'esclave
#define RESEND_PERIOD_MS  300          // renvoi périodique des paramètres

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

static WebServer server(80);
static DNSServer dnsServer;

static volatile bool paramsDirty = true;   // envoyer dès que possible
static uint32_t lastSendMs       = 0;
static String   serialLine;                // commandes tapées au moniteur série

// ---------------------------------------------------------------------------
// Page web (embarquée en flash) — jauges tactiles, thème "pédale"
// ---------------------------------------------------------------------------
static const char INDEX_HTML[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html lang="fr"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>SOAJ Pédale</title>
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
padding:16px 16px 10px;margin-bottom:12px;box-shadow:0 4px 18px #0006}
.ctl{margin-bottom:14px}
.row{display:flex;justify-content:space-between;align-items:baseline;margin-bottom:4px}
.row label{font-size:12px;letter-spacing:3px;color:var(--mut);text-transform:uppercase;font-weight:600}
.row output{font-size:17px;font-weight:800;color:var(--amber);font-variant-numeric:tabular-nums}
.hint{font-size:10px;color:#6d6455;letter-spacing:1px;margin-top:-2px}
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
.grid{display:grid;grid-template-columns:repeat(4,1fr);gap:8px;margin:8px 0 12px}
.grid button{padding:12px 4px;border-radius:12px;border:1px solid var(--line);
background:#1a1712;color:var(--mut);font-size:12px;font-weight:700;letter-spacing:1px;cursor:pointer;transition:.15s}
.grid button small{display:block;font-size:9px;font-weight:400;margin-top:3px;letter-spacing:0}
.grid button.sel{background:linear-gradient(#3a2c14,#2c210f);color:var(--amber);
border-color:var(--amber2);box-shadow:0 0 12px #ff7b1c44,inset 0 0 8px #ff7b1c22}
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
  <div><h1>S<b>O</b>AJ</h1><div class="sub">SATURATION &middot; ESP32</div></div>
  <div class="link"><span id="lktxt">liaison</span><span class="dot" id="dot"></span></div>
</header>

<div class="card">
  <div class="ctl"><div class="row"><label for="g">Drive</label><output id="og">0.50</output></div>
    <input type="range" id="g" min="0" max="1" step="0.01" value="0.5">
    <div class="hint">pot R5+R6 &mdash; gain &times;51 &agrave; &times;501</div></div>
  <div class="ctl"><div class="row"><label for="c">Clip</label><output id="oc">0.85</output></div>
    <input type="range" id="c" min="0.5" max="0.95" step="0.01" value="0.85">
    <div class="hint">seuil d'&eacute;cr&ecirc;tage &mdash; plus bas = &eacute;crase plus t&ocirc;t</div></div>
  <div class="ctl"><div class="row"><label for="t">Tone</label><output id="ot">0.50</output></div>
    <input type="range" id="t" min="0" max="1" step="0.01" value="0.5">
    <div class="hint">pot R8+R9 &mdash; 0 = sombre, 1 = brillant</div></div>
  <div class="ctl"><div class="row"><label for="v">Volume</label><output id="ov">0.50</output></div>
    <input type="range" id="v" min="0" max="1" step="0.01" value="0.5">
    <div class="hint">pot R10+R11 &mdash; niveau de sortie</div></div>
</div>

<div class="card">
  <div class="row"><label>Diodes d'&eacute;cr&ecirc;tage</label></div>
  <div class="grid" id="diodes">
    <button data-d="0">SANS<small>rails &plusmn;12 V</small></button>
    <button data-d="1">SILICIUM<small>&plusmn;0,6 V</small></button>
    <button data-d="2">LED<small>&plusmn;1,7 V</small></button>
    <button data-d="3">GERMA.<small>&plusmn;0,3 V</small></button>
  </div>
</div>

<div class="card fswrap">
  <button id="fsw" aria-label="effet"></button>
  <div id="fswtxt">BYPASS</div>
</div>
<footer>http://192.168.4.1 &mdash; SOAJ P&eacute;dale ESP32</footer>
</div>

<script>
var st={g:0.5,c:0.85,t:0.5,v:0.5,e:1,d:1};
var timer=null,lastEdit=0;
function $(id){return document.getElementById(id)}
function paint(){
  ['g','c','t','v'].forEach(function(k){
    var el=$(k),min=+el.min,max=+el.max;
    el.value=st[k];
    el.style.setProperty('--p',(100*(st[k]-min)/(max-min))+'%');
    $('o'+k).textContent=Number(st[k]).toFixed(2);
  });
  var on=(st.e==1);
  $('fsw').className=on?'on':'';
  $('fswtxt').textContent=on?'EFFET ON':'BYPASS';
  var bs=document.querySelectorAll('#diodes button');
  for(var i=0;i<bs.length;i++)bs[i].className=(+bs[i].getAttribute('data-d')==st.d)?'sel':'';
}
function link(ok){
  $('dot').className='dot'+(ok?' on':'');
  $('lktxt').textContent=ok?'connecté':'hors ligne';
}
function push(){
  clearTimeout(timer);
  timer=setTimeout(function(){
    fetch('/api/set?g='+st.g+'&c='+st.c+'&t='+st.t+'&v='+st.v+'&e='+st.e+'&d='+st.d)
      .then(function(r){link(r.ok)}).catch(function(){link(false)});
  },120);
}
['g','c','t','v'].forEach(function(k){
  $(k).addEventListener('input',function(){
    st[k]=+this.value;lastEdit=Date.now();paint();push();
  });
});
$('fsw').addEventListener('click',function(){
  st.e=st.e==1?0:1;lastEdit=Date.now();paint();push();
});
document.querySelectorAll('#diodes button').forEach(function(b){
  b.addEventListener('click',function(){
    st.d=+this.getAttribute('data-d');lastEdit=Date.now();paint();push();
  });
});
function poll(){
  fetch('/api/get').then(function(r){return r.json()}).then(function(j){
    link(true);
    if(Date.now()-lastEdit>2000){st=j;paint();}
  }).catch(function(){link(false)});
}
poll();setInterval(poll,3000);
</script>
</body></html>
)rawliteral";

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

static void logParams(const char *src) {
  Serial.printf("[%s] Paramètres : G=%.2f C=%.2f T=%.2f V=%.2f E=%d D=%d\n",
                src, params.gain, params.clip, params.tone, params.volume,
                params.effectOn, params.diode);
}

// ---------------------------------------------------------------------------
// Serveur web
// ---------------------------------------------------------------------------
static void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

// GET /api/get  ->  {"g":0.50,"c":0.85,"t":0.50,"v":0.50,"e":1,"d":1}
static void handleApiGet() {
  char buf[112];
  snprintf(buf, sizeof(buf),
           "{\"g\":%.2f,\"c\":%.2f,\"t\":%.2f,\"v\":%.2f,\"e\":%d,\"d\":%d}",
           params.gain, params.clip, params.tone, params.volume,
           params.effectOn, params.diode);
  server.send(200, "application/json", buf);
}

// GET /api/set?g=0.8&c=0.85&t=0.4&v=0.6&e=1&d=1  (chaque argument est optionnel)
static void handleApiSet() {
  bool changed = false;
  if (server.hasArg("g")) { params.gain   = clampf(server.arg("g").toFloat(), GAIN_MIN, GAIN_MAX);     changed = true; }
  if (server.hasArg("c")) { params.clip   = clampf(server.arg("c").toFloat(), CLIP_MIN, CLIP_MAX);     changed = true; }
  if (server.hasArg("t")) { params.tone   = clampf(server.arg("t").toFloat(), TONE_MIN, TONE_MAX);     changed = true; }
  if (server.hasArg("v")) { params.volume = clampf(server.arg("v").toFloat(), VOLUME_MIN, VOLUME_MAX); changed = true; }
  if (server.hasArg("e")) { params.effectOn = (server.arg("e").toInt() >= 1) ? 1 : 0;                  changed = true; }
  if (server.hasArg("d")) { params.diode  = (uint8_t)clampf((float)server.arg("d").toInt(), 0.0f, 3.0f); changed = true; }
  if (changed) {
    paramsDirty = true;
    logParams("WEB");
  }
  handleApiGet();  // répond avec l'état courant
}

// Portail captif : toute URL inconnue redirige vers la page de contrôle.
// (iOS/Android testent une URL au moment de la connexion WiFi : la
// redirection fait apparaître la page toute seule.)
static void handleNotFound() {
  server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
  server.send(302, "text/plain", "");
}

// ---------------------------------------------------------------------------
// Commandes texte sur le port série (bonus, même format qu'avant)
//   G:0.8;C:0.85;T:0.4;V:0.6;E:1;D:1
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
    while (i < n && (cmd.charAt(i) == ':' || cmd.charAt(i) == '=' || cmd.charAt(i) == ' ')) i++;

    int start = i;
    while (i < n && (isDigit(cmd.charAt(i)) || cmd.charAt(i) == '.' || cmd.charAt(i) == '-')) i++;
    if (start == i) continue;
    float v = cmd.substring(start, i).toFloat();

    switch (key) {
      case 'G': params.gain   = clampf(v, GAIN_MIN, GAIN_MAX);       changed = true; break;
      case 'C': params.clip   = clampf(v, CLIP_MIN, CLIP_MAX);       changed = true; break;
      case 'T': params.tone   = clampf(v, TONE_MIN, TONE_MAX);       changed = true; break;
      case 'V': params.volume = clampf(v, VOLUME_MIN, VOLUME_MAX);   changed = true; break;
      case 'E': params.effectOn = (v >= 0.5f) ? 1 : 0;               changed = true; break;
      case 'D': params.diode  = (uint8_t)clampf(v, 0.0f, 3.0f);      changed = true; break;
      default:  break;
    }
  }

  if (changed) {
    paramsDirty = true;
    logParams("Série");
  }
}

static void pollSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialLine.length() > 0) parseCommand(serialLine);
      serialLine = "";
    } else if (serialLine.length() < 120) {
      serialLine += c;
    }
  }
}

// ---------------------------------------------------------------------------
// Initialisation
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== SOAJ Pédale — MAÎTRE (WiFi + web) ===");

  // --- Point d'accès WiFi (fixe aussi le canal pour l'ESP-NOW) ---
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD, WIFI_CHANNEL);

  Serial.printf("[WiFi] Point d'accès : \"%s\"  mot de passe : \"%s\"  canal : %d\n",
                AP_SSID, AP_PASSWORD, WIFI_CHANNEL);
  Serial.print("[WiFi] Page de contrôle : http://");
  Serial.println(WiFi.softAPIP());

  // --- ESP-NOW (sur l'interface du point d'accès) ---
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] Échec init — redémarrage dans 3 s");
    delay(3000);
    ESP.restart();
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST_MAC, 6);
  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;
  peer.ifidx   = WIFI_IF_AP;   // on émet depuis l'interface point d'accès
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("[ESP-NOW] Échec ajout du pair broadcast");
  }

  // --- DNS captif : toutes les requêtes DNS pointent vers la pédale ---
  dnsServer.start(53, "*", WiFi.softAPIP());

  // --- Serveur web ---
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/get", HTTP_GET, handleApiGet);
  server.on("/api/set", HTTP_GET, handleApiSet);
  server.onNotFound(handleNotFound);
  server.begin();

  Serial.println("[Web] Serveur démarré. Connectez le téléphone au WiFi puis ouvrez http://192.168.4.1");
  Serial.println("[Série] Commandes acceptées aussi ici, ex : G:0.8;C:0.85;T:0.4;V:0.6;E:1;D:1");
}

// ---------------------------------------------------------------------------
// Boucle principale : web + DNS + série + envoi ESP-NOW
// ---------------------------------------------------------------------------
void loop() {
  dnsServer.processNextRequest();
  server.handleClient();
  pollSerialCommands();

  uint32_t now = millis();
  if (paramsDirty || (now - lastSendMs) >= RESEND_PERIOD_MS) {
    sendParamsEspNow();
    paramsDirty = false;
    lastSendMs = now;
  }

  delay(2);   // laisse la main au WiFi sans ralentir la page
}
