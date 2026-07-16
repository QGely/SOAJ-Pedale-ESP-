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
 *    - Sert une page web avec 7 jauges : drive, dist(saturation), low, mid,
 *      high, tone, volume + ON/OFF.
 *      L'esclave applique : Hin × Hampli(g) -> S(u,d) saturation ->
 *      EQ 3 bandes (b,m,h) -> Hsortie(t,v).
 *    - Valide et borne les paramètres, puis les diffuse à l'esclave en
 *      ESP-NOW (broadcast, même canal WiFi que le point d'accès).
 *    - Renvoie périodiquement les paramètres (si l'esclave démarre après).
 *    - Bonus : accepte aussi les commandes texte sur le port série
 *      (ex : "G:0.8;D:0.5;B:0.5;M:0.6;H:0.4;T:0.4;V:0.6;E:1").
 *
 *  NB : le paquet ESP-NOW a changé (7 paramètres) — re-téléverser l'ESCLAVE
 *  en même temps que ce fichier, sinon les réglages seront ignorés.
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
  float    gain;      // g : drive (0..1)
  float    dist;      // d : saturation (0 = aucune .. 1 = extrême)
  float    oct;       // o : octave fOXX (0 = off .. 1 = plein octaver)
  float    low;       // b : graves  (0..1, 0.5 = plat, ±12 dB)
  float    mid;       // m : médiums (0..1, 0.5 = plat, ±12 dB)
  float    high;      // h : aigus   (0..1, 0.5 = plat, ±12 dB)
  float    tone;      // t : tonalité (0..1)
  float    volume;    // v : volume (0..1)
  uint8_t  effectOn;  // 0 = bypass, 1 = effet actif
  uint8_t  profil;    // p : profil de saturation (0 = circuit SOAJ,
                      //     1 = TS9, 2 = RAT, 3 = BIG MUFF — pédale SATU)
} PedalParams;

// Nombre de profils embarqués dans l'esclave SATU (voir profils_pedales.h)
#define NB_PROFILS 4

// Valeurs de départ : potentiomètres à mi-course, EQ plat, crunch léger
static PedalParams params = {
  PARAMS_MAGIC,
  0.5f,    // drive
  0.3f,    // dist : léger crunch
  0.0f,    // oct : octave coupée
  0.5f,    // low  (plat)
  0.5f,    // mid  (plat)
  0.5f,    // high (plat)
  0.5f,    // tone
  0.5f,    // volume
  1,       // effet ON
  0        // profil : circuit SOAJ d'origine
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
  <div><h1>S<b>O</b>AJ</h1><div class="sub">SATURATION + EQ 3 BANDES &middot; ESP32</div></div>
  <div class="link"><span id="lktxt">liaison</span><span class="dot" id="dot"></span></div>
</header>

<div class="card">
  <div class="row"><label>P&eacute;dale connect&eacute;e</label></div>
  <div class="grid" style="grid-template-columns:repeat(2,1fr)" id="pedsel">
    <button data-m="satu" class="sel">SATU<small>fuzz / EQ / octave</small></button>
    <button data-m="psyche">PSYCHE<small>disto + r&eacute;verbe</small></button>
  </div>
</div>

<div class="card">
  <div class="row"><label>Pr&eacute;sets</label></div>
  <div class="grid" id="presets">
    <button data-p="clean">CLEAN<small>cristallin</small></button>
    <button data-p="crunch">CRUNCH<small>rock</small></button>
    <button data-p="muse">MUSE<small>fuzz Bellamy</small></button>
    <button data-p="psycho">PSYCHO<small>solo Muse</small></button>
    <button data-p="foxx">fOXX<small>octave fuzz</small></button>
    <button data-p="metal">METAL<small>serr&eacute; moderne</small></button>
    <button data-p="psyche">PSYCHE<small>p&eacute;dale r&eacute;verbe</small></button>
  </div>
</div>

<div class="card">
  <div class="row"><label>Saturation &mdash; profil de p&eacute;dale</label></div>
  <div class="grid" id="profsel">
    <button data-n="0" class="sel">SOAJ<small>circuit d'origine</small></button>
    <button data-n="1">TS9<small>overdrive doux</small></button>
    <button data-n="2">RAT<small>disto mordante</small></button>
    <button data-n="3">MUFF<small>fuzz compress&eacute;</small></button>
  </div>
  <div class="hint">profils convertis depuis des captures TONE3000 (pot Drive = gain du profil, Dist = intensit&eacute;)</div>
</div>

<div class="card">
  <div class="ctl"><div class="row"><label for="g">Drive</label><output id="og">0.50</output></div>
    <input type="range" id="g" min="0" max="1" step="0.01" value="0.5">
    <div class="hint">gain d'entr&eacute;e (pot R5 de la formule) &mdash; pousse l'&eacute;tage de saturation</div></div>
  <div class="ctl"><div class="row"><label for="d">Dist</label><output id="od">0.30</output></div>
    <input type="range" id="d" min="0" max="1" step="0.01" value="0.3">
    <div class="hint">0 = CLEAN PUR (signal brut, aucun traitement) &rarr; 1 = fuzz extr&ecirc;me</div></div>
  <div class="ctl"><div class="row"><label for="o" id="lb_o">Octave</label><output id="oo">0.00</output></div>
    <input type="range" id="o" min="0" max="1" step="0.01" value="0">
    <div class="hint" id="hi_o">fOXX Tone Machine &mdash; redressement |u| : la fondamentale devient l'octave sup&eacute;rieure</div></div>
</div>

<div class="card">
  <div class="ctl"><div class="row"><label for="b" id="lb_b">Low</label><output id="ob">0.50</output></div>
    <input type="range" id="b" min="0" max="1" step="0.01" value="0.5">
    <div class="hint" id="hi_b">graves 100 Hz &mdash; 0.5 = plat, &plusmn;12 dB</div></div>
  <div class="ctl"><div class="row"><label for="m" id="lb_m">Mid</label><output id="om">0.50</output></div>
    <input type="range" id="m" min="0" max="1" step="0.01" value="0.5">
    <div class="hint" id="hi_m">m&eacute;diums 700 Hz &mdash; 0.5 = plat, &plusmn;12 dB</div></div>
  <div class="ctl"><div class="row"><label for="h" id="lb_h">High</label><output id="oh">0.50</output></div>
    <input type="range" id="h" min="0" max="1" step="0.01" value="0.5">
    <div class="hint" id="hi_h">aigus 3,2 kHz &mdash; 0.5 = plat, &plusmn;12 dB</div></div>
</div>

<div class="card">
  <div class="ctl"><div class="row"><label for="t">Tone</label><output id="ot">0.50</output></div>
    <input type="range" id="t" min="0" max="1" step="0.01" value="0.5">
    <div class="hint">pot R8+R9 &mdash; 0 = sombre, 1 = brillant</div></div>
  <div class="ctl"><div class="row"><label for="v">Volume</label><output id="ov">0.50</output></div>
    <input type="range" id="v" min="0" max="1" step="0.01" value="0.5">
    <div class="hint">pot R10+R11 &mdash; niveau de sortie</div></div>
</div>

<div class="card fswrap">
  <button id="fsw" aria-label="effet"></button>
  <div id="fswtxt">BYPASS</div>
  <div class="grid" style="grid-template-columns:1fr;width:100%;margin-top:14px">
    <button id="direct">TEST DIRECT<small>ADC &rarr; DAC : rien que la guitare, aucun traitement</small></button>
  </div>
</div>
<footer>http://192.168.4.1 &mdash; SOAJ P&eacute;dale ESP32</footer>
</div>

<script>
var KEYS=['g','d','o','b','m','h','t','v'];
var st={g:0.5,d:0.3,o:0,b:0.5,m:0.5,h:0.5,t:0.5,v:0.5,e:1,p:0};
var timer=null,lastEdit=0;
function $(id){return document.getElementById(id)}
function paint(){
  KEYS.forEach(function(k){
    var el=$(k),min=+el.min,max=+el.max;
    el.value=st[k];
    el.style.setProperty('--p',(100*(st[k]-min)/(max-min))+'%');
    $('o'+k).textContent=Number(st[k]).toFixed(2);
  });
  var on=(st.e==1);
  $('fsw').className=on?'on':'';
  $('fswtxt').textContent=(st.e==2)?'TEST DIRECT':(on?'EFFET ON':'BYPASS');
  $('direct').className=(st.e==2)?'sel':'';
  var pb=document.querySelectorAll('#profsel button');
  for(var i=0;i<pb.length;i++)pb[i].className=(+pb[i].getAttribute('data-n')==(st.p||0))?'sel':'';
}
function link(ok){
  $('dot').className='dot'+(ok?' on':'');
  $('lktxt').textContent=ok?'connecté':'hors ligne';
}
function push(){
  clearTimeout(timer);
  timer=setTimeout(function(){
    var q=KEYS.map(function(k){return k+'='+st[k]}).join('&');
    fetch('/api/set?'+q+'&e='+st.e+'&p='+(st.p||0))
      .then(function(r){link(r.ok)}).catch(function(){link(false)});
  },120);
}
KEYS.forEach(function(k){
  $(k).addEventListener('input',function(){
    st[k]=+this.value;lastEdit=Date.now();paint();push();
  });
});
var LBL={
 satu:{o:['Octave','fOXX Tone Machine — redressement |u| : la fondamentale devient l\'octave supérieure'],
       b:['Low','graves 100 Hz — 0.5 = plat, ±12 dB'],
       m:['Mid','médiums 700 Hz — 0.5 = plat, ±12 dB'],
       h:['High','aigus 3,2 kHz — 0.5 = plat, ±12 dB']},
 psyche:{o:['Reverb','mix de réverbération — 0 = sec, 1 = cathédrale'],
       b:['Decay','durée de la queue de réverbe — 0 = courte, 1 = très longue'],
       m:['Warble','pulsation psychédélique de la réverbe (2 LFO lents)'],
       h:['Bright','brillance de la queue de réverbe']}
};
function relabel(mode){
  ['o','b','m','h'].forEach(function(k){
    $('lb_'+k).textContent=LBL[mode][k][0];
    $('hi_'+k).textContent=LBL[mode][k][1];
  });
  var bs=document.querySelectorAll('#pedsel button');
  for(var i=0;i<bs.length;i++)bs[i].className=(bs[i].getAttribute('data-m')==mode)?'sel':'';
}
document.querySelectorAll('#pedsel button').forEach(function(bt){
  bt.addEventListener('click',function(){relabel(this.getAttribute('data-m'));});
});
document.querySelectorAll('#profsel button').forEach(function(bt){
  bt.addEventListener('click',function(){
    st.p=+this.getAttribute('data-n');lastEdit=Date.now();paint();push();
  });
});
var PRESETS={
  clean:{g:0.30,d:0.00,o:0.00,b:0.50,m:0.50,h:0.50,t:0.60,v:0.60},
  crunch:{g:0.55,d:0.40,o:0.00,b:0.50,m:0.60,h:0.55,t:0.55,v:0.55},
  muse:{g:0.85,d:0.90,o:0.00,b:0.60,m:0.75,h:0.55,t:0.45,v:0.60},
  psycho:{g:0.90,d:0.95,o:0.00,b:0.85,m:0.80,h:0.45,t:0.50,v:0.65},
  foxx:{g:0.60,d:0.55,o:1.00,b:0.50,m:0.60,h:0.50,t:0.50,v:0.60},
  metal:{g:0.75,d:0.80,o:0.00,b:0.62,m:0.42,h:0.62,t:0.60,v:0.60},
  psyche:{g:0.45,d:0.50,o:0.70,b:0.65,m:0.50,h:0.55,t:0.65,v:0.60}
};
document.querySelectorAll('#presets button').forEach(function(bt){
  bt.addEventListener('click',function(){
    var id=this.getAttribute('data-p');
    var p=PRESETS[id];
    for(var k in p)st[k]=p[k];
    st.e=1;lastEdit=Date.now();
    relabel(id=='psyche'?'psyche':'satu');
    paint();push();
  });
});
$('fsw').addEventListener('click',function(){
  st.e=st.e==1?0:1;lastEdit=Date.now();paint();push();
});
$('direct').addEventListener('click',function(){
  st.e=st.e==2?1:2;lastEdit=Date.now();paint();push();
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
  Serial.printf("[%s] Paramètres : G=%.2f D=%.2f O=%.2f B=%.2f M=%.2f H=%.2f T=%.2f V=%.2f E=%d P=%d\n",
                src, params.gain, params.dist, params.oct, params.low, params.mid,
                params.high, params.tone, params.volume, params.effectOn, params.profil);
}

// ---------------------------------------------------------------------------
// Serveur web
// ---------------------------------------------------------------------------
static void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

// GET /api/get -> {"g":0.50,"d":0.30,"o":0.00,"b":0.50,"m":0.50,"h":0.50,"t":0.50,"v":0.50,"e":1,"p":0}
static void handleApiGet() {
  char buf[160];
  snprintf(buf, sizeof(buf),
           "{\"g\":%.2f,\"d\":%.2f,\"o\":%.2f,\"b\":%.2f,\"m\":%.2f,\"h\":%.2f,"
           "\"t\":%.2f,\"v\":%.2f,\"e\":%d,\"p\":%d}",
           params.gain, params.dist, params.oct, params.low, params.mid,
           params.high, params.tone, params.volume, params.effectOn, params.profil);
  server.send(200, "application/json", buf);
}

// GET /api/set?g=0.8&d=0.5&b=0.5&m=0.6&h=0.4&t=0.4&v=0.6&e=1  (arguments optionnels)
static void handleApiSet() {
  bool changed = false;
  if (server.hasArg("g")) { params.gain   = clampf(server.arg("g").toFloat(), GAIN_MIN, GAIN_MAX);     changed = true; }
  if (server.hasArg("d")) { params.dist   = clampf(server.arg("d").toFloat(), 0.0f, 1.0f);             changed = true; }
  if (server.hasArg("o")) { params.oct    = clampf(server.arg("o").toFloat(), 0.0f, 1.0f);             changed = true; }
  if (server.hasArg("b")) { params.low    = clampf(server.arg("b").toFloat(), 0.0f, 1.0f);             changed = true; }
  if (server.hasArg("m")) { params.mid    = clampf(server.arg("m").toFloat(), 0.0f, 1.0f);             changed = true; }
  if (server.hasArg("h")) { params.high   = clampf(server.arg("h").toFloat(), 0.0f, 1.0f);             changed = true; }
  if (server.hasArg("t")) { params.tone   = clampf(server.arg("t").toFloat(), TONE_MIN, TONE_MAX);     changed = true; }
  if (server.hasArg("v")) { params.volume = clampf(server.arg("v").toFloat(), VOLUME_MIN, VOLUME_MAX); changed = true; }
  if (server.hasArg("e")) { int ev = server.arg("e").toInt();
                            params.effectOn = (uint8_t)((ev < 0) ? 0 : (ev > 2 ? 2 : ev));             changed = true; }
  if (server.hasArg("p")) { int pv = server.arg("p").toInt();
                            params.profil = (uint8_t)((pv < 0) ? 0 : (pv >= NB_PROFILS ? NB_PROFILS - 1 : pv)); changed = true; }
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
//   G:0.8;D:0.5;B:0.5;M:0.6;H:0.4;T:0.4;V:0.6;E:1
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
      case 'D': params.dist   = clampf(v, 0.0f, 1.0f);               changed = true; break;
      case 'O': params.oct    = clampf(v, 0.0f, 1.0f);               changed = true; break;
      case 'B': params.low    = clampf(v, 0.0f, 1.0f);               changed = true; break;
      case 'M': params.mid    = clampf(v, 0.0f, 1.0f);               changed = true; break;
      case 'H': params.high   = clampf(v, 0.0f, 1.0f);               changed = true; break;
      case 'T': params.tone   = clampf(v, TONE_MIN, TONE_MAX);       changed = true; break;
      case 'V': params.volume = clampf(v, VOLUME_MIN, VOLUME_MAX);   changed = true; break;
      case 'E': params.effectOn = (v >= 1.5f) ? 2 : ((v >= 0.5f) ? 1 : 0); changed = true; break;
                // E:0 = bypass, E:1 = effet, E:2 = TEST DIRECT (diagnostic)
      case 'P': { int pv = (int)v;
                  params.profil = (uint8_t)((pv < 0) ? 0 : (pv >= NB_PROFILS ? NB_PROFILS - 1 : pv));
                  changed = true; break; }
                // P:0 = circuit SOAJ, P:1 = TS9, P:2 = RAT, P:3 = BIG MUFF
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
  Serial.println("[Série] Commandes acceptées aussi ici, ex : G:0.8;D:0.5;O:1;B:0.5;M:0.6;H:0.4;T:0.4;V:0.6;E:1;P:0");
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
