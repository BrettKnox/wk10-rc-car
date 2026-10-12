// WK-10 car brain. Board: "ESP32 Dev Module", Arduino-ESP32 core 3.x.
// Power: ESC BEC (5.8-6 V) -> dev board VIN/5V pin. Never the 2S pack directly.
//
// Three ways to drive it, one safety path:
//   1. ESP-NOW transmitter (firmware/tx). Wins whenever its link is alive.
//   2. Phone: join Wi-Fi "WK10-xxxx", open http://192.168.4.1, joystick + tuning page.
//   3. Code: HTTP GET /drive?s=<-1000..1000>&t=<-1000..1000>&d=1 at 10-50 Hz (tools/drive.py).
// Safety, in order:
//   - No valid command for cfg.failsafeMs -> throttle neutral, steering centre, pulses keep running.
//   - Throttle is zero unless the dead-man is held (TX button, phone finger down, d=1).
//   - Boot holds neutral for ARM_MS so the ESC arms on 1500 us.
//   - The mixing maths self-tests at boot; a failure parks the car at neutral.
//   - The access point stays off until AP_PASS is changed from the placeholder.
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ArduinoOTA.h>
#include <WkLink.h>   // firmware/libraries/WkLink

// ---- pins (LEDC-capable, not strapping pins) ----
const int PIN_STEER = 18;
const int PIN_ESC   = 19;
const int PIN_LED   = 2;
const int PIN_VBAT  = 34;          // ADC1, input only. 30k top / 10k bottom divider from the pack.

// ---- knobs you set once ----
const char *AP_PASS = "CHANGE-ME";  // 8+ chars. Also the OTA password. AP stays off until changed.
uint8_t TX_MAC[6] = {0, 0, 0, 0, 0, 0};  // bind to one transmitter; all zero = any WK10 packet
const float VBAT_RATIO = 4.0f;     // (top + bottom) / bottom. 0 = no divider fitted.
const int CELLS = 2;
const float CELL_LIMP_V = 3.4f;    // below this under load, throttle is capped at LIMP_THROTTLE
const float LIMP_THROTTLE = 0.3f;
const uint32_t ARM_MS = 3000;
const uint32_t ESPNOW_PRIORITY_MS = 500;   // web/API commands are ignored while the TX is talking

WkCfg cfg = WK_DEFAULTS;
Preferences prefs;
WebServer server(80);

// latest command from each source
struct Cmd { float steer = 0, thr = 0; bool deadman = false; uint32_t ms = 0; };
volatile Cmd nowCmd, webCmd;
volatile uint32_t badPackets = 0;
float slewed = 0, vbat = NAN;
bool limp = false, apOn = false;
int lastSteerUs = 1500, lastEscUs = 1500;
const char *source = "none";

static uint32_t usToDuty(int us) { return (uint32_t)us * 65535UL / 20000UL; }  // 16-bit at 50 Hz

static bool macBound() {
  for (int i = 0; i < 6; i++) if (TX_MAC[i]) return true;
  return false;
}

void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(WkPacket)) { badPackets++; return; }
  if (macBound() && memcmp(info->src_addr, TX_MAC, 6) != 0) { badPackets++; return; }
  WkPacket p;
  memcpy(&p, data, sizeof p);
  if (p.magic != WK_MAGIC || p.sum != wk_sum(p)) { badPackets++; return; }
  nowCmd.steer = wk_clampf(p.steer / 1000.0f, -1, 1);
  nowCmd.thr = wk_clampf(p.throttle / 1000.0f, -1, 1);
  nowCmd.deadman = p.flags & 1;
  nowCmd.ms = millis();
}

// ---------------------------------------------------------------- settings in flash
static void loadCfg() {
  WkCfg c;
  prefs.begin("wk10", true);
  size_t n = prefs.getBytes("cfg", &c, sizeof c);
  prefs.end();
  cfg = (n == sizeof c && wk_cfg_valid(c)) ? c : WK_DEFAULTS;
}

static bool saveCfg(const WkCfg &c) {
  if (!wk_cfg_valid(c)) return false;
  prefs.begin("wk10", false);
  bool ok = prefs.putBytes("cfg", &c, sizeof c) == sizeof c;
  prefs.end();
  if (ok) cfg = c;
  return ok;
}

// ---------------------------------------------------------------- web
static const char PAGE[] PROGMEM = R"HTML(<!doctype html><html lang="en"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no"><title>WK-10</title>
<style>body{font:16px system-ui;margin:0;padding:12px;background:#111;color:#eee}
#pad{touch-action:none;width:100%;max-width:420px;aspect-ratio:1;background:#222;border-radius:12px;position:relative;margin:auto}
#dot{position:absolute;width:56px;height:56px;border-radius:50%;background:#3a8;left:calc(50% - 28px);top:calc(50% - 28px)}
label{display:flex;justify-content:space-between;gap:8px;margin:6px 0}input{width:7em;font-size:16px}
button{font-size:16px;min-height:44px;padding:0 16px;margin:4px}#st{font-family:monospace;white-space:pre}</style>
<h1 style="font-size:20px">WK-10</h1><div id="pad" role="application" aria-label="Drive pad: hold to drive, up is throttle, sideways steers"><div id="dot"></div></div>
<div id="st">connecting</div>
<details><summary>Tuning (saved to the car)</summary><form id="f"></form>
<button id="save">Save</button><button id="def">Defaults</button></details>
<script>
const F=["steerCenter","steerLeft","steerRight","steerReverse","escNeutral","escFwdMax","escRevMax","throttleSlew","steerExpo","maxThrottle","failsafeMs"];
let s=0,t=0,held=0;const pad=document.getElementById('pad'),dot=document.getElementById('dot');
function mv(e){const r=pad.getBoundingClientRect();s=Math.max(-1,Math.min(1,(e.clientX-r.left)/r.width*2-1));
t=Math.max(-1,Math.min(1,1-(e.clientY-r.top)/r.height*2));dot.style.left=(s+1)/2*r.width-28+'px';dot.style.top=(1-t)/2*r.height-28+'px'}
pad.onpointerdown=e=>{held=1;pad.setPointerCapture(e.pointerId);mv(e)};pad.onpointermove=e=>{if(held)mv(e)};
pad.onpointerup=pad.onpointercancel=()=>{held=0;s=0;t=0;dot.style.left=dot.style.top='calc(50% - 28px)'};
async function tick(){try{const r=await fetch(`/drive?s=${Math.round(s*1000)}&t=${Math.round(t*1000)}&d=${held}`);
const j=await r.json();document.getElementById('st').textContent=`src ${j.src}  steer ${j.steer_us}us  esc ${j.esc_us}us\nbatt ${j.vbat??'n/a'} V ${j.limp?'LIMP':''}  bad pkts ${j.bad}`}
catch(e){document.getElementById('st').textContent='no link'}setTimeout(tick,50)}tick();
async function load(){const c=await (await fetch('/cfg')).json();document.getElementById('f').innerHTML=F.map(k=>
`<label>${k}<input name="${k}" value="${c[k]}" inputmode="decimal"></label>`).join('')}load();
document.getElementById('save').onclick=async()=>{const q=new URLSearchParams(new FormData(document.getElementById('f')));
const r=await fetch('/cfg?'+q,{method:'POST'});alert(r.ok?'saved':'rejected: '+await r.text());load()};
document.getElementById('def').onclick=async()=>{await fetch('/cfg/defaults',{method:'POST'});load()};
</script></html>)HTML";

static bool argInt(const char *k, long lo, long hi, long &out) {
  if (!server.hasArg(k)) return false;
  String v = server.arg(k);
  char *end;
  out = strtol(v.c_str(), &end, 10);
  return v.length() && *end == 0 && out >= lo && out <= hi;
}

static bool argFloat(const char *k, float &out) {
  if (!server.hasArg(k)) return false;
  String v = server.arg(k);
  char *end;
  out = strtof(v.c_str(), &end);
  return v.length() && *end == 0 && isfinite(out);
}

static void sendState() {
  char b[200];
  char vb[12];
  if (isnan(vbat)) strcpy(vb, "null"); else snprintf(vb, sizeof vb, "%.2f", vbat);
  snprintf(b, sizeof b, "{\"src\":\"%s\",\"steer_us\":%d,\"esc_us\":%d,\"vbat\":%s,\"limp\":%s,\"bad\":%lu}",
           source, lastSteerUs, lastEscUs, vb, limp ? "true" : "false", (unsigned long)badPackets);
  server.send(200, "application/json", b);
}

static void handleDrive() {
  long s, t, d;
  if (!argInt("s", -1000, 1000, s) || !argInt("t", -1000, 1000, t) || !argInt("d", 0, 1, d)) {
    server.send(400, "text/plain", "need s,t in -1000..1000 and d in 0..1");
    return;
  }
  webCmd.steer = s / 1000.0f;
  webCmd.thr = t / 1000.0f;
  webCmd.deadman = d == 1;
  webCmd.ms = millis();
  sendState();
}

static void handleCfgGet() {
  char b[300];
  snprintf(b, sizeof b,
           "{\"steerCenter\":%d,\"steerLeft\":%d,\"steerRight\":%d,\"steerReverse\":%d,\"escNeutral\":%d,"
           "\"escFwdMax\":%d,\"escRevMax\":%d,\"throttleSlew\":%.2f,\"steerExpo\":%.2f,\"maxThrottle\":%.2f,\"failsafeMs\":%u}",
           cfg.steerCenter, cfg.steerLeft, cfg.steerRight, cfg.steerReverse, cfg.escNeutral, cfg.escFwdMax,
           cfg.escRevMax, cfg.throttleSlew, cfg.steerExpo, cfg.maxThrottle, cfg.failsafeMs);
  server.send(200, "application/json", b);
}

static void handleCfgPost() {
  WkCfg c = cfg;
  long v;
  float f;
  bool ok = true;
  ok &= argInt("steerCenter", 800, 2200, v); c.steerCenter = v;
  ok &= argInt("steerLeft", 800, 2200, v); c.steerLeft = v;
  ok &= argInt("steerRight", 800, 2200, v); c.steerRight = v;
  ok &= argInt("steerReverse", 0, 1, v); c.steerReverse = v;
  ok &= argInt("escNeutral", 800, 2200, v); c.escNeutral = v;
  ok &= argInt("escFwdMax", 800, 2200, v); c.escFwdMax = v;
  ok &= argInt("escRevMax", 800, 2200, v); c.escRevMax = v;
  ok &= argFloat("throttleSlew", f); c.throttleSlew = f;
  ok &= argFloat("steerExpo", f); c.steerExpo = f;
  ok &= argFloat("maxThrottle", f); c.maxThrottle = f;
  ok &= argInt("failsafeMs", 100, 1000, v); c.failsafeMs = v;
  if (!ok || !saveCfg(c)) { server.send(400, "text/plain", "out of range: see wk_cfg_valid in WkLink.h"); return; }
  handleCfgGet();
}

// ---------------------------------------------------------------- setup / loop
static void park(int blinkMs, const char *why) {
  Serial.println(why);
  while (true) {
    ledcWrite(PIN_STEER, usToDuty(WK_DEFAULTS.steerCenter));
    ledcWrite(PIN_ESC, usToDuty(WK_DEFAULTS.escNeutral));
    digitalWrite(PIN_LED, millis() / blinkMs % 2);
    delay(10);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  ledcAttach(PIN_STEER, 50, 16);
  ledcAttach(PIN_ESC, 50, 16);
  ledcWrite(PIN_STEER, usToDuty(WK_DEFAULTS.steerCenter));
  ledcWrite(PIN_ESC, usToDuty(WK_DEFAULTS.escNeutral));

  int bad = wk_selftest();
  Serial.printf("selftest: %s (%d failed)\n", bad ? "FAIL" : "pass", bad);
  if (bad) park(60, "selftest failed: car parked at neutral");
  loadCfg();
  analogSetPinAttenuation(PIN_VBAT, ADC_11db);

  apOn = strcmp(AP_PASS, "CHANGE-ME") != 0 && strlen(AP_PASS) >= 8;
  WiFi.mode(apOn ? WIFI_AP_STA : WIFI_STA);
  esp_wifi_set_channel(WK_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (apOn) {
    uint8_t m[6];
    WiFi.macAddress(m);
    char ssid[16];
    snprintf(ssid, sizeof ssid, "WK10-%02X%02X", m[4], m[5]);
    WiFi.softAP(ssid, AP_PASS, WK_CHANNEL, 0, 2);
    server.on("/", [] { server.send_P(200, "text/html", PAGE); });
    server.on("/drive", handleDrive);
    server.on("/state", sendState);
    server.on("/cfg", HTTP_GET, handleCfgGet);
    server.on("/cfg", HTTP_POST, handleCfgPost);
    server.on("/cfg/defaults", HTTP_POST, [] { saveCfg(WK_DEFAULTS); handleCfgGet(); });
    server.begin();
    ArduinoOTA.setHostname(ssid);
    ArduinoOTA.setPassword(AP_PASS);
    ArduinoOTA.onStart([] { ledcWrite(PIN_ESC, usToDuty(WK_DEFAULTS.escNeutral)); });
    ArduinoOTA.begin();
    Serial.printf("Wi-Fi %s up, open http://192.168.4.1\n", ssid);
  } else {
    Serial.println("AP off: set AP_PASS (8+ chars) to enable phone/API control");
  }
  if (esp_now_init() != ESP_OK) park(100, "esp_now_init failed: car parked at neutral");
  esp_now_register_recv_cb(onRecv);
  Serial.print("car STA MAC (paste into tx.ino CAR_MAC): ");
  Serial.println(WiFi.macAddress());

  uint32_t t0 = millis();                       // ESC arming window
  while (millis() - t0 < ARM_MS) { digitalWrite(PIN_LED, millis() / 500 % 2); delay(10); }
}

static void readBattery() {
  static uint32_t lowSince = 0;
  if (VBAT_RATIO <= 0) return;
  float v = analogReadMilliVolts(PIN_VBAT) / 1000.0f * VBAT_RATIO;
  if (v < 1.0f) { vbat = NAN; return; }         // divider not fitted
  vbat = isnan(vbat) ? v : vbat * 0.9f + v * 0.1f;
  bool low = vbat / CELLS < CELL_LIMP_V;
  if (!low) lowSince = 0;
  else if (!lowSince) lowSince = millis();
  if (lowSince && millis() - lowSince > 2000) limp = true;   // latches until power-cycle
}

void loop() {
  static uint32_t lastMs = millis(), lastLog = 0;
  if (apOn) { server.handleClient(); ArduinoOTA.handle(); }
  uint32_t now = millis();
  float dt = (now - lastMs) / 1000.0f;
  lastMs = now;
  readBattery();

  Cmd c;
  bool txAlive = now - nowCmd.ms < ESPNOW_PRIORITY_MS && nowCmd.ms;
  if (txAlive) { c.steer = nowCmd.steer; c.thr = nowCmd.thr; c.deadman = nowCmd.deadman; c.ms = nowCmd.ms; source = "tx"; }
  else { c.steer = webCmd.steer; c.thr = webCmd.thr; c.deadman = webCmd.deadman; c.ms = webCmd.ms; source = "web"; }
  bool linkOk = c.ms && now - c.ms < cfg.failsafeMs;
  if (!linkOk) source = "failsafe";

  float cap = limp ? fminf(cfg.maxThrottle, LIMP_THROTTLE) : cfg.maxThrottle;
  float want = (linkOk && c.deadman) ? c.thr * (c.thr > 0 ? cap : 1.0f) : 0;
  slewed = wk_slew(slewed, want, cfg.throttleSlew, dt);
  lastSteerUs = wk_steer_us(cfg, linkOk ? c.steer : 0);
  lastEscUs = wk_esc_us(cfg, slewed);
  ledcWrite(PIN_STEER, usToDuty(lastSteerUs));
  ledcWrite(PIN_ESC, usToDuty(lastEscUs));
  digitalWrite(PIN_LED, linkOk ? HIGH : (now / 150 % 2));

  if (now - lastLog > 500) {
    lastLog = now;
    Serial.printf("src=%s steer=%dus esc=%dus vbat=%.2f limp=%d bad=%lu\n", source, lastSteerUs, lastEscUs,
                  vbat, limp, (unsigned long)badPackets);
  }
  delay(5);
}
