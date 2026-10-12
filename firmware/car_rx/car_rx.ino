// WK-10 car brain. Board: "ESP32 Dev Module", Arduino-ESP32 core 3.x.
// Power: ESC BEC (6 V) -> dev board VIN/5V pin. Never the 2S pack directly.
//
// Three ways to drive it, one safety path:
//   1. ESP-NOW transmitter (firmware/tx). Wins whenever its link is alive.
//   2. Phone: join Wi-Fi "WK10-xxxx", open http://192.168.4.1, joystick + tuning page.
//   3. Code: HTTP GET /drive?s=<-1000..1000>&t=<-1000..1000>&d=<0|1>&n=<seq> (tools/drive.py).
//
// Structure: controlTask (priority 10, core 1, 100 Hz) is the ONLY writer of the servo
// and ESC pulses and owns the failsafe. loop() runs the web server and OTA, which can
// block for seconds on a bad TCP connection; they only post commands under a spinlock.
//
// Safety, in order:
//   - No valid command for cfg.failsafeMs -> throttle neutral, steering centre, pulses keep running.
//   - Throttle needs the dead-man RELEASED then held on the current source after boot,
//     any failsafe and any source change (wk_gate), so a brownout reboot cannot launch the car.
//   - Throttle is zero unless the dead-man is held (TX button, phone finger down, d=1).
//   - The ESC arms on the saved neutral, and failsafe uses that same pulse.
//   - Unbound transmitters are ignored; packets must carry a newer seq than the last one.
//   - The control maths self-tests at boot against the config in use; a failure parks the car.
//   - Settings and OTA are refused unless the car is stopped.
//   - The access point stays off until AP_PASS is changed from the placeholder.
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_task_wdt.h>
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
uint8_t TX_MAC[6] = {0, 0, 0, 0, 0, 0};  // the transmitter's MAC; all zero = ignore every transmitter
const float VBAT_RATIO = 4.0f;     // (top + bottom) / bottom. 0 = no divider fitted.
const int CELLS = 2;
const float CELL_LIMP_V = 3.4f;    // below this under load, throttle is capped at LIMP_THROTTLE
const float LIMP_THROTTLE = 0.3f;
const uint32_t ARM_MS = 3000;
const uint32_t OTA_IDLE_MS = 5000; // OTA only after this long with no valid command

struct Cmd { float steer, thr; bool deadman; uint32_t ms; };
static Cmd txCmd = {0, 0, false, 0}, webCmd = {0, 0, false, 0};
static uint16_t txSeq = 0; static uint32_t webSeq = 0;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;   // guards txCmd, webCmd, cfg, status

WkCfg cfg = WK_DEFAULTS;
int bootNeutral = 1500;            // the pulse the ESC armed on; never changes until reboot
Preferences prefs;
WebServer server(80);
bool apOn = false;

// status written by the control task, read by the web handlers
struct Status { int steerUs, escUs; uint8_t src; bool armed, moving, limp; float vbat; uint32_t lastValidMs, stillSinceMs; };
static Status st = {1500, 1500, WK_SRC_NONE, false, false, false, NAN, 0, 0};
static volatile uint32_t badPackets = 0;

static uint32_t usToDuty(int us) { return (uint32_t)us * 65535UL / 20000UL; }  // 16-bit at 50 Hz

static bool macBound() {
  for (int i = 0; i < 6; i++) if (TX_MAC[i]) return true;
  return false;
}

void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (!macBound() || memcmp(info->src_addr, TX_MAC, 6) != 0) {
    static uint32_t lastLog = 0;
    if (millis() - lastLog > 2000) {          // help binding: say who is talking
      lastLog = millis();
      const uint8_t *m = info->src_addr;
      Serial.printf("ignored ESP-NOW from %02X:%02X:%02X:%02X:%02X:%02X (put it in TX_MAC)\n", m[0], m[1], m[2], m[3], m[4], m[5]);
    }
    badPackets++;
    return;
  }
  if (len != sizeof(WkPacket)) { badPackets++; return; }
  WkPacket p;
  memcpy(&p, data, sizeof p);
  if (p.magic != WK_MAGIC || p.sum != wk_sum(p)) { badPackets++; return; }
  uint32_t now = millis();
  portENTER_CRITICAL(&mux);
  bool fresh = wk_age(now, txCmd.ms) > 1000 || (int16_t)(p.seq - txSeq) > 0;   // replay/stale guard
  if (fresh) {
    txSeq = p.seq;
    txCmd = {wk_clampf(p.steer / 1000.0f, -1, 1), wk_clampf(p.throttle / 1000.0f, -1, 1), (p.flags & 1) != 0, now};
  }
  portEXIT_CRITICAL(&mux);
  if (!fresh) badPackets++;
}

// ---------------------------------------------------------------- control task
static void controlTask(void *) {
  esp_task_wdt_add(NULL);                       // if this task hangs, the chip resets; the ESC then sees no pulses
  WkGate gate = {WK_SRC_NONE, false};
  float slewed = 0, vbat = NAN;
  uint32_t lowSince = 0;
  bool limp = false;
  TickType_t wake = xTaskGetTickCount();
  uint32_t last = millis();
  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(10));
    esp_task_wdt_reset();
    if (VBAT_RATIO > 0) {                        // battery, debounced 2 s, latches until power-cycle
      float v = analogReadMilliVolts(PIN_VBAT) / 1000.0f * VBAT_RATIO;
      vbat = v < 1.0f ? NAN : (isnan(vbat) ? v : vbat * 0.98f + v * 0.02f);
      bool low = !isnan(vbat) && vbat / CELLS < CELL_LIMP_V;
      if (!low) lowSince = 0;
      else if (!lowSince) lowSince = millis();
      if (lowSince && millis() - lowSince > 2000) limp = true;
    }
    portENTER_CRITICAL(&mux);
    Cmd tx = txCmd, web = webCmd;
    WkCfg c = cfg;
    portEXIT_CRITICAL(&mux);
    uint32_t now = millis();                     // read AFTER the snapshot: ages are never negative
    float dt = (now - last) / 1000.0f;
    last = now;

    bool txLive = tx.ms && wk_age(now, tx.ms) < c.failsafeMs;
    bool webLive = web.ms && wk_age(now, web.ms) < c.failsafeMs && (int32_t)(web.ms - tx.ms) > 0;
    uint8_t src = txLive ? WK_SRC_TX : (webLive ? WK_SRC_WEB : WK_SRC_NONE);
    const Cmd &cmd = src == WK_SRC_TX ? tx : web;
    bool linkOk = src != WK_SRC_NONE;

    float cap = limp ? fminf(c.maxThrottle, LIMP_THROTTLE) : c.maxThrottle;
    float want = wk_gate(gate, src, linkOk, linkOk && cmd.deadman, linkOk ? cmd.thr : 0);
    if (want > 0) want *= cap;
    slewed = wk_slew(slewed, want, c.throttleSlew, dt);
    int steerUs = wk_steer_us(c, linkOk ? cmd.steer : 0);
    int escUs = wk_esc_us(c, bootNeutral, slewed);
    ledcWrite(PIN_STEER, usToDuty(steerUs));
    ledcWrite(PIN_ESC, usToDuty(escUs));
    digitalWrite(PIN_LED, linkOk ? (gate.armed ? HIGH : now / 500 % 2) : now / 150 % 2);

    portENTER_CRITICAL(&mux);
    bool moving = slewed != 0 || (linkOk && cmd.deadman);
    if (moving || !st.stillSinceMs) st.stillSinceMs = moving ? 0 : now;
    st.steerUs = steerUs; st.escUs = escUs; st.src = src; st.armed = gate.armed; st.moving = moving;
    st.limp = limp; st.vbat = vbat;
    if (linkOk) st.lastValidMs = now;
    portEXIT_CRITICAL(&mux);
  }
}

static Status status() {
  portENTER_CRITICAL(&mux);
  Status s = st;
  portEXIT_CRITICAL(&mux);
  return s;
}

// stopped for at least 1 s: the only state in which settings or OTA are accepted
static bool parked() {
  Status s = status();
  return !s.moving && s.stillSinceMs && wk_age(millis(), s.stillSinceMs) > 1000;
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
  if (ok) {
    portENTER_CRITICAL(&mux);
    cfg = c;
    portEXIT_CRITICAL(&mux);
  }
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
<div id="st" aria-live="polite">connecting</div>
<details><summary>Tuning (saved to the car; only while stopped)</summary><form id="f"></form>
<button id="save">Save</button><button id="def">Defaults</button></details>
<script>
const F=["steerCenter","steerLeft","steerRight","steerReverse","escNeutral","escFwdMax","escRevMax","throttleSlew","steerExpo","maxThrottle","failsafeMs"];
let s=0,t=0,held=0,n=Date.now()%2000000000;const pad=document.getElementById('pad'),dot=document.getElementById('dot');
function mv(e){const r=pad.getBoundingClientRect();s=Math.max(-1,Math.min(1,(e.clientX-r.left)/r.width*2-1));
t=Math.max(-1,Math.min(1,1-(e.clientY-r.top)/r.height*2));dot.style.left=(s+1)/2*r.width-28+'px';dot.style.top=(1-t)/2*r.height-28+'px'}
function up(){held=0;s=0;t=0;dot.style.left=dot.style.top='calc(50% - 28px)'}
pad.onpointerdown=e=>{held=1;pad.setPointerCapture(e.pointerId);mv(e)};pad.onpointermove=e=>{if(held)mv(e)};
pad.onpointerup=pad.onpointercancel=up;document.addEventListener('visibilitychange',up);window.addEventListener('blur',up);
async function tick(){try{const r=await fetch(`/drive?s=${Math.round(s*1000)}&t=${Math.round(t*1000)}&d=${held}&n=${++n}`);
const j=await r.json();document.getElementById('st').textContent=`src ${j.src} ${j.armed?'':'(release to arm)'}  steer ${j.steer_us}us  esc ${j.esc_us}us\nbatt ${j.vbat??'n/a'} V ${j.limp?'LIMP':''}  bad pkts ${j.bad}`}
catch(e){document.getElementById('st').textContent='no link'}setTimeout(tick,50)}tick();
async function load(){const c=await (await fetch('/cfg')).json();document.getElementById('f').innerHTML=F.map(k=>
`<label>${k}<input name="${k}" value="${c[k]}" inputmode="decimal"></label>`).join('')}load();
document.getElementById('save').onclick=async()=>{const q=new URLSearchParams(new FormData(document.getElementById('f')));
const r=await fetch('/cfg?'+q,{method:'POST'});alert(r.ok?'saved (escNeutral applies after a reboot)':'rejected: '+await r.text());load()};
document.getElementById('def').onclick=async()=>{const r=await fetch('/cfg/defaults',{method:'POST'});if(!r.ok)alert(await r.text());load()};
</script></html>)HTML";

static bool argInt(const char *k, long lo, long hi, long &out) {
  out = 0;
  if (!server.hasArg(k)) return false;
  String v = server.arg(k);
  char *end;
  out = strtol(v.c_str(), &end, 10);
  return v.length() && *end == 0 && out >= lo && out <= hi;
}

static bool argFloat(const char *k, float &out) {
  out = 0;
  if (!server.hasArg(k)) return false;
  String v = server.arg(k);
  char *end;
  out = strtof(v.c_str(), &end);
  return v.length() && *end == 0 && isfinite(out);
}

static void sendState() {
  Status s = status();
  char b[220], vb[12];
  if (isnan(s.vbat)) strcpy(vb, "null"); else snprintf(vb, sizeof vb, "%.2f", s.vbat);
  const char *src = s.src == WK_SRC_TX ? "tx" : s.src == WK_SRC_WEB ? "web" : "failsafe";
  snprintf(b, sizeof b, "{\"src\":\"%s\",\"armed\":%s,\"steer_us\":%d,\"esc_us\":%d,\"vbat\":%s,\"limp\":%s,\"bad\":%lu}",
           src, s.armed ? "true" : "false", s.steerUs, s.escUs, vb, s.limp ? "true" : "false",
           (unsigned long)badPackets);
  server.send(200, "application/json", b);
}

static void handleDrive() {
  long s, t, d, n = 0;
  if (!argInt("s", -1000, 1000, s) || !argInt("t", -1000, 1000, t) || !argInt("d", 0, 1, d)) {
    server.send(400, "text/plain", "need s,t in -1000..1000 and d in 0..1");
    return;
  }
  bool hasSeq = server.hasArg("n");
  if (hasSeq && !argInt("n", 0, 0x7FFFFFFF, n)) { server.send(400, "text/plain", "n must be a positive integer"); return; }
  uint32_t now = millis();
  portENTER_CRITICAL(&mux);
  // a request delivered late (after a newer one, or after the stop) must not resume driving
  bool fresh = !hasSeq || (uint32_t)n > webSeq || wk_age(now, webCmd.ms) > 1000;
  if (fresh) {
    if (hasSeq) webSeq = n;
    webCmd = {s / 1000.0f, t / 1000.0f, d == 1, now};
  }
  portEXIT_CRITICAL(&mux);
  if (!fresh) { server.send(409, "text/plain", "stale n"); return; }
  sendState();
}

static void handleCfgGet() {
  portENTER_CRITICAL(&mux);
  WkCfg c = cfg;
  portEXIT_CRITICAL(&mux);
  char b[320];
  snprintf(b, sizeof b,
           "{\"steerCenter\":%d,\"steerLeft\":%d,\"steerRight\":%d,\"steerReverse\":%d,\"escNeutral\":%d,"
           "\"escFwdMax\":%d,\"escRevMax\":%d,\"throttleSlew\":%.2f,\"steerExpo\":%.2f,\"maxThrottle\":%.2f,\"failsafeMs\":%u}",
           c.steerCenter, c.steerLeft, c.steerRight, c.steerReverse, c.escNeutral, c.escFwdMax,
           c.escRevMax, c.throttleSlew, c.steerExpo, c.maxThrottle, c.failsafeMs);
  server.send(200, "application/json", b);
}

static void handleCfgPost() {
  if (!parked()) { server.send(409, "text/plain", "stop the car first (no throttle, dead-man released, 1 s)"); return; }
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
  ok &= argInt("failsafeMs", 100, 500, v); c.failsafeMs = v;
  if (!ok || !saveCfg(c)) { server.send(400, "text/plain", "out of range: see wk_cfg_valid in WkLink.h"); return; }
  handleCfgGet();
}

// ---------------------------------------------------------------- setup / loop
static void park(int blinkMs, const char *why) {
  Serial.println(why);
  while (true) {
    ledcWrite(PIN_STEER, usToDuty(cfg.steerCenter));
    ledcWrite(PIN_ESC, usToDuty(bootNeutral));
    digitalWrite(PIN_LED, millis() / blinkMs % 2);
    delay(10);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  loadCfg();                                    // before the first pulse: arm on the saved neutral
  bootNeutral = cfg.escNeutral;
  if (!ledcAttach(PIN_STEER, 50, 16) || !ledcAttach(PIN_ESC, 50, 16)) {
    Serial.println("ledcAttach failed: no pulses will be sent");
    while (true) delay(1000);
  }
  ledcWrite(PIN_STEER, usToDuty(cfg.steerCenter));
  ledcWrite(PIN_ESC, usToDuty(bootNeutral));

  int bad = wk_selftest(cfg);
  Serial.printf("selftest: %s (%d failed)\n", bad ? "FAIL" : "pass", bad);
  if (bad) park(60, "selftest failed: car parked at neutral");
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
    server.on("/cfg/defaults", HTTP_POST, [] {
      if (!parked()) { server.send(409, "text/plain", "stop the car first"); return; }
      saveCfg(WK_DEFAULTS);
      handleCfgGet();
    });
    server.begin();
    ArduinoOTA.setHostname(ssid);
    ArduinoOTA.setPassword(AP_PASS);
    ArduinoOTA.begin();
    Serial.printf("Wi-Fi %s up, open http://192.168.4.1\n", ssid);
  } else {
    Serial.println("AP off: set AP_PASS (8+ chars) to enable phone/API control");
  }
  if (esp_now_init() != ESP_OK) park(100, "esp_now_init failed: car parked at neutral");
  esp_now_register_recv_cb(onRecv);
  Serial.print("car STA MAC (paste into tx.ino CAR_MAC): ");
  Serial.println(WiFi.macAddress());
  if (!macBound()) Serial.println("TX_MAC unbound: the handheld transmitter is ignored until you set it");

  uint32_t t0 = millis();                       // ESC arming window: neutral only
  while (millis() - t0 < ARM_MS) { digitalWrite(PIN_LED, millis() / 500 % 2); delay(10); }
  xTaskCreatePinnedToCore(controlTask, "control", 4096, nullptr, 10, nullptr, 1);
}

void loop() {
  if (apOn) {
    server.handleClient();
    Status s = status();
    if (wk_age(millis(), s.lastValidMs) > OTA_IDLE_MS && parked()) ArduinoOTA.handle();
  }
  static uint32_t lastLog = 0;
  if (millis() - lastLog > 500) {
    lastLog = millis();
    Status s = status();
    Serial.printf("src=%d steer=%dus esc=%dus vbat=%.2f limp=%d bad=%lu\n", s.src, s.steerUs, s.escUs, s.vbat, s.limp,
                  (unsigned long)badPackets);
  }
  delay(2);
}
