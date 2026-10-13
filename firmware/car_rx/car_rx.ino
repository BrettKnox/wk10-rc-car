// WK-10 car brain, rev F. Board: "ESP32 Dev Module", Arduino-ESP32 core 3.x.
// Library: WebSockets by Markus Sattler 2.7.2 (arduino-cli lib install WebSockets@2.7.2).
// Power: ESC BEC (6 V) -> dev board VIN/5V pin. Never the 2S pack directly.
//
// The hobby 2.4 GHz radio is the primary control link. A 74HCT157 mux feeds the servo and ESC
// from the receiver (S low) or from GPIO18/19 (S high). S is GPIO25 with a 10k pull-down, so a
// dead, resetting or booting ESP32 leaves the receiver in charge. wk_control_step() (WkLink.h)
// raises S only when receiver CH3 reads autonomy, CH1/CH2 are fresh, the control loop is on
// time, the battery checks pass and an ESP32 command source is armed and fresh.
//
// ESP32 command sources, used only while S is high:
//   1. Phone: join Wi-Fi "WK10-xxxx", open http://192.168.4.1 (WebSocket on port 81, 50 Hz).
//   2. Code: UDP port 4210, same frames, one state reply per frame (tools/drive.py).
//      Phone and code share one lock: the first token holds it until 1 s of silence.
//   3. Legacy ESP-NOW transmitter (firmware/tx). Wins over the web whenever its link is alive.
// Drive frame (text): "<token> <seq> <steer -1000..1000> <throttle -1000..1000> <deadman 0|1>".
// HTTP: GET /state (telemetry), GET/POST /cfg, POST /cfg/defaults.
//
// Structure: controlTask (priority 10, core 1, 100 Hz) is the ONLY writer of the servo and ESC
// pulses and of S. loop() runs the web server, WebSocket, UDP and OTA, which can block for
// seconds on a bad TCP connection; they only post commands under a spinlock.
//
// Safety, in order:
//   - Control task stalled 300 ms: the watchdog ISR drives S low and cuts GPIO19, then resets.
//     If the task cannot join the watchdog at all, it never raises S.
//   - Receiver lost, battery latch, 5 bad sensor readings, late loop, OTA, flash write: S low at once.
//   - CH3 off, brake end, command lost: S low at the next gap where all four signals are low.
//   - ESP32 command lost while S is high: brake (only if the ESC would brake, wk_brake_ok) for brakeMs, then S low.
//   - S rises only at neutral throttle after 500 ms of every check passing.
//   - Throttle needs the dead-man RELEASED then held on the current source (wk_gate).
//   - The control maths self-tests at boot against the config in use; a failure parks the car.
//   - Settings and OTA are refused unless the ESC has seen neutral for 3 s.
//   - The access point stays off until AP_PASS is changed from the placeholder.
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_task_wdt.h>
#include <esp_rom_gpio.h>
#include <driver/gpio.h>
#include <soc/gpio_sig_map.h>
#include <soc/gpio_struct.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Preferences.h>
#include <ArduinoOTA.h>
#include <algorithm>
#include <WkLink.h>   // firmware/libraries/WkLink

// ---- pins. Outputs are LEDC/GPIO-capable, none is a strapping pin (0, 2, 5, 12, 15) or flash (6..11) ----
const int PIN_STEER = 18;          // -> 74HCT157 1I1 (pin 3)
const int PIN_ESC   = 19;          // -> 74HCT157 2I1 (pin 6)
const int PIN_MUX   = 25;          // -> 74HCT157 S (pin 1), 10k to GND. HIGH = ESP32 drives.
const int PIN_RX_STEER = 32;       // receiver CH1 via buffer or divider (CHANGES-revF.md), never direct above 3.3 V
const int PIN_RX_THR   = 33;       // receiver CH2
const int PIN_RX_MODE  = 26;       // receiver CH3, 2-position switch
const int PIN_LED   = 23;          // external LED + 330R to GND (the DevKitC has no user LED)
const int PIN_BUZZ  = 22;          // passive piezo + 100R to GND
const int PIN_VBAT  = 34;          // ADC1, input only. Pack + -> 30k -> pin -> 10k -> GND, 100 nF at the pin.
const int PIN_VBEC  = 35;          // ADC1, input only. BEC + -> 20k -> pin -> 10k -> GND, 100 nF at the pin.

// ---- knobs you set once ----
const char *AP_PASS = "CHANGE-ME";  // 8+ chars. Also the OTA password. AP stays off until changed.
uint8_t TX_MAC[6] = {0, 0, 0, 0, 0, 0};  // legacy transmitter's MAC; all zero = ignore every transmitter
const int CELLS = 2;
const uint16_t UDP_PORT = 4210, WS_PORT = 81;
const uint32_t OTA_IDLE_MS = 5000; // OTA only after this long with no valid command
const uint32_t PARKED_MS = 3000;   // settings/OTA only after the ESC has seen neutral this long (no wheel sensor)
const uint32_t STATE_PUSH_MS = 200;// WebSocket clients get the state this often
const int BUZZ_HZ = 2700;          // piezo resonance; most 12 mm passive discs sit at 2.3..4 kHz
const uint32_t WDT_DRIVE_MS = 300, WDT_FLASH_MS = 5000;   // flash erases starve core 1; car is parked then

static WkCmd txCmd = {0, 0, false, 0}, webCmd = {0, 0, false, 0};
static uint16_t txSeq = 0;
static WkLock webLock = {0, 0, 0};
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;   // guards txCmd, webCmd, webLock, link stats, cfg, st
static volatile bool otaActive = false;  // set when an upload starts: S low, commands ignored
static volatile bool flashBusy = false;  // settings save in progress: S low

WkCfg cfg = WK_DEFAULTS;
Preferences prefs;
WebServer server(80);
WebSocketsServer ws(WS_PORT);
WiFiUDP udp;
bool apOn = false;

// receiver pulse capture: one CHANGE interrupt per channel, decoded by wk_edge()
struct RxCh { int pin; WkEdge e; uint32_t ms; };
static RxCh rxCh[3] = {{PIN_RX_STEER, {}, 0}, {PIN_RX_THR, {}, 0}, {PIN_RX_MODE, {}, 0}};
static portMUX_TYPE rxMux = portMUX_INITIALIZER_UNLOCKED;   // guards rxCh, muxWant, muxOut, muxWantMs
static bool muxWant = false, muxOut = false;   // S wanted by the control step, S on the pin
static uint32_t muxWantMs = 0;

// web link stats: inter-arrival gaps of the lock holder's frames
static uint16_t gaps[128];
static uint8_t gapN = 0, gapI = 0;
static uint32_t frames = 0, lostFrames = 0, rejected = 0;

// status written by the control task, read by the web handlers
struct Status { WkOut o; bool mux, armed, held; float vbat, vbec, rested; uint16_t rxUs[3]; uint32_t lastValidMs, quietSinceMs; };
static Status st = {};
static volatile uint32_t badPackets = 0;

static uint32_t usToDuty(int us) { return (uint32_t)us * 65535UL / 20000UL; }  // 16-bit at 50 Hz

// Runs inside the task-watchdog ISR when controlTask has stalled, before the panic handler
// spends time writing a core dump. S low hands the car to the receiver; GPIO19 is also
// detached from LEDC and held low so the ESP32 side carries no frozen throttle either.
extern "C" void IRAM_ATTR esp_task_wdt_isr_user_handler(void) {
  esp_rom_gpio_connect_out_signal(PIN_ESC, SIG_GPIO_OUT_IDX, false, false);
  GPIO.out_w1tc = (1UL << PIN_ESC) | (1UL << PIN_MUX);
}

static inline bool pinHigh(int p) { return p < 32 ? (GPIO.in >> p) & 1 : (GPIO.in1.data >> (p - 32)) & 1; }

static inline void setMux(bool v) {             // call with rxMux held
  if (v) GPIO.out_w1ts = 1UL << PIN_MUX; else GPIO.out_w1tc = 1UL << PIN_MUX;
  muxOut = v;
}

// The CH2 falling edge is where a deferred S change lands: CH2 has just ended, and CH1 and both
// ESP32 pulses must read low too, so no pulse is cut or merged. Pads 18/19 have input enabled
// so their LEDC level reads back through GPIO.in.
static void ARDUINO_ISR_ATTR rxEdge(void *arg) {
  RxCh *c = (RxCh *)arg;
  uint32_t t = micros();
  bool high = pinHigh(c->pin);
  portENTER_CRITICAL_ISR(&rxMux);
  if (wk_edge(c->e, high, t)) c->ms = millis();
  if (!high && c == &rxCh[WK_RX_THR] && muxOut != muxWant &&
      !pinHigh(PIN_RX_STEER) && !pinHigh(PIN_STEER) && !pinHigh(PIN_ESC)) setMux(muxWant);
  portEXIT_CRITICAL_ISR(&rxMux);
}

static bool setWdt(uint32_t ms) {
  esp_task_wdt_config_t c = {ms, 0, true};      // subscribed tasks only, panic -> reset
  return esp_task_wdt_reconfigure(&c) == ESP_OK;
}

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
    badPackets = badPackets + 1;
    return;
  }
  if (len != sizeof(WkPacket)) { badPackets = badPackets + 1; return; }
  WkPacket p;
  memcpy(&p, data, sizeof p);
  if (p.magic != WK_MAGIC || p.sum != wk_sum(p)) { badPackets = badPackets + 1; return; }
  uint32_t now = millis();
  portENTER_CRITICAL(&mux);
  bool fresh = wk_age(now, txCmd.ms) > 1000 || (int16_t)(p.seq - txSeq) > 0;   // replay/stale guard
  if (fresh) {
    txSeq = p.seq;
    txCmd = {wk_clampf(p.steer / 1000.0f, -1, 1), wk_clampf(p.throttle / 1000.0f, -1, 1), (p.flags & 1) != 0, now};
  }
  portEXIT_CRITICAL(&mux);
  if (!fresh) badPackets = badPackets + 1;
}

// Volts at the source, or NaN when the one conversion sits at a rail (open divider, or above
// the ADC's calibrated range).
static float readV(int pin, float ratio) {
  int mv = analogReadMilliVolts(pin);
  if (mv < 50 || mv > 3000) return NAN;
  return mv / 1000.0f * ratio;
}

// Filter good samples. A bad sample is NaN for this step but leaves the filter alone, so
// wk_control_step() can count bad samples in a row.
static float sample(float &filtered, float v) {
  if (isnan(v)) return NAN;
  filtered = isnan(filtered) ? v : filtered * 0.98f + v * 0.02f;
  return filtered;
}

// ---------------------------------------------------------------- control task
static void controlTask(void *) {
  if (esp_task_wdt_add(NULL) != ESP_OK) {        // hang > 300 ms drops S, cuts GPIO19 and resets the chip
    Serial.println("control task not on the watchdog: ESP32 parked, the radio drives");
    for (;;) {                                   // never raise S without the watchdog behind it
      portENTER_CRITICAL(&rxMux);
      muxWant = false; setMux(false);
      portEXIT_CRITICAL(&rxMux);
      ledcWrite(PIN_ESC, usToDuty(cfg.escNeutral));
      digitalWrite(PIN_LED, millis() / 60 % 2);
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
  WkCtl ctl = WK_CTL_INIT;
  float vbat = NAN, vbec = NAN;
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(10));
    esp_task_wdt_reset();
    portENTER_CRITICAL(&mux);
    WkIn in = {txCmd, webCmd};
    WkCfg c = cfg;
    portEXIT_CRITICAL(&mux);
    in.cellV = sample(vbat, readV(PIN_VBAT, c.vbatRatio)) / CELLS;
    in.becV = sample(vbec, readV(PIN_VBEC, c.becRatio));
    in.inhibit = otaActive || flashBusy;
    portENTER_CRITICAL(&rxMux);
    for (int i = 0; i < 3; i++) { in.rxUs[i] = rxCh[i].e.us; in.rxMs[i] = rxCh[i].ms; }
    in.modePrevUs = rxCh[WK_RX_MODE].e.prevUs;
    in.muxNow = muxOut;
    portEXIT_CRITICAL(&rxMux);
    uint32_t now = millis();                     // read AFTER the snapshots: ages are never negative

    WkOut o = wk_control_step(ctl, c, in, now);
    ledcWrite(PIN_STEER, usToDuty(o.steerUs));
    ledcWrite(PIN_ESC, usToDuty(o.escUs));
    // S: a drop for a fault lands now; anything else waits for the CH2 ISR's common low gap,
    // forced after WK_SWITCH_MAX_MS (receiver frames slower than that, or signals never all low).
    portENTER_CRITICAL(&rxMux);
    if (o.select != muxWant) { muxWant = o.select; muxWantMs = now; }
    if (muxOut != muxWant && (o.urgent || wk_age(now, muxWantMs) >= WK_SWITCH_MAX_MS)) setMux(muxWant);
    bool muxNow = muxOut;
    portEXIT_CRITICAL(&rxMux);

    // LED: solid = ESP32 driving, flicker = ESP32 braking, slow blink = radio driving, fast = receiver or battery fault
    bool fault = !o.rxOk || (o.why & (WK_WHY_BATT_LOW | WK_WHY_SENSOR));
    digitalWrite(PIN_LED, muxNow ? (o.braking ? now / 50 % 2 : HIGH) : fault ? now / 125 % 2 : now / 500 % 2);
    // Buzzer: battery = 0.5 s beeps, link = fast beeps, engaged = chirps for 600 ms
    bool buzz = o.alarm == WK_ALARM_BATT ? now % 1000 < 500
              : o.alarm == WK_ALARM_LINK ? now % 200 < 100
              : o.alarm == WK_ALARM_ENGAGED ? (now - ctl.engagedMs) % 300 < 100 : false;
    ledcWrite(PIN_BUZZ, buzz ? 128 : 0);

    const WkCmd &cmd = o.src == WK_SRC_TX ? in.tx : in.web;
    portENTER_CRITICAL(&mux);
    st.o = o; st.mux = muxNow; st.armed = ctl.gate.armed; st.held = o.src != WK_SRC_NONE && cmd.deadman;
    st.vbat = vbat; st.vbec = vbec; st.rested = ctl.restedCellV * CELLS;
    for (int i = 0; i < 3; i++) st.rxUs[i] = wk_rx_fresh(in, i, now) ? in.rxUs[i] : 0;
    st.quietSinceMs = ctl.quietSinceMs;
    if (o.src != WK_SRC_NONE) st.lastValidMs = now;
    portEXIT_CRITICAL(&mux);
  }
}

static Status status() {
  portENTER_CRITICAL(&mux);
  Status s = st;
  portEXIT_CRITICAL(&mux);
  return s;
}

// The ESC has seen neutral (ours or the receiver's) for PARKED_MS and no dead-man is held: the
// only state in which settings or OTA are accepted. There is no wheel-speed sensor, so a
// coasting car counts as parked: stop it before you tune it.
static bool parked() {
  Status s = status();
  return !s.held && s.quietSinceMs && wk_age(millis(), s.quietSinceMs) > PARKED_MS;
}

// ---------------------------------------------------------------- web drive frames
// One frame from the WebSocket or UDP. Trust boundary: every field is range-checked.
static void takeFrame(const uint8_t *p, size_t len) {
  char b[64];
  unsigned long tok, n;
  long s, t, d;
  int used = 0;
  if (len == 0 || len >= sizeof b) { portENTER_CRITICAL(&mux); rejected++; portEXIT_CRITICAL(&mux); return; }
  memcpy(b, p, len);
  b[len] = 0;
  bool ok = sscanf(b, "%lu %lu %ld %ld %ld%n", &tok, &n, &s, &t, &d, &used) == 5 && used == (int)len &&
            s >= -1000 && s <= 1000 && t >= -1000 && t <= 1000 && (d == 0 || d == 1);
  uint32_t now = millis();
  portENTER_CRITICAL(&mux);
  WkLock was = webLock;
  ok = ok && wk_lock_accept(webLock, tok, n, now);
  if (ok) {
    webCmd = {s / 1000.0f, t / 1000.0f, d == 1, now};
    frames++;
    if (was.tok == tok && wk_age(now, was.ms) < WK_LOCK_MS) {   // same holder: link stats
      lostFrames += n - was.seq - 1;
      gaps[gapI] = (uint16_t)std::min<int32_t>(wk_age(now, was.ms), 65535);
      gapI = (gapI + 1) % 128;
      if (gapN < 128) gapN++;
    }
  } else {
    rejected++;
  }
  portEXIT_CRITICAL(&mux);
}

static void onWs(uint8_t, WStype_t type, uint8_t *payload, size_t len) {
  if (type == WStype_TEXT) takeFrame(payload, len);
}

static uint16_t p99() {
  uint16_t v[128];
  portENTER_CRITICAL(&mux);
  uint8_t n = gapN;
  memcpy(v, gaps, sizeof v);
  portEXIT_CRITICAL(&mux);
  if (!n) return 0;
  std::sort(v, v + n);
  return v[(n * 99 + 99) / 100 - 1];             // ceil(0.99 n)-th smallest
}

static void fmtV(char *b, size_t n, float v) { if (isnan(v)) strcpy(b, "null"); else snprintf(b, n, "%.2f", v); }

static int stateJson(char *b, size_t n) {
  Status s = status();
  portENTER_CRITICAL(&mux);
  uint32_t tok = webLock.tok, fr = frames, lost = lostFrames, rej = rejected;
  portEXIT_CRITICAL(&mux);
  char vb[12], vr[12], ve[12];
  fmtV(vb, sizeof vb, s.vbat); fmtV(vr, sizeof vr, s.rested); fmtV(ve, sizeof ve, s.vbec);
  const char *src = s.o.src == WK_SRC_TX ? "tx" : s.o.src == WK_SRC_WEB ? "web" : "none";
  static const char *AL[] = {"none", "engaged", "link", "batt"};
  return snprintf(b, n,
    "{\"src\":\"%s\",\"armed\":%s,\"mux\":%s,\"why\":%u,\"brake\":%s,\"alarm\":\"%s\",\"steer_us\":%d,\"esc_us\":%d,"
    "\"rx\":[%u,%u,%u],\"rx_ok\":%s,\"auto_sw\":%s,\"vbat\":%s,\"rested\":%s,\"bec\":%s,"
    "\"lock\":\"%08lx\",\"p99_ms\":%u,\"frames\":%lu,\"lost\":%lu,\"rejected\":%lu,\"bad\":%lu}",
    src, s.armed ? "true" : "false", s.mux ? "true" : "false", s.o.why, s.o.braking ? "true" : "false",
    AL[s.o.alarm & 3], s.o.steerUs, s.o.escUs, s.rxUs[0], s.rxUs[1], s.rxUs[2],
    s.o.rxOk ? "true" : "false", s.o.autoSw ? "true" : "false", vb, vr, ve,
    (unsigned long)tok, p99(), (unsigned long)fr, (unsigned long)lost, (unsigned long)rej, (unsigned long)badPackets);
}

static void sendState() {
  char b[512];
  stateJson(b, sizeof b);
  server.send(200, "application/json", b);
}

static void pollUdp() {
  for (int i = 0; i < 8; i++) {                 // bounded: never starve the rest of loop()
    int len = udp.parsePacket();
    if (len <= 0) return;
    uint8_t b[64];
    int got = udp.read(b, sizeof b);
    takeFrame(b, len == got ? got : sizeof b);  // oversize datagram: rejected
    char out[512];
    int m = stateJson(out, sizeof out);
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.write((const uint8_t *)out, m < (int)sizeof out ? m : sizeof out - 1);
    udp.endPacket();
  }
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
  flashBusy = true;                             // control task drops S before the erase stalls it
  delay(30);
  setWdt(WDT_FLASH_MS);                         // a sector erase can take hundreds of ms; car is parked
  prefs.begin("wk10", false);
  bool ok = prefs.putBytes("cfg", &c, sizeof c) == sizeof c;
  prefs.end();
  setWdt(WDT_DRIVE_MS);
  if (ok) {
    portENTER_CRITICAL(&mux);
    cfg = c;
    portEXIT_CRITICAL(&mux);
  }
  flashBusy = false;
  return ok;
}

// ---------------------------------------------------------------- web page
static const char PAGE[] PROGMEM = R"HTML(<!doctype html><html lang="en"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no"><title>WK-10</title>
<style>body{font:16px system-ui;margin:0;padding:12px;background:#111;color:#eee}
#pad{touch-action:none;width:100%;max-width:420px;aspect-ratio:1;background:#222;border-radius:12px;position:relative;margin:auto}
.k{position:absolute;width:56px;height:56px;border-radius:50%;transform:translate(-50%,-50%);display:none;pointer-events:none}
#org{border:2px solid #777}#dot{background:#3a8}
#brk{touch-action:none;user-select:none;max-width:420px;height:88px;margin:8px auto;background:#522;border-radius:12px;display:flex;align-items:center;justify-content:center;font-weight:600}
#brk.on{background:#d33}
label{display:flex;justify-content:space-between;gap:8px;margin:6px 0}input{width:7em;font-size:16px}
button{font-size:16px;min-height:44px;padding:0 16px;margin:4px}#st{font-family:monospace;white-space:pre-wrap}</style>
<h1 style="font-size:20px">WK-10</h1>
<div id="pad" role="application" aria-label="Drive pad: touch anywhere, slide up from there for throttle and sideways to steer"><div id="org" class="k"></div><div id="dot" class="k"></div></div>
<div id="brk" role="button" aria-label="Brake: hold to brake. Release and hold again after a stop to reverse">BRAKE / REVERSE</div>
<div id="st" aria-live="polite">connecting</div>
<details><summary>Tuning (saved to the car; only while stopped)</summary><form id="f"></form>
<button id="save">Save</button><button id="def">Defaults</button><div id="msg" aria-live="polite"></div></details>
<script>
const F=["steerCenter","steerLeft","steerRight","steerReverse","escNeutral","escFwdMax","escRevMax","throttleSlew","steerExpo","maxThrottle","failsafeMs","steerRate","steerLimitAt","steerLimitFrac","brakeFrac","brakeMs","escMode","vbatRatio","becRatio"];
const $=id=>document.getElementById(id),pad=$('pad'),org=$('org'),dot=$('dot'),brk=$('brk');
let tok=0;while(!tok)tok=crypto.getRandomValues(new Uint32Array(1))[0];
const me=tok.toString(16).padStart(8,'0');
let n=0,s=0,t=0,stick=null,brake=null,ox=0,oy=0,ws=null,heard=0;
const put=(k,x,y)=>{k.style.left=x+'px';k.style.top=y+'px';k.style.display='block'};
function mv(e){if(e.pointerId!==stick)return;const r=pad.getBoundingClientRect(),R=r.width*0.3,x=e.clientX-r.left,y=e.clientY-r.top;
s=Math.max(-1,Math.min(1,(x-ox)/R));t=Math.max(0,Math.min(1,(oy-y)/R));put(dot,ox+s*R,oy-t*R)}
pad.onpointerdown=e=>{if(stick!==null)return;stick=e.pointerId;pad.setPointerCapture(e.pointerId);const r=pad.getBoundingClientRect();
ox=e.clientX-r.left;oy=e.clientY-r.top;put(org,ox,oy);mv(e)};
pad.onpointermove=mv;
pad.onpointerup=pad.onpointercancel=e=>{if(e.pointerId!==stick)return;stick=null;s=t=0;org.style.display=dot.style.display='none'};
brk.onpointerdown=e=>{brake=e.pointerId;brk.setPointerCapture(e.pointerId);brk.classList.add('on')};
brk.onpointerup=brk.onpointercancel=e=>{if(e.pointerId===brake){brake=null;brk.classList.remove('on')}};
function all(){stick=brake=null;s=t=0;org.style.display=dot.style.display='none';brk.classList.remove('on')}
document.addEventListener('visibilitychange',all);window.addEventListener('blur',all);
function show(j){$('st').textContent=`${j.lock!==me&&j.lock!=='00000000'?'LOCKED: another client is driving\n':''}${j.mux?'ESP32 DRIVING':'radio driving'}${j.brake?' BRAKING':''}  src ${j.src}${j.armed?'':' (release to arm)'}  why ${j.why}
rx ${j.rx.join(' ')} us  autonomy switch ${j.auto_sw?'on':'off'}  alarm ${j.alarm}
pack ${j.vbat??'n/a'} V (rested ${j.rested??'n/a'})  BEC ${j.bec??'n/a'} V
link p99 ${j.p99_ms} ms  lost ${j.lost}  rejected ${j.rejected}`}
function open(){ws=new WebSocket(`ws://${location.hostname}:81/`);ws.onmessage=e=>{heard=Date.now();show(JSON.parse(e.data))};
ws.onclose=()=>{ws=null;setTimeout(open,500)}}
open();
setInterval(()=>{if(Date.now()-heard>1000)$('st').textContent='no link';
if(!ws||ws.readyState!==1||ws.bufferedAmount>64)return;   // fire and forget: never queue stale frames
const held=stick!==null||brake!==null,th=brake!==null?-1:t;
ws.send(`${tok} ${++n} ${Math.round(s*1000)} ${Math.round(th*1000)} ${held?1:0}`)},20);
async function load(){const c=await (await fetch('/cfg')).json();$('f').innerHTML=F.map(k=>
`<label>${k}<input name="${k}" value="${c[k]}" inputmode="decimal"></label>`).join('')}load();
const say=m=>$('msg').textContent=m;
$('save').onclick=async()=>{const q=new URLSearchParams(new FormData($('f')));
const r=await fetch('/cfg?'+q,{method:'POST'});say(r.ok?'saved':'rejected: '+await r.text());load()};
$('def').onclick=async()=>{const r=await fetch('/cfg/defaults',{method:'POST'});say(r.ok?'defaults restored':await r.text());load()};
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

static void handleCfgGet() {
  portENTER_CRITICAL(&mux);
  WkCfg c = cfg;
  portEXIT_CRITICAL(&mux);
  char b[560];
  snprintf(b, sizeof b,
           "{\"steerCenter\":%d,\"steerLeft\":%d,\"steerRight\":%d,\"steerReverse\":%d,\"escNeutral\":%d,"
           "\"escFwdMax\":%d,\"escRevMax\":%d,\"throttleSlew\":%.2f,\"steerExpo\":%.2f,\"maxThrottle\":%.2f,\"failsafeMs\":%u,"
           "\"steerRate\":%.2f,\"steerLimitAt\":%.2f,\"steerLimitFrac\":%.2f,\"brakeFrac\":%.2f,\"brakeMs\":%u,"
           "\"escMode\":%d,\"vbatRatio\":%.4f,\"becRatio\":%.4f}",
           c.steerCenter, c.steerLeft, c.steerRight, c.steerReverse, c.escNeutral, c.escFwdMax,
           c.escRevMax, c.throttleSlew, c.steerExpo, c.maxThrottle, c.failsafeMs,
           c.steerRate, c.steerLimitAt, c.steerLimitFrac, c.brakeFrac, c.brakeMs, c.escMode, c.vbatRatio, c.becRatio);
  server.send(200, "application/json", b);
}

static void handleCfgPost() {
  if (!parked()) { server.send(409, "text/plain", "stop the car first (ESC at neutral, dead-man released, 3 s)"); return; }
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
  ok &= argFloat("steerRate", f); c.steerRate = f;
  ok &= argFloat("steerLimitAt", f); c.steerLimitAt = f;
  ok &= argFloat("steerLimitFrac", f); c.steerLimitFrac = f;
  ok &= argFloat("brakeFrac", f); c.brakeFrac = f;
  ok &= argInt("brakeMs", 0, 3000, v); c.brakeMs = v;
  ok &= argInt("escMode", 0, 2, v); c.escMode = v;
  ok &= argFloat("vbatRatio", f); c.vbatRatio = f;
  ok &= argFloat("becRatio", f); c.becRatio = f;
  if (!ok || !saveCfg(c)) { server.send(400, "text/plain", "out of range: see wk_cfg_valid in WkLink.h"); return; }
  handleCfgGet();
}

// ---------------------------------------------------------------- setup / loop
static void park(int blinkMs, const char *why) {
  Serial.println(why);
  while (true) {
    portENTER_CRITICAL(&rxMux);
    muxWant = false; setMux(false);
    portEXIT_CRITICAL(&rxMux);
    ledcWrite(PIN_STEER, usToDuty(cfg.steerCenter));
    ledcWrite(PIN_ESC, usToDuty(cfg.escNeutral));
    digitalWrite(PIN_LED, millis() / blinkMs % 2);
    delay(10);
  }
}

void setup() {
  digitalWrite(PIN_MUX, LOW);                   // the receiver has the car from the first instruction
  pinMode(PIN_MUX, OUTPUT);
  digitalWrite(PIN_MUX, LOW);
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  loadCfg();
  if (!ledcAttach(PIN_STEER, 50, 16) || !ledcAttach(PIN_ESC, 50, 16) || !ledcAttach(PIN_BUZZ, BUZZ_HZ, 8)) {
    Serial.println("ledcAttach failed: no pulses will be sent");
    while (true) delay(1000);                   // S stays low: the radio drives
  }
  ledcWrite(PIN_STEER, usToDuty(cfg.steerCenter));
  ledcWrite(PIN_ESC, usToDuty(cfg.escNeutral));
  ledcWrite(PIN_BUZZ, 0);
  gpio_input_enable((gpio_num_t)PIN_STEER);     // read the LEDC outputs back in the CH2 ISR
  gpio_input_enable((gpio_num_t)PIN_ESC);

  int bad = wk_selftest(cfg);
  Serial.printf("selftest: %s (%d failed)\n", bad ? "FAIL" : "pass", bad);
  if (bad) park(60, "selftest failed: ESP32 parked, the radio drives");
  analogSetPinAttenuation(PIN_VBAT, ADC_11db);
  analogSetPinAttenuation(PIN_VBEC, ADC_11db);
  for (RxCh &c : rxCh) {                        // the buffer or divider bottom resistor holds an unplugged lead low
    pinMode(c.pin, INPUT);
    attachInterruptArg(c.pin, rxEdge, &c, CHANGE);
  }

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
    server.on("/state", sendState);
    server.on("/cfg", HTTP_GET, handleCfgGet);
    server.on("/cfg", HTTP_POST, handleCfgPost);
    server.on("/cfg/defaults", HTTP_POST, [] {
      if (!parked()) { server.send(409, "text/plain", "stop the car first"); return; }
      saveCfg(WK_DEFAULTS);
      handleCfgGet();
    });
    server.begin();
    ws.begin();
    ws.onEvent(onWs);
    ws.enableHeartbeat(1000, 500, 2);           // drop a phone that walked out of range so it can reconnect
    udp.begin(UDP_PORT);
    ArduinoOTA.setHostname(ssid);
    ArduinoOTA.setPassword(AP_PASS);
    ArduinoOTA.onStart([] { otaActive = true; setWdt(WDT_FLASH_MS); });   // S low; erases may starve core 1
    ArduinoOTA.onError([](ota_error_t) { otaActive = false; setWdt(WDT_DRIVE_MS); });
    ArduinoOTA.begin();
    Serial.printf("Wi-Fi %s up, open http://192.168.4.1 (UDP %u, WebSocket %u)\n", ssid, UDP_PORT, WS_PORT);
  } else {
    Serial.println("AP off: set AP_PASS (8+ chars) to enable phone/API control");
  }
  if (esp_now_init() != ESP_OK) park(100, "esp_now_init failed: ESP32 parked, the radio drives");
  esp_now_register_recv_cb(onRecv);
  Serial.print("car STA MAC (paste into tx.ino CAR_MAC): ");
  Serial.println(WiFi.macAddress());
  if (!macBound()) Serial.println("TX_MAC unbound: the legacy transmitter is ignored until you set it");

  if (!setWdt(WDT_DRIVE_MS)) park(200, "watchdog reconfigure failed: ESP32 parked, the radio drives");
  xTaskCreatePinnedToCore(controlTask, "control", 4096, nullptr, 10, nullptr, 1);
}

void loop() {
  if (apOn) {
    server.handleClient();
    ws.loop();
    pollUdp();
    static uint32_t lastPush = 0;
    if (millis() - lastPush >= STATE_PUSH_MS) {
      lastPush = millis();
      char b[512];
      stateJson(b, sizeof b);
      ws.broadcastTXT(b);
    }
    Status s = status();
    if (wk_age(millis(), s.lastValidMs) > OTA_IDLE_MS && parked()) ArduinoOTA.handle();
  }
  static uint32_t lastLog = 0;
  if (millis() - lastLog > 500) {
    lastLog = millis();
    Status s = status();
    Serial.printf("mux=%d why=%u src=%d rx=%u/%u/%u steer=%dus esc=%dus vbat=%.2f bec=%.2f alarm=%u bad=%lu\n",
                  s.mux, s.o.why, s.o.src, s.rxUs[0], s.rxUs[1], s.rxUs[2], s.o.steerUs, s.o.escUs,
                  s.vbat, s.vbec, s.o.alarm, (unsigned long)badPackets);
  }
  delay(2);
}
