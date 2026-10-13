// Runs the same wk_selftest() the car runs at boot, on the PC, then drives wk_control_step()
// on a simulated clock through the mux and failsafe scenarios.
// python -m ziglang c++ -std=c++17 -Wall -Wextra -I firmware/libraries/WkLink firmware/test/host_selftest.cpp -o build/selftest.exe
#include <cstdio>
#include "WkLink.h"

static int fails = 0;
#define CHECK(cond, name) do { if (!(cond)) { fails++; std::printf("FAIL %s: %s (line %d)\n", name, #cond, __LINE__); } } while (0)

// A car on the bench: receiver on (a frame every rxPeriod ms), CH3 on autonomy, healthy 2S pack
// and 6 V BEC, phone page open. Steps jitter around 10 ms like a loaded RTOS tick. S as applied
// lags the wanted S like the firmware: urgent drops at once, other changes at the next receiver
// frame (the common low gap), or after WK_SWITCH_MAX_MS.
struct Sim {
  WkCtl s = WK_CTL_INIT;
  WkCfg c = WK_DEFAULTS;
  WkIn in = {};
  WkOut o = {};
  uint32_t t = 1000, nextFrame = 1000, wantSince = 0;
  int rxPeriod = 20, jitter = 0;
  bool rxOn = true, modeOn = true, webOn = true;
  uint16_t steerUs = 1500, thrUs = 1500, modeUs = 2000;
  float cellV = 4.0f, becV = 6.0f;
  int badSamples = 0;                         // the next N steps read NaN on the pack
  int seenMin = 9999, seenMax = 0;            // ESP32 throttle pulses actually applied to the ESC
  bool ambiguousSent = false;                 // an applied ESP32 pulse inside the neutral band but not neutral
  Sim() { web(0, 0, false); }
  void web(float steer, float thr, bool d) { in.web.steer = steer; in.web.thr = thr; in.web.deadman = d; }
  void step(int count = 1) {
    static const int J[] = {10, 7, 13, 10, 12, 8, 10, 11, 9};
    for (int i = 0; i < count; i++) {
      t += jitter ? J[(jitter++) % 9] : 10;
      bool frame = false;
      while ((int32_t)(t - nextFrame) >= 0) {   // receiver frames that arrived since the last step
        nextFrame += rxPeriod;
        frame = true;
        if (!rxOn) continue;
        in.rxUs[WK_RX_STEER] = steerUs; in.rxUs[WK_RX_THR] = thrUs;
        in.rxMs[WK_RX_STEER] = in.rxMs[WK_RX_THR] = nextFrame - rxPeriod;
        if (modeOn) { in.modePrevUs = in.rxUs[WK_RX_MODE]; in.rxUs[WK_RX_MODE] = modeUs; in.rxMs[WK_RX_MODE] = nextFrame - rxPeriod; }
      }
      if (webOn) in.web.ms = t;
      in.cellV = badSamples > 0 ? NAN : cellV;
      if (badSamples > 0) badSamples--;
      in.becV = becV;
      o = wk_control_step(s, c, in, t);
      if (o.select != in.muxNow) {
        if (!wantSince) wantSince = t;
        if (o.urgent || (frame && rxOn) || wk_age(t, wantSince) >= WK_SWITCH_MAX_MS) { in.muxNow = o.select; wantSince = 0; }
      } else {
        wantSince = 0;
      }
      if (in.muxNow) {
        if (o.escUs < seenMin) seenMin = o.escUs;
        if (o.escUs > seenMax) seenMax = o.escUs;
        if (o.escUs != c.escNeutral && o.escUs >= c.escNeutral - WK_ESC_BAND_US && o.escUs <= c.escNeutral + WK_ESC_BAND_US &&
            o.escUs < c.escNeutral) ambiguousSent = true;
      }
    }
  }
  void ms(int m) { uint32_t end = t + m; while ((int32_t)(end - t) > 0) step(); }
  bool engage() { ms(3000); return o.select && in.muxNow; }   // 2 s rest + 0.5 s debounce, with margin
  void loseWeb() { webOn = false; ms(c.failsafeMs + 20); }
};

static void t_engage() {
  Sim m;
  m.ms(1000);
  CHECK(!m.o.select && (m.o.why & WK_WHY_REST), "engage waits for a rested pack reading");
  CHECK(m.engage(), "engages when every check passes");
  CHECK(m.o.why == 0 && m.o.src == WK_SRC_WEB, "engaged state is clean");
  CHECK(m.o.alarm == WK_ALARM_NONE, "engaged chirp is over after 600 ms");
  Sim q;                                        // the debounce: S does not rise the moment checks pass
  q.ms(2000);
  int firstUp = -1;
  for (int i = 0; i < 200 && firstUp < 0; i++) { q.step(); if (q.o.select) firstUp = i; }
  CHECK(firstUp >= 0 && q.o.alarm == WK_ALARM_ENGAGED && !q.o.urgent, "engaged chirp plays when S rises; engage waits for a gap");
  Sim h;                                        // dead-man held with throttle while the switch flips: no engage
  h.modeUs = 1000; h.ms(2500);
  h.web(0, 0.5f, true); h.ms(100);
  h.modeUs = 2000; h.ms(3000);
  CHECK(!h.o.select && (h.o.why & WK_WHY_THR), "never engages with throttle held");
  Sim j; j.jitter = 1; j.rxPeriod = 14;         // jittered steps and a 14 ms receiver
  CHECK(j.engage(), "engages with jittered steps and a 14 ms receiver frame");
}

static void t_receiver_failsafe() {
  Sim m;
  CHECK(m.engage(), "rx: engaged first");
  m.rxOn = false;                               // receiver in failsafe with no pulses, or unplugged
  m.ms(WK_RX_FRESH_MS + 30);
  CHECK(!m.o.select && !m.in.muxNow && (m.o.why & WK_WHY_RX), "rx: lost receiver forces manual");
  CHECK(m.o.alarm == WK_ALARM_LINK, "rx: lost receiver alarms");
  m.rxOn = true; m.ms(5000);
  CHECK(m.o.select, "rx: re-engages once the receiver is back");
  Sim f;                                        // ageing: 60 ms frames stay fresh, 120 ms frames go stale between pulses
  f.rxPeriod = 60;
  CHECK(f.engage(), "rx: a 60 ms frame period still counts as fresh");
  Sim g; g.rxPeriod = 120;
  bool everUp = false, everStale = false;
  for (int i = 0; i < 500; i++) { g.step(); everUp |= g.o.select; everStale |= (g.o.why & WK_WHY_RX) != 0; }
  CHECK(!everUp && everStale, "rx: pulses 120 ms apart go stale and never engage");
  Sim n;
  n.rxOn = false; n.ms(5000);
  CHECK(!n.o.select && n.o.alarm == WK_ALARM_NONE, "rx: never engages without a receiver, quiet on a USB bench");
  Sim b;
  b.steerUs = 2500; b.ms(5000);                 // out-of-range steering pulses are not a live receiver
  CHECK(!b.o.select && (b.o.why & WK_WHY_RX), "rx: bad CH1 width blocks");
  Sim u;
  CHECK(u.engage(), "rx: engaged (urgent case)");
  u.rxOn = false;
  bool urgentSeen = false;
  for (int i = 0; i < 20; i++) { u.step(); if (!u.o.select) { urgentSeen = u.o.urgent; break; } }
  CHECK(urgentSeen, "rx: receiver loss drops S urgently");
}

static void t_ch3() {
  Sim m;
  CHECK(m.engage(), "ch3: engaged first");
  m.modeUs = 1000; m.ms(30);                    // driver flips to manual (or receiver failsafe sets CH3 manual)
  CHECK(!m.o.select && (m.o.why & WK_WHY_MODE) && !m.o.urgent, "ch3: off wants manual at the next frame");
  CHECK(!m.in.muxNow, "ch3: S applied within one frame");
  const uint16_t notAuto[] = {1799, 2201, 1500, 0};
  for (uint16_t us : notAuto) {
    Sim k; k.modeUs = us; k.ms(4000);
    CHECK(!k.o.select, "ch3: outside 1800..2200 never engages");
  }
  Sim e; e.modeUs = 1800; CHECK(e.engage(), "ch3: 1800 is autonomy");
  Sim f; f.modeUs = 2200; CHECK(f.engage(), "ch3: 2200 is autonomy");
  Sim g;
  CHECK(g.engage(), "ch3: engaged before the CH3 lead fails");
  g.modeOn = false;                             // CH3 lead falls off, CH1/CH2 still fine
  g.ms(WK_RX_FRESH_MS + 30);
  CHECK(!g.o.select && (g.o.why & WK_WHY_MODE), "ch3: stale CH3 forces manual");
  Sim one;                                      // one autonomy pulse between manual ones is not enough
  one.modeUs = 1000; one.ms(3000);
  one.in.modePrevUs = 1000; one.in.rxUs[WK_RX_MODE] = 2000; one.in.rxMs[WK_RX_MODE] = one.t;
  one.modeOn = false; one.step();
  CHECK(!one.o.autoSw, "ch3: a single autonomy pulse does not count");
}

static void t_brake_after_forward() {
  Sim m;
  CHECK(m.engage(), "brake: engaged first");
  m.web(0, 0.3f, true); m.ms(1000);            // partial forward
  CHECK(m.o.escUs > m.c.escNeutral + WK_ESC_BAND_US, "brake: driving forward");
  m.loseWeb();                                  // phone Wi-Fi drops
  int brakeUs = wk_esc_us(m.c, m.c.escNeutral, -m.c.brakeFrac);
  CHECK(m.o.select && m.o.braking && m.o.escUs == brakeUs, "brake: stale command brakes with S high");
  CHECK(brakeUs < m.c.escNeutral - WK_ESC_BAND_US, "brake: default brake pulse is below the neutral band");
  CHECK(m.o.alarm == WK_ALARM_LINK, "brake: link alarm while braking");
  CHECK(m.s.quietSinceMs == 0, "brake: braking never counts as quiet (no settings or OTA)");
  m.ms(m.c.brakeMs - 300);
  CHECK(m.o.select && m.o.escUs == brakeUs, "brake: still braking before brakeMs");
  m.ms(400);
  CHECK(!m.o.select && !m.o.braking && m.o.escUs == m.c.escNeutral && !m.in.muxNow, "brake: then neutral and the receiver");
  Sim r;                                        // brake cut short by the driver flipping CH3
  CHECK(r.engage(), "brake: engaged (CH3 case)");
  r.web(0, 1, true); r.ms(500); r.loseWeb();
  CHECK(r.o.braking, "brake: braking (CH3 case)");
  r.modeUs = 1000; r.ms(30);
  CHECK(!r.o.select && !r.o.braking && !r.in.muxNow, "brake: CH3 to manual cuts the brake short");
  Sim h;                                        // operator holding the brake when the link dies: keep braking
  CHECK(h.engage(), "brake: engaged (held brake case)");
  h.web(0, 0.4f, true); h.ms(800);
  h.web(0, -0.6f, true); h.ms(300);
  h.loseWeb();
  CHECK(h.o.braking && h.o.select, "brake: a brake held since forward keeps braking on link loss");
  Sim q;                                        // defence in depth: a config that skipped wk_cfg_valid
  q.c.brakeFrac = 0.1f;                         // 1475 us brake, inside the neutral band
  CHECK(q.engage(), "brake: engaged (in-band brake case)");
  q.web(0, 0.4f, true); q.ms(800); q.loseWeb();
  q.ms(WK_REST_MS + 100);
  CHECK(q.o.braking && q.s.quietSinceMs == 0, "brake: even an in-band brake never counts as quiet");
}

static void t_brake_never_reverses() {
  Sim m;                                        // review case: forward, light brake, neutral, light brake again
  CHECK(m.engage(), "fwd: engaged first");
  m.web(0, 0.3f, true); m.ms(500);
  m.web(0, -0.1f, true); m.ms(200);            // 1475 us if sent: inside the band, so it is snapped
  m.web(0, 0, true); m.ms(200);
  m.web(0, -0.1f, true); m.ms(200);
  CHECK(!m.ambiguousSent, "fwd: no ESP32 pulse inside the neutral band ever reaches the ESC");
  m.loseWeb();
  CHECK(m.o.braking, "fwd: ESC only saw forward and neutral, so a brake is a brake");
  Sim r;                                        // forward, clear brake, neutral, clear below = reversing
  CHECK(r.engage(), "fwd: engaged (reverse case)");
  r.web(0, 0.3f, true); r.ms(500);
  r.web(0, -0.3f, true); r.ms(300);
  r.web(0, 0, true); r.ms(200);
  r.web(0, -0.3f, true); r.ms(300);
  r.loseWeb();
  CHECK(!r.o.braking && !r.o.select, "fwd: reversing gets no brake pulse, S low");
  bool everBelow = false;
  for (int i = 0; i < 100; i++) { r.step(); everBelow |= r.o.escUs < r.c.escNeutral; }
  CHECK(!everBelow, "fwd: no below-neutral pulse after reverse");
  // the tracker on its own, with the ambiguous pulses a receiver can send
  const int n = 1500;
  uint8_t e = wk_esc_track(WK_ESC_UNKNOWN, 1700, n);
  e = wk_esc_track(e, 1475, n);                 // ESC may brake or stay neutral
  e = wk_esc_track(e, 1500, n);
  e = wk_esc_track(e, 1475, n);                 // may now be reverse
  CHECK(!wk_brake_ok(WK_DEFAULTS, e), "fwd: ambiguous brake, neutral, ambiguous never allows a brake");
  CHECK(wk_esc_track(WK_ESC_FWD, 1495, n) == WK_ESC_FWD, "fwd: 5 us below neutral is neutral");
  CHECK(wk_esc_track(WK_ESC_FWD, 1489, n) == WK_ESC_BRAKE, "fwd: 11 us below neutral may be a brake");
  CHECK(wk_esc_track(WK_ESC_FWD, 0, n) == WK_ESC_UNKNOWN, "fwd: no signal makes the ESC state unknown");
  CHECK(wk_esc_track(WK_ESC_UNKNOWN, 1300, n) == WK_ESC_UNKNOWN, "fwd: below from unknown stays unknown");
  Sim p;                                        // never moved: no brake
  CHECK(p.engage(), "fwd: engaged (parked)");
  p.loseWeb();
  CHECK(!p.o.select && !p.o.braking, "fwd: parked car is not braked, S low");
  Sim x;                                        // the receiver drove forward before the handover: S change = unknown
  x.thrUs = 1700; x.modeUs = 1000; x.ms(1000);
  x.thrUs = 1500; x.modeUs = 2000;
  CHECK(x.engage(), "fwd: engaged after manual forward");
  x.loseWeb();
  CHECK(!x.o.braking, "fwd: ESC state is unknown after the handover, no brake");
}

static void t_esc_modes() {
  Sim f;                                        // Fwd/Brake ESC: below neutral is always a brake
  f.c.escMode = WK_ESC_MODE_FWD_BRAKE;
  CHECK(f.engage(), "mode: engaged (Fwd/Brake)");
  f.web(0, -1, true); f.ms(500); f.web(0, 0, false); f.ms(200);
  f.loseWeb();
  CHECK(f.o.braking && f.o.select, "mode: Fwd/Brake brakes even after a below-neutral command");
  Sim r;                                        // Fwd/Rev ESC: below neutral is always reverse
  r.c.escMode = WK_ESC_MODE_FWD_REV;
  CHECK(r.engage(), "mode: engaged (Fwd/Rev)");
  r.web(0, 0.5f, true); r.ms(800);
  r.loseWeb();
  CHECK(!r.o.braking && !r.o.select && r.o.escUs == r.c.escNeutral, "mode: Fwd/Rev never brakes, neutral and hand back");
  WkCfg bad = WK_DEFAULTS; bad.escMode = 3;
  CHECK(!wk_cfg_valid(bad), "mode: unknown ESC mode rejected");
}

static void t_low_battery() {
  Sim m;
  CHECK(m.engage(), "batt: engaged first");
  m.cellV = 3.2f; m.ms(1000);                  // short sag under load
  CHECK(m.o.select, "batt: a 1 s sag does not release");
  m.cellV = 4.0f; m.ms(100);
  m.cellV = 3.2f; m.ms(WK_LOW_MS + 30);
  CHECK(!m.o.select && (m.o.why & WK_WHY_BATT_LOW) && m.o.urgent, "batt: low under load releases S urgently");
  CHECK(m.o.alarm == WK_ALARM_BATT, "batt: low under load alarms");
  m.cellV = 3.9f; m.ms(5000);                  // pack recovers at rest
  CHECK(!m.o.select && m.o.alarm == WK_ALARM_BATT, "batt: latched until power-cycle");
  Sim r; r.cellV = 3.6f; r.ms(5000);
  CHECK(!r.o.select && (r.o.why & WK_WHY_REST), "batt: below 3.7 V/cell rested refuses to engage");
  Sim i; i.cellV = 6.0f; i.ms(5000);
  CHECK(!i.o.select && (i.o.why & WK_WHY_SENSOR), "batt: implausible pack reading blocks");
}

static void t_sensor_glitch() {
  Sim m;
  CHECK(m.engage(), "adc: engaged first");
  m.badSamples = 1; m.step();
  CHECK(m.o.select, "adc: one bad sample does not drop S mid-run");
  m.ms(100);
  m.badSamples = WK_SENSOR_BAD_STEPS - 1; m.ms(100);
  CHECK(m.o.select, "adc: four bad samples in a row do not drop S");
  m.ms(100);
  m.badSamples = WK_SENSOR_BAD_STEPS; m.step(WK_SENSOR_BAD_STEPS);
  CHECK(!m.o.select && (m.o.why & WK_WHY_SENSOR) && m.o.alarm == WK_ALARM_BATT, "adc: five in a row release and alarm");
  Sim n; n.cellV = NAN; n.ms(5000);
  CHECK(!n.o.select && (n.o.why & WK_WHY_SENSOR) && n.o.alarm == WK_ALARM_BATT, "adc: NaN pack blocks and alarms");
  Sim b;                                        // a single bad sample restarts the engage debounce
  b.ms(2300);
  int ups = 0;
  for (int i = 0; i < 60; i++) { if (i % 20 == 0) b.badSamples = 1; b.step(); ups += b.o.select; }
  CHECK(ups == 0, "adc: one bad sample every 200 ms blocks engaging");
  Sim l;                                        // a bad sample does not reset the low-cell timer
  CHECK(l.engage(), "adc: engaged (low timer case)");
  l.cellV = 3.2f;
  for (int i = 0; i < 30; i++) { l.badSamples = 1; l.ms(100); }
  CHECK(!l.o.select && (l.o.why & WK_WHY_BATT_LOW), "adc: bad samples do not hide a sagging pack");
}

static void t_bec() {
  Sim m; m.becV = 6.8f; m.ms(5000);
  CHECK(!m.o.select && (m.o.why & WK_WHY_BEC), "bec: over 6.5 V refuses to engage");
  m.becV = 6.0f;
  CHECK(m.engage(), "bec: engages once the BEC is back at 6 V");
  Sim n; n.becV = NAN; n.ms(5000);
  CHECK(!n.o.select && (n.o.why & WK_WHY_SENSOR) && n.o.alarm == WK_ALARM_BATT, "bec: NaN BEC blocks and alarms");
}

static void t_loop_and_inhibit() {
  Sim m;
  CHECK(m.engage(), "loop: engaged first");
  m.t += 100; m.step();                         // the control task ran 110 ms late
  CHECK(!m.o.select && (m.o.why & WK_WHY_LOOP) && m.o.urgent, "loop: a late step drops S urgently");
  Sim q;
  CHECK(q.engage(), "inhibit: engaged first");
  q.in.inhibit = true; q.step();
  CHECK(!q.o.select && q.o.src == WK_SRC_NONE && q.o.urgent, "inhibit: OTA or flash write drops S urgently");
}

static void t_steering() {
  Sim m;
  m.c.steerRate = 0.8f;
  CHECK(m.engage(), "steer: engaged first");
  m.web(1, 0, true); m.ms(100);
  CHECK(m.o.steerUs == wk_steer_us(m.c, 0.8f), "steer: dual rate");
  m.c.maxThrottle = 1.0f;
  m.web(1, 1, true); m.ms(1000);
  CHECK(m.o.steerUs == wk_steer_us(m.c, m.c.steerLimitFrac), "steer: limited at full throttle");
}

static void t_cfg() {
  WkCfg c = WK_DEFAULTS; c.brakeFrac = 0.1f;   // 1475 us: inside the neutral band
  CHECK(!wk_cfg_valid(c), "cfg: brake pulse inside the neutral band rejected");
  c.brakeFrac = 0; CHECK(wk_cfg_valid(c), "cfg: brake off accepted");
  c = WK_DEFAULTS; c.brakeMs = 3001;
  CHECK(!wk_cfg_valid(c), "cfg: brakeMs over 3000 rejected");
}

static void t_edge() {
  WkEdge e = {};
  uint32_t t = 0x7FFFFFF0u;                     // wraps during the test
  CHECK(!wk_edge(e, true, t) && !wk_edge(e, false, t + 1500), "edge: first pulse after boot is dropped (no gap seen)");
  t += 20000;
  wk_edge(e, true, t);
  CHECK(wk_edge(e, false, t + 1000) && e.us == 1000, "edge: valid pulse");
  CHECK(!wk_edge(e, false, t + 1900), "edge: a stray low interrupt after the fall is rejected (review case)");
  CHECK(e.us == 1000, "edge: the stray interrupt did not overwrite the width");
  t += 20000;
  wk_edge(e, true, t);
  CHECK(wk_edge(e, false, t + 2000) && e.prevUs == 0, "edge: the stray interrupt broke the run");
  t += 20000;
  wk_edge(e, true, t);
  CHECK(wk_edge(e, false, t + 2000) && e.prevUs == 2000, "edge: two in a row");
  wk_edge(e, true, t + 3000);                   // rise 1 ms after a fall: noise
  CHECK(!wk_edge(e, false, t + 4900), "edge: rise inside the low gap rejected");
  t += 20000;
  wk_edge(e, true, t);
  CHECK(!wk_edge(e, false, t + 500) && !wk_edge(e, false, t + 1900), "edge: short pulse rejected, no second chance");
  t += 20000;
  wk_edge(e, true, t);
  CHECK(!wk_edge(e, false, t + 2300), "edge: too long rejected");
}

static void t_lock() {
  WkLock l = {0, 0, 0};
  CHECK(wk_lock_accept(l, 7, 1, 1000), "lock: first client takes it");
  CHECK(!wk_lock_accept(l, 9, 50, 1020), "lock: second client refused");
  CHECK(!wk_lock_accept(l, 7, 1, 1030), "lock: repeated seq refused");
  CHECK(wk_lock_accept(l, 7, 2, 1040), "lock: holder's next frame accepted");
  CHECK(!wk_lock_accept(l, 9, 51, 1040 + WK_LOCK_MS - 1), "lock: still held just under 1 s");
  CHECK(wk_lock_accept(l, 9, 51, 1040 + WK_LOCK_MS), "lock: free after 1 s of silence");
  CHECK(!wk_lock_accept(l, 0, 99, 9000), "lock: token 0 never accepted");
}

int main() {
  int bad = wk_selftest(WK_DEFAULTS);
  WkCfg odd = WK_DEFAULTS;                 // asymmetric endpoints + reversed steering + trimmed neutral
  odd.steerLeft = 1100; odd.steerRight = 1880; odd.steerCenter = 1490; odd.steerReverse = 1;
  odd.escNeutral = 1520; odd.escFwdMax = 1950; odd.escRevMax = 1300;
  odd.steerRate = 0.7f; odd.steerLimitFrac = 0.3f; odd.brakeFrac = 0.5f;
  bad += wk_selftest(odd);
  bad += wk_selftest(WkCfg{}) == 0;        // an all-zero config must fail (version 0)
  std::printf("boot selftest: %s (%d failed)\n", bad ? "FAIL" : "pass", bad);
  t_engage(); t_receiver_failsafe(); t_ch3(); t_brake_after_forward(); t_brake_never_reverses(); t_esc_modes();
  t_low_battery(); t_sensor_glitch(); t_bec(); t_loop_and_inhibit(); t_steering(); t_cfg(); t_edge(); t_lock();
  std::printf("control scenarios: %s (%d failed)\n", fails ? "FAIL" : "pass", fails);
  bad += fails;
  std::printf("host selftest: %s (%d failed)\n", bad ? "FAIL" : "pass", bad);
  return bad != 0;
}
