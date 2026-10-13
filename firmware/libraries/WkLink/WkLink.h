// Shared by the transmitter and the car: the radio packet, the tuning record,
// the pure control maths and the control decision (kept here so wk_selftest() can check
// it at boot and firmware/test/host_selftest.cpp can drive it on a simulated clock).
#pragma once
#include <stdint.h>
#include <math.h>

#define WK_MAGIC   0x574B3130UL   // "WK10"
#define WK_CHANNEL 1              // ESP-NOW and the car's access point share this channel

struct __attribute__((packed)) WkPacket {
  uint32_t magic;
  uint16_t seq;       // increments every packet; the car rejects repeats and old ones
  int16_t  steer;     // -1000 (full left) .. +1000 (full right)
  int16_t  throttle;  // -1000 (full reverse/brake) .. +1000 (full forward)
  uint8_t  flags;     // bit0 = dead-man held
  uint8_t  sum;       // xor of every byte before it
};

static inline uint8_t wk_sum(const WkPacket &p) {
  const uint8_t *b = (const uint8_t *)&p;
  uint8_t s = 0;
  for (unsigned i = 0; i < sizeof(WkPacket) - 1; i++) s ^= b[i];
  return s;
}

// ESC running mode, as programmed into the ESC (Hobbywing "running mode" item).
enum { WK_ESC_MODE_FWD_REV_BRAKE = 0, WK_ESC_MODE_FWD_BRAKE = 1, WK_ESC_MODE_FWD_REV = 2 };

// Neutral band the ESP32 assumes the ESC uses. The ESP32 never sends a pulse in
// escNeutral - WK_ESC_BAND_US .. escNeutral - 1 (it snaps to neutral), so the ESC can never read
// one of our pulses either way. The ESC's own neutral range must be narrower than this band.
#define WK_ESC_BAND_US 40         // above escNeutral + this = forward; at or above escNeutral - this may be neutral
#define WK_ESC_BELOW_US 10        // below escNeutral - this, the ESC may read "below neutral"

// Everything you tune on the bench. Saved to flash from the web page.
struct WkCfg {
  uint16_t version;      // bump when the layout or meaning changes so old flash is ignored
  int16_t steerCenter, steerLeft, steerRight;   // us
  uint8_t steerReverse;
  int16_t escNeutral, escFwdMax, escRevMax;     // us; escNeutral must match the radio's throttle neutral
  float   throttleSlew;  // full scale per second, upward only
  float   steerExpo;     // 0 linear .. 0.9
  float   maxThrottle;   // 0..1, beginner limit
  uint16_t failsafeMs;   // ESP32 command older than this = lost
  float   steerRate;     // dual rate for ESP32 sources, 0.2..1
  float   steerLimitAt;  // throttle at and above which steering is held to steerLimitFrac
  float   steerLimitFrac;// 1 = no throttle-scaled limit
  float   brakeFrac;     // failsafe brake, fraction of the reverse span (escNeutral..escRevMax); 0 = coast
  uint16_t brakeMs;      // failsafe brake time (max 3000), then neutral and the receiver takes over
  uint8_t escMode;       // WK_ESC_MODE_*: must match the ESC's programmed running mode
  float   vbatRatio;     // pack divider: pack volts per pin volt (30k/10k = 4.0), trim to a meter
  float   becRatio;      // BEC divider: BEC volts per pin volt (20k/10k = 3.0), trim to a meter
};

static const WkCfg WK_DEFAULTS = {
  6,
  1500, 1150, 1850, 0,     // set left/right on the bench to 28 deg lock (about 36 deg of servo)
  1500, 2000, 1250,        // reverse/brake capped at half
  4.0f,                    // launch is traction-limited at ~0.66 g; faster than this just spins
  0.35f,
  0.5f,                    // start at half power; raise on the web page
  250,
  1.0f, 0.6f, 0.4f,        // full steering to 30% throttle, ramping to 40% steering at 60%
  0.35f, 3000,             // brake at 35% of the reverse span (1413 us) for 3 s
  WK_ESC_MODE_FWD_REV_BRAKE,
  4.0f, 3.0f
};

static inline float wk_clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// t: -1 full reverse/brake .. +1 full forward, around `neutral`.
static inline int wk_esc_us(const WkCfg &c, int neutral, float t) {
  t = wk_clampf(t, -1, 1);
  return t >= 0 ? (int)lroundf(neutral + t * (c.escFwdMax - neutral))
                : (int)lroundf(neutral + t * (neutral - c.escRevMax));
}

// Trust boundary: every value from the web page or flash goes through this. false = reject.
// Written so a NaN fails every float check.
static inline bool wk_cfg_valid(const WkCfg &c) {
  bool ok = c.version == WK_DEFAULTS.version &&
         c.steerCenter >= 1000 && c.steerCenter <= 2000 &&
         c.steerLeft >= 800 && c.steerLeft < c.steerCenter &&
         c.steerRight <= 2200 && c.steerRight > c.steerCenter &&
         c.steerReverse <= 1 &&
         c.escNeutral >= 1470 && c.escNeutral <= 1530 &&      // keep it near 1500, like the radio
         c.escFwdMax >= c.escNeutral + 100 && c.escFwdMax <= 2100 &&
         c.escRevMax <= c.escNeutral - 100 && c.escRevMax >= 900 &&
         c.throttleSlew >= 0.5f && c.throttleSlew <= 50.0f &&
         c.steerExpo >= 0.0f && c.steerExpo <= 0.9f &&
         c.maxThrottle >= 0.0f && c.maxThrottle <= 1.0f &&
         c.failsafeMs >= 100 && c.failsafeMs <= 500 &&
         c.steerRate >= 0.2f && c.steerRate <= 1.0f &&
         c.steerLimitAt >= 0.1f && c.steerLimitAt <= 1.0f &&
         c.steerLimitFrac >= 0.1f && c.steerLimitFrac <= 1.0f &&
         c.brakeFrac >= 0.0f && c.brakeFrac <= 1.0f &&
         c.brakeMs <= 3000 &&
         c.escMode <= WK_ESC_MODE_FWD_REV &&
         c.vbatRatio >= 3.0f && c.vbatRatio <= 5.0f &&
         c.becRatio >= 2.0f && c.becRatio <= 4.0f;
  // A brake pulse inside the neutral band would be read as neutral by the ESC (and as "quiet"
  // here, unlocking settings mid-brake): it must sit clearly below the band, or be off.
  return ok && (c.brakeFrac == 0 || wk_esc_us(c, c.escNeutral, -c.brakeFrac) < c.escNeutral - WK_ESC_BAND_US);
}

static inline float wk_expo(float x, float e) { return (1 - e) * x + e * x * x * x; }

// x: -1 left .. +1 right
static inline int wk_steer_us(const WkCfg &c, float x) {
  x = wk_expo(wk_clampf(x, -1, 1), c.steerExpo) * (c.steerReverse ? -1 : 1);
  return x >= 0 ? (int)lroundf(c.steerCenter + x * (c.steerRight - c.steerCenter))
                : (int)lroundf(c.steerCenter + x * (c.steerCenter - c.steerLeft));
}

// Slew-limit only when pushing harder forward. Lifting off and braking are instant,
// and going from reverse to forward starts the ramp at zero, not at the reverse value.
static inline float wk_slew(float cur, float want, float perSec, float dt) {
  if (want > 0 && cur < 0) cur = 0;
  if (want > cur && want > 0) { float n = cur + perSec * dt; return n < want ? n : want; }
  return want;
}

// Dual rate, then a limit that shrinks with throttle: full lock up to half of steerLimitAt,
// a linear ramp down to steerLimitFrac at steerLimitAt, flat above. `thr` is the throttle
// actually sent (after maxThrottle), because that is what sets the speed.
static inline float wk_steer_cmd(const WkCfg &c, float steer, float thr) {
  float a = fabsf(thr), half = c.steerLimitAt * 0.5f, lim = 1.0f;
  if (a >= c.steerLimitAt) lim = c.steerLimitFrac;
  else if (a > half) lim = 1.0f - (1.0f - c.steerLimitFrac) * (a - half) / half;
  return wk_clampf(wk_clampf(steer, -1, 1) * c.steerRate, -lim, lim);
}

// Arming gate. Throttle passes only after the operator has been seen with the dead-man
// RELEASED on the current source since the last boot, failsafe or source change.
// Stops a car that rebooted mid-run (brownout) or switched source from launching.
enum { WK_SRC_NONE = 0, WK_SRC_TX = 1, WK_SRC_WEB = 2 };
struct WkGate { uint8_t src; bool armed; };

static inline float wk_gate(WkGate &g, uint8_t src, bool linkOk, bool deadman, float thr) {
  if (!linkOk) src = WK_SRC_NONE;
  if (src != g.src) { g.src = src; g.armed = false; }
  if (src != WK_SRC_NONE && !deadman) g.armed = true;
  return (src != WK_SRC_NONE && g.armed && deadman) ? thr : 0.0f;
}

// Signed age in ms; a timestamp written just after `now` was read counts as fresh, never as 49 days old.
static inline int32_t wk_age(uint32_t now, uint32_t then) {
  int32_t a = (int32_t)(now - then);
  return a < 0 ? 0 : a;
}

// ---------------------------------------------------------------- web client lock
// One web client (phone WebSocket or UDP script) drives at a time. The holder's frames are
// accepted when their seq is newer; any other token, or any seq, only after WK_LOCK_MS of silence.
#define WK_LOCK_MS 1000
struct WkLock { uint32_t tok, seq, ms; };

static inline bool wk_lock_accept(WkLock &l, uint32_t tok, uint32_t seq, uint32_t now) {
  if (!tok) return false;
  bool idle = !l.tok || wk_age(now, l.ms) >= WK_LOCK_MS;
  if (!idle && (tok != l.tok || (int32_t)(seq - l.seq) <= 0)) return false;
  l.tok = tok; l.seq = seq; l.ms = now;
  return true;
}

// ---------------------------------------------------------------- receiver pulse decoding
#define WK_RX_MIN_US 800          // a receiver pulse outside 800..2200 us is noise
#define WK_RX_MAX_US 2200
#define WK_RX_MIN_GAP_US 2500     // a rise this soon after any fall is noise (receivers idle 5+ ms between pulses)
#define WK_RX_FRESH_MS 100        // no valid pulse for this long = receiver lost (5 frames at 50 Hz)

// Edge decoder for one receiver channel, fed from a CHANGE interrupt with the pin level and
// micros(). A width counts only for the first fall after a rise that itself followed a full
// low gap, so a stray interrupt can drop a pulse but never invent one. prevUs is the width of
// the pulse before, or 0 if anything was rejected in between (CH3 needs two in a row).
struct WkEdge { uint32_t riseUs, fallUs, okUs; bool rose, seenFall, run; uint16_t us, prevUs; };

static inline bool wk_edge(WkEdge &e, bool high, uint32_t t) {
  if (high) {
    e.rose = e.seenFall && t - e.fallUs >= WK_RX_MIN_GAP_US;
    e.riseUs = t;
    return false;
  }
  uint32_t w = t - e.riseUs;
  bool ok = e.rose && w >= WK_RX_MIN_US && w <= WK_RX_MAX_US;
  e.rose = false;                                    // one fall per rise
  e.seenFall = true;
  e.fallUs = t;
  if (!ok) { e.run = false; return false; }
  e.prevUs = (e.run && t - e.okUs < WK_RX_FRESH_MS * 1000UL) ? e.us : 0;
  e.us = (uint16_t)w;
  e.okUs = t;
  e.run = true;
  return true;
}

// ---------------------------------------------------------------- ESC direction tracking
// Hobbywing Fwd/Rev/Brake: after forward, the first push below neutral brakes (also after a
// return to neutral); once it has braked, neutral then below reverses. Tracks what the ESC
// has seen so the failsafe never sends a "brake" that the ESC would take as reverse.
// A pulse between escNeutral - WK_ESC_BAND_US and escNeutral - WK_ESC_BELOW_US could be read
// either way, so it counts as both, always against braking.
static const uint8_t WK_ESC_UNKNOWN = 0, WK_ESC_FWD = 1, WK_ESC_BRAKE = 2, WK_ESC_AFTER_BRAKE = 3, WK_ESC_REV = 4;

static inline uint8_t wk_esc_track(uint8_t st, int seen, int n) {
  if (!seen) return WK_ESC_UNKNOWN;                  // ESC sees no signal: its state is unknown
  if (seen > n + WK_ESC_BAND_US) return WK_ESC_FWD;
  bool below = seen < n - WK_ESC_BELOW_US;           // the ESC may read it as below neutral
  bool neutral = seen >= n - WK_ESC_BAND_US;         // the ESC may read it as neutral
  bool braking = st == WK_ESC_FWD || st == WK_ESC_BRAKE;
  if (below && !neutral) return braking ? WK_ESC_BRAKE : (st == WK_ESC_UNKNOWN ? WK_ESC_UNKNOWN : WK_ESC_REV);
  if (below) return st == WK_ESC_FWD ? WK_ESC_BRAKE : st == WK_ESC_BRAKE ? WK_ESC_AFTER_BRAKE : st;
  return st == WK_ESC_BRAKE ? WK_ESC_AFTER_BRAKE : st;   // neutral: a brake becomes "next push reverses"
}

// May the failsafe send a below-neutral pulse and have the ESC brake, not reverse?
static inline bool wk_brake_ok(const WkCfg &c, uint8_t st) {
  if (!c.brakeMs || !(c.brakeFrac > 0)) return false;
  if (c.escMode == WK_ESC_MODE_FWD_BRAKE) return true;         // below neutral is always brake
  if (c.escMode == WK_ESC_MODE_FWD_REV) return false;          // below neutral is always reverse
  return st == WK_ESC_FWD || st == WK_ESC_BRAKE;               // forward, or already braking since forward
}

// ---------------------------------------------------------------- control decision
// The hobby receiver drives the servo and ESC through a 74HCT157 unless the mux select S is
// high. wk_control_step() is the only thing that decides S. Any doubt means S low.
#define WK_AUTO_MIN_US 1800       // CH3 "autonomy" window; anything else is manual
#define WK_AUTO_MAX_US 2200
#define WK_LOOP_MAX_MS 50         // a step this late after the last one = control loop stalled
#define WK_ENGAGE_MS 500          // every engage check must hold this long before S rises
#define WK_SWITCH_MAX_MS 50       // a deferred S change is forced after this long without a common low gap
#define WK_REST_MS 2000           // quiet this long = pack voltage counts as rested
#define WK_REST_MAX_AGE_MS 60000  // a rested reading older than this is not trusted for engaging
#define WK_LOW_MS 2000            // under-load low cell this long = latched until power-cycle
#define WK_SENSOR_BAD_STEPS 5     // this many bad readings in a row release S; one blocks engaging
#define WK_CELL_ENGAGE_V 3.7f     // rested volts per cell needed to engage
#define WK_CELL_RELEASE_V 3.3f    // under-load volts per cell that release S and alarm
#define WK_CELL_MIN_V 2.5f        // outside 2.5..4.5 V per cell = sensor fault
#define WK_CELL_MAX_V 4.5f
#define WK_BEC_ARM_MAX_V 6.5f     // BEC above this refuses to engage (BEC mis-set to 7.4 V)
#define WK_BEC_MIN_V 3.0f         // outside 3..9 V = sensor fault
#define WK_BEC_MAX_V 9.0f
#define WK_CHIRP_MS 600           // "engaged" chirps play this long after S rises

static const uint8_t WK_ALARM_NONE = 0, WK_ALARM_ENGAGED = 1, WK_ALARM_LINK = 2, WK_ALARM_BATT = 3;
// Why S is low (bitmask). The first group drops S at once; the second only blocks engaging,
// except CMD, which brakes first (see below).
static const uint16_t
  WK_WHY_INHIBIT = 1, WK_WHY_LOOP = 2, WK_WHY_RX = 4, WK_WHY_MODE = 8, WK_WHY_BATT_LOW = 16, WK_WHY_SENSOR = 32,
  WK_WHY_CMD = 64, WK_WHY_REST = 128, WK_WHY_BEC = 256, WK_WHY_THR = 512, WK_WHY_WAIT = 1024;
enum { WK_RX_STEER = 0, WK_RX_THR = 1, WK_RX_MODE = 2 };

struct WkCmd { float steer, thr; bool deadman; uint32_t ms; };   // ms = 0: never received
struct WkIn {
  WkCmd tx, web;
  uint16_t rxUs[3];   // last valid receiver pulse per channel (WK_RX_*), 0 = never
  uint32_t rxMs[3];   // millis() of that pulse
  uint16_t modePrevUs;// CH3 pulse before rxUs[WK_RX_MODE], 0 = not consecutive
  float cellV, becV;  // this sample: filtered volts per cell and BEC volts; NaN = unreadable
  bool inhibit;       // OTA or flash write in progress: no ESP32 source, S low
  bool muxNow;        // S as actually applied to the pin right now
};
struct WkCtl {
  WkGate gate;
  float slewed;
  uint32_t lastMs;            // previous step; 0 = none yet (first step counts as a stall)
  bool engaged, braking, rxSeen, prevMux;
  uint32_t okSinceMs, engagedMs, brakeSinceMs;
  uint8_t escState;           // WK_ESC_*, what the ESC has seen; UNKNOWN after every S change
  uint8_t badRun;             // consecutive bad battery/BEC samples
  uint32_t quietSinceMs;      // ESC-seen throttle neutral since; 0 = not quiet
  float restedCellV;
  uint32_t restedMs, lowSinceMs;
  bool lowLatched;
};
static const WkCtl WK_CTL_INIT = {{WK_SRC_NONE, false}, 0, 0, false, false, false, false, 0, 0, 0,
                                  WK_ESC_UNKNOWN, 0, 0, NAN, 0, 0, false};
struct WkOut {
  bool select;          // S wanted: true = ESP32 drives the servo and ESC
  bool urgent;          // apply a drop now; otherwise wait for a common low gap (WK_SWITCH_MAX_MS at most)
  int steerUs, escUs;   // ESP32 pulses (always valid, also while S is low)
  uint8_t src, alarm;
  uint16_t why;         // WK_WHY_* reasons S is low or not yet allowed up; 0 while engaged and commanded
  bool braking, rxOk, autoSw;
};

static inline bool wk_rx_fresh(const WkIn &in, int ch, uint32_t now) {
  return in.rxUs[ch] >= WK_RX_MIN_US && in.rxUs[ch] <= WK_RX_MAX_US && wk_age(now, in.rxMs[ch]) < WK_RX_FRESH_MS;
}
static inline bool wk_is_auto(uint16_t us) { return us >= WK_AUTO_MIN_US && us <= WK_AUTO_MAX_US; }

// One 100 Hz control step. Pure: everything it knows comes in through `in` and `s`.
// Which failsafe wins:
//   - Receiver lost, battery latch, 5 bad sensor readings in a row, loop late, OTA: S low in this
//     step (urgent); the receiver (and its own failsafe, set to brake) has the car.
//   - CH3 to manual: S low at the next common low gap of all four signals (WK_SWITCH_MAX_MS at most).
//   - Only the ESP32 command lost (receiver fine, CH3 on autonomy): S stays high for brakeMs
//     while the ESP32 brakes, then S low. Flipping CH3 to manual cuts the brake short.
//   - No brake when the ESC might take below neutral as reverse (wk_brake_ok): S low at once.
static inline WkOut wk_control_step(WkCtl &s, const WkCfg &c, const WkIn &in, uint32_t now) {
  WkOut o = {};
  const int n = c.escNeutral;
  bool loopOk = s.lastMs && wk_age(now, s.lastMs) <= WK_LOOP_MAX_MS;
  float dt = loopOk ? wk_age(now, s.lastMs) / 1000.0f : 0;
  s.lastMs = now;

  // ESP32 command source: the ESP-NOW transmitter wins while alive, else the newest web frame.
  bool txLive = in.tx.ms && wk_age(now, in.tx.ms) < c.failsafeMs;
  bool webLive = in.web.ms && wk_age(now, in.web.ms) < c.failsafeMs && (!in.tx.ms || (int32_t)(in.web.ms - in.tx.ms) > 0);
  uint8_t src = in.inhibit ? WK_SRC_NONE : txLive ? WK_SRC_TX : (webLive ? WK_SRC_WEB : WK_SRC_NONE);
  const WkCmd &cmd = src == WK_SRC_TX ? in.tx : in.web;
  bool linkOk = src != WK_SRC_NONE;
  float want = wk_gate(s.gate, src, linkOk, linkOk && cmd.deadman, linkOk ? cmd.thr : 0);
  bool cmdOk = linkOk && s.gate.armed;

  bool rxOk = wk_rx_fresh(in, WK_RX_STEER, now) && wk_rx_fresh(in, WK_RX_THR, now);
  if (rxOk) s.rxSeen = true;
  bool autoSw = wk_rx_fresh(in, WK_RX_MODE, now) && wk_is_auto(in.rxUs[WK_RX_MODE]) && wk_is_auto(in.modePrevUs);

  bool sampleBad = !(in.cellV >= WK_CELL_MIN_V && in.cellV <= WK_CELL_MAX_V) ||   // NaN fails both
                   !(in.becV >= WK_BEC_MIN_V && in.becV <= WK_BEC_MAX_V);
  s.badRun = sampleBad ? (s.badRun < 255 ? s.badRun + 1 : 255) : 0;
  bool sensorFault = s.badRun >= WK_SENSOR_BAD_STEPS;
  if (!sampleBad) {                                  // a bad sample leaves the low-cell timer alone
    if (in.cellV < WK_CELL_RELEASE_V) {
      if (!s.lowSinceMs) s.lowSinceMs = now;
      if (wk_age(now, s.lowSinceMs) >= WK_LOW_MS) s.lowLatched = true;
    } else {
      s.lowSinceMs = 0;
    }
  }
  bool restOk = s.restedMs && wk_age(now, s.restedMs) < WK_REST_MAX_AGE_MS && s.restedCellV >= WK_CELL_ENGAGE_V;

  uint16_t drop = (in.inhibit ? WK_WHY_INHIBIT : 0) | (loopOk ? 0 : WK_WHY_LOOP) | (rxOk ? 0 : WK_WHY_RX) |
                  (autoSw ? 0 : WK_WHY_MODE) | (s.lowLatched ? WK_WHY_BATT_LOW : 0) | (sensorFault ? WK_WHY_SENSOR : 0);
  uint16_t why = drop | (sampleBad ? WK_WHY_SENSOR : 0) | (cmdOk ? 0 : WK_WHY_CMD) | (restOk ? 0 : WK_WHY_REST) |
                 (!sampleBad && in.becV > WK_BEC_ARM_MAX_V ? WK_WHY_BEC : 0) | (want != 0 ? WK_WHY_THR : 0);

  if (s.engaged) {
    if (drop) {
      s.engaged = s.braking = false;                 // the receiver takes over
    } else if (!cmdOk && !s.braking) {
      if (wk_brake_ok(c, s.escState)) { s.braking = true; s.brakeSinceMs = now; }
      else s.engaged = false;
    }
    if (s.braking && wk_age(now, s.brakeSinceMs) >= c.brakeMs) s.engaged = s.braking = false;   // then neutral, receiver
  } else if (why) {
    s.okSinceMs = 0;
  } else if (!s.okSinceMs) {
    s.okSinceMs = now;
  } else if (wk_age(now, s.okSinceMs) >= WK_ENGAGE_MS) {
    s.engaged = true; s.engagedMs = now; s.okSinceMs = 0;
  }

  float steer = 0;
  if (s.engaged && !s.braking) {
    if (want > 0) want *= c.maxThrottle;
    s.slewed = wk_slew(s.slewed, want, c.throttleSlew, dt);
    steer = wk_steer_cmd(c, cmd.steer, s.slewed);
  } else {
    s.slewed = 0;
  }
  o.select = s.engaged;
  o.urgent = !s.engaged && (drop & ~WK_WHY_MODE);
  o.escUs = wk_esc_us(c, n, s.braking ? -c.brakeFrac : s.slewed);
  if (o.escUs < n && o.escUs >= n - WK_ESC_BAND_US) o.escUs = n;   // never a pulse the ESC could read either way
  o.steerUs = wk_steer_us(c, steer);

  // What the ESC sees now: our pulse while S is applied high, the receiver's otherwise (none if
  // lost). Any S change makes its state unknown: the switch can land mid-frame.
  int seen = in.muxNow ? o.escUs : (wk_rx_fresh(in, WK_RX_THR, now) ? in.rxUs[WK_RX_THR] : 0);
  s.escState = in.muxNow != s.prevMux ? WK_ESC_UNKNOWN : wk_esc_track(s.escState, seen, n);
  s.prevMux = in.muxNow;
  bool quiet = !s.braking && (!seen || (seen >= n - WK_ESC_BAND_US && seen <= n + WK_ESC_BAND_US));
  if (!quiet) s.quietSinceMs = 0;
  else if (!s.quietSinceMs) s.quietSinceMs = now;
  if (quiet && wk_age(now, s.quietSinceMs) >= WK_REST_MS && !sampleBad) { s.restedCellV = in.cellV; s.restedMs = now; }

  // Alarms sound only once the receiver has been seen, so a USB-only bench stays quiet.
  bool live = s.rxSeen;
  o.alarm = (live && (s.lowLatched || sensorFault)) ? WK_ALARM_BATT
          : (s.braking || (live && !rxOk)) ? WK_ALARM_LINK
          : (s.engaged && wk_age(now, s.engagedMs) < WK_CHIRP_MS) ? WK_ALARM_ENGAGED : WK_ALARM_NONE;
  o.src = src;
  o.why = s.engaged ? (s.braking ? WK_WHY_CMD : 0) : (why ? why : WK_WHY_WAIT);
  o.braking = s.braking; o.rxOk = rxOk; o.autoSw = autoSw;
  return o;
}

// Returns the number of failed checks (0 = pass). Runs on the car at every boot,
// against the defaults AND against the config the car is about to use.
static inline int wk_selftest(const WkCfg &c) {
  int bad = 0;
  const int n = c.escNeutral;
  bad += !wk_cfg_valid(WK_DEFAULTS);
  bad += !wk_cfg_valid(c);
  bad += wk_steer_us(c, 0) != c.steerCenter;
  bad += wk_steer_us(c, 2) != wk_steer_us(c, 1);                       // clamps
  int lo = c.steerReverse ? c.steerRight : c.steerLeft, hi = c.steerReverse ? c.steerLeft : c.steerRight;
  bad += wk_steer_us(c, -1) != lo || wk_steer_us(c, 1) != hi;
  WkCfg rev = c; rev.steerReverse = !c.steerReverse;                   // reverse really mirrors
  bad += wk_steer_us(rev, 1) != wk_steer_us(c, -1) || wk_steer_us(rev, -0.5f) != wk_steer_us(c, 0.5f);
  bad += wk_esc_us(c, n, 0) != n;
  bad += wk_esc_us(c, n, 1) != c.escFwdMax;
  bad += wk_esc_us(c, n, -1) != c.escRevMax;
  int brk = wk_esc_us(c, n, -c.brakeFrac);                              // brake pulse sits in the reverse span
  bad += brk > n || brk < c.escRevMax;
  bad += fabsf(wk_steer_cmd(c, 1, 0) - c.steerRate) > 1e-4f;           // dual rate at rest
  bad += fabsf(wk_steer_cmd(c, -1, 1)) > c.steerLimitFrac + 1e-4f;     // limit at full throttle
  bad += fabsf(wk_steer_cmd(c, 1, c.steerLimitAt * 0.75f)) > fabsf(wk_steer_cmd(c, 1, c.steerLimitAt * 0.4f)) + 1e-4f;
  bad += fabsf(wk_slew(0, 1, 4, 0.1f) - 0.4f) > 1e-4f;                // ramps up
  bad += wk_slew(0.8f, 0, 4, 0.01f) != 0;                               // lift-off instant
  bad += wk_slew(0.5f, -1, 4, 0.01f) != -1;                             // brake instant
  bad += fabsf(wk_slew(-1, 1, 4, 0.1f) - 0.4f) > 1e-4f;               // reverse -> forward ramps from 0
  WkCfg broken = c; broken.steerLeft = c.steerCenter + 10;             // left past centre: reject
  bad += wk_cfg_valid(broken);
  broken = c; broken.escNeutral = 1600;                                // neutral that would creep: reject
  bad += wk_cfg_valid(broken);
  broken = c; broken.vbatRatio = NAN;                                  // NaN calibration: reject
  bad += wk_cfg_valid(broken);
  // Fwd/Rev/Brake: forward, brake, neutral, below = reverse; the tracker must not allow a brake then
  uint8_t e = wk_esc_track(WK_ESC_UNKNOWN, n + 200, n);
  e = wk_esc_track(e, n - 100, n);
  bad += e != WK_ESC_BRAKE;
  e = wk_esc_track(e, n, n);
  bad += e != WK_ESC_AFTER_BRAKE;
  bad += wk_esc_track(e, n - 100, n) != WK_ESC_REV;
  WkGate g = {WK_SRC_NONE, false};
  bad += wk_gate(g, WK_SRC_TX, true, true, 1) != 0;                    // held at boot: no throttle
  bad += wk_gate(g, WK_SRC_TX, true, false, 1) != 0;                   // released: arms, still 0
  bad += wk_gate(g, WK_SRC_TX, true, true, 1) != 1;                    // held again: drives
  bad += wk_gate(g, WK_SRC_TX, false, true, 1) != 0;                   // failsafe: 0
  bad += wk_gate(g, WK_SRC_TX, true, true, 1) != 0;                    // link back, still held: 0
  wk_gate(g, WK_SRC_TX, true, false, 1);
  bad += wk_gate(g, WK_SRC_WEB, true, true, 1) != 0;                   // source change: disarms
  bad += wk_age(100, 101) != 0 || wk_age(0x10, 0xFFFFFFF0u) != 0x20;   // race and wrap
  WkPacket p = {WK_MAGIC, 1, 10, 20, 1, 0}; p.sum = wk_sum(p);
  WkPacket q = p; q.throttle = 999;                                    // tampered byte fails the sum
  bad += q.sum == wk_sum(q);
  return bad;
}
