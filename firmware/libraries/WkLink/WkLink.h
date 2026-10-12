// Shared by the transmitter and the car: the radio packet, the tuning record,
// and the pure control maths (kept here so wk_selftest() can check it at boot).
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

// Everything you tune on the bench. Saved to flash from the web page.
struct WkCfg {
  uint16_t version;      // bump when the layout or meaning changes so old flash is ignored
  int16_t steerCenter, steerLeft, steerRight;   // us
  uint8_t steerReverse;
  int16_t escNeutral, escFwdMax, escRevMax;     // us; escNeutral takes effect at the next boot
  float   throttleSlew;  // full scale per second, upward only
  float   steerExpo;     // 0 linear .. 0.9
  float   maxThrottle;   // 0..1, beginner limit
  uint16_t failsafeMs;
};

static const WkCfg WK_DEFAULTS = {
  4,
  1500, 1150, 1850, 0,     // set left/right on the bench to 28 deg lock (about 36 deg of servo)
  1500, 2000, 1250,        // reverse/brake capped at half
  4.0f,                    // launch is traction-limited at ~0.66 g; faster than this just spins
  0.35f,
  0.5f,                    // start at half power; raise on the web page
  250
};

static inline float wk_clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Trust boundary: every value from the web page or flash goes through this. false = reject.
static inline bool wk_cfg_valid(const WkCfg &c) {
  return c.version == WK_DEFAULTS.version &&
         c.steerCenter >= 1000 && c.steerCenter <= 2000 &&
         c.steerLeft >= 800 && c.steerLeft < c.steerCenter &&
         c.steerRight <= 2200 && c.steerRight > c.steerCenter &&
         c.steerReverse <= 1 &&
         c.escNeutral >= 1470 && c.escNeutral <= 1530 &&      // ESC arms on this: keep it near 1500
         c.escFwdMax >= c.escNeutral + 100 && c.escFwdMax <= 2100 &&
         c.escRevMax <= c.escNeutral - 100 && c.escRevMax >= 900 &&
         c.throttleSlew >= 0.5f && c.throttleSlew <= 50.0f &&
         c.steerExpo >= 0.0f && c.steerExpo <= 0.9f &&
         c.maxThrottle >= 0.0f && c.maxThrottle <= 1.0f &&
         c.failsafeMs >= 100 && c.failsafeMs <= 500;
}

static inline float wk_expo(float x, float e) { return (1 - e) * x + e * x * x * x; }

// x: -1 left .. +1 right
static inline int wk_steer_us(const WkCfg &c, float x) {
  x = wk_expo(wk_clampf(x, -1, 1), c.steerExpo) * (c.steerReverse ? -1 : 1);
  return x >= 0 ? (int)lroundf(c.steerCenter + x * (c.steerRight - c.steerCenter))
                : (int)lroundf(c.steerCenter + x * (c.steerCenter - c.steerLeft));
}

// t: -1 full reverse/brake .. +1 full forward. `neutral` is the boot-time neutral the ESC armed on.
static inline int wk_esc_us(const WkCfg &c, int neutral, float t) {
  t = wk_clampf(t, -1, 1);
  return t >= 0 ? (int)lroundf(neutral + t * (c.escFwdMax - neutral))
                : (int)lroundf(neutral + t * (neutral - c.escRevMax));
}

// Slew-limit only when pushing harder forward. Lifting off and braking are instant,
// and going from reverse to forward starts the ramp at zero, not at the reverse value.
static inline float wk_slew(float cur, float want, float perSec, float dt) {
  if (want > 0 && cur < 0) cur = 0;
  if (want > cur && want > 0) { float n = cur + perSec * dt; return n < want ? n : want; }
  return want;
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
  bad += fabsf(wk_slew(0, 1, 4, 0.1f) - 0.4f) > 1e-4f;                // ramps up
  bad += wk_slew(0.8f, 0, 4, 0.01f) != 0;                               // lift-off instant
  bad += wk_slew(0.5f, -1, 4, 0.01f) != -1;                             // brake instant
  bad += fabsf(wk_slew(-1, 1, 4, 0.1f) - 0.4f) > 1e-4f;               // reverse -> forward ramps from 0
  WkCfg broken = c; broken.steerLeft = c.steerCenter + 10;             // left past centre: reject
  bad += wk_cfg_valid(broken);
  broken = c; broken.escNeutral = 1600;                                // neutral that would creep: reject
  bad += wk_cfg_valid(broken);
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
