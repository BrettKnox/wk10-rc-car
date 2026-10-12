// Shared by the transmitter and the car: the radio packet, the tuning record,
// and the pure mixing maths (kept here so wk_selftest() can check it at boot).
#pragma once
#include <stdint.h>
#include <math.h>

#define WK_MAGIC   0x574B3130UL   // "WK10"
#define WK_CHANNEL 1              // ESP-NOW and the car's access point share this channel

struct __attribute__((packed)) WkPacket {
  uint32_t magic;
  uint16_t seq;
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
  uint16_t version;      // bump when the layout changes so old flash is ignored
  int16_t steerCenter, steerLeft, steerRight;   // us
  uint8_t steerReverse;
  int16_t escNeutral, escFwdMax, escRevMax;     // us
  float   throttleSlew;  // full scale per second, upward only
  float   steerExpo;     // 0 linear .. 0.9
  float   maxThrottle;   // 0..1, beginner limit
  uint16_t failsafeMs;
};

static const WkCfg WK_DEFAULTS = {
  3,
  1500, 1150, 1850, 0,     // set left/right on the bench to 28 deg lock (about 37 deg of servo)
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
         c.escNeutral >= 1400 && c.escNeutral <= 1600 &&
         c.escFwdMax > c.escNeutral && c.escFwdMax <= 2100 &&
         c.escRevMax < c.escNeutral && c.escRevMax >= 900 &&
         c.throttleSlew >= 0.5f && c.throttleSlew <= 50.0f &&
         c.steerExpo >= 0.0f && c.steerExpo <= 0.9f &&
         c.maxThrottle >= 0.0f && c.maxThrottle <= 1.0f &&
         c.failsafeMs >= 100 && c.failsafeMs <= 1000;
}

static inline float wk_expo(float x, float e) { return (1 - e) * x + e * x * x * x; }

// x: -1 left .. +1 right
static inline int wk_steer_us(const WkCfg &c, float x) {
  x = wk_expo(wk_clampf(x, -1, 1), c.steerExpo) * (c.steerReverse ? -1 : 1);
  return x >= 0 ? (int)lroundf(c.steerCenter + x * (c.steerRight - c.steerCenter))
                : (int)lroundf(c.steerCenter + x * (c.steerCenter - c.steerLeft));
}

// t: -1 full reverse/brake .. +1 full forward (after maxThrottle and slew)
static inline int wk_esc_us(const WkCfg &c, float t) {
  t = wk_clampf(t, -1, 1);
  return t >= 0 ? (int)lroundf(c.escNeutral + t * (c.escFwdMax - c.escNeutral))
                : (int)lroundf(c.escNeutral + t * (c.escNeutral - c.escRevMax));
}

// Slew-limit only when pushing harder forward. Lifting off and braking are instant.
static inline float wk_slew(float cur, float want, float perSec, float dt) {
  if (want > cur && want > 0) { float n = cur + perSec * dt; return n < want ? n : want; }
  return want;
}

// Returns the number of failed checks (0 = pass). Runs on the car at every boot.
static inline int wk_selftest() {
  int bad = 0;
  const WkCfg &c = WK_DEFAULTS;
  bad += !wk_cfg_valid(c);
  bad += wk_steer_us(c, 0) != c.steerCenter;
  bad += wk_steer_us(c, 1) != c.steerRight;
  bad += wk_steer_us(c, -1) != c.steerLeft;
  bad += wk_steer_us(c, 2) != c.steerRight;               // clamps
  bad += wk_steer_us(c, 0.5f) <= c.steerCenter || wk_steer_us(c, 0.5f) >= c.steerRight;
  bad += wk_esc_us(c, 0) != c.escNeutral;
  bad += wk_esc_us(c, 1) != c.escFwdMax;
  bad += wk_esc_us(c, -1) != c.escRevMax;
  bad += fabsf(wk_slew(0, 1, 4, 0.1f) - 0.4f) > 1e-4f;    // ramps up
  bad += wk_slew(0.8f, 0, 4, 0.01f) != 0;                  // lift-off instant
  bad += wk_slew(0.5f, -1, 4, 0.01f) != -1;                // brake instant
  WkCfg broken = c; broken.steerLeft = 1700;               // left past centre must be rejected
  bad += wk_cfg_valid(broken);
  WkPacket p = {WK_MAGIC, 1, 10, 20, 1, 0}; p.sum = wk_sum(p);
  bad += p.sum != wk_sum(p);
  return bad;
}
