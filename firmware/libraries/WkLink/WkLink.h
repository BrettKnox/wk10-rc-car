// Packet shared by the transmitter and the car. Both sketches include this file.
#pragma once
#include <stdint.h>

#define WK_MAGIC   0x574B3130UL   // "WK10"
#define WK_CHANNEL 1              // both ends must sit on the same Wi-Fi channel

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
