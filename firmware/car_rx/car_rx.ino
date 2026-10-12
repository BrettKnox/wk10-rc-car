// WK-10 car receiver: ESP-NOW in, 50 Hz servo + ESC pulses out.
// Board: "ESP32 Dev Module", Arduino-ESP32 core 3.x.
// Power: ESC BEC (6 V) -> dev board VIN/5V pin. Never the 2S pack directly.
//
// Safety, in order of importance:
//   1. No valid packet for FAILSAFE_MS -> throttle neutral, steering centre.
//   2. Throttle only moves while the TX dead-man button is held.
//   3. Boot holds neutral for ARM_MS so the ESC arms on 1500 us, never on a stale value.
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WkLink.h>   // firmware/libraries/WkLink

// ---- pins (LEDC-capable, not strapping pins) ----
const int PIN_STEER = 18;
const int PIN_ESC   = 19;
const int PIN_LED   = 2;

// ---- calibration knobs: set these on the bench, see README "Bring-up" ----
// Bind to one transmitter. Leave all zero to accept any WK10 packet (bench only).
uint8_t TX_MAC[6] = {0, 0, 0, 0, 0, 0};
const int STEER_CENTER_US = 1500;   // trim until the car tracks straight
const int STEER_LEFT_US   = 1150;   // stop at 28 deg lock (the CAD sweep is cleared to 28)
const int STEER_RIGHT_US  = 1850;
const bool STEER_REVERSE  = false;
const int ESC_NEUTRAL_US  = 1500;
const int ESC_FWD_MAX_US  = 2000;
const int ESC_REV_MAX_US  = 1250;   // reverse/brake capped at half
const float THROTTLE_SLEW = 4.0;    // full scale per second upward: launch is traction-limited at ~0.66 g
const float STEER_EXPO    = 0.35;   // 0 = linear
const uint32_t FAILSAFE_MS = 250;
const uint32_t ARM_MS      = 3000;

volatile int16_t rxSteer = 0, rxThrottle = 0;
volatile uint8_t rxFlags = 0;
volatile uint32_t lastRxMs = 0;
volatile uint32_t badPackets = 0;

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
  rxSteer = constrain(p.steer, -1000, 1000);
  rxThrottle = constrain(p.throttle, -1000, 1000);
  rxFlags = p.flags;
  lastRxMs = millis();
}

static float expo(float x, float e) { return (1 - e) * x + e * x * x * x; }

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  ledcAttach(PIN_STEER, 50, 16);
  ledcAttach(PIN_ESC, 50, 16);
  ledcWrite(PIN_STEER, usToDuty(STEER_CENTER_US));
  ledcWrite(PIN_ESC, usToDuty(ESC_NEUTRAL_US));

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(WK_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) {
    Serial.println("esp_now_init failed: outputs stay neutral");
    while (true) { digitalWrite(PIN_LED, millis() / 100 % 2); delay(10); }
  }
  esp_now_register_recv_cb(onRecv);
  Serial.print("car MAC (paste into tx.ino CAR_MAC): ");
  Serial.println(WiFi.macAddress());

  uint32_t t0 = millis();                       // ESC arming window
  while (millis() - t0 < ARM_MS) { digitalWrite(PIN_LED, millis() / 500 % 2); delay(10); }
}

void loop() {
  static float thr = 0;                         // slewed throttle, -1..1
  static uint32_t lastMs = millis(), lastLog = 0;
  uint32_t now = millis();
  float dt = (now - lastMs) / 1000.0f;
  lastMs = now;

  bool linkOk = (now - lastRxMs) < FAILSAFE_MS;
  float wantSteer = linkOk ? rxSteer / 1000.0f : 0;
  float wantThr = (linkOk && (rxFlags & 1)) ? rxThrottle / 1000.0f : 0;

  // slew limit only when pushing harder forward; letting off and braking stay instant
  if (wantThr > thr && wantThr > 0) thr = min(wantThr, thr + THROTTLE_SLEW * dt);
  else thr = wantThr;

  float st = expo(wantSteer, STEER_EXPO) * (STEER_REVERSE ? -1 : 1);
  int steerUs = st >= 0 ? STEER_CENTER_US + st * (STEER_RIGHT_US - STEER_CENTER_US)
                        : STEER_CENTER_US + st * (STEER_CENTER_US - STEER_LEFT_US);
  int escUs = thr >= 0 ? ESC_NEUTRAL_US + thr * (ESC_FWD_MAX_US - ESC_NEUTRAL_US)
                       : ESC_NEUTRAL_US + thr * (ESC_NEUTRAL_US - ESC_REV_MAX_US);
  ledcWrite(PIN_STEER, usToDuty(steerUs));
  ledcWrite(PIN_ESC, usToDuty(escUs));
  digitalWrite(PIN_LED, linkOk ? HIGH : (now / 150 % 2));

  if (now - lastLog > 500) {
    lastLog = now;
    Serial.printf("link=%d steer=%dus esc=%dus bad=%lu\n", linkOk, steerUs, escUs, (unsigned long)badPackets);
  }
  delay(5);
}
