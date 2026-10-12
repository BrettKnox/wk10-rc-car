// WK-10 transmitter: a second ESP32 dev board + two 10k pots + one tact switch.
// Board: "ESP32 Dev Module", Arduino-ESP32 core 3.x. Power from a USB power bank.
//
// Wiring: steer pot wiper -> GPIO34, throttle pot wiper -> GPIO35, pot ends -> 3V3/GND.
//         dead-man tact switch GPIO27 -> GND (internal pull-up). LED on GPIO2.
//         Each pot end goes through a 1k resistor (10k pot), so real travel reads about 340..3755.
//         A reading outside that band means a broken wire: the TX sends stop, not full lock/throttle.
// The throttle pot has no return spring, so throttle is zero unless the dead-man is held,
// and the TX refuses to start until the throttle pot sits within the deadband of centre.
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WkLink.h>   // firmware/libraries/WkLink

const int PIN_STEER = 34, PIN_THR = 35, PIN_DEADMAN = 27, PIN_LED = 2;
// Car MAC printed by car_rx on boot. The TX will not send until this is set.
uint8_t CAR_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
const int RAW_MIN = 340, RAW_MAX = 3755, RAW_FAULT_MARGIN = 150;   // with 1k end resistors on a 10k pot
const int DEADBAND = 40;          // of 1000
const bool INVERT_STEER = false, INVERT_THR = false;
const uint32_t PERIOD_MS = 20;    // 50 Hz, same as the servo frame

int centerSteer = 2048, centerThr = 2048;

static int readAvg(int pin) {
  long s = 0;
  for (int i = 0; i < 16; i++) s += analogRead(pin);
  return s / 16;
}

static bool potFault(int raw) { return raw < RAW_MIN - RAW_FAULT_MARGIN || raw > RAW_MAX + RAW_FAULT_MARGIN; }

static int16_t scale(int raw, int center, bool invert) {
  int span = raw >= center ? RAW_MAX - center : center - RAW_MIN;
  int v = span > 0 ? (long)(raw - center) * 1000 / span : 0;
  if (abs(v) < DEADBAND) v = 0;
  v = constrain(v, -1000, 1000);
  return invert ? -v : v;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_DEADMAN, INPUT_PULLUP);
  analogReadResolution(12);
  centerSteer = readAvg(PIN_STEER);             // whatever the knobs read at power-on is centre
  centerThr = readAvg(PIN_THR);
  if (abs(centerThr - 2048) > 400) {            // throttle knob not near mid-travel: refuse
    Serial.println("throttle knob not centred: turn it to the middle and reset");
    while (true) { digitalWrite(PIN_LED, millis() / 80 % 2); delay(10); }
  }

  bool unbound = true;
  for (int i = 0; i < 6; i++) unbound &= CAR_MAC[i] == 0xFF;
  if (unbound) {                                // never broadcast: every WK10 car in range would listen
    Serial.println("set CAR_MAC to the MAC car_rx prints, then reflash");
    while (true) { digitalWrite(PIN_LED, millis() / 300 % 2); delay(10); }
  }
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(WK_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) { Serial.println("esp_now_init failed"); while (true) delay(1000); }
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, CAR_MAC, 6);
  peer.channel = WK_CHANNEL;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
  Serial.print("tx MAC (paste into car_rx.ino TX_MAC): ");
  Serial.println(WiFi.macAddress());
}

void loop() {
  static uint16_t seq = 0;
  static uint32_t next = 0;
  if ((int32_t)(millis() - next) < 0) return;  // wrap-safe
  next = millis() + PERIOD_MS;

  WkPacket p;
  p.magic = WK_MAGIC;
  p.seq = seq++;
  int rs = readAvg(PIN_STEER), rt = readAvg(PIN_THR);
  bool fault = potFault(rs) || potFault(rt);
  p.steer = fault ? 0 : scale(rs, centerSteer, INVERT_STEER);
  bool held = !fault && digitalRead(PIN_DEADMAN) == LOW;
  p.throttle = held ? scale(rt, centerThr, INVERT_THR) : 0;
  p.flags = held ? 1 : 0;
  p.sum = wk_sum(p);
  bool ok = esp_now_send(CAR_MAC, (uint8_t *)&p, sizeof p) == ESP_OK;
  digitalWrite(PIN_LED, fault ? millis() / 100 % 2 : ok && held);
}
