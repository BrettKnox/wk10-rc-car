// WK-10 transmitter: a second ESP32 dev board + two 10k pots + one tact switch.
// Board: "ESP32 Dev Module", Arduino-ESP32 core 3.x. Power from a USB power bank.
//
// Wiring: steer pot wiper -> GPIO34, throttle pot wiper -> GPIO35.
//         Each pot: 3V3 -> 4.7k -> pot -> 2.2k -> GND, so travel sits at about 0.43..2.38 V,
//         inside the range IDF calibrates well (0.15..2.45 V at 12 dB). A reading outside
//         MV_MIN..MV_MAX (with margin), or a raw reading pinned at either rail, means a broken
//         wire, and the TX sends stop instead of full lock or throttle.
//         Dead-man tact switch GPIO27 -> GND (internal pull-up; a broken wire reads "released").
//         LED on GPIO2.
// The throttle pot has no return spring, so throttle is zero unless the dead-man is held,
// and the TX refuses to start until the throttle pot sits near centre.
// Serial prints the pot millivolts twice a second: turn each knob to both stops and copy the
// numbers into MV_MIN / MV_MAX.
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WkLink.h>   // firmware/libraries/WkLink

const int PIN_STEER = 34, PIN_THR = 35, PIN_DEADMAN = 27, PIN_LED = 2;
// Car MAC printed by car_rx on boot. The TX will not send until this is set.
uint8_t CAR_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
const int MV_MIN = 430, MV_MAX = 2380;   // measured knob stops (calibration knob); 4.7k top / 2.2k bottom, 10k pot
const int MV_FAULT_MARGIN = 150;         // beyond the stops by this much = broken wire
const int RAW_RAIL_LO = 60, RAW_RAIL_HI = 4000;   // raw ADC pinned at a rail = open GND or 3V3 lead
const int DEADBAND = 40;                 // of 1000
const bool INVERT_STEER = false, INVERT_THR = false;
const uint32_t PERIOD_MS = 20;           // 50 Hz, same as the servo frame

int centerSteer = 1650, centerThr = 1650;

static int readMv(int pin) {
  long s = 0;
  for (int i = 0; i < 8; i++) s += analogReadMilliVolts(pin);
  return s / 8;
}

static bool potFault(int pin, int mv) {
  int raw = analogRead(pin);                   // independent of the mV calibration curve
  return raw <= RAW_RAIL_LO || raw >= RAW_RAIL_HI || mv < MV_MIN - MV_FAULT_MARGIN || mv > MV_MAX + MV_FAULT_MARGIN;
}

static int16_t scale(int mv, int center, bool invert) {
  int span = mv >= center ? MV_MAX - center : center - MV_MIN;
  int v = span > 0 ? (long)(mv - center) * 1000 / span : 0;
  if (abs(v) < DEADBAND) v = 0;
  v = constrain(v, -1000, 1000);
  return invert ? -v : v;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_DEADMAN, INPUT_PULLUP);
  analogSetAttenuation(ADC_11db);
  WiFi.mode(WIFI_STA);
  Serial.print("tx MAC (paste into car_rx.ino TX_MAC): ");
  Serial.println(WiFi.macAddress());

  centerSteer = readMv(PIN_STEER);              // whatever the knobs read at power-on is centre
  centerThr = readMv(PIN_THR);
  int mid = (MV_MIN + MV_MAX) / 2;
  if (abs(centerThr - mid) > (MV_MAX - MV_MIN) / 6) {   // throttle knob not near mid-travel: refuse
    Serial.printf("throttle knob reads %d mV, centre is about %d: turn it to the middle and reset\n", centerThr, mid);
    while (true) { digitalWrite(PIN_LED, millis() / 80 % 2); delay(10); }
  }

  bool unbound = true;
  for (int i = 0; i < 6; i++) unbound &= CAR_MAC[i] == 0xFF;
  if (unbound) {                                // never broadcast: every WK10 car in range would listen
    Serial.println("set CAR_MAC to the MAC car_rx prints, then reflash");
    while (true) { digitalWrite(PIN_LED, millis() / 300 % 2); delay(10); }
  }
  esp_wifi_set_channel(WK_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) { Serial.println("esp_now_init failed"); while (true) delay(1000); }
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, CAR_MAC, 6);
  peer.channel = WK_CHANNEL;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
}

void loop() {
  static uint16_t seq = 0;
  static uint32_t next = 0, lastLog = 0;
  if ((int32_t)(millis() - next) < 0) return;  // wrap-safe
  next = millis() + PERIOD_MS;

  WkPacket p;
  p.magic = WK_MAGIC;
  p.seq = seq++;
  int ms = readMv(PIN_STEER), mt = readMv(PIN_THR);
  bool fault = potFault(PIN_STEER, ms) || potFault(PIN_THR, mt);
  p.steer = fault ? 0 : scale(ms, centerSteer, INVERT_STEER);
  bool held = !fault && digitalRead(PIN_DEADMAN) == LOW;
  p.throttle = held ? scale(mt, centerThr, INVERT_THR) : 0;
  p.flags = held ? 1 : 0;
  p.sum = wk_sum(p);
  bool ok = esp_now_send(CAR_MAC, (uint8_t *)&p, sizeof p) == ESP_OK;
  digitalWrite(PIN_LED, fault ? millis() / 100 % 2 : ok && held);
  if (millis() - lastLog > 500) {
    lastLog = millis();
    Serial.printf("steer %d mV  throttle %d mV  %s\n", ms, mt, fault ? "POT FAULT" : "");
  }
}
