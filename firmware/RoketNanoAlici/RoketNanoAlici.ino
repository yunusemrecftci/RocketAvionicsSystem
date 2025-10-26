#include <Arduino.h>
#include <SoftwareSerial.h>
#include "LoRa_E22.h"   // xreef LoRa E22 kütüphanesi

// --- Uno pinleri ---
// E22 TX -> Uno RX_PIN,  E22 RX -> Uno TX_PIN
#define RX_PIN 8   // Uno dijital pin 5
#define TX_PIN 9   // Uno dijital pin 4

SoftwareSerial e22Serial(RX_PIN, TX_PIN); // RX, TX
LoRa_E22 e22(&e22Serial);

static const uint8_t  HDR        = 0x67;
static const size_t   FRAME_SIZE = 46;

enum RxState { WAIT_HDR, COLLECT } rxState = WAIT_HDR;
uint8_t frame[FRAME_SIZE];
uint8_t idx = 0;

static const uint32_t GAP_TIMEOUT_MS = 30;
uint32_t last_rx_ms = 0;

struct TelePkt {
  uint8_t hdr;      // 0xFF
  uint8_t body[44]; // 11 float (LE)
  uint8_t state;    // durum
};

static inline float leFloat(const uint8_t* p) {
  float f; memcpy(&f, p, 4); return f;
}

void setup() {
  // USB seri monitor için
  Serial.begin(19200);
  while (!Serial) { ; } // Uno'da gerekli değil ama uyumluluk için

  // LoRa UART (SoftwareSerial)
  e22Serial.begin(9600);
  e22.begin();

  Serial.println(F("LoRa E22 RX (Arduino Uno)"));
  Serial.println(F("46 byte TelePkt -> tek satir JSON olarak USB'ye yazilir"));
}

void printJsonOneLine(const TelePkt& p) {
  const uint8_t* b = p.body;

  float alt   = leFloat(&b[ 0]);
  float gpsA  = leFloat(&b[ 4]);
  float lat   = leFloat(&b[ 8]);
  float lng   = leFloat(&b[12]);
  float eulX  = leFloat(&b[16]);
  float eulZ  = leFloat(&b[20]);
  float eulY  = leFloat(&b[24]);
  float pitch = leFloat(&b[28]);
  float accX  = leFloat(&b[32]);
  float accZ  = leFloat(&b[36]);
  float accY  = leFloat(&b[40]);

  Serial.print(F("{\"alt\":"));   Serial.print(alt, 2);
  Serial.print(F(",\"gpsAlt\":")); Serial.print(gpsA, 2);
  Serial.print(F(",\"lat\":"));   Serial.print(lat, 6);
  Serial.print(F(",\"lng\":"));   Serial.print(lng, 6);
  Serial.print(F(",\"eulX\":"));  Serial.print(eulX, 2);
  Serial.print(F(",\"eulY\":"));  Serial.print(eulY, 2);
  Serial.print(F(",\"eulZ\":"));  Serial.print(eulZ, 2);
  Serial.print(F(",\"pitch\":")); Serial.print(pitch, 2);
  Serial.print(F(",\"accX\":"));  Serial.print(accX, 3);
  Serial.print(F(",\"accY\":"));  Serial.print(accY, 3);
  Serial.print(F(",\"accZ\":"));  Serial.print(accZ, 3);
  Serial.print(F(",\"state\":")); Serial.print(p.state);
  Serial.println("}");
}

void loop() {
  while (e22Serial.available()) {
    uint8_t b = (uint8_t)e22Serial.read();
    last_rx_ms = millis();

    if (rxState == WAIT_HDR) {
      if (b == HDR) {
        frame[0] = b;
        idx = 1;
        rxState = COLLECT;
      }
    } else {
      if (idx < FRAME_SIZE) frame[idx++] = b;
      if (idx >= FRAME_SIZE) {
        const TelePkt* p = reinterpret_cast<const TelePkt*>(frame);
        if (p->hdr == HDR) {
          printJsonOneLine(*p);
        }
        rxState = WAIT_HDR;
        idx = 0;
      }
    }
  }

  if (rxState == COLLECT && (millis() - last_rx_ms) > GAP_TIMEOUT_MS) {
    rxState = WAIT_HDR;
    idx = 0;
  }
}