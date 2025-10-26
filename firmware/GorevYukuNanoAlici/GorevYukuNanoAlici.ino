#include <SoftwareSerial.h>
#include "LoRa_E22.h"

// *** LoRa için pin tanımları (Nano tarafı) ***
#define LORA_RX_PIN 8   // Nano RX (LoRa TX'e bağlanacak)
#define LORA_TX_PIN 9   // Nano TX (LoRa RX'e bağlanacak)
#define LORA_BAUD_RATE 9600

SoftwareSerial loraSerial(LORA_RX_PIN, LORA_TX_PIN);
LoRa_E22 e22ttl(&loraSerial);

// === Union ve struct aynı ===
typedef union { float f; byte b[4]; } FloatToBytes;

struct DataPacket {
  byte header;              
  FloatToBytes latitude;
  FloatToBytes longitude;
  FloatToBytes altitude;
  FloatToBytes rms_internal;
  FloatToBytes rms_external;
};

void setup() {
  Serial.begin(115200);       // PC için JSON çıkış
  loraSerial.begin(LORA_BAUD_RATE);
  e22ttl.begin();

  Serial.println("=== LoRa Receiver NANO ===");
}

void loop() {
  if (e22ttl.available() > 0) {
    // Gelen paket boyutu: 21 byte
    ResponseStructContainer rsc = e22ttl.receiveMessage(sizeof(DataPacket));
    if (rsc.status.code == 1) {
      DataPacket packet = *(DataPacket*)rsc.data;

      // Paket kontrolü
      if (packet.header == 0x52) {
        float lat      = packet.latitude.f;
        float lng      = packet.longitude.f;
        float alt      = packet.altitude.f;
        float rms_int  = packet.rms_internal.f;
        float rms_ext  = packet.rms_external.f;

        // JSON formatında bilgisayara gönder
        Serial.print("{\"header\":"); Serial.print(packet.header);
        Serial.print(",\"lat\":");    Serial.print(lat, 6);
        Serial.print(",\"lng\":");    Serial.print(lng, 6);
        Serial.print(",\"alt\":");    Serial.print(alt, 1);
        Serial.print(",\"rms_internal\":"); Serial.print(rms_int, 4);
        Serial.print(",\"rms_external\":"); Serial.print(rms_ext, 4);
        Serial.println("}");
      }
    }
    rsc.close();
  }
}
