/*
 * Rocket Flight Computer - C++ Implementation
 * 
 * This file contains the complete flight computer implementation
 * for autonomous rocket control and recovery.
 * 
 * Hardware: Teensy 4.x + LoRa E22 + BMP388 + BNO055 + GPS
 * 

 */

#ifndef ROCKET_FLIGHT_COMPUTER_H
#define ROCKET_FLIGHT_COMPUTER_H

// =====================================================================
// EK-6 Rev4 Uyumlu UKB Test Firmware (SIT + SUT + Durum Paketleri)
// (Teensy 4.x + LoRa E22 + BMP388 + BNO055 + TinyGPS++ + RS232)
// =====================================================================

#include <Arduino.h>
#include <Wire.h> // I2C iletişimi için gerekli
#include <Adafruit_Sensor.h> // Adafruit sensör kütüphanesi
#include <Adafruit_BMP3XX.h> // BMP388 sensör kütüphanesi
#include <Adafruit_BNO055.h> // BNO055 IMU sensör kütüphanesi
#include <TinyGPS++.h> // TinyGPS++ kütüphanesi
#include "LoRa_E22.h" // LoRa E22 modül kütüphanesi
#include <utility/imumaths.h> // IMU matematiksel fonksiyonları için (BNO055 kütüphanesinden gelir)
#include <RunningAverage.h> // RunningAverage kütüphanesi eklendi
#include <HardwareSerial.h> // Donanımsal UART kullanımı için gerekli kütüphane

// =====================================================================
// CONFIGURATION CONSTANTS
// =====================================================================

// -------- Kullanıcı Ayarları --------
const float ANGLE_THR       = 40.0; // Pitch açısı eşiği (derece)
const unsigned long ANGLE_HOLD_MS = 150; // Pitch eşiği üzerinde kalma süresi (ms)
const float DESC_DROP_M     = 10.0; // İniş algılaması için irtifa düşüşü (metre)
constexpr float SEA_LEVEL_HPA = 1013.25; // Deniz seviyesi barometrik basıncı (hPa) - Kendi konumunuza göre kalibre edin!

// -------- Pin ve Sabitler --------
constexpr uint8_t SDA_PIN     = 17, SCL_PIN     = 16; // I2C SDA ve SCL pinleri
constexpr uint8_t PYRO1_PIN   = 23, PYRO2_PIN   = 38; // Pyro ateşleme pinleri
const uint8_t FRAME_HDR      = 0xAA; // Komutlar için Başlık
const uint8_t HDR_SUT        = 0xAB; // Sentetik Veri Paketleri (SUT) ve SIT Veri Paketleri için Başlık
const uint8_t FRAME_FTR1     = 0x0D; // Paket sonu baytı 1
const uint8_t FRAME_FTR2     = 0x0A; // Paket sonu baytı 2
const uint8_t CMD_SIT        = 0x20; // SIT (Veri Gönderim) modunu başlatma komutu (önceki CMD_START)
const uint8_t CMD_SUT        = 0x22; // SUT (Sentetik Veri Alma) modunu başlatma komutu
const uint8_t CMD_STOP       = 0x24; // Modu durdurma (NORMAL moda dönme) komutu
static unsigned long pyro1Timer = 0, pyro2Timer = 0;
const unsigned long PYRO_PULSE_MS = 1000; // 1 saniye pulse süresi

// -------- Debug Flagleri --------
#define DEBUG_FLIGHT_ALGO // Uçuş algoritması debug çıktılarını etkinleştirir
#define DEBUG_SENSORS     // Sensör okuma debug çıktılarını etkinleştirir

// -------- UART Tanımları --------
#define GPSSerial    Serial8 // GPS için UART
#define LoraSerial   Serial1 // LoRa için UART
#define RS232Serial Serial2 // RS232 (Yer İstasyonu) için UART

// =====================================================================
// GLOBAL VARIABLES
// =====================================================================

// -------- Sensör ve Modüller --------
LoRa_E22          e22(&LoraSerial); // LoRa E22 modülü objesi - AUX pini yok
TinyGPSPlus       gps; // TinyGPS++ objesi
Adafruit_BMP3XX bmp; // BMP388 barometrik sensör objesi
Adafruit_BNO055 bno(55, 0x28, &Wire1); // BNO055 IMU sensör objesi (I2C adresi 0x28, Wire1 bus'ı)

// -------- Global Sensör Verileri --------
sensors_event_t gEuler, gAcc; // BNO055'ten okunan Euler açıları ve ivme verileri
float baroHpa = 0; // Barometrik basınç (hPa)

// -------- Burnout Algoritması Değişkenleri --------
float maxAccelTotal = 0; // Burnout maksimum ivme
bool burnoutCandidate = false; // Burnout aday durumu
unsigned long burnoutCandidateTime = 0; // Burnout aday zamanı

// -------- Uçuş Değişkenleri --------
float   fAlt = 0; // Filtrelenmiş irtifa (metre)
float   pitchDeg = 0; // Pitch açısı (derece)
float   maxAlt = 0; // Ulaşılan maksimum irtifa
float   sutMaxAlt = 0; // SUT verileri için maksimum irtifa
bool    launch = false; // Fırlatma durumu
bool    burnout = false; // Motor yanma bitiş durumu
bool    desc = false; // İniş (apogee sonrası) durumu
bool    angleFlag = false; // Belirli bir açı eşiğinin üzerinde kalma durumu
bool    sep1 = false; // Drogue paraşütü ayrılma durumu
bool    sep2 = false; // Ana paraşüt ayrılma durumu
bool    belowMin = false; // İrtifaın minimum drogue irtifasının altına düşme durumu
uint16_t statusBits = 0; // Uçuş durum bitlerini içeren değişken

// -------- RunningAverage Filtresi --------
RunningAverage runningAvgAlt(1); // İrtifa filtresi - hızlı güncelleme
RunningAverage runningAvgAccX(2); // İvme X filtresi - hızlı güncelleme
RunningAverage runningAvgAccY(2); // İvme Y filtresi - hızlı güncelleme
RunningAverage runningAvgAccZ(2); // İvme Z filtresi - hızlı güncelleme
RunningAverage runningAvgAngleX(2); // Açı X filtresi - hızlı güncelleme
RunningAverage runningAvgAngleY(2); // Açı Y filtresi - hızlı güncelleme
RunningAverage runningAvgAngleZ(2); // Açı Z filtresi - hızlı güncelleme

// Çalışma modlarını tanımlayan enum yapısı
enum Mode { NORMAL } currentMode = NORMAL; // Yalnızca NORMAL mod
unsigned long tSens = 0; // Sensör okuma zamanlayıcısı

// Veri gönderme sıklığı için zamanlayıcı
unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL_MS = 150; // LoRa ve SIT veri gönderme aralığı (ms) - 5 Hz için

// -------- Telemetri Paket --------
// Union kullanımı, float verilerini byte dizisine dönüştürmek için yaygın ve etkili bir yöntem.
typedef union { float f; uint8_t b[4]; } F2B;
// __attribute__((packed)) kullanmak bellek hizalamasını optimize eder ve paketin boyutunu düşürür.
struct __attribute__((packed)) TelePkt { uint8_t hdr; uint8_t body[44]; uint8_t state; };
TelePkt pkt46; // LoRa telemetri paketi

// UART2 yapılandırması (RS232 bağlantısı için)
// Teensy 4.x için Serial2 kullanıyoruz - pinler otomatik atanır

// Sabit tanımlar (paket yapısı ve komutlar)
#define PKT_HEADER     0xAA  // Tüm paketlerin başında olacak sabit değer
#define PKT_TELEM_HDR  0xAB  // Telemetri verisi paketi başlangıç baytı
#define PKT_FOOTER1    0x0D  // Paket sonu (carriage return)
#define PKT_FOOTER2    0x0A  // Paket sonu (newline)
#define CMD_SIT_START  0x20  // SİT modunu başlatan komut
#define CMD_SUT_START  0x22  // SUT modunu başlatan komut
#define CMD_STOP       0x24  // Her iki modu durduran komut

// Mod durumlarını tutan bayraklar
bool sit_active = false; // SİT modunun aktif olup olmadığını tutar
bool sut_active = false; // SUT modunun aktif olup olmadığını tutar

// Gönderim zamanlayıcıları (millis ile periyodik gönderim için)
unsigned long lastSend = 0;         // SİT verisinin en son gönderildiği zaman
unsigned long lastDurumSend = 0;    // Durum paketinin en son gönderildiği zaman

// Durum bilgilendirme bitleri (Table 3'e göre düzenlendi)
bool durum_roket_kalkisi             = false;  // Bit 0: Roket kalkışı algılandı
bool durum_motor_yanma_onlem_suresi = false;  // Bit 1: Motor yanma önlem süresi
bool durum_mini_irtifa_esigi_asildi = false;  // Bit 2: Minimum irtifa eşiği aşıldı
bool durum_roket_govde_acisi_fazla  = false;  // Bit 3: Roket gövde açısı fazla
bool durum_irtifa_alcalmaya_basladi = false;  // Bit 4: Roket irtifası alçalmaya başladı
bool durum_suruklenme_parasut_emri  = false;  // Bit 5: Sürüklenme paraşüt emri
bool durum_irtifa_belirlenen_altinda= false;  // Bit 6: İrtifa belirlenen altında
bool durum_ana_parasut_emri         = false;  // Bit 7: Ana paraşüt emri

// float'ı byte dizisine dönüştürmek için birlik (union)
union FloatUnion {
  float f;
  byte bytes[4];
};

// float değerini Big Endian olarak byte dizisine yazar
void writeFloatBigEndian(byte* buf, int offset, float val) {
  byte* p = (byte*)&val;
  buf[offset + 0] = p[3]; // En yüksek bayt
  buf[offset + 1] = p[2];
  buf[offset + 2] = p[1];
  buf[offset + 3] = p[0]; // En düşük bayt
}

// Big Endian byte dizisini float'a çevirir
float readFloatBigEndian(byte* buf, int offset) {
  FloatUnion fu;
  fu.bytes[0] = buf[offset + 3]; // En düşük bayt
  fu.bytes[1] = buf[offset + 2];
  fu.bytes[2] = buf[offset + 1];
  fu.bytes[3] = buf[offset + 0]; // En yüksek bayt
  return fu.f;
}

// Little Endian byte dizisini float'a çevirir
float readFloatLittleEndian(byte* buf, int offset) {
  FloatUnion fu;
  fu.bytes[0] = buf[offset + 0]; // En düşük bayt
  fu.bytes[1] = buf[offset + 1];
  fu.bytes[2] = buf[offset + 2];
  fu.bytes[3] = buf[offset + 3]; // En yüksek bayt
  return fu.f;
}

// SUT paketi için checksum hesaplar (0-32. baytlar dahil)
byte calculateChecksum(byte* buf) {
  uint32_t sum = 0;
  for (int i = 0; i <= 32; i++) {
    sum += buf[i]; // Toplamı al
  }
  return (byte)(sum & 0xFF); // Son 8 biti geri döndür
}

// Komut paketini doğrular (header ve footer kontrolü)
bool verifyCommandPacket(byte* pkt) {
  return (pkt[0] == PKT_HEADER &&
          pkt[3] == PKT_FOOTER1 &&
          pkt[4] == PKT_FOOTER2); // Paket formatı geçerli mi
}

// Little-Endian'dan Big-Endian'a çeviren fonksiyon
inline void toBigEndian(uint8_t* bytes) { 
  uint8_t temp = bytes[0];
  bytes[0] = bytes[3];
  bytes[3] = temp;
  temp = bytes[1];
  bytes[1] = bytes[2];
  bytes[2] = temp;
}

// =====================================================================
// FUNCTION DECLARATIONS
// =====================================================================

// Core functions
void setup();
void loop();

// Sensor functions
void readSensors();
void gpsDecode(unsigned long ms);

// Flight algorithm functions
void flightAlgo();

// Communication functions
void sendLoRa();
void readCommand();
void sendSitTelemetry();
void receiveSutPacket();
void sendDurumBilgilendirme();
bool waitForLoRaReady(unsigned long timeout = 1000);

// Utility functions
byte calculateChecksum(byte* buf);
bool verifyCommandPacket(byte* pkt);
void writeFloatBigEndian(byte* buf, int offset, float val);
float readFloatBigEndian(byte* buf, int offset);
float readFloatLittleEndian(byte* buf, int offset);
inline void toBigEndian(uint8_t* bytes);

// =====================================================================
// IMPLEMENTATION
// =====================================================================

// Sabit tanımlar (paket yapısı ve komutlar)
#define PKT_HEADER     0xAA  // Tüm paketlerin başında olacak sabit değer
#define PKT_TELEM_HDR  0xAB  // Telemetri verisi paketi başlangıç baytı
#define PKT_FOOTER1    0x0D  // Paket sonu (carriage return)
#define PKT_FOOTER2    0x0A  // Paket sonu (newline)
#define CMD_SIT_START  0x20  // SİT modunu başlatan komut
#define CMD_SUT_START  0x22  // SUT modunu başlatan komut
#define CMD_STOP       0x24  // Her iki modu durduran komut

// Mod durumlarını tutan bayraklar
bool sit_active = false; // SİT modunun aktif olup olmadığını tutar
bool sut_active = false; // SUT modunun aktif olup olmadığını tutar

// Gönderim zamanlayıcıları (millis ile periyodik gönderim için)
unsigned long lastSend = 0;         // SİT verisinin en son gönderildiği zaman
unsigned long lastDurumSend = 0;    // Durum paketinin en son gönderildiği zaman

// Durum bilgilendirme bitleri (Table 3'e göre düzenlendi)
bool durum_roket_kalkisi             = false;  // Bit 0: Roket kalkışı algılandı
bool durum_motor_yanma_onlem_suresi = false;  // Bit 1: Motor yanma önlem süresi
bool durum_mini_irtifa_esigi_asildi = false;  // Bit 2: Minimum irtifa eşiği aşıldı
bool durum_roket_govde_acisi_fazla  = false;  // Bit 3: Roket gövde açısı fazla
bool durum_irtifa_alcalmaya_basladi = false;  // Bit 4: Roket irtifası alçalmaya başladı
bool durum_suruklenme_parasut_emri  = false;  // Bit 5: Sürüklenme paraşüt emri
bool durum_irtifa_belirlenen_altinda= false;  // Bit 6: İrtifa belirlenen altında
bool durum_ana_parasut_emri         = false;  // Bit 7: Ana paraşüt emri

// float'ı byte dizisine dönüştürmek için birlik (union)
union FloatUnion {
  float f;
  byte bytes[4];
};

// float değerini Big Endian olarak byte dizisine yazar
void writeFloatBigEndian(byte* buf, int offset, float val) {
  byte* p = (byte*)&val;
  buf[offset + 0] = p[3]; // En yüksek bayt
  buf[offset + 1] = p[2];
  buf[offset + 2] = p[1];
  buf[offset + 3] = p[0]; // En düşük bayt
}

// Big Endian byte dizisini float'a çevirir
float readFloatBigEndian(byte* buf, int offset) {
  FloatUnion fu;
  fu.bytes[0] = buf[offset + 3]; // En düşük bayt
  fu.bytes[1] = buf[offset + 2];
  fu.bytes[2] = buf[offset + 1];
  fu.bytes[3] = buf[offset + 0]; // En yüksek bayt
  return fu.f;
}

// Little Endian byte dizisini float'a çevirir
float readFloatLittleEndian(byte* buf, int offset) {
  FloatUnion fu;
  fu.bytes[0] = buf[offset + 0]; // En düşük bayt
  fu.bytes[1] = buf[offset + 1];
  fu.bytes[2] = buf[offset + 2];
  fu.bytes[3] = buf[offset + 3]; // En yüksek bayt
  return fu.f;
}

// SUT paketi için checksum hesaplar (0-32. baytlar dahil)
byte calculateChecksum(byte* buf) {
  uint32_t sum = 0;
  for (int i = 0; i <= 32; i++) {
    sum += buf[i]; // Toplamı al
  }
  return (byte)(sum & 0xFF); // Son 8 biti geri döndür
}

// Komut paketini doğrular (header ve footer kontrolü)
bool verifyCommandPacket(byte* pkt) {
  return (pkt[0] == PKT_HEADER &&
          pkt[3] == PKT_FOOTER1 &&
          pkt[4] == PKT_FOOTER2); // Paket formatı geçerli mi
}

// Little-Endian'dan Big-Endian'a çeviren fonksiyon
inline void toBigEndian(uint8_t* bytes) { 
  uint8_t temp = bytes[0];
  bytes[0] = bytes[3];
  bytes[3] = temp;
  temp = bytes[1];
  bytes[1] = bytes[2];
  bytes[2] = temp;
}

// =====================================================================
// CORE FUNCTIONS
// =====================================================================

void setup() {
  Serial.begin(115200); // PC ile haberleşme için seri portu başlat (genellikle debug için)
  RS232Serial.begin(115200); // Harici cihaz (Yer İstasyonu) ile haberleşme için seri portu başlat
  GPSSerial.begin(9600);    // GPS UART'ı başlat
  LoraSerial.begin(9600);   // LoRa UART'ı başlat
  RS232Serial.setTimeout(50);
  e22.begin(); // LoRa E22 modülünü başlat (AUX pini kullanılmadığı varsayılıyor)

  // UART2 başlat
  Serial2.begin(115200); // UART2 başlat (Teensy 4.x için pin ayarları otomatik)

  Wire1.setSDA(SDA_PIN); // SDA pinini ayarla
  Wire1.setSCL(SCL_PIN); // SCL pinini ayarla
  Wire1.begin(); // Wire1 (I2C) iletişimi başlatılıyor.

  if (!bmp.begin_I2C(0x77, &Wire1)) {
    while (1) { }
  }
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ); // Sensör okuma sıklığı.

  if (!bno.begin()) {
    while (1) { }
  }
  delay(1000); // BNO055 kalibrasyonu için kısa bir bekleme süresi faydalı olabilir.
  bno.setExtCrystalUse(true); // Harici kristal kullanımı daha kararlı zamanlama sağlar.
  digitalWrite(PYRO1_PIN, LOW); // Başlangıçta LOW - güvenli
  digitalWrite(PYRO2_PIN, LOW);
  pinMode(PYRO1_PIN, OUTPUT);
  pinMode(PYRO2_PIN, OUTPUT);
  // Başlangıçta LOW - güvenli
  
  // Pyro pinlerini güvenli başlat
  Serial.println("Pyro pinleri güvenli başlatıldı (LOW)");
  
  pkt46.hdr = 0xFF; // Telemetri paket başlığı başlangıç değeri.

  // RS232Serial hazır olana kadar bekle (maksimum 5 saniye)
  unsigned long startWait = millis();
  while (!RS232Serial && (millis() - startWait < 5000)) {
    delay(10);
  }
  if (!RS232Serial) {
    // RS232 başlatılamadı
  }

  // İlk sensör okumalarını yap
  Serial.println("Sensörler kalibre ediliyor...");
  for (int i = 0; i < 20; i++) {
    readSensors();
    delay(50); // Gecikme, sensör okumaları arasında stabilite sağlar.
  }
  
  // BMP sensörünü olduğu gibi kabul et
  maxAlt = fAlt; // Maksimum irtifa başlangıç değerinden başlasın
  
  Serial.print("Başlangıç irtifası: ");
  Serial.print(fAlt);
  Serial.println(" metre");
  
  Serial.println("Sensör kalibrasyonu tamamlandı!");
}

void loop() {
  gpsDecode(100); // GPS verilerini okumak için bloklamayan bir yaklaşım (süre artırıldı).
  readCommand(); // Gelen komutları kontrol et

  // NORMAL mod (Uçuş algoritması ve LoRa telemetri)
  if (millis() - tSens >= 100) { // Sensör okuma sıklığı kontrolü
    tSens = millis();
    readSensors(); // Gerçek sensör verilerini oku
    
    // SIT modunda uçuş algoritması çalışmasın
    if (!sit_active) {
      flightAlgo(); // Gerçek veri ile uçuş algoritması
    }
  }
  
  // LoRa gönderme sıklığı kontrolü (bağımsız)
  if (millis() - lastSendTime >= SEND_INTERVAL_MS) {
    lastSendTime = millis();
    sendLoRa();
  }

  // SİT modunda isek verileri gönder (sensör okumasından bağımsız)
  if (sit_active && millis() - lastSend >= 50) { // 20 Hz (50ms) - daha hızlı
    sendSitTelemetry();
    lastSend = millis();
  }

  // SUT modunda isek hem veri al hem durum gönder
  if (sut_active) {
    receiveSutPacket(); // RS232 üzerinden gelen veriyi al

    if (millis() - lastDurumSend >= 100) {
      sendDurumBilgilendirme(); // 10 Hz durum paketi gönder
      lastDurumSend = millis();
    }
  }
}

// =====================================================================
// SENSOR FUNCTIONS
// =====================================================================

void readSensors() {
  if (!bmp.performReading()) {
    return;
  }
  baroHpa = bmp.pressure / 100.0f; // Pascal'dan hPa'ya dönüşüm.
  float rawAlt = bmp.readAltitude(SEA_LEVEL_HPA);
  
  // BMP sensörünü olduğu gibi kabul et - filtreleme yok
  fAlt = rawAlt;

  bno.getEvent(&gEuler, Adafruit_BNO055::VECTOR_EULER); // Euler açıları (Roll, Pitch, Yaw)
  bno.getEvent(&gAcc, Adafruit_BNO055::VECTOR_LINEARACCEL); // Doğrusal ivme (yerçekimi çıkarılmış)
  
  // Normal sensör verilerini ortak filtrelerle filtrele
  runningAvgAccX.addValue(gAcc.acceleration.x);
  runningAvgAccY.addValue(gAcc.acceleration.y);
  runningAvgAccZ.addValue(gAcc.acceleration.z);
  runningAvgAngleX.addValue(gEuler.orientation.x);
  runningAvgAngleY.addValue(gEuler.orientation.y);
  runningAvgAngleZ.addValue(gEuler.orientation.z);
  
  // Filtrelenmiş değerleri geri ata
  gAcc.acceleration.x = runningAvgAccX.getAverage();
  gAcc.acceleration.y = runningAvgAccY.getAverage();
  gAcc.acceleration.z = runningAvgAccZ.getAverage();
  gEuler.orientation.x = runningAvgAngleX.getAverage();
  gEuler.orientation.y = runningAvgAngleY.getAverage();
  gEuler.orientation.z = runningAvgAngleZ.getAverage();
  
  pitchDeg = fabsf(gEuler.orientation.y);
}

// =====================================================================
// FLIGHT ALGORITHM FUNCTIONS
// =====================================================================

void flightAlgo() {
  // Fırlatma algılama - Sadece geçerli irtifa değerleri için
  if (!launch && fAlt > 5.0f && fAlt < 10000) {
    launch = true;
  }

  // >>> BURNOUT TESPİT ALGORİTMASI - YENİ VERSİYON <<<
  // Bu algoritma ivmenin tepe noktasından sonraki kararlı düşüşü algılar.
  // Global değişkenler kullanılıyor (maxAccelTotal, burnoutCandidate, burnoutCandidateTime)
  // m/s^2 cinsinden, burnout'un başladığına dair ivme eşiği
  const float BURNOUT_ACCEL_THRESHOLD = 5.0f; 
  // ms cinsinden, burnout'u doğrulamak için geçmesi gereken süre
  const unsigned long BURNOUT_CONFIRM_TIME = 200; 

  if (launch && !burnout) {
    // Toplam ivme vektörünün büyüklüğünü hesapla
    float totalAccel = sqrt(gAcc.acceleration.x * gAcc.acceleration.x + 
                            gAcc.acceleration.y * gAcc.acceleration.y + 
                            gAcc.acceleration.z * gAcc.acceleration.z);
    
    // Fırlatma anından itibaren maksimum ivmeyi takip et
    if (totalAccel > maxAccelTotal) {
      maxAccelTotal = totalAccel;
    }

    // İvme, maksimum değerin %50'sinin altına düşerse ve
    // belirli bir eşiğin altına inerse burnout adayı olarak işaretle
    if (totalAccel < maxAccelTotal * 0.5f && totalAccel < BURNOUT_ACCEL_THRESHOLD) {
      if (!burnoutCandidate) {
        burnoutCandidate = true;
        burnoutCandidateTime = millis();
        #ifdef DEBUG_FLIGHT_ALGO
        Serial.println("Burnout adayi tespit edildi.");
        #endif
      } else {
        // Aday durumu belirli bir süre devam ederse burnout'u onayla
        if (millis() - burnoutCandidateTime >= BURNOUT_CONFIRM_TIME) {
          burnout = true;
          #ifdef DEBUG_FLIGHT_ALGO
          Serial.println("Burnout onaylandi!");
          #endif
        }
      }
    } else {
      // İvme tekrar yükselirse veya eşiğin üzerine çıkarsa adayı sıfırla
      burnoutCandidate = false;
    }
  }
  // >>> BURNOUT TESPİT ALGORİTMASI SONU <<<

  // Pitch eşiği - 40° için 100ms kontrol (optimize edildi)
  static unsigned long angleT = 0;
  static bool angleDetected = false;
  static unsigned long lastAngleCheck = 0;
  
  // Sadece 50ms'de bir kontrol et
  if (millis() - lastAngleCheck >= 50) {
    lastAngleCheck = millis();
    
    float absAngle = fabs(gEuler.orientation.y);
    if (absAngle > 10.0f) {
      if (!angleDetected) {
        angleT = millis();
        angleDetected = true;
      }
    } else {
      angleDetected = false;
    }
  }
  
  angleFlag = (angleDetected && (millis() - angleT) >= 100);

  // Apogee & Desc - Sadece geçerli irtifa değerleri için (SUT modunda değilse)
  if (!sut_active && fAlt > 0 && fAlt < 10000) { // Geçerli irtifa aralığı
    if (fAlt > maxAlt) {
      maxAlt = fAlt;
    }
  }
  
  // Alçalma kontrolü (basitleştirildi)
  if (!sut_active && !desc && maxAlt > 0 && fAlt > 20.0f && fAlt < 10000.0f && (maxAlt - fAlt) >= DESC_DROP_M) {
    desc = true;
  }

  // Drogue (sep1) açılma koşulları - Burnout gerekmez (MOSFET tetikleme) - Sadece 1 kere
  if (!sep1 && desc && fAlt > 100.0f) { // DROGUE_ALT_MIN yerine 100m
    digitalWrite(PYRO1_PIN, HIGH); // MOSFET tetikleme
    pyro1Timer = millis();
    sep1 = true;
    Serial.println("MOSFET Pyro 1 tetiklendi! (1 kere)");
  }
  // Pyro ateşleme süresi kontrolü (SUT modunda özel süreler)
  unsigned long pyro1PulseTime = PYRO_PULSE_MS; // Varsayılan 500ms
  unsigned long pyro2PulseTime = PYRO_PULSE_MS; // Varsayılan 500ms
  
  if (sut_active) {
    // SUT modunda özel süreler
    pyro1PulseTime = 2000; // Sürüklenme paraşütü 2 saniye
    pyro2PulseTime = 2000; // Ana paraşüt 2 saniye
  }
  
  if (sep1 && pyro1Timer != 0 && (millis() - pyro1Timer >= pyro1PulseTime)) {
    digitalWrite(PYRO1_PIN, LOW);
    pyro1Timer = 0;
  }
  // BelowMin irtifa kontrolü
  if (!belowMin && sep1 && fAlt <= 100.0f) {
    belowMin = true;
  }
  // Main (sep2) açılma koşulları - İrtifa azalmaya başladıktan sonra 400-600m aralığında (MOSFET tetikleme) - Sadece 1 kere
  if (!sep2 && desc && fAlt <= 600.0f && fAlt >= 400.0f) {
    digitalWrite(PYRO2_PIN, HIGH); // MOSFET tetikleme
    pyro2Timer = millis();
    sep2 = true;
    Serial.print("MOSFET Pyro 2 tetiklendi! (1 kere) İrtifa: "); Serial.print(fAlt); Serial.println("m");
  }
  if (sep2 && pyro2Timer != 0 && (millis() - pyro2Timer >= pyro2PulseTime)) {
    digitalWrite(PYRO2_PIN, LOW);
    pyro2Timer = 0;
  }

  // Durum bitleri (Yeni koşullara göre düzenlendi)
  statusBits = 0; // Her çağrıda sıfırlanması önemli.
  
  // Bit 0: Roket kalkışı - irtifa 10 metreyi geçince
  bool roket_kalkisi = (fAlt > 10.0f && fAlt < 10000 && fAlt > 5.0f); // Minimum 5 metre üzerinde olmalı
  statusBits |= (uint16_t(roket_kalkisi) << 0);
  
  // Bit 1: Motor yanma önlem süresi (burnout bayrağı ile)
  statusBits |= (uint16_t(burnout) << 1);
  
  // Bit 2: Minimum irtifa eşiği aşıldı - 1000 metreden sonra
  bool mini_irtifa_esigi = (fAlt > 1000.0f && fAlt < 10000 );
  statusBits |= (uint16_t(mini_irtifa_esigi) << 2);
  
  // Bit 3: Roket gövde açısı fazla - Y ekseni ile 40 dereceyi aşması ve 100ms devam etmesi
  bool roket_govde_acisi = angleFlag;
  statusBits |= (uint16_t(roket_govde_acisi) << 3);
  
  // Bit 4: Roket irtifası alçalmaya başladı - apogee sonrası
  statusBits |= (uint16_t(desc) << 4);
  
  // Bit 5: Sürüklenme paraşüt emri - irtifa azalmaya başladıktan sonra Y ekseni mutlak değeri 40'ı geçince ve 100ms devam edince (burnout gerekmez)
  bool suruklenme_parasut = (desc && angleFlag);
  statusBits |= (uint16_t(suruklenme_parasut) << 5);
  
  // Bit 6: İrtifa belirlenen altında - 600 metrenin altına düşerse (alçalma bağımsız)
  bool irtifa_altinda = (fAlt <= 800.0f && desc);
  statusBits |= (uint16_t(irtifa_altinda) << 6);
  
  // Bit 7: Ana paraşüt emri - 400-600m aralığında (alçalma bağımsız)
  bool ana_parasut = (fAlt <=600.0f && fAlt >= 400.0f && desc);
  statusBits |= (uint16_t(ana_parasut) << 7);
  
  // Durum bilgilendirme bitlerini güncelle (sadece SUT modunda)
  if (sut_active) {
    durum_roket_kalkisi = roket_kalkisi;
    durum_motor_yanma_onlem_suresi = burnout;
    durum_mini_irtifa_esigi_asildi = mini_irtifa_esigi;
    durum_roket_govde_acisi_fazla = roket_govde_acisi;
    durum_irtifa_alcalmaya_basladi = desc;
    durum_suruklenme_parasut_emri = suruklenme_parasut;
    durum_irtifa_belirlenen_altinda = irtifa_altinda;
    durum_ana_parasut_emri = ana_parasut;
  }
}

// =====================================================================
// COMMUNICATION FUNCTIONS
// =====================================================================

void sendLoRa() {
  uint8_t* p = pkt46.body;
  // Float'ı bayt dizisine kopyalamak ve işaretçiyi ilerletmek için lambda fonksiyonu
  auto pf = [&](float v) {
    F2B u;
    u.f = v;
    memcpy(p, u.b, 4);
    p += 4;
  };

  // Telemetri paketini sensör verileri ve uçuş durumuyla doldur
  pf(fAlt); // Filtrelenmiş irtifa
  
  // GPS verilerini güvenli şekilde ekle
  if (gps.altitude.isValid()) {
    pf(gps.altitude.meters()); // GPS irtifası
  } else {
    pf(0.0f); // GPS irtifası geçersizse 0 gönder
  }
  
  if (gps.location.isValid()) {
    pf(gps.location.lat()); // GPS enlemi
    pf(gps.location.lng()); // GPS boylamı
  } else {
    pf(0.0f); // GPS enlemi geçersizse 0 gönder
    pf(0.0f); // GPS boylamı geçersizse 0 gönder
  }
  pf(gEuler.orientation.x); // Jiroskop X (Euler açısı X)
  pf(gEuler.orientation.z); // Jiroskop Z (Euler açısı Z)
  pf(gEuler.orientation.y); // Jiroskop Y (Euler açısı Y)
  pf(pitchDeg); // Eğim açısı
  pf(gAcc.acceleration.x); // Doğrusal ivme X
  pf(gAcc.acceleration.z); // Doğrusal ivme Z
  pf(gAcc.acceleration.y); // Doğrusal ivme Y
  pkt46.state = uint8_t(statusBits & 0xFF); // Mevcut uçuş durumu

  // Telemetri paketini LoRa üzerinden gönder
  if (waitForLoRaReady(200)) { // 200ms timeout ile hazır olmasını bekle
    ResponseStatus rs = e22.sendFixedMessage(0, 2, 0x06, (uint8_t*)&pkt46, sizeof(pkt46));
    if (rs.code != 1) {
      Serial.print("LoRa gönderme hatası: ");
      Serial.println(rs.getResponseDescription());
    }
  } else {
    Serial.println("LoRa modülü hazır değil, paket gönderilemedi!");
  }
}

void gpsDecode(unsigned long ms) { // GPS verilerini okuyan ve işleyen fonksiyon
  unsigned long start = millis();
  do {
    while (GPSSerial.available()) {
      gps.encode(GPSSerial.read());
    }
  } while (millis() - start < ms);
  
  // GPS verilerini kontrol et ve debug bilgisi ver
  if (gps.altitude.isValid()) {
    Serial.print("GPS Roket irtifa: ");
    Serial.println(gps.altitude.meters());
  } 
  
  // GPS bağlantı kontrolü
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println(F("GPS verisi alınamıyor: bağlantıyı kontrol edin"));
  }
}

void readCommand() {
  static byte buffer[5]; // Gelen komutu saklayan tampon
  static byte index = 0; // Okuma indexi

  while (Serial2.available()) {
    byte b = Serial2.read(); // RS232 üzerinden gelen baytı oku

    if (index == 0 && b != PKT_HEADER) continue; // İlk bayt header değilse atla

    buffer[index++] = b; // Baytı buffera ekle

    if (index == 5) { // 5 bayt tamamlandıysa
      index = 0;

      if (verifyCommandPacket(buffer)) { // Komut geçerli mi
        byte cmd = buffer[1]; // Komut byte'ı

        if (cmd == CMD_SIT_START) {
          sit_active = true;
          sut_active = false;
          
          // SIT modunda değerleri bağımsız tut - diğerlerini etkileme
          // Normal değerler değişmez, sadece SIT verileri gönderilir
          
          Serial.println("SIT modu başlatıldı - Bağımsız veri gönderimi!");
        } else if (cmd == CMD_SUT_START) {
          sit_active = false;
          sut_active = true;
          
          // SUT testinde girişte maxları sıfırla
          sutMaxAlt = 0; // SUT maksimum irtifasını sıfırla
          maxAlt = 0; // Normal maksimum irtifasını sıfırla
          launch = false; // Fırlatma durumunu sıfırla
          burnout = false; // Burnout durumunu sıfırla
          desc = false; // Alçalma durumunu sıfırla
          sep1 = false; // Paraşüt durumlarını sıfırla
          sep2 = false;
          belowMin = false; // Minimum durumunu sıfırla
          angleFlag = false; // Açı bayrağını sıfırla
          
          // Pyro timer'ları da sıfırla
          pyro1Timer = 0;
          pyro2Timer = 0;
          
          // Burnout algoritması değişkenlerini sıfırla
          maxAccelTotal = 0;
          burnoutCandidate = false;
          burnoutCandidateTime = 0;
          
          Serial.println("SUT testi başlatıldı - Tüm max değerler ve burnout sıfırlandı!");
        } else if (cmd == CMD_STOP) {
          sit_active = false;
          sut_active = false;
        } else {
          // Tanımsız komut
        }
      } else {
        // Komut paketi geçersiz
      }
    }
  }
}

void sendSitTelemetry() {
  // SIT için ham sensör verilerini oku (filtreleme yok)
  if (!bmp.performReading()) {
    return;
  }
  
  float sitBaroHpa = bmp.pressure / 100.0f; // Pascal'dan hPa'ya dönüşüm
  float sitRawAlt = bmp.readAltitude(SEA_LEVEL_HPA);
  
  // BMP sensörünü olduğu gibi kabul et (SIT için) - 10m ekle
  float sitAlt = sitRawAlt + 10.0f;
  
  sensors_event_t sitEuler, sitAcc;
  bno.getEvent(&sitEuler, Adafruit_BNO055::VECTOR_EULER); // Euler açıları
  bno.getEvent(&sitAcc, Adafruit_BNO055::VECTOR_LINEARACCEL); // Doğrusal ivme
  
  byte pkt[36];
  pkt[0] = PKT_TELEM_HDR; // Telemetri paketi başlığı

  // Ham sensör verilerini doğru sırayla pakete yaz (Big Endian)
  int offset = 1;
  writeFloatBigEndian(pkt, offset, sitAlt); offset += 4;    // BMP388 İrtifa (ham değer)
  writeFloatBigEndian(pkt, offset, sitBaroHpa); offset += 4;   // BMP388 Basınç
  writeFloatBigEndian(pkt, offset, sitAcc.acceleration.x); offset += 4; // BNO055 İvme X
  writeFloatBigEndian(pkt, offset, sitAcc.acceleration.y); offset += 4; // BNO055 İvme Y
  writeFloatBigEndian(pkt, offset, sitAcc.acceleration.z); offset += 4; // BNO055 İvme Z
  writeFloatBigEndian(pkt, offset, sitEuler.orientation.x); offset += 4; // BNO055 Açı X
  writeFloatBigEndian(pkt, offset, sitEuler.orientation.y); offset += 4; // BNO055 Açı Y
  writeFloatBigEndian(pkt, offset, sitEuler.orientation.z); offset += 4; // BNO055 Açı Z

  pkt[offset++] = calculateChecksum(pkt); // Checksum ekle
  pkt[offset++] = PKT_FOOTER1;
  pkt[offset++] = PKT_FOOTER2;

  Serial2.write(pkt, 36); // RS232 üzerinden gönder
}

void receiveSutPacket() {
  if (Serial2.available() < 36) return; // Yeterli veri yoksa çık

  // Paket başlangıcını ara
  while (Serial2.available() > 0) {
    if (Serial2.peek() == PKT_TELEM_HDR) {
      break; // Doğru başlangıç bulundu
    }
    Serial2.read(); // Yanlış byte'ı at
  }
  
  if (Serial2.available() < 36) return; // Yeterli veri yoksa çık

  byte buf[36];
  Serial2.readBytes(buf, 36); // Tüm paketi oku

  // Paket kontrolü (başlık ve son)
  if (buf[0] != PKT_TELEM_HDR || buf[34] != PKT_FOOTER1 || buf[35] != PKT_FOOTER2) {
    return;
  }

  // Checksum doğrulama
  byte recvChecksum = buf[33];
  byte calcChecksum = calculateChecksum(buf);

  if (recvChecksum == calcChecksum) {
    // Float verileri çözümleme (Big Endian olarak oku)
    float irtifa  = readFloatBigEndian(buf, 1);
    float basinc  = readFloatBigEndian(buf, 5);
    float ivmeX   = readFloatBigEndian(buf, 9);
    float ivmeY   = readFloatBigEndian(buf, 13);
    float ivmeZ   = readFloatBigEndian(buf, 17);
    float aciX    = readFloatBigEndian(buf, 21);
    float aciY    = readFloatBigEndian(buf, 25);
    float aciZ    = readFloatBigEndian(buf, 29);

    // Float değerlerini doğrula (NaN, Infinity, çok büyük değerler için)
    auto validateFloat = [](float val, const char* name) -> float {
      if (isnan(val) || isinf(val) || val > 1e6 || val < -1e6) {
        return 0.0f;
      }
      return val;
    };
    
    irtifa = validateFloat(irtifa, "İrtifa");
    basinc = validateFloat(basinc, "Basınç");
    ivmeX = validateFloat(ivmeX, "İvme X");
    ivmeY = validateFloat(ivmeY, "İvme Y");
    ivmeZ = validateFloat(ivmeZ, "İvme Z");
    aciX = validateFloat(aciX, "Açı X");
    aciY = validateFloat(aciY, "Açı Y");
    aciZ = validateFloat(aciZ, "Açı Z");

    // Geçerli verileri global değişkenlere ata
    if (irtifa != 0.0f) {
      // SUT verilerini tamamen ayrı işle
      
      // SUT verilerini doğrudan kullan
      
      // SUT verilerini aynı filtre ile filtrele
      runningAvgAlt.addValue(irtifa);
      
      // SUT modunda fAlt'ı SUT verileri ile güncelle (filtrelenmiş değil, ham veri)
      fAlt = irtifa; // Ham SUT irtifa verisini kullan
      
      // SUT verileri için ayrı maxAlt güncelleme (anında)
      if (irtifa > 0 && irtifa < 10000) {
        // Anında güncelleme (filtre beklemeden)
        if (irtifa > sutMaxAlt) {
          sutMaxAlt = irtifa;
          Serial.print("SUT - Yeni maxAlt: "); Serial.println(sutMaxAlt);
        }
      }
      
      // Normal sensör verileri için maxAlt güncelleme (SUT modunda değilse)
      if (!sut_active && fAlt > 0 && fAlt < 10000) {
        if (fAlt > maxAlt) {
          maxAlt = fAlt;
        }
      }
      
      // SUT modunda alçalma kontrolü (basitleştirildi)
      if (!desc && sutMaxAlt > 0 && irtifa > 20.0f && irtifa < 10000.0f && 
          (sutMaxAlt - irtifa) >= DESC_DROP_M) {
        desc = true;
      }
      
      // SUT açı verileri güncellendikten sonra uçuş algoritması çalıştır
      flightAlgo();
      
      // SUT modunda burnout kontrolü (normal mod ile aynı mantık)
      if (launch && !burnout && irtifa > 100.0f) {
        float sutTotalAccel = sqrt(ivmeX * ivmeX + ivmeY * ivmeY + ivmeZ * ivmeZ);
        if (sutTotalAccel > 20.0f) {
          burnout = true;
        }
      }
      
      // SUT modunda pyro ateşleme kontrolü (2 saniye süreler) - MOSFET tetikleme - Sadece 1 kere
      static bool sutPyro1Fired = false; // SUT Pyro 1 ateşlendi bayrağı
      static bool sutPyro2Fired = false; // SUT Pyro 2 ateşlendi bayrağı
      
      // SUT modunda pyro pinlerini güvenli başlat
      static bool sutPyroInitialized = false;
      if (!sutPyroInitialized) {
        digitalWrite(PYRO1_PIN, LOW); // SUT başlangıçta LOW
        digitalWrite(PYRO2_PIN, LOW); // SUT başlangıçta LOW
        sutPyroInitialized = true;
        Serial.println("SUT - Pyro pinleri güvenli başlatıldı (LOW)");
      }
      
      if (sep1 && pyro1Timer == 0 && !sutPyro1Fired) {
        pyro1Timer = millis();
        digitalWrite(PYRO1_PIN, HIGH); // MOSFET tetikleme
        sutPyro1Fired = true; // Bayrağı set et
        Serial.println("SUT - MOSFET Pyro 1 tetiklendi! (1 kere, 2s)");
      }
      
      if (sep2 && pyro2Timer == 0 && !sutPyro2Fired) {
        pyro2Timer = millis();
        digitalWrite(PYRO2_PIN, HIGH); // MOSFET tetikleme
        sutPyro2Fired = true; // Bayrağı set et
        Serial.println("SUT - MOSFET Pyro 2 tetiklendi! (1 kere, 2s)");
      }
      
      // SUT pyro ateşleme süresi kontrolü (2 saniye)
      if (pyro1Timer > 0 && millis() - pyro1Timer >= 2000) {
        digitalWrite(PYRO1_PIN, LOW);
        pyro1Timer = 0;
        Serial.println("SUT - Pyro 1 söndürüldü! (2s sonra)");
      }
      
      if (pyro2Timer > 0 && millis() - pyro2Timer >= 2000) {
        digitalWrite(PYRO2_PIN, LOW);
        pyro2Timer = 0;
        Serial.println("SUT - Pyro 2 söndürüldü! (2s sonra)");
      }
    }
    if (basinc != 0.0f) baroHpa = basinc;
    
    // SUT verilerini ortak filtrelerle filtrele
    if (ivmeX != 0.0f) runningAvgAccX.addValue(ivmeX);
    if (ivmeY != 0.0f) runningAvgAccY.addValue(ivmeY);
    if (ivmeZ != 0.0f) runningAvgAccZ.addValue(ivmeZ);
    
    // Açı verilerini ortak filtrelerle filtrele (±180° sistemi)
    if (aciX >= -180.0f && aciX <= 180.0f && aciX != 0.0f) runningAvgAngleX.addValue(aciX);
    if (aciY >= -180.0f && aciY <= 180.0f && aciY != 0.0f) runningAvgAngleY.addValue(aciY);
    if (aciZ >= -180.0f && aciZ <= 180.0f && aciZ != 0.0f) runningAvgAngleZ.addValue(aciZ);
    
    // Filtrelenmiş değerleri ata
    gAcc.acceleration.x = runningAvgAccX.getAverage();
    gAcc.acceleration.y = runningAvgAccY.getAverage();
    gAcc.acceleration.z = runningAvgAccZ.getAverage();
    gEuler.orientation.x = runningAvgAngleX.getAverage();
    gEuler.orientation.y = runningAvgAngleY.getAverage();
    gEuler.orientation.z = runningAvgAngleZ.getAverage();
    
    // Pitch değerini güncelle
    pitchDeg = fabsf(gEuler.orientation.y);
    
    // SUT modunda durum bitleri zaten flightAlgo() içinde güncellenir
    
  } else {
    // SUT paketi geçersiz (checksum)
    // Hatalı paketi temizle
    while (Serial2.available() > 0) {
      Serial2.read();
    }
  }
}

bool waitForLoRaReady(unsigned long timeout) {
  // Sabit delay kullan (AUX pin kontrolü yok)
  delay(50); // 50ms sabit bekleme süresi (5 Hz için optimize edildi)
  return true; // Her zaman başarılı
}

void sendDurumBilgilendirme() {
  uint16_t durumBits = 0; // 16-bit durum bilgisi

  // Her bir biti kontrol edip yerleştiriyoruz (Table 3'e göre)
  if (durum_roket_kalkisi)              durumBits |= (1 << 0); // Bit 0: Roket kalkışı
  if (durum_motor_yanma_onlem_suresi)   durumBits |= (1 << 1); // Bit 1: Motor yanma önlem
  if (durum_mini_irtifa_esigi_asildi)   durumBits |= (1 << 2); // Bit 2: İrtifa eşiği
  if (durum_roket_govde_acisi_fazla)    durumBits |= (1 << 3); // Bit 3: Gövde açısı
  if (durum_irtifa_alcalmaya_basladi)   durumBits |= (1 << 4); // Bit 4: İrtifa alçalma
  if (durum_suruklenme_parasut_emri)    durumBits |= (1 << 5); // Bit 5: Sürüklenme paraşüt
  if (durum_irtifa_belirlenen_altinda)  durumBits |= (1 << 6); // Bit 6: İrtifa altında
  if (durum_ana_parasut_emri)           durumBits |= (1 << 7); // Bit 7: Ana paraşüt

  byte data1 = (durumBits >> 0) & 0xFF; // Düşük byte
  byte data2 = (durumBits >> 8) & 0xFF; // Yüksek byte

  // Paket yapısı: Header, Data1, Data2, Checksum, Footer1, Footer2
  byte paket[6];
  paket[0] = PKT_HEADER;
  paket[1] = data1;
  paket[2] = data2;
  paket[3] = paket[0] + paket[1] + paket[2]; // Checksum
  paket[4] = PKT_FOOTER1;
  paket[5] = PKT_FOOTER2;

  Serial2.write(paket, 6); // Durum paketini gönder
}

#endif // ROCKET_FLIGHT_COMPUTER_H