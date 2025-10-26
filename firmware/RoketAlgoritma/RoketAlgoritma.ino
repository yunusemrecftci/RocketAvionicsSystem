// =====================================================================
// EK-6 Rev4 Uyumlu UKB Test Firmware (SIT + SUT + Durum Paketleri)
// (Teensy 4.x + LoRa E22 + BMP388 + BNO055 + TinyGPS++ + RS232)
// 
// *** FİLTRE SİSTEMİ KALDIRILDI ***
// - Ham sensör verileri direkt kullanılıyor
// - Filtreleme yok - maksimum hız ve tepki süresi
// =====================================================================

#include <Wire.h> // I2C iletişimi için gerekli
#include <Adafruit_Sensor.h> // Adafruit sensör kütüphanesi
#include <Adafruit_BMP3XX.h> // BMP388 sensör kütüphanesi
#include <Adafruit_BNO055.h> // BNO055 IMU sensör kütüphanesi
#include <TinyGPS++.h> // TinyGPS++ kütüphanesi
#include "LoRa_E22.h" // LoRa E22 modül kütüphanesi
#include <utility/imumaths.h> // IMU matematiksel fonksiyonları için (BNO055 kütüphanesinden gelir)
#include <HardwareSerial.h> // Donanımsal UART kullanımı için gerekli kütüphane
#include <SimpleKalmanFilter.h>



// -------- Uçuş Konfigürasyonu --------
struct FlightConfig {
  // Açı Eşikleri
  static constexpr float ANGLE_DETECTION_THRESHOLD = 20.0f; // Açı algılama için minimum eşik
  static constexpr unsigned long ANGLE_HOLD_MS = 50;       // Pitch eşiği üzerinde kalma süresi (ms)
  
  // İrtifa Eşikleri
  static constexpr float LAUNCH_ALT_MIN = 5.0f;            // Minimum fırlatma irtifası (m)
  static constexpr float LAUNCH_ALT_MAX = 10.0f;           // Maksimum fırlatma irtifası (m)
  static constexpr float VALID_ALT_MAX = 10000.0f;         // Geçerli irtifa maksimum değeri (m)
  static constexpr float DESC_DROP_M = 10.0f;              // İniş algılaması için irtifa düşüşü (m)
  static constexpr float MIN_ALT_THRESHOLD = 1000.0f;      // Minimum irtifa eşiği (m)
  static constexpr float DROGUE_ALT_MIN = 100.0f;          // Drogue paraşütü minimum irtifa (m)
  static constexpr float MAIN_ALT_MAX = 600.0f;            // Ana paraşüt maksimum irtifa (m)
  static constexpr float MAIN_ALT_MIN = 400.0f;            // Ana paraşüt minimum irtifa (m)
  static constexpr float BELOW_MIN_ALT = 100.0f;           // Minimum altında sayılan irtifa (m)
  static constexpr float STATUS_ALT_THRESHOLD = 800.0f;    // Durum biti için irtifa eşiği (m)
  
  // Burnout Algoritması (GERÇEKÇİ PARAMETRELER)
  static constexpr float BURNOUT_ACCEL_THRESHOLD = 2.0f;   // Normal burnout ivme eşiği (m/s²)
  static constexpr float BURNOUT_ACCEL_RATIO = 0.8f;       // Maksimum ivmenin %70'i (%30 azalma algılama)
  static constexpr unsigned long BURNOUT_CONFIRM_TIME = 100; // Normal burnout doğrulama süresi (ms)
  
  // SUT Özel Burnout Parametreleri (Daha Hızlı Algılama)
  static constexpr float SUT_BURNOUT_ACCEL_THRESHOLD = 3.0f; // SUT burnout ivme eşiği (m/s²) - düşürüldü
  static constexpr float SUT_BURNOUT_ACCEL_RATIO = 0.7f;     // SUT maksimum ivmenin %70'i - çok daha agresif
  static constexpr unsigned long SUT_BURNOUT_CONFIRM_TIME = 100; // SUT burnout doğrulama süresi (ms) - çok hızlı
  static constexpr float SUT_BURNOUT_ALT = 100.0f;         // SUT burnout irtifa eşiği (m)
  static constexpr float SUT_BURNOUT_ACCEL = 20.0f;        // SUT burnout ivme eşiği (m/s²)
  
  // Pyro Ayarları
  static constexpr unsigned long PYRO_PULSE_MS = 500;      // Normal pyro pulse süresi (ms)
  static constexpr unsigned long SUT_PYRO_PULSE_MS = 500;  // SUT pyro pulse süresi (ms)
  
  // Zamanlayıcı Ayarları
  static constexpr unsigned long SENSOR_READ_INTERVAL = 100;  // Sensör okuma aralığı (ms)
  static constexpr unsigned long LORA_SEND_INTERVAL = 300;    // LoRa gönderim aralığı (ms)
  static constexpr unsigned long SIT_SEND_INTERVAL = 100;     // SIT telemetri aralığı (ms)
  static constexpr unsigned long SUT_STATUS_INTERVAL = 100;   // SUT durum paketi aralığı (ms)
  static constexpr unsigned long ANGLE_CHECK_INTERVAL = 50;   // Açı kontrol aralığı (ms)
  
  // Sensör Ayarları
  static constexpr float SEA_LEVEL_HPA = 1013.25f;         // Deniz seviyesi basıncı (hPa)
  static constexpr unsigned long LORA_TIMEOUT = 200;       // LoRa timeout (ms)
  static constexpr unsigned long LORA_DELAY = 50;          // LoRa sabit gecikme (ms)
};

// Geriye uyumluluk için
const unsigned long ANGLE_HOLD_MS = FlightConfig::ANGLE_HOLD_MS;
const float DESC_DROP_M = FlightConfig::DESC_DROP_M;
constexpr float SEA_LEVEL_HPA = FlightConfig::SEA_LEVEL_HPA;

// -------- Pin ve Sabitler --------
constexpr uint8_t SDA_PIN     = 17, SCL_PIN     = 16; // I2C SDA ve SCL pinleri
constexpr uint8_t PYRO1_PIN   = 23, PYRO2_PIN   = 38; // Pyro ateşleme pinleri
constexpr uint8_t LORA_AUX_PIN = 27; // LoRa E22 AUX pini
const uint8_t FRAME_HDR      = 0xAA; // Komutlar için Başlık
const uint8_t HDR_SUT        = 0xAB; // Sentetik Veri Paketleri (SUT) ve SIT Veri Paketleri için Başlık
const uint8_t FRAME_FTR1     = 0x0D; // Paket sonu baytı 1
const uint8_t FRAME_FTR2     = 0x0A; // Paket sonu baytı 2
const uint8_t CMD_SIT        = 0x20; // SIT (Veri Gönderim) modunu başlatma komutu (önceki CMD_START)
const uint8_t CMD_SUT        = 0x22; // SUT (Sentetik Veri Alma) modunu başlatma komutu
const uint8_t CMD_STOP       = 0x24; // Modu durdurma (NORMAL moda dönme) komutu
// Pyro timer'ları TimerManager struct'ında tanımlandı

// -------- Debug Flagleri --------
#define DEBUG_FLIGHT_ALGO // Uçuş algoritması debug çıktılarını etkinleştirir
#define DEBUG_SENSORS     // Sensör okuma debug çıktılarını etkinleştirir

// -------- UART Tanımları --------
#define GPSSerial    Serial8 // GPS için UART
#define LoraSerial   Serial1 // LoRa için UART
#define RS232Serial Serial2 // RS232 (Yer İstasyonu) için UART

// -------- Sensör ve Modüller --------
LoRa_E22          e22(&LoraSerial); // LoRa E22 modülü objesi
TinyGPSPlus       gps; // TinyGPS++ objesi
Adafruit_BMP3XX bmp; // BMP388 barometrik sensör objesi
Adafruit_BNO055 bno(55, 0x28, &Wire1); // BNO055 IMU sensör objesi (I2C adresi 0x28, Wire1 bus'ı)

// -------- Veri Yapıları --------
struct SensorData {
  sensors_event_t euler, accel;  // BNO055 sensör verileri
  float baroHpa = 0.0f;         // Barometrik basınç (hPa)
  float altitude = 0.0f;        // Filtrelenmiş irtifa (m)
  float pitchDeg = 0.0f;        // Pitch açısı (derece)
  
  void reset() {
    baroHpa = 0.0f;
    altitude = 0.0f;
    pitchDeg = 0.0f;
  }
};

struct FlightState {
  float maxAlt = 0.0f;          // Ulaşılan maksimum irtifa (Normal ve SUT için birleşik)
  bool launch = false;          // Fırlatma durumu
  bool burnout = false;         // Motor yanma bitiş durumu
  bool descent = false;         // İniş (apogee sonrası) durumu
  bool angleFlag = false;       // Açı eşiği aşma durumu
  bool sep1 = false;            // Drogue paraşütü ayrılma
  bool sep2 = false;            // Ana paraşüt ayrılma
  bool belowMin = false;        // Minimum irtifa altında
  uint16_t statusBits = 0;      // Uçuş durum bitleri
  
  void reset() {
    maxAlt = 0.0f;
    launch = false;
    burnout = false;
    descent = false;
    angleFlag = false;
    sep1 = false;
    sep2 = false;
    belowMin = false;
    statusBits = 0;
  }
};

struct BurnoutDetection {
  float maxAccelTotal = 0.0f;           // Maksimum ivme
  bool candidate = false;               // Burnout aday durumu
  unsigned long candidateTime = 0;      // Aday zamanı
  
  void reset() {
    maxAccelTotal = 0.0f;
    candidate = false;
    candidateTime = 0;
  }
};

// -------- Hata Yönetimi --------
struct ErrorManager {
  bool bmpError = false;        // BMP388 sensör hatası
  bool bnoError = false;        // BNO055 sensör hatası  
  bool gpsError = false;        // GPS hatası
  bool loraError = false;       // LoRa haberleşme hatası
  bool rs232Error = false;      // RS232 haberleşme hatası
  unsigned long lastBmpRead = 0;   // Son başarılı BMP okuma
  unsigned long lastBnoRead = 0;   // Son başarılı BNO okuma
  unsigned long lastGpsRead = 0;   // Son başarılı GPS okuma
  
  void reset() {
    bmpError = false;
    bnoError = false;
    gpsError = false;
    loraError = false;
    rs232Error = false;
    lastBmpRead = 0;
    lastBnoRead = 0;
    lastGpsRead = 0;
  }
  
  bool hasAnyError() const {
    return bmpError || bnoError || gpsError || loraError || rs232Error;
  }
  
  void checkTimeouts() {
    unsigned long now = millis();
    if (now - lastBmpRead > 5000) bmpError = true;   // 5s timeout
    if (now - lastBnoRead > 5000) bnoError = true;   // 5s timeout
    if (now - lastGpsRead > 30000) gpsError = true;  // 30s timeout
  }
};

// -------- Kalman Filter Instances --------
// Barometrik irtifa için: Ölçüm hatası (1 m), tahmin hatası (1 m), süreç gürültüsü (0.01 m)
SimpleKalmanFilter baroKalman(1, 1, 0.01);

// Açı verileri için: Ölçüm hatası (2 derece), tahmin hatası (2 derece), süreç gürültüsü (0.1 derece)
// Açı değişimleri daha hızlı olabilir, bu yüzden süreç gürültüsü biraz daha yüksek
SimpleKalmanFilter angleKalman(2, 2, 0.1);

// Global veri yapıları
SensorData sensors;
FlightState flight;
BurnoutDetection burnoutDetector;
ErrorManager errors;

// Yerel rakım için global değişken
float groundLevel = 0.0f;
bool groundLevelSet = false;

// Başlangıç açısı için global değişken
float initialPitch = 0.0f;
bool initialPitchSet = false;

// Başlangıç irtifası için global değişken
float initialAltitude = 0.0f;
bool initialAltitudeSet = false;

// SUT modu için ayrı kalibrasyon değişkenleri
float sutInitialAltitude = 0.0f;
bool sutInitialAltitudeSet = false;
float sutInitialPitch = 0.0f;
bool sutInitialPitchSet = false;

// Geriye uyumluluk için global erişim
#define gEuler sensors.euler
#define gAcc sensors.accel
#define baroHpa sensors.baroHpa
#define fAlt sensors.altitude
#define pitchDeg sensors.pitchDeg
#define maxAlt flight.maxAlt
#define launch flight.launch
#define burnout flight.burnout
#define desc flight.descent
#define angleFlag flight.angleFlag
#define sep1 flight.sep1
#define sep2 flight.sep2
#define belowMin flight.belowMin
#define statusBits flight.statusBits
#define maxAccelTotal burnoutDetector.maxAccelTotal
#define burnoutCandidate burnoutDetector.candidate
#define burnoutCandidateTime burnoutDetector.candidateTime

// -------- Ham Sensör Verisi Sistemi --------
// Filtreleme yok - maksimum hız ve tepki süresi

// -------- Zamanlayıcı Yapısı --------
struct TimerManager {
  unsigned long sensorRead = 0;        // Sensör okuma zamanlayıcısı
  unsigned long loRaSend = 0;          // LoRa gönderim zamanlayıcısı
  unsigned long sitSend = 0;           // SIT telemetri zamanlayıcısı
  unsigned long sutStatus = 0;        // SUT durum paketi zamanlayıcısı
  unsigned long pyro1 = 0;            // Pyro1 zamanlayıcısı
  unsigned long pyro2 = 0;            // Pyro2 zamanlayıcısı
  unsigned long angleCheck = 0;       // Açı kontrol zamanlayıcısı
  unsigned long sensorReconnect = 0;  // Sensör yeniden bağlanma zamanlayıcısı
  
  void reset() {
    sensorRead = 0;
    loRaSend = 0;
    sitSend = 0;
    sutStatus = 0;
    pyro1 = 0;
    pyro2 = 0;
    angleCheck = 0;
    sensorReconnect = 0;
  }
};

// Çalışma modları
enum Mode { NORMAL } currentMode = NORMAL;

// Global zamanlayıcı
TimerManager timers;

// Geriye uyumluluk için
#define tSens timers.sensorRead
#define lastSendTime timers.loRaSend
#define lastSend timers.sitSend
#define lastDurumSend timers.sutStatus
#define pyro1Timer timers.pyro1
#define pyro2Timer timers.pyro2

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

// Gönderim zamanlayıcıları TimerManager struct'ında tanımlandı

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

// -------- Prototipler --------
void readSensors(); // Sensör verilerini okur ve global değişkenleri günceller
void flightAlgo(); // Ana uçuş algoritması koordinatörü
void sendLoRa(); // LoRa üzerinden telemetri paketi gönderir
void gpsDecode(unsigned long ms); // GPS verilerini okur ve işler
void readCommand(); // Komutları okur ve modları değiştirir
void sendSitTelemetry(); // SİT modu - sabit verileri gönderir
void receiveSutPacket(); // SUT modunda gelen veriyi çözümler ve float değişkenlere yazar
void sendDurumBilgilendirme(); // SUT modunda durum bilgilendirme paketini 10 Hz gönderir
bool waitForLoRaReady(unsigned long timeout = 1000); // LoRa modülünün hazır olmasını bekler
void checkSensorReconnection(); // Sensör yeniden bağlanma kontrolü
uint8_t computeEventFlags(); // LoRa state alanı için paraşüt kombine durumu (1..4)

// -------- Uçuş Algoritması Alt Fonksiyonları --------
bool detectLaunch(); // Fırlatma algılama
bool detectBurnout(); // Burnout algılama
bool detectAngleThreshold(); // Açı eşiği algılama
bool detectDescent(); // İniş algılama
void controlPyroChannels(); // Pyro kanal kontrolü
void updateStatusBits(); // Durum bitlerini güncelle
void updateFlightData(); // Uçuş verilerini güncelle (irtifa, maxAlt)


// ====================================================================
void setup() {
  delay(1000); // BNO055 kalibrasyonu için kısa bir bekleme süresi faydalı olabilir.
  Serial.begin(115200); // PC ile haberleşme için seri portu başlat (genellikle debug için)
  RS232Serial.begin(115200); // Harici cihaz (Yer İstasyonu) ile haberleşme için seri portu başlat
  GPSSerial.begin(9600);    // GPS UART'ı başlat
  LoraSerial.begin(9600);   // LoRa UART'ı başlat
  RS232Serial.setTimeout(50);
  e22.begin(); // LoRa E22 modülünü başlat

  // LoRa AUX pini giriş olarak ayarlanır (E22 AUX çıkıştır)
  pinMode(LORA_AUX_PIN, INPUT);

  // UART2 başlat
  Serial2.begin(115200); // UART2 başlat (Teensy 4.x için pin ayarları otomatik)

  Wire1.setSDA(SDA_PIN); // SDA pinini ayarla
  Wire1.setSCL(SCL_PIN); // SCL pinini ayarla
  Wire1.begin(); // Wire1 (I2C) iletişimi başlatılıyor.

  if (!bmp.begin_I2C(0x77, &Wire1)) {
    errors.bmpError = true;
    Serial.println("BMP388 başlatma hatası! Sistem devam ediyor...");
  } else {
    errors.bmpError = false;
    Serial.println("BMP388 başarıyla başlatıldı");
  }
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ); // Sensör okuma sıklığı.

  if (!bno.begin()) {
    errors.bnoError = true;
    Serial.println("BNO055 başlatma hatası! Sistem devam ediyor...");
  } else {
    errors.bnoError = false;
    Serial.println("BNO055 başarıyla başlatıldı");
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
  
  

  pkt46.hdr = 0x67; // Telemetri paket başlığı başlangıç değeri.

  // RS232Serial hazır olana kadar bekle (maksimum 5 saniye)
  unsigned long startWait = millis();
  while (!RS232Serial && (millis() - startWait < 5000)) {
    delay(10);
  }
  if (!RS232Serial) {
    // RS232 başlatılamadı
  }

  // ÖNCE kalibrasyonları yap
  Serial.println("Sensörler kalibre ediliyor...");
  
  // Başlangıç açısını kalibre et
  for (int i = 0; i < 5; i++) {
    sensors_event_t tempEuler;
    if (bno.getEvent(&tempEuler, Adafruit_BNO055::VECTOR_EULER) && 
        !initialPitchSet && tempEuler.orientation.y != 0.0f) {
      initialPitch = tempEuler.orientation.y;
      initialPitchSet = true;
      Serial.print("Başlangıç açısı kalibre edildi: ");
      Serial.print(initialPitch, 2);
      Serial.println("°");
      break;
    }
    delay(100);
  }
  
  // Başlangıç irtifasını kalibre et - HAM BMP değerini kullan
  if (!initialAltitudeSet) {
    float rawAlt = bmp.readAltitude(FlightConfig::SEA_LEVEL_HPA);
    if (rawAlt > -1000.0f && rawAlt < FlightConfig::VALID_ALT_MAX) {
      initialAltitude = rawAlt;  // Ham değeri kaydet
      initialAltitudeSet = true;
      Serial.print("Başlangıç irtifası kalibre edildi: ");
      Serial.print(initialAltitude, 2);
      Serial.println(" metre (HAM değer)");
    }
  }
  
  // Şimdi sensör okumalarını yap (kalibrasyon tamamlandıktan sonra)
  for (int i = 0; i < 20; i++) {
    readSensors();
    delay(50); // Gecikme, sensör okumaları arasında stabilite sağlar.
  }
  
  // MaxAlt'ı doğru fAlt değeri ile başlat
  maxAlt = fAlt; // Artık fAlt doğru hesaplanmış
  
  Serial.print("Mevcut irtifa: ");
  Serial.print(fAlt);
  Serial.println(" metre");
  
  Serial.println("Sensör kalibrasyonu tamamlandı!");
}

// ====================================================================
void loop() {
  // GPS sadece SIT modunda değilse çalışsın
  if (!sit_active) {
    gpsDecode(100); // GPS verilerini okumak için bloklamayan bir yaklaşım (süre artırıldı).
  }
  
  readCommand(); // Gelen komutları kontrol et

  // Sensör yeniden bağlanma kontrolü (5 saniyede bir) - SIT modunda da gerekli
  if (millis() - timers.sensorReconnect >= 5000) {
    timers.sensorReconnect = millis();
    checkSensorReconnection();
  }
  
  // NORMAL mod (Uçuş algoritması ve LoRa telemetri)
  if (millis() - tSens >= FlightConfig::SENSOR_READ_INTERVAL) {
    tSens = millis();
    readSensors(); // Gerçek sensör verilerini oku
    
    // SIT ve SUT modlarında uçuş algoritması çalışmasın (SUT kendi çağırır)
    if (!sit_active && !sut_active) {
      flightAlgo(); // Gerçek veri ile uçuş algoritması
    }
  }
  
  // LoRa gönderme sıklığı kontrolü (SIT modunda devre dışı)
  if (!sit_active && millis() - lastSendTime >= FlightConfig::LORA_SEND_INTERVAL) {
    lastSendTime = millis();
    sendLoRa();
  }

  // SİT modunda isek verileri gönder (sensör okumasından bağımsız)
  if (sit_active && millis() - lastSend >= FlightConfig::SIT_SEND_INTERVAL) {
    sendSitTelemetry();
    lastSend = millis();
  }

  // SUT modunda isek hem veri al hem durum gönder
  if (sut_active) {
    receiveSutPacket(); // RS232 üzerinden gelen veriyi al

    if (millis() - lastDurumSend >= FlightConfig::SUT_STATUS_INTERVAL) {
      sendDurumBilgilendirme(); // 10 Hz durum paketi gönder
      lastDurumSend = millis();
    }
  }
}

// ====================================================================
void readSensors() {
  // BMP388 sensör okuma
  if (!bmp.performReading()) {
    errors.bmpError = true;
    #ifdef DEBUG_SENSORS
    Serial.println("BMP388 okuma hatası!");
    #endif
  } else {
    errors.bmpError = false;
    errors.lastBmpRead = millis();
    baroHpa = bmp.pressure / 100.0f;

    float rawAlt = bmp.readAltitude(FlightConfig::SEA_LEVEL_HPA);

    // Geçerli irtifa kontrolü
    if (rawAlt > -1000.0f && rawAlt < FlightConfig::VALID_ALT_MAX) {
      // Kalman filtresi uyguluyoruz
      float kalmanAlt = baroKalman.updateEstimate(rawAlt);
      
      // Başlangıç irtifasından sapma olarak hesapla
      if (initialAltitudeSet) {
        fAlt = kalmanAlt - initialAltitude; // Başlangıç irtifasından sapma
      } else {
        fAlt = kalmanAlt; // Henüz kalibre edilmemişse ham değer
      }

      #ifdef DEBUG_SENSORS
      Serial.print("Ham irtifa: ");
      Serial.print(rawAlt, 2);
      Serial.print(" m  |  Kalman irtifa: ");
      Serial.print(kalmanAlt, 2);
      Serial.print(" m  |  Sapma: ");
      Serial.print(fAlt, 2);
      Serial.println(" m");
      #endif
    }
  }

  // BNO055 sensör okuma
  bool eulerValid = bno.getEvent(&gEuler, Adafruit_BNO055::VECTOR_EULER);
  bool accelValid = bno.getEvent(&gAcc, Adafruit_BNO055::VECTOR_LINEARACCEL);
  
  if (!eulerValid || !accelValid) {
    errors.bnoError = true;
    #ifdef DEBUG_SENSORS
    Serial.println("BNO055 okuma hatası!");
    #endif
    // return kaldırıldı - son değerler korunuyor
  } else {
    // BNO055 başarılı okuma
    errors.bnoError = false;
    errors.lastBnoRead = millis();
    
    // Açı verilerini filtrele (başlangıç açısından sapma olarak)
    float rawPitch = gEuler.orientation.y - initialPitch;
    float absPitch = fabsf(rawPitch);
    
    // Açı geçerlilik kontrolü
    if (absPitch >= 0.0f && absPitch <= 180.0f) {
      // Kalman filtresi uyguluyoruz
      float kalmanPitch = angleKalman.updateEstimate(absPitch);
      pitchDeg = kalmanPitch; // artık uçuş algoritması filtrelenmiş açı veriyi kullanıyor

      #ifdef DEBUG_SENSORS
      Serial.print("Ham sapma: ");
      Serial.print(rawPitch, 2);
      Serial.print("°  |  Mutlak sapma: ");
      Serial.print(absPitch, 2);
      Serial.print("°  |  Kalman sapma: ");
      Serial.print(kalmanPitch, 2);
      Serial.println("°");
      #endif
    }
  }
  
  // Hata timeout kontrolü
  errors.checkTimeouts();
}

// ====================================================================
// Ana uçuş algoritması koordinatörü
void flightAlgo() {
  updateFlightData();       // Uçuş verilerini güncelle
  detectLaunch();          // Fırlatma algıla
  detectBurnout();         // Burnout algıla
  detectAngleThreshold();  // Açı eşiği algıla
  detectDescent();         // İniş algıla
  controlPyroChannels();   // Pyro kanallarını kontrol et
  updateStatusBits();      // Durum bitlerini güncelle
}

// ====================================================================
// Uçuş verilerini güncelle (irtifa, maxAlt)
void updateFlightData() {
  // İrtifa değerini sürekli raporla
      #ifdef DEBUG_FLIGHT_ALGO
    Serial.print("İrtifa sapması: ");
    Serial.print(fAlt, 1);
    Serial.println("m");
    #endif
  
  // Maksimum irtifa güncelleme (Normal ve SUT için birleşik)
  if (fAlt > 0) { // fAlt artık sapma olduğu için sadece pozitif kontrol
    if (fAlt > maxAlt) {
      maxAlt = fAlt;
      #ifdef DEBUG_FLIGHT_ALGO
      if (sut_active) {
        Serial.print("→ SUT - Yeni maxAlt: ");
      } else {
        Serial.print("→ Yeni maxAlt: ");
      }
      Serial.print(maxAlt, 1);
      Serial.println("m");
      #endif
    }
  }
}

// ====================================================================
// Fırlatma algılama
bool detectLaunch() {
  // Başlangıç irtifasından minimum yükselme kontrolü
  
  // İlk çalışmada ground level'ı belirle
  if (!groundLevelSet && fAlt > 0) {
    groundLevel = fAlt;
    groundLevelSet = true;
    Serial.print("Ground level ayarlandı: ");
    Serial.print(groundLevel, 1);
    Serial.println("m");
  }
  
  // Ground level'dan minimum yükselme kontrolü (artık fAlt zaten sapma)
  float altGain = fAlt; // fAlt zaten başlangıç irtifasından sapma
  if (!launch && altGain > FlightConfig::LAUNCH_ALT_MIN && 
      altGain < 100.0f) {
    launch = true;
    
    // Status biti updateStatusBits() fonksiyonunda güncellenir
    
    #ifdef DEBUG_FLIGHT_ALGO
    if (sut_active) {
      Serial.print("SUT - Fırlatma algılandı! Başlangıç: ");
    } else {
      Serial.print("Fırlatma algılandı! Başlangıç: ");
    }
    Serial.print(initialAltitude, 1);
    Serial.print("m, Sapma: ");
    Serial.print(fAlt, 1);
    Serial.print("m, Gain: ");
    Serial.print(altGain, 1);
    Serial.println("m");
    #endif
  }
  return launch;
}

// ====================================================================
// Burnout algılama
bool detectBurnout() {
  if (!launch || burnout) return burnout;

  // Toplam ivme vektörünün büyüklüğünü hesapla
  float totalAccel = sqrt(gAcc.acceleration.x * gAcc.acceleration.x + 
                          gAcc.acceleration.y * gAcc.acceleration.y + 
                          gAcc.acceleration.z * gAcc.acceleration.z);
  
  // Fırlatma anından itibaren maksimum ivmeyi takip et
  if (totalAccel > maxAccelTotal) {
    maxAccelTotal = totalAccel;
  }

  // Detaylı debug için ivme değerlerini raporla
  #ifdef DEBUG_FLIGHT_ALGO
  static int burnoutDebugCounter = 0;
  if (++burnoutDebugCounter % 5 == 0) { // Daha sık - her 500ms
    Serial.print("BURNOUT - İvme: ");
    Serial.print(totalAccel, 1);
    Serial.print(", Max: ");
    Serial.print(maxAccelTotal, 1);
    Serial.print(", Ratio: ");
    Serial.print(maxAccelTotal * FlightConfig::SUT_BURNOUT_ACCEL_RATIO, 1);
    Serial.print(", Thresh: ");
    Serial.print(FlightConfig::SUT_BURNOUT_ACCEL_THRESHOLD, 1);
    Serial.print(", Cand: ");
    Serial.print(burnoutCandidate ? "Y" : "N");
    if (burnoutCandidate) {
      Serial.print(" (");
      Serial.print(millis() - burnoutCandidateTime);
      Serial.print("ms)");
    }
    Serial.println();
  }
  #endif

  // Burnout koşullarını kontrol et (SUT parametreleri her iki mod için kullanılır)
  // SUT parametreleri daha gerçekçi ve test edilmiş
  float accelThreshold = FlightConfig::SUT_BURNOUT_ACCEL_THRESHOLD;  // 3.0 m/s²
  float accelRatio = FlightConfig::SUT_BURNOUT_ACCEL_RATIO;          // 0.7 (30% düşüş)
  unsigned long confirmTime = FlightConfig::SUT_BURNOUT_CONFIRM_TIME; // 100ms
  
  float ratioThreshold = maxAccelTotal * accelRatio;
  bool condition1 = totalAccel < ratioThreshold;  // İvme maksimumun %70'inin altına düştü
  bool condition2 = totalAccel < accelThreshold;  // İvme mutlak eşiğin altına düştü
  
  // Birleşik burnout mantığı: Ratio kontrolü VE minimum max ivme kontrolü
  bool burnoutCondition = condition1 && (maxAccelTotal > 5.0f);
  
  #ifdef DEBUG_FLIGHT_ALGO
  // Koşulları debug et
  static bool lastBurnoutCondition = false;
  if (burnoutCondition != lastBurnoutCondition) {
    Serial.print("BURNOUT KOŞUL DEĞİŞTİ - İvme: ");
    Serial.print(totalAccel, 1);
    Serial.print(", Koşul1 (<");
    Serial.print(ratioThreshold, 1);
    Serial.print("): ");
    Serial.print(condition1 ? "OK" : "FAIL");
    Serial.print(", Koşul2 (<");
    Serial.print(accelThreshold, 1);
    Serial.print("): ");
    Serial.print(condition2 ? "OK" : "FAIL");
    Serial.print(" = ");
    Serial.print(burnoutCondition ? "BURNOUT!" : "NO");
    Serial.print(" (Birleşik mantık, MaxAcc:");
    Serial.print(maxAccelTotal, 1);
    Serial.println(")");
    lastBurnoutCondition = burnoutCondition;
  }
  #endif
  
  if (burnoutCondition) {
    if (!burnoutCandidate) {
      burnoutCandidate = true;
      burnoutCandidateTime = millis();
      #ifdef DEBUG_FLIGHT_ALGO
      Serial.print("Burnout adayı tespit edildi - İvme: ");
      Serial.print(totalAccel, 2);
      Serial.print(", Threshold: ");
      Serial.println(accelThreshold, 2);
      #endif
    } else {
      // Aday durumu belirli bir süre devam ederse burnout'u onayla
      if (millis() - burnoutCandidateTime >= confirmTime) {
        burnout = true;
        
        // Status biti updateStatusBits() fonksiyonunda güncellenir
        
        #ifdef DEBUG_FLIGHT_ALGO
        Serial.print("Burnout onaylandı! Final İvme: ");
        Serial.print(totalAccel, 2);
        Serial.print(", Max: ");
        Serial.println(maxAccelTotal, 2);
        #endif
      }
    }
  } else {
    // İvme tekrar yükselirse veya eşiğin üzerine çıkarsa adayı sıfırla
    burnoutCandidate = false;
  }
  
  return burnout;
}

// ====================================================================
// Açı eşiği algılama
bool detectAngleThreshold() {
  static unsigned long angleT = 0;
  static bool angleDetected = false;
  static unsigned long lastAngleCheck = 0;
  
  // Belirlenen aralıkta kontrol et
  if (millis() - lastAngleCheck >= FlightConfig::ANGLE_CHECK_INTERVAL) {
    lastAngleCheck = millis();
    
    float currentAngle = gEuler.orientation.y - initialPitch;
    float absAngle = fabsf(currentAngle);
    
    // Açı değerini sürekli raporla (irtifa gibi)
    #ifdef DEBUG_FLIGHT_ALGO
    Serial.print("Açı sapması: ");
    Serial.print(currentAngle, 1);
    Serial.print("° (Mutlak: ");
    Serial.print(absAngle, 1);
    Serial.println("°)");
    #endif
    
    if (absAngle > FlightConfig::ANGLE_DETECTION_THRESHOLD) {
      if (!angleDetected) {
        angleT = millis();
        angleDetected = true;
        #ifdef DEBUG_FLIGHT_ALGO
        Serial.println("→ EŞIK AŞILDI!");
        #endif
      }
    } else {
      angleDetected = false;
    }
  }
  
  bool newAngleFlag = (angleDetected && (millis() - angleT) >= FlightConfig::ANGLE_HOLD_MS);
  if (newAngleFlag && !angleFlag) {
    #ifdef DEBUG_FLIGHT_ALGO
    Serial.println("AngleFlag aktif oldu!");
    #endif
    
    // Status biti updateStatusBits() fonksiyonunda güncellenir
  }
  angleFlag = newAngleFlag;
  return angleFlag;
}

// ====================================================================
// İniş algılama
bool detectDescent() {
  // Alçalma kontrolü (basitleştirildi) - Normal ve SUT modlarında çalışır
  if (!desc && maxAlt > 0 && fAlt > 20.0f && 
      (maxAlt - fAlt) >= FlightConfig::DESC_DROP_M) {
    desc = true;
    
    // Status biti updateStatusBits() fonksiyonunda güncellenir
    
    #ifdef DEBUG_FLIGHT_ALGO
    if (sut_active) {
      Serial.println("SUT - İniş algılandı!");
    } else {
      Serial.println("İniş algılandı!");
    }
    #endif
  }
  return desc;
}

// ====================================================================
// Birleşik Pyro kanal kontrolü (Normal ve SUT modları için)
void controlPyroChannels() {
  // Pyro sürelerini belirle
  unsigned long pyro1PulseTime = sut_active ? FlightConfig::SUT_PYRO_PULSE_MS : FlightConfig::PYRO_PULSE_MS;
  unsigned long pyro2PulseTime = sut_active ? FlightConfig::SUT_PYRO_PULSE_MS : FlightConfig::PYRO_PULSE_MS;
  
  // Pyro tetikleme sadece bir kez olsun diye static flag'lar
  static bool pyro1Fired = false;
  static bool pyro2Fired = false;
  
  // SUT moduna geçişte flag'ları sıfırla
  static bool lastSutState = false;
  if (sut_active != lastSutState) {
    if (sut_active) {
      pyro1Fired = false;
      pyro2Fired = false; 
      Serial.println("SUT - Pyro flag'ları sıfırlandı");
    }
    lastSutState = sut_active;
  }

  // Drogue paraşüt kontrolü - MUTLAK İRTİFA KULLAN (yer seviyesinden)
  float drogueAbsoluteAltitude = 0.0f;
  if (sut_active && sutInitialAltitudeSet) {
    drogueAbsoluteAltitude = fAlt + sutInitialAltitude; // SUT: sapma + başlangıç
  } else if (!sut_active && initialAltitudeSet) {
    drogueAbsoluteAltitude = fAlt + initialAltitude;    // Normal: sapma + başlangıç  
  } else {
    drogueAbsoluteAltitude = fAlt; // Kalibrasyon yoksa ham değer
  }
  
  if (!sep1 && desc && angleFlag && drogueAbsoluteAltitude > FlightConfig::DROGUE_ALT_MIN) {
    if (!pyro1Fired) {
      digitalWrite(PYRO1_PIN, HIGH);
      pyro1Timer = millis();
      pyro1Fired = true;
      sep1 = true;
      
      // Status biti updateStatusBits() fonksiyonunda güncellenir
      
      if (sut_active) {
        Serial.print("SUT - MOSFET Pyro 1 tetiklendi! (Drogue, 500ms) Mutlak İrtifa: ");
      } else {
        Serial.print("MOSFET Pyro 1 tetiklendi! (Drogue, 500ms) Mutlak İrtifa: ");
      }
      Serial.print(drogueAbsoluteAltitude);
      Serial.print("m (Sapma: ");
      Serial.print(fAlt);
      Serial.println("m)");
    }
  }
  
  // Pyro1 süre kontrolü
  if (pyro1Timer != 0 && (millis() - pyro1Timer >= pyro1PulseTime)) {
    digitalWrite(PYRO1_PIN, LOW);
    pyro1Timer = 0;
    
    if (sut_active) {
      Serial.println("SUT - Pyro 1 söndürüldü!");
    } else {
      Serial.println("Pyro 1 söndürüldü!");
    }
  }
  
  // BelowMin irtifa kontrolü
  if (!belowMin && sep1 && fAlt <= FlightConfig::BELOW_MIN_ALT) {
    belowMin = true;
  }
  
  // Ana paraşüt kontrolü - MUTLAK İRTİFA KULLAN (yer seviyesinden)
  float absoluteAltitude = 0.0f;
  if (sut_active && sutInitialAltitudeSet) {
    absoluteAltitude = fAlt + sutInitialAltitude; // SUT: sapma + başlangıç
  } else if (!sut_active && initialAltitudeSet) {
    absoluteAltitude = fAlt + initialAltitude;    // Normal: sapma + başlangıç  
  } else {
    absoluteAltitude = fAlt; // Kalibrasyon yoksa ham değer
  }
  
  if (!sep2 && desc && absoluteAltitude <= FlightConfig::MAIN_ALT_MAX && absoluteAltitude >= FlightConfig::MAIN_ALT_MIN) {
    if (!pyro2Fired) {
      digitalWrite(PYRO2_PIN, HIGH);
      pyro2Timer = millis();
      pyro2Fired = true;
      sep2 = true;
      
      // Status biti updateStatusBits() fonksiyonunda güncellenir
      
      if (sut_active) {
        Serial.print("SUT - MOSFET Pyro 2 tetiklendi! (Main, 500ms) Mutlak İrtifa: ");
      } else {
        Serial.print("MOSFET Pyro 2 tetiklendi! (Main, 500ms) Mutlak İrtifa: ");
      }
      Serial.print(absoluteAltitude);
      Serial.print("m (Sapma: ");
      Serial.print(fAlt);
      Serial.println("m)");
    }
  }
  
  // Pyro2 süre kontrolü
  if (pyro2Timer != 0 && (millis() - pyro2Timer >= pyro2PulseTime)) {
    digitalWrite(PYRO2_PIN, LOW);
    pyro2Timer = 0;
    
    if (sut_active) {
      Serial.println("SUT - Pyro 2 söndürüldü!");
    } else {
      Serial.println("Pyro 2 söndürüldü!");
    }
  }
}

// ====================================================================
// Durum bitlerini güncelle - Tüm bitleri her çağrıda yeniden hesapla
void updateStatusBits() {
  // StatusBits'i sıfırla ve yeniden hesapla
  statusBits = 0;
  
  // Bit 0: Roket kalkışı
  if (launch) {
    statusBits |= (1 << 0);
    durum_roket_kalkisi = true;
  } else {
    durum_roket_kalkisi = false;
  }
  
  // Bit 1: Motor yanma önlem süresi
  if (burnout) {
    statusBits |= (1 << 1);
    durum_motor_yanma_onlem_suresi = true;
  } else {
    durum_motor_yanma_onlem_suresi = false;
  }
  
  // Bit 2: Minimum irtifa eşiği aşıldı
  bool mini_irtifa_esigi = (fAlt > FlightConfig::MIN_ALT_THRESHOLD);
  if (mini_irtifa_esigi) {
    statusBits |= (1 << 2);
    durum_mini_irtifa_esigi_asildi = true;
  } else {
    durum_mini_irtifa_esigi_asildi = false;
  }
  
  // Bit 3: Roket gövde açısı fazla
  if (angleFlag) {
    statusBits |= (1 << 3);
    durum_roket_govde_acisi_fazla = true;
  } else {
    durum_roket_govde_acisi_fazla = false;
  }
  
  // Bit 4: Roket irtifası alçalmaya başladı
  if (desc) {
    statusBits |= (1 << 4);
    durum_irtifa_alcalmaya_basladi = true;
  } else {
    durum_irtifa_alcalmaya_basladi = false;
  }
  
  // Bit 5: Sürüklenme paraşüt emri
  if (sep1) {
    statusBits |= (1 << 5);
    durum_suruklenme_parasut_emri = true;
  } else {
    durum_suruklenme_parasut_emri = false;
  }
  
  // Bit 6: İrtifa belirlenen altında
  bool irtifa_altinda = (fAlt <= FlightConfig::STATUS_ALT_THRESHOLD && desc);
  if (irtifa_altinda) {
    statusBits |= (1 << 6);
    durum_irtifa_belirlenen_altinda = true;
  } else {
    durum_irtifa_belirlenen_altinda = false;
  }
  
  // Bit 7: Ana paraşüt emri
  if (sep2) {
    statusBits |= (1 << 7);
    durum_ana_parasut_emri = true;
  } else {
    durum_ana_parasut_emri = false;
  }
  
  // Debug: Status bitlerini raporla (sadece önemli değişiklikler)
  static uint16_t lastStatusBits = 0;
  if (statusBits != lastStatusBits) {
    #ifdef DEBUG_FLIGHT_ALGO
    Serial.print("StatusBits güncellendi: 0x");
    Serial.print(statusBits, HEX);
    Serial.print(" (Complete refresh)");
    Serial.println();
    #endif
    lastStatusBits = statusBits;
  }
}

// ====================================================================


// ====================================================================
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
  // MUTLAK İRTİFA GÖNDER (yer seviyesinden)
  float absoluteAltitude = 0.0f;
  if (sut_active && sutInitialAltitudeSet) {
    absoluteAltitude = fAlt + sutInitialAltitude; // SUT: sapma + başlangıç
  } else if (!sut_active && initialAltitudeSet) {
    absoluteAltitude = fAlt + initialAltitude;    // Normal: sapma + başlangıç
  } else {
    absoluteAltitude = fAlt; // Kalibrasyon yoksa ham değer
  }
  pf(absoluteAltitude); // Mutlak irtifa (yer seviyesinden)
  
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
  // LoRa state alanı: statusBits yerine 4 bağımsız durum biti
  // Bit0: Launch, Bit1: Burnout, Bit2: Sep1, Bit3: Sep2
  pkt46.state = computeEventFlags();

  // Telemetri paketini LoRa üzerinden gönder
  if (waitForLoRaReady(FlightConfig::LORA_TIMEOUT)) { // Timeout ile hazır olmasını bekle
    ResponseStatus rs = e22.sendFixedMessage(0, 2, 0x06, (uint8_t*)&pkt46, sizeof(pkt46));
    if (rs.code == 1) {
      errors.loraError = false; // Başarılı gönderim
    } else {
      errors.loraError = true;
      #ifdef DEBUG_SENSORS
      Serial.print("LoRa gönderme hatası: ");
      Serial.println(rs.getResponseDescription());
      #endif
    }
  } else {
    errors.loraError = true;
    #ifdef DEBUG_SENSORS
    Serial.println("LoRa modülü hazır değil, paket gönderilemedi!");
    #endif
  }
}

// ====================================================================
void gpsDecode(unsigned long ms) { // GPS verilerini okuyan ve işleyen fonksiyon
  unsigned long start = millis();
  bool dataReceived = false;
  
  do {
    while (GPSSerial.available()) {
      if (gps.encode(GPSSerial.read())) {
        dataReceived = true;
      }
    }
  } while (millis() - start < ms);
  
  // GPS hata durumunu güncelle
  if (dataReceived && (gps.altitude.isValid() || gps.location.isValid())) {
    errors.gpsError = false;
    errors.lastGpsRead = millis();
    
    #ifdef DEBUG_SENSORS
    if (gps.altitude.isValid()) {
      Serial.print("GPS irtifa: ");
      Serial.println(gps.altitude.meters());
    }
    #endif
  }
  
  // GPS bağlantı kontrolü (5 saniye sonra)
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    errors.gpsError = true;
    #ifdef DEBUG_SENSORS
    Serial.println("GPS verisi alınamıyor: bağlantıyı kontrol edin");
    #endif
  }
}

// ====================================================================
// Komutları okur ve modları değiştirir
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
          
          Serial.println("SIT modu başlatıldı - GPS ve LoRa durduruldu, sadece RS232 aktif!");
        } else if (cmd == CMD_SUT_START) {
          sit_active = false;
          sut_active = true;
          
          // SUT testinde tüm durumları sıfırla
          flight.reset();         // Uçuş durumlarını sıfırla
          sensors.reset();        // Sensör verilerini sıfırla  
          burnoutDetector.reset(); // Burnout algoritmasını sıfırla
          timers.reset();         // Zamanlayıcıları sıfırla
          
          // SUT kalibrasyon değişkenlerini sıfırla
          sutInitialAltitude = 0.0f;
          sutInitialAltitudeSet = false;
          sutInitialPitch = 0.0f;
          sutInitialPitchSet = false;
          
          // Status flaglarını sıfırla
          durum_roket_kalkisi             = false;
          durum_motor_yanma_onlem_suresi = false;
          durum_mini_irtifa_esigi_asildi = false;
          durum_roket_govde_acisi_fazla  = false;
          durum_irtifa_alcalmaya_basladi = false;
          durum_suruklenme_parasut_emri  = false;
          durum_irtifa_belirlenen_altinda = false;
          durum_ana_parasut_emri          = false;
          
          // Pyro pinlerini güvenli başlat
          digitalWrite(PYRO1_PIN, LOW);
          digitalWrite(PYRO2_PIN, LOW);
          
          Serial.println("SUT testi başlatıldı - Tüm durumlar, status flagları ve pyro sıfırlandı!");
        } else if (cmd == CMD_STOP) {
          sit_active = false;
          sut_active = false;
          
          // Normal moda dönerken pyro pinlerini güvenli konuma getir
          digitalWrite(PYRO1_PIN, LOW);
          digitalWrite(PYRO2_PIN, LOW);
          
          Serial.println("Mod durduruldu - GPS ve LoRa tekrar aktif, Pyro pinleri güvenli konumda");
        } else {
          // Tanımsız komut
        }
      } else {
        // Komut paketi geçersiz
      }
    }
  }
}

// ====================================================================
// SİT modu - ham sensör verilerini okuyup gönderir (filtreleme yok)
void sendSitTelemetry() {
  // SIT için ham sensör verilerini oku (filtreleme yok)
  if (!bmp.performReading()) {
    return;
  }
  
  float sitBaroHpa = bmp.pressure / 100.0f; // Pascal'dan hPa'ya dönüşüm
  float sitRawAlt = bmp.readAltitude(SEA_LEVEL_HPA);
  
  // BMP sensörünü ham değeri olduğu gibi gönder (SIT için)
  float sitAlt = sitRawAlt;
  
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

// ====================================================================
// SUT modunda gelen veriyi çözümler ve float değişkenlere yazar
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

    // SUT modunda ilk veri geldiğinde kendi kalibrasyonunu yap
    if (!sutInitialAltitudeSet && irtifa != 0.0f) {
      sutInitialAltitude = irtifa;
      sutInitialAltitudeSet = true;
      Serial.print("SUT başlangıç irtifası kalibre edildi: ");
      Serial.print(sutInitialAltitude, 2);
      Serial.println(" metre");
      
      // MaxAlt'ı SUT verisiyle sıfırla
      maxAlt = 0.0f;
    }
    
    if (!sutInitialPitchSet && aciY >= -180.0f && aciY <= 180.0f && aciY != 0.0f) {
      sutInitialPitch = aciY;
      sutInitialPitchSet = true;
      Serial.print("SUT başlangıç açısı kalibre edildi: ");
      Serial.print(sutInitialPitch, 2);
      Serial.println("°");
    }

    // SUT verilerini ÖNCE global değişkenlere aktar
    if (basinc != 0.0f) baroHpa = basinc;
    
    // SUT ivme verilerini ham olarak direkt kullan
    if (ivmeX != 0.0f) gAcc.acceleration.x = ivmeX;
    if (ivmeY != 0.0f) gAcc.acceleration.y = ivmeY;
    if (ivmeZ != 0.0f) gAcc.acceleration.z = ivmeZ;
    
    // SUT açı verilerini global sensör değişkenlerine aktar
    if (aciX >= -180.0f && aciX <= 180.0f && aciX != 0.0f) gEuler.orientation.x = aciX;
    if (aciY >= -180.0f && aciY <= 180.0f && aciY != 0.0f) gEuler.orientation.y = aciY;
    if (aciZ >= -180.0f && aciZ <= 180.0f && aciZ != 0.0f) gEuler.orientation.z = aciZ;
    
    // İrtifa ve pitch değerlerini güncelle
    if (irtifa != 0.0f) {
      // SUT irtifa verisini SUT başlangıç irtifasından sapma olarak hesapla
      if (sutInitialAltitudeSet) {
        fAlt = irtifa - sutInitialAltitude; // SUT başlangıç irtifasından sapma
      } else {
        fAlt = irtifa; // Henüz kalibre edilmemişse ham değer
      }
    }
    
    // SUT açı verisi için de Kalman filtresi uygula (SUT başlangıç açısından sapma olarak)
    if (sutInitialPitchSet) {
      float rawSutPitch = gEuler.orientation.y - sutInitialPitch;
      float absSutPitch = fabsf(rawSutPitch);
      if (absSutPitch >= 0.0f && absSutPitch <= 180.0f) {
        pitchDeg = angleKalman.updateEstimate(absSutPitch); // SUT açı verisi de filtreleniyor
      }
    }
    
    // ARTIK tüm SUT verileri güncellenmiş durumda - algoritma çalıştır
    if (irtifa != 0.0f) {
      flightAlgo(); // SUT verileri ile uçuş algoritması
    }
    
    // SUT modunda durum bitleri zaten flightAlgo() içinde güncellenir
    
  } else {
    // SUT paketi geçersiz (checksum)
    // Hatalı paketi temizle
    while (Serial2.available() > 0) {
      Serial2.read();
    }
  }
}

// ====================================================================
// LoRa modülünün hazır olmasını bekler (sabit delay ile)
bool waitForLoRaReady(unsigned long timeout) {
  unsigned long start = millis();
  // Önce kısa sabit bir gecikme uygula (işlem tamamlanma için)
  if (FlightConfig::LORA_DELAY > 0) {
    delay(FlightConfig::LORA_DELAY);
  }
  // AUX pinini HIGH olana kadar bekle veya timeout'a kadar
  while (digitalRead(LORA_AUX_PIN) == LOW) {
    if (millis() - start >= timeout) {
      return false; // Zaman aşımı
    }
    delay(1);
  }
  return true;
}

// ====================================================================
// Paraşüt kombinasyon kodu üretir (resimdeki tabloya uygun):
// 1: Hiçbiri tetiklenmedi
// 2: Sürüklenme (Drogue) tetiklendi, Ana tetiklenmedi
// 3: Sürüklenme tetiklenmedi, Ana tetiklendi
// 4: Her ikisi tetiklendi
uint8_t computeEventFlags() {
  bool drogue = sep1; // Sürüklenme paraşütü
  bool mainP  = sep2; // Ana paraşüt

  if (!drogue && !mainP) return 1;
  if (drogue && !mainP)  return 2;
  if (!drogue && mainP)  return 3;
  return 4; // drogue && mainP
}

// ====================================================================
// SUT modunda durum bilgilendirme paketini 10 Hz gönderir
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

// ====================================================================
// Sensör yeniden bağlanma kontrolü (5 saniyede bir çalışır)
void checkSensorReconnection() {
  // BMP388 sensörü yeniden bağlanma deneimi
  if (errors.bmpError) {
    #ifdef DEBUG_SENSORS
    Serial.println("BMP388 yeniden bağlanma deneniyor...");
    #endif
    
    if (bmp.begin_I2C(0x77, &Wire1)) {
      errors.bmpError = false;
      errors.lastBmpRead = millis();
      
      // BMP388 ayarlarını yeniden yapılandır
      bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
      bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
      bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
      bmp.setOutputDataRate(BMP3_ODR_50_HZ);
      
      Serial.println("BMP388 yeniden bağlandı!");
    } else {
      #ifdef DEBUG_SENSORS
      Serial.println("BMP388 yeniden bağlanma başarısız");
      #endif
    }
  }
  
  // BNO055 sensörü yeniden bağlanma deneimi
  if (errors.bnoError) {
    #ifdef DEBUG_SENSORS
    Serial.println("BNO055 yeniden bağlanma deneniyor...");
    #endif
    
    if (bno.begin()) {
      errors.bnoError = false;
      errors.lastBnoRead = millis();
      
      // BNO055 ayarlarını yeniden yapılandır
      delay(100); // Kısa bekleme
      bno.setExtCrystalUse(true);
      
      Serial.println("BNO055 yeniden bağlandı!");
    } else {
      #ifdef DEBUG_SENSORS
      Serial.println("BNO055 yeniden bağlanma başarısız");
      #endif
    }
  }
  
  // Hata durumu raporu (sadece hata varsa)
  if (errors.hasAnyError()) {
    Serial.print("Sensör durumu - BMP: ");
    Serial.print(errors.bmpError ? "HATA" : "OK");
    Serial.print(", BNO: ");
    Serial.print(errors.bnoError ? "HATA" : "OK");
    Serial.print(", GPS: ");
    Serial.print(errors.gpsError ? "HATA" : "OK");
    Serial.println();
  }
}