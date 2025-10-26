#include <Wire.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "LoRa_E22.h"
#include <SD.h>
#include <SPI.h>

// *** GPS için Pin Tanımlamaları ***
#define GPS_RX_PIN 7
#define GPS_TX_PIN 8
#define GPS_BAUD_RATE 9600
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus gps;

// *** LoRa E22 için Pin Tanımlamaları ***
#define LORA_RX_PIN 21
#define LORA_TX_PIN 20
#define LORA_BAUD_RATE 9600
SoftwareSerial loraSerial(LORA_RX_PIN, LORA_TX_PIN);
LoRa_E22 e22ttl(&loraSerial);

// *** SD Kart için Pin Tanımlamaları ***
#define SD_CS_PIN BUILTIN_SDCARD  // Teensy 4.1 dahili SD kart
File dataFile;
char fileName[32]; // Dosya ismi için buffer

// *** MPU6050 için Pin Tanımlamaları ***
#define I2C_MPU6050_BUS Wire2
const int MPU6050_ADDR_INTERNAL = 0x68; // İç sensör
const int MPU6050_ADDR_EXTERNAL = 0x69; // Dış sensör
const int PWR_MGMT_1 = 0x6B;
const int ACCEL_XOUT_H = 0x3B;

// === Kalman Filter Class ===
class KalmanFilter {
public:
    float Q;  // Process noise covariance
    float R;  // Measurement noise covariance
    float x;  // Value
    float P;  // Estimation error covariance
    float K;  // Kalman gain

    KalmanFilter(float q = 0.01, float r = 0.1, float initial_value = 0) {
        Q = q;
        R = r;
        P = 1;
        x = initial_value;
    }

    float update(float measurement) {
        P = P + Q;
        K = P / (P + R);
        x = x + K * (measurement - x);
        P = (1 - K) * P;
        return x;
    }
};

// *** Sensör verisi ***
struct SensorData {
    int16_t accelX_raw, accelY_raw, accelZ_raw;
    float accelX_offset, accelY_offset, accelZ_offset;

    // Kalman filtreleri
    KalmanFilter kfX, kfY, kfZ;

    // RMS hesaplama için buffer
    float accelX_history[30];
    float accelY_history[30];
    float accelZ_history[30];
    int sample_index;
    bool buffer_full;
    float rms_total;
};

// *** Float to Byte Union (RoketAlgoritma.ino mantığı) ***
typedef union { float f; uint8_t b[4]; } F2B;

// *** Veri paketi yapısı - Union Tabanlı ***
struct __attribute__((packed)) DataPacket {
    uint8_t header;           // 0x52 - Paket başlığı
    uint8_t body[20];         // 5 float * 4 byte = 20 byte
};

SensorData internal_sensor;
SensorData external_sensor;

// *** Global değişkenler ***
float gpsLatitude = 0.0, gpsLongitude = 0.0, gpsAltitude = 0.0;
int gpsSatellites = 0;
unsigned long lastLoRaTransmission = 0;
const unsigned long LORA_INTERVAL = 165; // 200ms -> 5 Hz

// *** SD Kart Zamanlayıcı ***
unsigned long lastSDWriteTime = 0;
const unsigned long SD_WRITE_INTERVAL = 500; // 500ms -> 2 Hz

// *** MPU Durum Kontrolü ***
bool mpuInternalAvailable = false;
bool mpuExternalAvailable = false;

// === SD Fonksiyonları ===
void createFileName() {
    if (gps.date.isValid() && gps.time.isValid()) {
        const char* monthNames[] = {"JAN","FEB","MAR","APR","MAY","JUN",
                                    "JUL","AUG","SEP","OCT","NOV","DEC"};
        sprintf(fileName, "%s_%02d%02d.csv", 
                monthNames[gps.date.month() - 1],
                gps.time.hour(), gps.time.minute());
    } else {
        sprintf(fileName, "flight_%lu.csv", millis());
    }
    Serial.print("Dosya oluşturuluyor: ");
    Serial.println(fileName);
}

void initSDCard() {
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("SD kart başlatılamadı!");
        while (1) delay(1000);
    }
    Serial.println("SD kart OK");
    createFileName();
    dataFile = SD.open(fileName, FILE_WRITE);
    if (dataFile) {
        dataFile.println("Tarih,Saat,Enlem,Boylam,Yukseklik,Uydu,RMS_Internal,RMS_External,MPU_Internal,MPU_External");
        dataFile.flush();
    }
}

void writeDataToSD() {
  if (dataFile) {
    char dateStr[16], timeStr[16];
    if (gps.date.isValid() && gps.time.isValid()) {
        sprintf(dateStr, "%02d/%02d/%04d", gps.date.day(), gps.date.month(), gps.date.year());
        sprintf(timeStr, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
    } else {
        strcpy(dateStr, "00/00/0000");
        strcpy(timeStr, "00:00:00");
    }

    dataFile.print(dateStr); dataFile.print(",");
    dataFile.print(timeStr); dataFile.print(",");
    dataFile.print(gpsLatitude, 6); dataFile.print(",");
    dataFile.print(gpsLongitude, 6); dataFile.print(",");
    dataFile.print(gpsAltitude, 1); dataFile.print(",");
    dataFile.print(gpsSatellites); dataFile.print(",");
    dataFile.print(internal_sensor.rms_total, 4); dataFile.print(",");
    dataFile.print(external_sensor.rms_total, 4); dataFile.print(",");
    dataFile.print(mpuInternalAvailable ? "YES" : "NO"); dataFile.print(",");
    dataFile.println(mpuExternalAvailable ? "YES" : "NO");
    dataFile.flush();
  }
}

// === GPS Fonksiyonu ===
static void smartGPSTimeout(unsigned long ms) {
    unsigned long start = millis();
    do {
        while (gpsSerial.available()) {
            if (gps.encode(gpsSerial.read())) {
                if (gps.location.isValid()) {
                    gpsLatitude = gps.location.lat();
                    gpsLongitude = gps.location.lng();
                }
                if (gps.altitude.isValid()) gpsAltitude = gps.altitude.meters();
                if (gps.satellites.isValid()) gpsSatellites = gps.satellites.value();
            }
        }
    } while (millis() - start < ms);
}

// === MPU Fonksiyonları ===
bool readMPU6050Data(int addr, SensorData &sensor) {
    I2C_MPU6050_BUS.beginTransmission(addr);
    I2C_MPU6050_BUS.write(ACCEL_XOUT_H);
    I2C_MPU6050_BUS.endTransmission(0);
    I2C_MPU6050_BUS.requestFrom(addr, 6, 1);
    
    if (I2C_MPU6050_BUS.available() == 6) {
        sensor.accelX_raw = I2C_MPU6050_BUS.read() << 8 | I2C_MPU6050_BUS.read();
        sensor.accelY_raw = I2C_MPU6050_BUS.read() << 8 | I2C_MPU6050_BUS.read();
        sensor.accelZ_raw = I2C_MPU6050_BUS.read() << 8 | I2C_MPU6050_BUS.read();
        return true;
    }
    return false;
}

bool initMPU6050(int addr) {
    I2C_MPU6050_BUS.beginTransmission(addr);
    I2C_MPU6050_BUS.write(PWR_MGMT_1);
    I2C_MPU6050_BUS.write(0x00); // Wake up
    return I2C_MPU6050_BUS.endTransmission(1) == 0;
}

void calibrateSensor(int addr, SensorData &sensor) {
    float ax_sum=0, ay_sum=0, az_sum=0;
    int valid=0;
    for (int i=0;i<1000;i++) {
        if (readMPU6050Data(addr,sensor)) {
            ax_sum+=sensor.accelX_raw;
            ay_sum+=sensor.accelY_raw;
            az_sum+=sensor.accelZ_raw;
            valid++;
        }
        delay(2);
    }
    if (valid>0) {
        sensor.accelX_offset=ax_sum/valid;
        sensor.accelY_offset=ay_sum/valid;
        float avgZ=az_sum/valid;
        sensor.accelZ_offset=(avgZ>0)?(avgZ-16384):(avgZ+16384);
    }
}

void updateRMSBuffer(SensorData &sensor,float ax,float ay,float az){
    sensor.accelX_history[sensor.sample_index]=ax;
    sensor.accelY_history[sensor.sample_index]=ay;
    sensor.accelZ_history[sensor.sample_index]=az;
    sensor.sample_index++;
    if(sensor.sample_index>=30){sensor.sample_index=0;sensor.buffer_full=true;}
}

void calculateRMS(SensorData &sensor){
    if(!sensor.buffer_full&&sensor.sample_index<10) return;
    int samples=sensor.buffer_full?30:sensor.sample_index;
    float meanX=0,meanY=0,meanZ=0;
    for(int i=0;i<samples;i++){meanX+=sensor.accelX_history[i];meanY+=sensor.accelY_history[i];meanZ+=sensor.accelZ_history[i];}
    meanX/=samples;meanY/=samples;meanZ/=samples;
    float sumsq=0;
    for(int i=0;i<samples;i++){
        float dx=sensor.accelX_history[i]-meanX;
        float dy=sensor.accelY_history[i]-meanY;
        float dz=sensor.accelZ_history[i]-meanZ;
        sumsq+=dx*dx+dy*dy+dz*dz;
    }
    sensor.rms_total=sqrt(sumsq/samples);
}

// === LoRa Fonksiyonu ===

void sendLoRaData() {
    DataPacket packet;
    
    // Header'ı ata
    packet.header = 0x52;
    
    // Float'ı bayt dizisine kopyalamak ve işaretçiyi ilerletmek için lambda fonksiyonu (RoketAlgoritma.ino mantığı)
    uint8_t* p = packet.body;
    auto pf = [&](float v) {
        F2B u;
        u.f = v;
        memcpy(p, u.b, 4);
        p += 4;
    };
    
    // RMS değerlerini kontrol et
    float rms_int = 0.0;
    float rms_ext = 0.0;
    
    // İç sensör kontrolü
    if (mpuInternalAvailable && internal_sensor.buffer_full) {
        rms_int = internal_sensor.rms_total;
        // Geçersiz değer kontrolü
        if (isnan(rms_int) || isinf(rms_int) || rms_int < 0 || rms_int > 1.0) {
            rms_int = 0.0;
        }
    }
    
    // Dış sensör kontrolü - daha sıkı
    if (mpuExternalAvailable && external_sensor.buffer_full) {
        rms_ext = external_sensor.rms_total;
        // Geçersiz değer kontrolü
        if (isnan(rms_ext) || isinf(rms_ext) || rms_ext < 0 || rms_ext > 1.0) {
            rms_ext = 0.0;
        }
    } else {
        // Dış sensör yoksa kesinlikle 0
        rms_ext = 0.0;
    }
    
    // Telemetri paketini sensör verileriyle doldur (RoketAlgoritma.ino mantığı)
    pf(gpsLatitude);    // GPS enlem
    pf(gpsLongitude);   // GPS boylam
    pf(gpsAltitude);    // GPS yükseklik
    pf(rms_int);        // İç sensör RMS
    pf(rms_ext);        // Dış sensör RMS

    // Debug: Gönderilen veriyi kontrol et
    Serial.print("Gönderilen - Lat: "); Serial.print(gpsLatitude, 6);
    Serial.print(" Lng: "); Serial.print(gpsLongitude, 6);
    Serial.print(" Alt: "); Serial.print(gpsAltitude, 1);
    Serial.print(" RMS_Int: "); Serial.print(rms_int, 4);
    Serial.print(" RMS_Ext: "); Serial.print(rms_ext, 4);
    Serial.print(" g | İç: "); Serial.print(mpuInternalAvailable ? "OK" : "NO");
    Serial.print(" Dış: "); Serial.println(mpuExternalAvailable ? "OK" : "NO");

    // Union tabanlı struct'ı gönder
    ResponseStatus rs = e22ttl.sendFixedMessage(0, 1, 16, (uint8_t*)&packet, sizeof(packet));
    if(rs.code == 1) {
        Serial.println("LoRa Gönderim Başarılı");
    } else {
        Serial.print("Hata: "); Serial.println(rs.getResponseDescription());
    }
}

// === Setup ===
void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("=== GPS + MPU6050 (Kalman) + LoRa + SD ===");

    initSDCard();
    gpsSerial.begin(GPS_BAUD_RATE);
    loraSerial.begin(LORA_BAUD_RATE);
    e22ttl.begin();

    I2C_MPU6050_BUS.begin();
    I2C_MPU6050_BUS.setClock(400000);

    // Sensörleri başlat
    mpuInternalAvailable=initMPU6050(MPU6050_ADDR_INTERNAL);
    mpuExternalAvailable=initMPU6050(MPU6050_ADDR_EXTERNAL);

    if(mpuInternalAvailable) calibrateSensor(MPU6050_ADDR_INTERNAL,internal_sensor);
    if(mpuExternalAvailable) calibrateSensor(MPU6050_ADDR_EXTERNAL,external_sensor);

    // Kalman filtre init
    internal_sensor.kfX=KalmanFilter(0.01,0.1,0);
    internal_sensor.kfY=KalmanFilter(0.01,0.1,0);
    internal_sensor.kfZ=KalmanFilter(0.01,0.1,0);
    external_sensor.kfX=KalmanFilter(0.01,0.1,0);
    external_sensor.kfY=KalmanFilter(0.01,0.1,0);
    external_sensor.kfZ=KalmanFilter(0.01,0.1,0);
}

// === Loop ===
void loop() {
    static unsigned long lastGPS=0;
    if(millis()-lastGPS>100){smartGPSTimeout(10);lastGPS=millis();}

    if(mpuInternalAvailable&&readMPU6050Data(MPU6050_ADDR_INTERNAL,internal_sensor)){
        float ax_raw=(internal_sensor.accelX_raw-internal_sensor.accelX_offset)/16384.0;
        float ay_raw=(internal_sensor.accelY_raw-internal_sensor.accelY_offset)/16384.0;
        float az_raw=(internal_sensor.accelZ_raw-internal_sensor.accelZ_offset)/16384.0;
        float ax=internal_sensor.kfX.update(ax_raw);
        float ay=internal_sensor.kfY.update(ay_raw);
        float az=internal_sensor.kfZ.update(az_raw);
        updateRMSBuffer(internal_sensor,ax,ay,az);
        calculateRMS(internal_sensor);
    }

    if(mpuExternalAvailable&&readMPU6050Data(MPU6050_ADDR_EXTERNAL,external_sensor)){
        float ax_raw=(external_sensor.accelX_raw-external_sensor.accelX_offset)/16384.0;
        float ay_raw=(external_sensor.accelY_raw-external_sensor.accelY_offset)/16384.0;
        float az_raw=(external_sensor.accelZ_raw-external_sensor.accelZ_offset)/16384.0;
        float ax=external_sensor.kfX.update(ax_raw);
        float ay=external_sensor.kfY.update(ay_raw);
        float az=external_sensor.kfZ.update(az_raw);
        updateRMSBuffer(external_sensor,ax,ay,az);
        calculateRMS(external_sensor);
    }

    static unsigned long lastPrint=0;
    if(millis()-lastPrint>1000){
        lastPrint=millis();
        Serial.print("GPS Sat: ");Serial.print(gpsSatellites);
        Serial.print(" | İç RMS: ");Serial.print(internal_sensor.rms_total,4);
        Serial.print("g | Dış RMS: ");Serial.print(external_sensor.rms_total,4);
        Serial.println("g");
    }

    if(millis()-lastLoRaTransmission>LORA_INTERVAL){
        lastLoRaTransmission=millis();
        sendLoRaData();
    }

    if(millis()-lastSDWriteTime>=SD_WRITE_INTERVAL){
        lastSDWriteTime=millis();
        writeDataToSD();
    }
    delay(20);
}
