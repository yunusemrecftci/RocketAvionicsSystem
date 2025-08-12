/*
 * Rocket Flight Computer - Configuration Example
 * 
 * This file contains example configuration settings for the flight computer.
 * Copy this file to firmware/config.h and modify the values as needed.
 */

#ifndef CONFIG_H
#define CONFIG_H

// =====================================================================
// FLIGHT PARAMETERS
// =====================================================================

// Angle threshold for pitch detection (degrees)
const float ANGLE_THR = 40.0;

// Time to hold above angle threshold (milliseconds)
const unsigned long ANGLE_HOLD_MS = 150;

// Altitude drop for descent detection (meters)
const float DESC_DROP_M = 10.0;

// Sea level pressure for altitude calculation (hPa)
// Adjust this value for your location and current weather
constexpr float SEA_LEVEL_HPA = 1013.25;

// =====================================================================
// PIN DEFINITIONS
// =====================================================================

// I2C pins for sensors
constexpr uint8_t SDA_PIN = 17;
constexpr uint8_t SCL_PIN = 16;

// Pyro control pins (MOSFET gates)
constexpr uint8_t PYRO1_PIN = 23;  // Drogue parachute
constexpr uint8_t PYRO2_PIN = 38;  // Main parachute

// Pyro pulse duration (milliseconds)
const unsigned long PYRO_PULSE_MS = 1000;  // 1 second

// =====================================================================
// COMMUNICATION SETTINGS
// =====================================================================

// LoRa transmission interval (milliseconds)
const unsigned long SEND_INTERVAL_MS = 150;  // 5 Hz

// UART baud rates
#define GPS_BAUDRATE 9600
#define LORA_BAUDRATE 9600
#define RS232_BAUDRATE 115200

// =====================================================================
// SENSOR CONFIGURATION
// =====================================================================

// BMP388 settings
#define BMP_TEMP_OVERSAMPLING BMP3_OVERSAMPLING_8X
#define BMP_PRESSURE_OVERSAMPLING BMP3_OVERSAMPLING_4X
#define BMP_IIR_FILTER_COEFF BMP3_IIR_FILTER_COEFF_3
#define BMP_DATA_RATE BMP3_ODR_50_HZ

// BNO055 settings
#define BNO_I2C_ADDRESS 0x28
#define BNO_USE_EXTERNAL_CRYSTAL true

// =====================================================================
// FLIGHT ALGORITHM PARAMETERS
// =====================================================================

// Launch detection altitude threshold (meters)
const float LAUNCH_ALTITUDE_THRESHOLD = 5.0;

// Maximum valid altitude (meters)
const float MAX_VALID_ALTITUDE = 10000.0;

// Burnout detection parameters
const float BURNOUT_ACCEL_THRESHOLD = 5.0;  // m/s²
const unsigned long BURNOUT_CONFIRM_TIME = 200;  // ms

// Parachute deployment altitudes (meters)
const float DROGUE_MIN_ALTITUDE = 100.0;
const float MAIN_PARACHUTE_MIN = 400.0;
const float MAIN_PARACHUTE_MAX = 600.0;

// =====================================================================
// FILTER SETTINGS
// =====================================================================

// Running average filter sizes
#define ALTITUDE_FILTER_SIZE 1
#define ACCELERATION_FILTER_SIZE 2
#define ANGLE_FILTER_SIZE 2

// =====================================================================
// DEBUG SETTINGS
// =====================================================================

// Enable debug output
#define DEBUG_FLIGHT_ALGO
#define DEBUG_SENSORS

// Debug serial baud rate
#define DEBUG_BAUDRATE 115200

// =====================================================================
// TEST MODE SETTINGS
// =====================================================================

// SIT mode transmission rate (milliseconds)
const unsigned long SIT_INTERVAL_MS = 50;  // 20 Hz

// SUT mode status transmission rate (milliseconds)
const unsigned long SUT_STATUS_INTERVAL_MS = 100;  // 10 Hz

// SUT mode pyro pulse duration (milliseconds)
const unsigned long SUT_PYRO_PULSE_MS = 2000;  // 2 seconds

// =====================================================================
// SAFETY SETTINGS
// =====================================================================

// Maximum pyro activation time (milliseconds)
const unsigned long MAX_PYRO_ACTIVATION_TIME = 5000;

// Minimum time between pyro activations (milliseconds)
const unsigned long MIN_PYRO_INTERVAL = 1000;

// Emergency shutdown altitude (meters)
const float EMERGENCY_SHUTDOWN_ALTITUDE = 50.0;

// =====================================================================
// COMPETITION SETTINGS
// =====================================================================

// Team ID (1-255)
#define TEAM_ID 1

// Competition protocol settings
#define HYI_PACKET_RATE 2  // Hz
#define HYI_TIMEOUT_MS 500

// =====================================================================
// ADVANCED SETTINGS
// =====================================================================

// GPS timeout (milliseconds)
const unsigned long GPS_TIMEOUT_MS = 5000;

// LoRa timeout (milliseconds)
const unsigned long LORA_TIMEOUT_MS = 200;

// RS232 command timeout (milliseconds)
const unsigned long RS232_TIMEOUT_MS = 50;

// Sensor reading interval (milliseconds)
const unsigned long SENSOR_READ_INTERVAL_MS = 100;  // 10 Hz

// =====================================================================
// CALIBRATION VALUES
// =====================================================================

// Accelerometer calibration offsets
const float ACCEL_OFFSET_X = 0.0;
const float ACCEL_OFFSET_Y = 0.0;
const float ACCEL_OFFSET_Z = 0.0;

// Gyroscope calibration offsets
const float GYRO_OFFSET_X = 0.0;
const float GYRO_OFFSET_Y = 0.0;
const float GYRO_OFFSET_Z = 0.0;

// Magnetometer calibration (if using)
const float MAG_OFFSET_X = 0.0;
const float MAG_OFFSET_Y = 0.0;
const float MAG_OFFSET_Z = 0.0;

// =====================================================================
// DEPRECATED SETTINGS (for backward compatibility)
// =====================================================================

// These settings are kept for compatibility but should not be modified
#define FRAME_HDR 0xAA
#define HDR_SUT 0xAB
#define FRAME_FTR1 0x0D
#define FRAME_FTR2 0x0A
#define CMD_SIT 0x20
#define CMD_SUT 0x22
#define CMD_STOP 0x24

#endif // CONFIG_H
