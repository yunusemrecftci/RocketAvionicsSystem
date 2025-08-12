# 🚀 Rocket Flight Computer

A comprehensive Arduino/Teensy-based flight computer for rocket telemetry and autonomous control, featuring real-time sensor integration, flight algorithms, and communication protocols.

## 🎯 Purpose & Mission

This flight computer is designed for **autonomous rocket control and recovery**. Its primary mission is to:

- **Detect Launch**: Automatically identify when the rocket leaves the ground
- **Monitor Flight**: Track altitude, orientation, and acceleration in real-time
- **Detect Apogee**: Identify the highest point of the rocket's trajectory
- **Deploy Parachutes**: Automatically trigger drogue and main parachutes at optimal altitudes
- **Ensure Safe Recovery**: Guarantee the rocket returns safely to the ground

### Key Capabilities:
- **Autonomous Operation**: No human intervention required during flight
- **Real-time Processing**: 100Hz sensor reading and decision making
- **Fail-safe Systems**: Multiple safety mechanisms to prevent accidents
- **Competition Ready**: Compatible with rocket competition protocols
- **Long-range Communication**: LoRa telemetry for ground monitoring

## 📋 Table of Contents

- [Purpose & Mission](#-purpose--mission)
- [Overview](#overview)
- [Hardware Overview](#hardware-overview)
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Project Structure](#project-structure)
- [Configuration](#configuration)
- [API Documentation](#api-documentation)
- [Contributing](#contributing)
- [License](#license)

## 🎯 Overview

This project implements a complete rocket flight computer system featuring:

- **Multi-Sensor Integration**: BMP388 (altitude/pressure), BNO055 (IMU), GPS
- **Real-time Flight Algorithms**: Launch detection, burnout detection, apogee detection
- **Autonomous Control**: Automatic parachute deployment based on flight conditions
- **Multiple Communication Modes**: LoRa telemetry, RS232 command interface
- **Test Modes**: SIT (System Integration Test) and SUT (System Under Test) modes

## 🔧 Hardware Overview

### Flight Computer Board

![Rocket Flight Computer Board](images/board.jpeg)

*The flight computer features a Teensy 4.x microcontroller with integrated sensors and communication modules for autonomous rocket control.*

### Key Components:
- **Teensy 4.x Microcontroller**: High-performance ARM Cortex-M7 processor
- **BMP388 Sensor**: Barometric pressure and altitude measurement
- **BNO055 IMU**: 9-DOF motion and orientation sensing
- **GPS Module**: Global positioning and navigation
- **LoRa E22 Module**: Long-range wireless communication (15km tested range)
- **RS232 Interface**: High-speed serial communication
- **Pyro Control Circuits**: MOSFET-based parachute deployment

### Pin Layout:
```
┌─────────────────────────────────────┐
│ Teensy 4.x Flight Computer         │
├─────────────────────────────────────┤
│ I2C: SDA=17, SCL=16                │
│ LoRa: TX=0, RX=1                   │
│ RS232: TX=9, RX=10                 │
│ GPS: TX=14, RX=15                  │
│ Pyro: CH1=23, CH2=38               │
└─────────────────────────────────────┘
```

## ✨ Features

### Flight Computer (Arduino/Teensy)
- **Multi-sensor Integration**: BMP388 (altitude/pressure), BNO055 (IMU), GPS
- **Real-time Flight Algorithms**: Launch detection, burnout detection, apogee detection
- **Autonomous Control**: Automatic parachute deployment based on flight conditions
- **Multiple Communication Modes**: LoRa telemetry, RS232 command interface
- **Test Modes**: SIT (System Integration Test) and SUT (System Under Test) modes

### Communication Protocols
- **LoRa E22**: Long-range wireless telemetry
  - **Rocket Antenna**: 5dB Omni antenna
  - **Ground Antenna**: 12dB 866MHz Yagi antenna
  - **Tested Range**: 15km (verified in field tests)
  - **Frequency**: 866MHz (configurable)
  - **Packet Structure**: 46-byte binary format
    ```
    Header (1 byte) + Body (44 bytes) + State (1 byte) = 46 bytes total
    ```
  - **Byte-by-Byte Structure**:
    ```
    Byte 0:   Header (0xFF)
    Bytes 1-4:   Altitude (float, 4 bytes)
    Bytes 5-8:   GPS Altitude (float, 4 bytes)
    Bytes 9-12:  GPS Latitude (float, 4 bytes)
    Bytes 13-16: GPS Longitude (float, 4 bytes)
    Bytes 17-20: Gyroscope X (float, 4 bytes)
    Bytes 21-24: Gyroscope Z (float, 4 bytes)
    Bytes 25-28: Gyroscope Y (float, 4 bytes)
    Bytes 29-32: Pitch (float, 4 bytes)
    Bytes 33-36: Acceleration X (float, 4 bytes)
    Bytes 37-40: Acceleration Z (float, 4 bytes)
    Bytes 41-44: Acceleration Y (float, 4 bytes)
    Byte 45:     State (8-bit flags)
    ```
- **RS232**: High-speed serial communication
- **HYI Protocol**: Competition-standard data format

## 🔧 Hardware Requirements

### Flight Computer
- **Microcontroller**: Teensy 4.x (recommended) or Arduino compatible
- **Sensors**:
  - BMP388 (Barometric pressure/altitude)
  - BNO055 (9-DOF IMU)
  - GPS module (NEO-6M or compatible)
- **Communication**:
  - LoRa E22 module
  - RS232 interface
- **Actuators**:
  - 2x Pyro channels (MOSFET controlled)
- **Power**: 3.3V regulated supply

## 💻 Software Requirements

### Flight Computer
- **Arduino IDE** 1.8.x or later
- **Teensyduino** (if using Teensy)
- **Required Libraries**:
  - `Adafruit_Sensor`
  - `Adafruit_BMP3XX`
  - `Adafruit_BNO055`
  - `TinyGPS++`
  - `LoRa_E22`
  - `RunningAverage`

## 📦 Installation

### 1. Clone the Repository
```bash
git clone https://github.com/yourusername/rocket-flight-computer.git
cd rocket-flight-computer
```

### 2. Flight Computer Setup
1. Install Arduino IDE and Teensyduino
2. Install required libraries via Library Manager:
   ```
   Adafruit Unified Sensor
   Adafruit BMP3XX
   Adafruit BNO055
   TinyGPS++
   LoRa_E22
   RunningAverage
   ```
3. Connect hardware according to pin definitions in `docs/hardware.md`
4. Upload `firmware/RocketFlightAlgorithm.ino` to your microcontroller

## 🚀 Usage

### Flight Computer Operation
1. **Power Up**: Connect power and wait for sensor calibration
2. **Pre-flight Check**: Verify all sensors are reading correctly
3. **Launch**: System automatically detects launch and begins flight algorithms
4. **Recovery**: Automatic parachute deployment based on flight conditions

### Communication Modes
- **Normal Mode**: Standard flight operation
- **SIT Mode**: System Integration Test - sends fixed test data
- **SUT Mode**: System Under Test - receives external data for testing

## 📁 Project Structure

```
rocket-flight-computer/
├── README.md                 # This file
├── LICENSE                   # License information
├── .gitignore               # Git ignore rules
├── images/                   # Project images
│   └── flight_computer_board.jpg  # Flight computer board image
├── docs/                    # Documentation
│   ├── hardware.md          # Hardware setup guide
│   ├── software.md          # Software architecture
│   └── protocols.md         # Communication protocols
├── firmware/                # Flight computer code
│   └── RocketFlightAlgorithm.ino  # Main Arduino sketch
├── schematics/              # Hardware schematics
└── examples/                # Example configurations
```

## ⚙️ Configuration

### Flight Computer Configuration
Edit the constants in `firmware/RocketFlightAlgorithm.ino`:

```cpp
// Flight parameters
const float ANGLE_THR = 40.0;           // Pitch angle threshold (degrees)
const float DESC_DROP_M = 10.0;         // Descent detection altitude drop (meters)
constexpr float SEA_LEVEL_HPA = 1013.25; // Sea level pressure (adjust for location)

// Pin definitions
constexpr uint8_t SDA_PIN = 17, SCL_PIN = 16;
constexpr uint8_t PYRO1_PIN = 23, PYRO2_PIN = 38;
```

## 📚 API Documentation

### Flight Computer Functions
- `readSensors()`: Read all sensor data
- `flightAlgo()`: Execute flight control algorithms
- `sendLoRa()`: Transmit telemetry via LoRa
- `sendSitTelemetry()`: Send SIT test data
- `receiveSutPacket()`: Process SUT test data

### Flight Algorithms

#### Launch Detection
```cpp
// Detect when rocket leaves ground
if (!launch && fAlt > 5.0f && fAlt < 10000) {
    launch = true;
}
```

#### Burnout Detection
```cpp
// Monitor acceleration for motor burnout
float totalAccel = sqrt(accX² + accY² + accZ²);
if (totalAccel < maxAccelTotal * 0.5f && totalAccel < threshold) {
    burnout = true;
}
```

#### Parachute Deployment
```cpp
// Drogue parachute (apogee detection)
if (!sep1 && desc && fAlt > 100.0f) {
    digitalWrite(PYRO1_PIN, HIGH);
    sep1 = true;
}

// Main parachute (altitude-based)
if (!sep2 && desc && fAlt <= 600.0f && fAlt >= 400.0f) {
    digitalWrite(PYRO2_PIN, HIGH);
    sep2 = true;
}
```

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

### Development Guidelines
- Follow Arduino coding standards for firmware
- Add comments for complex algorithms
- Include error handling for robust operation
- Test thoroughly before submitting

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## ⚠️ Safety Notice

- This system controls pyrotechnic devices
- Always follow proper safety procedures
- Test thoroughly in safe conditions
- Ensure proper failsafes are in place
- Comply with local regulations

## 📞 Support

For questions and support:
- Create an issue on GitHub
- Check the documentation in the `docs/` folder
- Review example configurations in `examples/`

---

**Version**: 1.0.0  
**Last Updated**: December 2024  
**Maintainer**: Your Name
