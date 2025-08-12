# Hardware Setup Guide

## Overview

This document provides detailed instructions for setting up the hardware components of the rocket telemetry system.

## Components List

### Microcontroller
- **Teensy 4.x** (recommended) or Arduino compatible board
- **Power Supply**: 3.3V regulated supply capable of 500mA
- **Programming**: USB connection for code upload

### Sensors

#### BMP388 (Barometric Pressure/Altitude)
- **I2C Address**: 0x77 (default)
- **Connections**:
  - VCC → 3.3V
  - GND → GND
  - SDA → Pin 17 (configurable)
  - SCL → Pin 16 (configurable)

#### BNO055 (9-DOF IMU)
- **I2C Address**: 0x28 (default)
- **Connections**:
  - VCC → 3.3V
  - GND → GND
  - SDA → Pin 17 (shared with BMP388)
  - SCL → Pin 16 (shared with BMP388)

#### GPS Module (NEO-6M or compatible)
- **UART Connection**:
  - VCC → 3.3V
  - GND → GND
  - TX → Pin 14 (Serial8 RX)
  - RX → Pin 15 (Serial8 TX)

### Communication Modules

#### LoRa E22 Module
- **UART Connection**:
  - VCC → 3.3V
  - GND → GND
  - TX → Pin 0 (Serial1 RX)
  - RX → Pin 1 (Serial1 TX)

#### RS232 Interface
- **UART Connection**:
  - VCC → 3.3V
  - GND → GND
  - TX → Pin 9 (Serial2 TX)
  - RX → Pin 10 (Serial2 RX)

### Actuators

#### Pyro Channels (MOSFET Controlled)
- **Pyro 1** (Drogue Parachute):
  - Control Pin → Pin 23
  - MOSFET Gate → Pin 23
  - Load → Pyro device
  - Power → 12V supply

- **Pyro 2** (Main Parachute):
  - Control Pin → Pin 38
  - MOSFET Gate → Pin 38
  - Load → Pyro device
  - Power → 12V supply

## Wiring Diagram

```
Teensy 4.x
├── I2C Bus (Pins 16, 17)
│   ├── BMP388 (0x77)
│   └── BNO055 (0x28)
├── UART1 (Pins 0, 1)
│   └── LoRa E22
├── UART2 (Pins 9, 10)
│   └── RS232 Interface
├── UART8 (Pins 14, 15)
│   └── GPS Module
├── Digital Outputs
│   ├── Pin 23 → Pyro 1 MOSFET
│   └── Pin 38 → Pyro 2 MOSFET
└── Power
    ├── 3.3V → All sensors and modules
    └── 12V → Pyro devices
```

## Power Requirements

### Voltage Levels
- **3.3V**: Sensors, communication modules, microcontroller
- **12V**: Pyro devices (through MOSFETs)

### Current Requirements
- **Microcontroller**: ~100mA
- **Sensors**: ~50mA total
- **Communication**: ~100mA total
- **Pyro devices**: ~2A per channel (pulsed)

### Power Supply Recommendations
- Use a regulated 3.3V supply for logic circuits
- Use a separate 12V supply for pyro devices
- Include reverse polarity protection
- Add filtering capacitors (100nF ceramic + 10µF electrolytic)

## Mounting Considerations

### Vibration Isolation
- Mount sensors on vibration-damping material
- Use flexible wiring where possible
- Secure all connections with strain relief

### Thermal Management
- Ensure adequate airflow around components
- Monitor temperature-sensitive components
- Consider thermal insulation for high-altitude flights

### EMI Protection
- Shield sensitive analog circuits
- Use ferrite beads on power lines
- Keep high-current pyro circuits separate from logic circuits

## Testing and Calibration

### Pre-flight Checklist
1. **Power Test**: Verify all voltage levels
2. **Sensor Test**: Check all sensor readings
3. **Communication Test**: Verify LoRa and RS232 links
4. **Actuator Test**: Test pyro circuits (with dummy loads)
5. **GPS Test**: Verify satellite lock
6. **IMU Calibration**: Perform sensor calibration

### Calibration Procedures

#### BMP388 Calibration
1. Set sea level pressure for your location
2. Verify altitude readings against known elevation
3. Check temperature compensation

#### BNO055 Calibration
1. Perform accelerometer calibration
2. Perform magnetometer calibration
3. Verify gyroscope zero offset
4. Check Euler angle accuracy

#### GPS Calibration
1. Verify satellite lock time
2. Check position accuracy
3. Test altitude readings

## Safety Considerations

### Pyro Safety
- Always use dummy loads for testing
- Verify MOSFET ratings before use
- Include failsafe mechanisms
- Follow local regulations

### Electrical Safety
- Double-check all connections
- Use appropriate wire gauges
- Include fuses where necessary
- Test insulation integrity

### Mechanical Safety
- Secure all components firmly
- Use appropriate mounting hardware
- Consider shock and vibration loads
- Plan for recovery system deployment

## Troubleshooting

### Common Issues

#### Sensor Communication Problems
- Check I2C address configuration
- Verify wiring connections
- Test with known good sensors
- Check power supply stability

#### Communication Link Issues
- Verify baud rate settings
- Check antenna connections
- Test with known good modules
- Monitor signal strength

#### Pyro Circuit Problems
- Verify MOSFET operation
- Check power supply capacity
- Test with dummy loads
- Monitor current draw

### Diagnostic Tools
- Multimeter for voltage/current measurement
- Oscilloscope for signal analysis
- Logic analyzer for communication debugging
- Temperature sensor for thermal monitoring

## Maintenance

### Regular Checks
- Inspect all connections monthly
- Test all systems before each flight
- Calibrate sensors as needed
- Update firmware when available

### Storage
- Store in dry, temperature-controlled environment
- Protect from dust and moisture
- Disconnect power during storage
- Document any modifications

## Resources

### Datasheets
- [Teensy 4.x Reference Manual](https://www.pjrc.com/teensy/)
- [BMP388 Datasheet](https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/bmp388/)
- [BNO055 Datasheet](https://www.bosch-sensortec.com/products/motion-sensors/absolute-orientation-sensors/bno055/)
- [LoRa E22 Datasheet](https://www.waveshare.com/wiki/LoRa_E22_Series_Module)

### Community Resources
- [Arduino Forum](https://forum.arduino.cc/)
- [Teensy Forum](https://forum.pjrc.com/)
- [Rocketry Forum](https://www.rocketryforum.com/)

---

**Last Updated**: December 2024  
**Version**: 1.0.0
