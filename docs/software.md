# Software Architecture

## Overview

The rocket flight computer system consists of a single main software component:

**Flight Computer Software** (Arduino/Teensy)

## Flight Computer Architecture

### Core Components

#### 1. Sensor Management
```cpp
// Sensor objects
Adafruit_BMP3XX bmp;        // Barometric pressure/altitude
Adafruit_BNO055 bno;        // 9-DOF IMU
TinyGPSPlus gps;            // GPS module
LoRa_E22 e22;               // LoRa communication
```

#### 2. Data Processing
- **RunningAverage Filters**: Smooth sensor readings
- **Flight Algorithms**: Autonomous decision making
- **Data Validation**: Check for sensor errors

#### 3. Communication Protocols
- **LoRa Telemetry**: Real-time data transmission
- **RS232 Commands**: External control interface
- **SIT/SUT Modes**: Testing and simulation

### Main Program Flow

```cpp
void setup() {
    // Initialize sensors and communication
    // Perform calibration
    // Set initial states
}

void loop() {
    // Read sensors (100Hz)
    // Execute flight algorithms
    // Send telemetry (5Hz)
    // Process commands
    // Handle test modes
}
```

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

### Communication Modes

#### Normal Mode
- Real sensor data processing
- Autonomous flight control
- LoRa telemetry transmission

#### SIT Mode (System Integration Test)
- Send fixed test data
- Simulate flight conditions
- Test external reception

#### SUT Mode (System Under Test)
- Receive external test data
- Process simulated sensor readings
- Test flight algorithms

## Data Flow Architecture

### Sensor Data Flow
```
Sensors → Processing → Flight Algorithms → Control Outputs
```

### Communication Flow
```
Flight Computer → LoRa/RS232 → External Systems
```

### Command Flow
```
External Systems → RS232 → Flight Computer → Mode Change/Control
```

## Error Handling

### Flight Computer
- **Sensor Failures**: Graceful degradation
- **Communication Errors**: Retry mechanisms
- **Power Issues**: Safe shutdown procedures
- **Algorithm Failures**: Failsafe modes

## Performance Considerations

### Flight Computer
- **Real-time Requirements**: 100Hz sensor reading
- **Memory Usage**: Optimized data structures
- **Power Efficiency**: Sleep modes when possible
- **Reliability**: Redundant systems

## Security Considerations

### Data Integrity
- **Checksums**: Packet validation
- **Error Detection**: Corrupted data handling
- **Authentication**: Team ID verification
- **Validation**: Input data checking

### System Safety
- **Failsafes**: Automatic recovery systems
- **Validation**: Input data checking
- **Logging**: Event recording
- **Testing**: Comprehensive verification

## Testing Strategy

### Unit Testing
- **Sensor Functions**: Individual component testing
- **Algorithms**: Flight logic verification
- **Communication**: Protocol testing

### Integration Testing
- **End-to-end**: Complete system testing
- **Performance**: Load and stress testing
- **Safety**: Failsafe verification

### Field Testing
- **Ground Tests**: Pre-flight verification
- **Flight Tests**: Real-world validation
- **Recovery Tests**: System restoration
- **Competition Tests**: Rule compliance

## Deployment

### Flight Computer
1. **Code Compilation**: Arduino IDE
2. **Hardware Verification**: Component testing
3. **Calibration**: Sensor setup
4. **Integration**: System assembly

## Maintenance

### Code Updates
- **Version Control**: Git repository
- **Documentation**: Code comments and guides
- **Backup**: Regular code backups
- **Testing**: Update verification

### System Monitoring
- **Performance**: Resource usage tracking
- **Errors**: Log analysis
- **Updates**: Dependency management
- **Security**: Vulnerability assessment

---

**Last Updated**: December 2024  
**Version**: 1.0.0
