# Communication Protocols

## Overview

The rocket flight computer uses multiple communication protocols for different purposes:

1. **LoRa Telemetry**: Long-range wireless data transmission
2. **RS232 Commands**: External control interface
3. **HYI Protocol**: Competition-standard data format
4. **JSON Format**: Flexible data exchange

## LoRa Telemetry Protocol

### Packet Structure
```
Header (1 byte) + Body (44 bytes) + State (1 byte) = 46 bytes total
```

### Detailed Format
```cpp
struct TelePkt {
    uint8_t hdr;        // Header: 0xFF
    uint8_t body[44];   // Sensor data (11 float values × 4 bytes each)
    uint8_t state;      // Flight state bits
};
```

### Data Fields (Body)
1. **Altitude** (4 bytes): Filtered altitude in meters
2. **GPS Altitude** (4 bytes): GPS altitude in meters
3. **GPS Latitude** (4 bytes): GPS latitude in degrees
4. **GPS Longitude** (4 bytes): GPS longitude in degrees
5. **Gyroscope X** (4 bytes): Euler angle X in degrees
6. **Gyroscope Z** (4 bytes): Euler angle Z in degrees
7. **Gyroscope Y** (4 bytes): Euler angle Y in degrees
8. **Pitch** (4 bytes): Pitch angle in degrees
9. **Acceleration X** (4 bytes): Linear acceleration X in m/s²
10. **Acceleration Z** (4 bytes): Linear acceleration Z in m/s²
11. **Acceleration Y** (4 bytes): Linear acceleration Y in m/s²

### State Bits
```
Bit 0: Rocket launch detected
Bit 1: Motor burnout detected
Bit 2: Minimum altitude threshold exceeded
Bit 3: Rocket body angle excessive
Bit 4: Altitude decreasing (descent)
Bit 5: Drogue parachute command
Bit 6: Altitude below threshold
Bit 7: Main parachute command
```

### Transmission Parameters
- **Frequency**: 433 MHz (configurable)
- **Baud Rate**: 9600 bps
- **Transmission Rate**: 5 Hz (every 200ms)
- **Range**: Up to 15 km (tested with 5dB rocket antenna + 12dB ground Yagi)
- **Rocket Antenna**: 5dB Omni antenna
- **Ground Antenna**: 12dB 866MHz Yagi antenna
- **Field Tested**: Verified range of 15km in real-world conditions

## RS232 Command Protocol

### Packet Structure
```
Header (1 byte) + Command (1 byte) + Data (1 byte) + Footer (2 bytes) = 5 bytes total
```

### Packet Format
```cpp
byte packet[5] = {
    0xAA,   // Header
    cmd,    // Command byte
    data,   // Data byte
    0x0D,   // Footer 1 (CR)
    0x0A    // Footer 2 (LF)
};
```

### Commands
| Command | Value | Description |
|---------|-------|-------------|
| CMD_SIT_START | 0x20 | Start SIT mode |
| CMD_SUT_START | 0x22 | Start SUT mode |
| CMD_STOP | 0x24 | Stop test modes |

### Communication Parameters
- **Baud Rate**: 115200 bps
- **Data Bits**: 8
- **Parity**: None
- **Stop Bits**: 1
- **Flow Control**: None

## SIT Mode Protocol

### Outgoing Data (Flight Computer → External System)
```
Header (1 byte) + Sensor Data (32 bytes) + Checksum (1 byte) + Footer (2 bytes) = 36 bytes total
```

### Data Format
```cpp
byte packet[36] = {
    0xAB,   // Header
    // 8 float values (32 bytes):
    altitude,       // BMP388 altitude
    pressure,       // BMP388 pressure
    accX, accY, accZ,  // BNO055 acceleration
    angleX, angleY, angleZ,  // BNO055 angles
    checksum,       // Checksum (sum of bytes 0-32)
    0x0D, 0x0A      // Footer
};
```

### Transmission Rate
- **Frequency**: 20 Hz (every 50ms)
- **Purpose**: System integration testing

## SUT Mode Protocol

### Incoming Data (External System → Flight Computer)
```
Header (1 byte) + Sensor Data (32 bytes) + Checksum (1 byte) + Footer (2 bytes) = 36 bytes total
```

### Data Format
```cpp
byte packet[36] = {
    0xAB,   // Header
    // 8 float values (32 bytes):
    altitude,       // Simulated altitude
    pressure,       // Simulated pressure
    accX, accY, accZ,  // Simulated acceleration
    angleX, angleY, angleZ,  // Simulated angles
    checksum,       // Checksum (sum of bytes 0-32)
    0x0D, 0x0A      // Footer
};
```

### Status Response (Flight Computer → External System)
```
Header (1 byte) + Status (2 bytes) + Checksum (1 byte) + Footer (2 bytes) = 6 bytes total
```

### Status Bits
```
Bit 0: Rocket launch detected
Bit 1: Motor burnout detected
Bit 2: Minimum altitude threshold exceeded
Bit 3: Rocket body angle excessive
Bit 4: Altitude decreasing (descent)
Bit 5: Drogue parachute command
Bit 6: Altitude below threshold
Bit 7: Main parachute command
```

### Transmission Rate
- **Status Updates**: 10 Hz (every 100ms)
- **Purpose**: System under test validation

## HYI Competition Protocol

### Packet Structure
```
Header (4 bytes) + Data (70 bytes) + Footer (4 bytes) = 78 bytes total
```

### Detailed Format
```cpp
struct HYIPacket {
    // Header
    uint8_t header[4];    // 0xFF 0xFF 0x54 0x52
    
    // Data
    uint8_t team_id;      // Team identification
    uint8_t packet_counter; // Packet sequence number
    
    // 15 float values (60 bytes)
    float altitude;
    float rocket_gps_altitude;
    float rocket_latitude;
    float rocket_longitude;
    float payload_gps_altitude;
    float payload_latitude;
    float payload_longitude;
    float stage_gps_altitude;
    float stage_latitude;
    float stage_longitude;
    float gyroscope_x;
    float gyroscope_y;
    float gyroscope_z;
    float acceleration_x;
    float acceleration_y;
    float acceleration_z;
    float angle;
    
    uint8_t status;       // Flight status
    
    uint8_t checksum;     // Checksum (sum of bytes 4-75)
    
    // Footer
    uint8_t footer[2];    // 0x0D 0x0A
};
```

### Data Validation
- **Checksum**: Sum of all data bytes (4-75) modulo 256
- **Float Validation**: Check for NaN, Infinity, and range limits
- **Status Validation**: Ensure status bits are valid

## JSON Data Format

### Rocket Data Format
```json
{
    "kaynak": "anakart",
    "fAlt": 150.5,
    "gpsAlt": 150.2,
    "gpsLat": 39.9334,
    "gpsLng": 32.8597,
    "gyroX": 0.5,
    "gyroY": -1.2,
    "gyroZ": 0.8,
    "accX": 0.1,
    "accY": 0.2,
    "accZ": 9.8,
    "pitch": 15.3,
    "state": 2
}
```

### Field Descriptions
- **kaynak**: Data source ("anakart" = rocket)
- **fAlt**: Altitude in meters
- **gpsAlt**: GPS altitude in meters
- **gpsLat**: GPS latitude in degrees
- **gpsLng**: GPS longitude in degrees
- **gyroX/Y/Z**: Gyroscope readings in degrees
- **accX/Y/Z**: Acceleration in m/s²
- **pitch**: Pitch angle in degrees
- **state**: Flight status (integer)

## Error Handling

### Checksum Validation
```cpp
byte calculateChecksum(byte* buf, int length) {
    uint32_t sum = 0;
    for (int i = 0; i < length; i++) {
        sum += buf[i];
    }
    return (byte)(sum & 0xFF);
}
```

### Data Validation
```cpp
bool validateFloat(float val) {
    return !isnan(val) && !isinf(val) && 
           val > -1e6 && val < 1e6;
}
```

### Timeout Handling
- **LoRa**: 200ms timeout for transmission
- **RS232**: 50ms timeout for commands
- **GPS**: 5-second timeout for data

## Performance Metrics

### Transmission Rates
| Protocol | Rate | Purpose |
|----------|------|---------|
| LoRa Telemetry | 5 Hz | Real-time monitoring |
| RS232 Commands | On-demand | Control interface |
| SIT Data | 20 Hz | Integration testing |
| SUT Status | 10 Hz | System validation |
| HYI Protocol | 2 Hz | Competition data |

### Data Sizes
| Protocol | Size | Efficiency |
|----------|------|------------|
| LoRa | 46 bytes | 92% (data/overhead) |
| RS232 Commands | 5 bytes | 60% (data/overhead) |
| SIT/SUT | 36 bytes | 89% (data/overhead) |
| HYI | 78 bytes | 77% (data/overhead) |
| JSON | Variable | ~85% (data/overhead) |

### Reliability Features
- **Retry Mechanisms**: Automatic retransmission
- **Error Detection**: Checksums and validation
- **Failsafe Modes**: Graceful degradation
- **Data Logging**: Complete transmission records

## Security Considerations

### Data Integrity
- **Checksums**: All protocols include validation
- **Sequence Numbers**: Packet ordering verification
- **Time Stamps**: Data freshness validation
- **Source Authentication**: Team ID verification

### Access Control
- **Team ID**: Unique identification
- **Command Validation**: Authorized operations only
- **Data Encryption**: Sensitive information protection
- **Audit Logging**: Complete activity records

## Testing Procedures

### Protocol Testing
1. **Unit Tests**: Individual protocol validation
2. **Integration Tests**: End-to-end communication
3. **Stress Tests**: High data rate handling
4. **Error Tests**: Corrupted data handling

### Field Testing
1. **Range Tests**: Maximum communication distance
2. **Interference Tests**: RF environment validation
3. **Reliability Tests**: Long-term operation
4. **Compatibility Tests**: Multi-device operation

---

**Last Updated**: December 2024  
**Version**: 1.0.0
