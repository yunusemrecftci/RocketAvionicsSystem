# Rocket Flight Computer - C++ Implementation

This file contains the Arduino .ino file converted to standard C++ format. This allows you to use more professional development tools and better organize your code.

## 📁 File Structure

```
firmware/
├── RocketFlightAlgorithm.ino    # Original Arduino file
├── rocket_flight_computer.cpp   # C++ version
├── Makefile                     # Make build system
├── CMakeLists.txt              # CMake build system
└── README_CPP.md               # This file
```

## 🔧 Requirements

### Software Requirements
- **C++ Compiler**: GCC 7+ or Clang 6+
- **CMake**: 3.16+ (optional)
- **Make**: Standard Unix/Linux make
- **Arduino IDE**: For libraries
- **Teensyduino**: For Teensy 4.x support

### Library Requirements
The following Arduino libraries must be installed via Arduino IDE Library Manager:

- **Wire** (built-in)
- **Adafruit Sensor**
- **Adafruit BMP3XX**
- **Adafruit BNO055**
- **TinyGPS++**
- **LoRa E22**
- **RunningAverage**

## 🚀 Building

### Building with Make

```bash
cd firmware
make
```

### Building with CMake

```bash
cd firmware
mkdir build
cd build
cmake ..
make
```

### Cleaning

```bash
# With Make
make clean

# With CMake
make clean-all
```

## 📋 C++ vs Arduino .ino Differences

### ✅ C++ Advantages

1. **Header Guards**: `#ifndef ROCKET_FLIGHT_COMPUTER_H` for multiple include protection
2. **Function Declarations**: All functions declared at the beginning
3. **Modular Structure**: Code sections organized
4. **Build Systems**: Professional build tools like Make, CMake
5. **IDE Support**: Better support in IDEs like VS Code, CLion
6. **Debugging**: More advanced debugging tools
7. **Version Control**: Better integration with Git

### ⚠️ Important Notes

1. **Arduino Libraries**: Libraries installed via Arduino IDE are required
2. **Path Settings**: Paths in Makefile and CMakeLists.txt must be adjusted for your system
3. **Platform Dependencies**: Special settings may be required for Teensy 4.x

## 🔄 Using with Arduino IDE

To use the C++ file in Arduino IDE:

1. Open `rocket_flight_computer.cpp` in Arduino IDE
2. Install required libraries
3. Select Teensy 4.x board
4. Upload

## 🛠️ Development Tips

### Adding New Features

1. Add function declaration to header section
2. Add function definition to implementation section
3. Add new libraries to Makefile/CMakeLists.txt if needed

### Debug

```cpp
#ifdef DEBUG_FLIGHT_ALGO
Serial.println("Debug message");
#endif
```

### Configuration

```cpp
// Modify in user settings section
const float ANGLE_THR = 40.0; // Pitch angle threshold
const unsigned long ANGLE_HOLD_MS = 150; // Pitch threshold hold time
```

## 📚 Library References

- **Wire**: I2C communication
- **Adafruit_Sensor**: Sensor base classes
- **Adafruit_BMP3XX**: Barometric pressure sensor
- **Adafruit_BNO055**: 9-DOF IMU sensor
- **TinyGPS++**: GPS module
- **LoRa_E22**: LoRa wireless communication
- **RunningAverage**: Data filtering

## 🔧 Troubleshooting

### Build Errors

1. **Library Not Found**: Install libraries via Arduino IDE
2. **Path Error**: Check paths in Makefile/CMakeLists.txt
3. **C++ Standard**: Ensure C++17 support

### Runtime Errors

1. **Sensor Connection**: Check I2C pins
2. **LoRa Connection**: Check UART pins
3. **GPS Connection**: Ensure GPS module is working

## 📞 Support

For issues:
1. Test in Arduino IDE
2. Check debug messages in Serial Monitor
3. Check sensor connections
4. Check library versions

## 📄 License

This C++ implementation is licensed under the MIT License.
