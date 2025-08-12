# Rocket Flight Computer - PlatformIO Setup

This guide explains how to use PlatformIO for developing and building the Rocket Flight Computer firmware.

## 📁 PlatformIO Project Structure

```
caresizkod/
├── platformio.ini              # PlatformIO configuration
├── src/
│   └── main.cpp                # Main firmware source (converted from .ino)
├── lib/                        # Custom libraries (if any)
├── scripts/
│   ├── pre_build.py           # Pre-build validation script
│   └── post_build.py          # Post-build reporting script
├── output/                     # Build artifacts (generated)
├── firmware/                   # Original Arduino files
│   ├── RocketFlightAlgorithm.ino
│   ├── rocket_flight_computer.cpp
│   ├── Makefile
│   └── CMakeLists.txt
└── README_PLATFORMIO.md        # This file
```

## 🔧 Requirements

### Software Requirements
- **PlatformIO Core**: Latest version
- **PlatformIO IDE**: VS Code extension or standalone
- **Python**: 3.7+ (for build scripts)

### Hardware Requirements
- **Teensy 4.1** (recommended) or **Teensy 4.0**
- **Required Sensors**: BMP388, BNO055, GPS module
- **Communication**: LoRa E22 module, RS232 interface

## 🚀 Quick Start

### 1. Install PlatformIO

```bash
# Install PlatformIO Core
pip install platformio

# Or install PlatformIO IDE extension in VS Code
```

### 2. Clone and Setup Project

```bash
git clone https://github.com/yunusemrecftci/RocketAvionicsSystem.git
cd RocketAvionicsSystem
```

### 3. Build and Upload

```bash
# Build for Teensy 4.1
pio run -e teensy41

# Upload to Teensy 4.1
pio run -e teensy41 -t upload

# Monitor serial output
pio device monitor -e teensy41
```

## 📋 Available Environments

### Development Environments

#### `teensy41` (Default)
- **Board**: Teensy 4.1
- **Debug Flags**: Enabled
- **Optimization**: -O2
- **Use Case**: General development

#### `teensy41_debug`
- **Board**: Teensy 4.1
- **Debug Flags**: All enabled
- **Optimization**: -O0 (no optimization)
- **Use Case**: Debugging and development

#### `teensy41_production`
- **Board**: Teensy 4.1
- **Debug Flags**: Disabled
- **Optimization**: -O3 (maximum optimization)
- **Use Case**: Production deployment

#### `teensy40`
- **Board**: Teensy 4.0
- **Debug Flags**: Enabled
- **Optimization**: -O2
- **Use Case**: Teensy 4.0 compatibility

## 🛠️ Build Commands

### Basic Commands

```bash
# Build all environments
pio run

# Build specific environment
pio run -e teensy41

# Clean build
pio run -t clean

# Upload firmware
pio run -e teensy41 -t upload

# Monitor serial output
pio device monitor -e teensy41

# List available devices
pio device list
```

### Advanced Commands

```bash
# Build with verbose output
pio run -e teensy41 -v

# Build and upload in one command
pio run -e teensy41 -t upload

# Build and monitor in one command
pio run -e teensy41 -t upload && pio device monitor -e teensy41

# Check library dependencies
pio lib list

# Update libraries
pio lib update
```

## 📦 Library Management

### Automatic Library Installation
PlatformIO automatically installs required libraries from `platformio.ini`:

```ini
lib_deps = 
    adafruit/Adafruit Unified Sensor @ ^1.1.9
    adafruit/Adafruit BMP3XX Library @ ^2.2.4
    adafruit/Adafruit BNO055 @ ^1.6.1
    mikalhart/TinyGPSPlus @ ^1.0.3
    e22-400t22s/LoRa_E22 @ ^1.0.0
    RobTillaart/RunningAverage @ ^1.4.0
```

### Manual Library Management

```bash
# Install specific library
pio lib install "Adafruit Unified Sensor"

# Search for libraries
pio lib search "BMP388"

# Show library information
pio lib show "Adafruit BMP3XX Library"
```

## 🔍 Debugging

### Serial Monitor

```bash
# Monitor with specific baud rate
pio device monitor -e teensy41 --baud 115200

# Monitor with filters
pio device monitor -e teensy41 --filter time,log2file
```

### Debug Build

```bash
# Build with debug symbols
pio run -e teensy41_debug

# Upload debug version
pio run -e teensy41_debug -t upload
```

## 📊 Build Scripts

### Pre-build Script (`scripts/pre_build.py`)
- Validates configuration
- Checks dependencies
- Performs pre-compilation tasks

### Post-build Script (`scripts/post_build.py`)
- Generates build reports
- Copies build artifacts
- Provides build statistics

### Running Scripts Manually

```bash
# Run pre-build script
python scripts/pre_build.py

# Run post-build script
python scripts/post_build.py
```

## 🔧 Configuration

### Customizing Build Flags

Edit `platformio.ini` to modify build configuration:

```ini
[env:teensy41]
build_flags = 
    -D DEBUG_FLIGHT_ALGO
    -D DEBUG_SENSORS
    -D CUSTOM_FLAG
    -std=gnu++17
    -O2
```

### Adding New Environments

```ini
[env:custom_board]
platform = teensy
board = teensy40
framework = arduino
build_flags = 
    -D CUSTOM_CONFIG
    -std=gnu++17
    -O2
upload_speed = 6000000
monitor_speed = 115200
lib_deps = 
    adafruit/Adafruit Unified Sensor @ ^1.1.9
    # Add other dependencies
```

## 🚨 Troubleshooting

### Common Issues

#### Build Errors
```bash
# Clean and rebuild
pio run -t clean
pio run -e teensy41

# Check library conflicts
pio lib list --outdated
pio lib update
```

#### Upload Errors
```bash
# Check device connection
pio device list

# Reset Teensy manually
# Press reset button on Teensy board
```

#### Library Issues
```bash
# Remove and reinstall libraries
pio lib uninstall "Library Name"
pio lib install "Library Name"
```

### Debug Information

```bash
# Verbose build output
pio run -e teensy41 -v

# Check environment configuration
pio run -e teensy41 --dry-run
```

## 📈 Performance Optimization

### Production Build

```bash
# Build optimized version
pio run -e teensy41_production

# Check firmware size
pio run -e teensy41_production -t size
```

### Memory Optimization

```ini
[env:teensy41_optimized]
platform = teensy
board = teensy41
framework = arduino
build_flags = 
    -std=gnu++17
    -O3
    -DNDEBUG
    -ffunction-sections
    -fdata-sections
build_unflags = -Os
lib_deps = 
    # Minimal dependencies
```

## 📞 Support

For PlatformIO-specific issues:
1. Check PlatformIO documentation
2. Verify library compatibility
3. Test with different environments
4. Check build scripts output

## 📄 License

This PlatformIO configuration is part of the Rocket Flight Computer project and is licensed under the MIT License.
