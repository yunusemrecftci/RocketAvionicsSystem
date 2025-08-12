# Makefile for Rocket Flight Computer
# Compiles C++ code for Teensy 4.x with Arduino/Teensyduino libraries

# Compiler settings
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2
INCLUDES = -I/Applications/Arduino.app/Contents/Java/hardware/teensy/avr/cores/teensy4 \
           -I/Applications/Arduino.app/Contents/Java/hardware/teensy/avr/libraries \
           -I/Applications/Arduino.app/Contents/Java/libraries \
           -I/Applications/Arduino.app/Contents/Java/hardware/tools/arm/arm-none-eabi/include

# Library paths (adjust these paths according to your Arduino installation)
ARDUINO_PATH = /Applications/Arduino.app/Contents/Java
TEENSY_PATH = $(ARDUINO_PATH)/hardware/teensy/avr
LIBRARIES_PATH = $(ARDUINO_PATH)/libraries

# Source files
SOURCES = rocket_flight_computer.cpp
OBJECTS = $(SOURCES:.cpp=.o)
TARGET = rocket_flight_computer

# Default target
all: $(TARGET)

# Compile the main executable
$(TARGET): $(OBJECTS)
	$(CXX) $(OBJECTS) -o $(TARGET) $(LDFLAGS)

# Compile source files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# Clean build files
clean:
	rm -f $(OBJECTS) $(TARGET)

# Install dependencies (Arduino libraries)
install-deps:
	@echo "Installing required Arduino libraries..."
	# Note: These libraries should be installed via Arduino IDE Library Manager
	@echo "Required libraries:"
	@echo "- Wire (built-in)"
	@echo "- Adafruit Sensor"
	@echo "- Adafruit BMP3XX"
	@echo "- Adafruit BNO055"
	@echo "- TinyGPS++"
	@echo "- LoRa E22"
	@echo "- RunningAverage"

# Help target
help:
	@echo "Available targets:"
	@echo "  all          - Build the flight computer executable"
	@echo "  clean        - Remove build files"
	@echo "  install-deps - Show required dependencies"
	@echo "  help         - Show this help message"

.PHONY: all clean install-deps help
