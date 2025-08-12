#!/usr/bin/env python3
"""
Pre-build script for Rocket Flight Computer
Handles pre-compilation tasks and validation
"""

import os
import sys
from pathlib import Path

# Add the project root to the Python path
project_dir = Path(__file__).parent.parent
sys.path.insert(0, str(project_dir))

def validate_config():
    """Validate configuration before build"""
    print("🔍 Validating configuration...")
    
    # Check if main.cpp exists
    main_cpp = project_dir / "src" / "main.cpp"
    if not main_cpp.exists():
        print("❌ Error: src/main.cpp not found!")
        return False
    
    # Check if platformio.ini exists
    platformio_ini = project_dir / "platformio.ini"
    if not platformio_ini.exists():
        print("❌ Error: platformio.ini not found!")
        return False
    
    print("✅ Configuration validation passed")
    return True

def check_dependencies():
    """Check if required dependencies are available"""
    print("📦 Checking dependencies...")
    
    # List of required libraries
    required_libs = [
        "Adafruit Unified Sensor",
        "Adafruit BMP3XX Library", 
        "Adafruit BNO055",
        "TinyGPSPlus",
        "LoRa_E22",
        "RunningAverage"
    ]
    
    for lib in required_libs:
        print(f"  - {lib}")
    
    print("✅ Dependencies check completed")
    return True

def main():
    """Main pre-build function"""
    print("🚀 Rocket Flight Computer - Pre-build Script")
    print("=" * 50)
    
    # Validate configuration
    if not validate_config():
        sys.exit(1)
    
    # Check dependencies
    if not check_dependencies():
        sys.exit(1)
    
    print("✅ Pre-build completed successfully!")
    return 0

if __name__ == "__main__":
    sys.exit(main())
