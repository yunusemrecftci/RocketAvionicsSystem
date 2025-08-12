#!/usr/bin/env python3
"""
Post-build script for Rocket Flight Computer
Handles post-compilation tasks and reporting
"""

import os
import sys
import json
from pathlib import Path
from datetime import datetime

# Add the project root to the Python path
project_dir = Path(__file__).parent.parent
sys.path.insert(0, str(project_dir))

def generate_build_report():
    """Generate build report with statistics"""
    print("📊 Generating build report...")
    
    # Get build information
    build_info = {
        "timestamp": datetime.now().isoformat(),
        "project": "Rocket Flight Computer",
        "version": "1.0.0",
        "target": os.environ.get("PIOTEST_ENV", "unknown"),
        "framework": "Arduino",
        "platform": "Teensy"
    }
    
    # Save build report
    report_file = project_dir / "build_report.json"
    with open(report_file, 'w') as f:
        json.dump(build_info, f, indent=2)
    
    print(f"✅ Build report saved to: {report_file}")
    return True

def check_build_output():
    """Check if build output files exist"""
    print("🔍 Checking build output...")
    
    # Check for common build output files
    build_dir = project_dir / ".pio" / "build"
    if build_dir.exists():
        for env_dir in build_dir.iterdir():
            if env_dir.is_dir():
                firmware_file = env_dir / "firmware.hex"
                if firmware_file.exists():
                    size = firmware_file.stat().st_size
                    print(f"✅ Firmware built: {firmware_file} ({size} bytes)")
                else:
                    print(f"⚠️  Firmware not found in {env_dir}")
    
    return True

def copy_build_artifacts():
    """Copy build artifacts to output directory"""
    print("📁 Copying build artifacts...")
    
    # Create output directory
    output_dir = project_dir / "output"
    output_dir.mkdir(exist_ok=True)
    
    # Copy firmware files
    build_dir = project_dir / ".pio" / "build"
    if build_dir.exists():
        for env_dir in build_dir.iterdir():
            if env_dir.is_dir():
                firmware_file = env_dir / "firmware.hex"
                if firmware_file.exists():
                    output_firmware = output_dir / f"firmware_{env_dir.name}.hex"
                    import shutil
                    shutil.copy2(firmware_file, output_firmware)
                    print(f"✅ Copied: {output_firmware}")
    
    return True

def main():
    """Main post-build function"""
    print("🚀 Rocket Flight Computer - Post-build Script")
    print("=" * 50)
    
    # Generate build report
    generate_build_report()
    
    # Check build output
    check_build_output()
    
    # Copy build artifacts
    copy_build_artifacts()
    
    print("✅ Post-build completed successfully!")
    return 0

if __name__ == "__main__":
    sys.exit(main())
