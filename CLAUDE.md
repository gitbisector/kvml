# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Claude Code Configuration

### Model Default
Use `sonnet` (claude-sonnet-4-20250514) as the default model for this project.

## Project Overview

KVML (Keyboard and Video Mouse Link) is a dual ESP32-S3 Bluetooth HID multiplexer that routes USB keyboard/mouse input to two computers via Bluetooth. It uses a master/slave architecture with inter-board UART communication.

## Essential Commands

### Environment Setup

#### Using the Development Container (Recommended)
```bash
# Open the kvml/ directory in VS Code
# Click "Reopen in Container" when prompted
# ESP-IDF environment is automatically configured
```

#### Manual Environment Setup
```bash
# ESP-IDF is already available in some container environments
# If not available, install ESP-IDF manually
# No additional environment setup needed in devcontainer

# Configure WiFi credentials for web UI
./setup_wifi_config.sh
# Then edit main/wifi_config.h with actual credentials
```

### Build and Flash
```bash
# Set target (required for fresh builds)
idf.py set-target esp32s3

# Build project
idf.py build

# Flash firmware to boards
idf.py -p /dev/ttyACM0 flash  # Master board
idf.py -p /dev/ttyACM1 flash  # Slave board

# Monitor logs (user handles separately)
# DO NOT use idf.py monitor - it locks the serial port
```

## Commands NOT to use
- **Never run `idf.py monitor`** - it locks up the serial port and causes issues in this environment
- Use only `idf.py build` and `idf.py flash` 
- The user will handle monitoring separately through their own terminal

## Architecture

### Component Structure
- **kvml/**: Core multiplexer logic, host switching, device discovery
- **ble_hid/**: Bluetooth HID keyboard/mouse implementation  
- **usb_host/**: USB HID host for input device detection
- **uart_protocol/**: Inter-board communication between master/slave ESP32s
- **web_ui/**: RESTful API and web interface for configuration
- **common/**: Shared utilities, NVS config management, error handling
- **taskmaster_ai/**: AI task management and screen region handling

### Data Flow
USB devices → USB Host → KVML Core → BLE HID → Target computers
Master/slave coordination via UART Protocol

### Key Configuration Files
- `partitions.csv`: Memory layout (NVS, dual OTA, SPIFFS)
- `sdkconfig.defaults`: ESP32-S3 build configuration  
- `main/idf_component.yml`: Component dependencies
- `main/wifi_config.h`: WiFi credentials (created by setup script)

### Storage Layout
- **NVS**: BLE bonding keys and persistent configuration
- **Dual OTA**: Reliable firmware updates with fallback
- **SPIFFS**: Web UI assets and configuration files

## Development Notes

- **Development Container**: Use `.devcontainer/` for complete ESP32-S3 development environment
- ESP-IDF is pre-configured in the container environment
- Use two USB ports for master/slave board development (`/dev/ttyACM0`, `/dev/ttyACM1`)
- Web UI requires WiFi configuration for remote access
- Components follow ESP-IDF standards with CMakeLists.txt and separate include/src structure
- UART protocol enables seamless switching between host computers

## Container Development

The project includes a complete VS Code devcontainer setup:
- Pre-installed ESP-IDF with ESP32-S3 support
- C/C++ IntelliSense and ESP-IDF extensions
- Automatic USB device mounting for flashing
- Claude Code integration for AI-assisted development
