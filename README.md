# KVML - Keyboard and Video Mouse Link

A dual ESP32-S3 based Bluetooth HID multiplexer that routes input from a single USB keyboard and mouse to two separate computers via Bluetooth.

## Features

- Dual ESP32-S3 board system with master/slave architecture
- Bluetooth HID interface for keyboard and absolute pointer
- USB host for keyboard and mouse input detection
- UART communication protocol between boards
- Web-based configuration interface with device management
- OTA firmware updates with dual app partitions
- Persistent BLE identity and configuration storage
- Seamless switching between two host computers

## Hardware Requirements

- 2x ESP32-S3 development boards with USB-OTG support
- USB cables for programming and power
- Host computers with Bluetooth support

## Development Environment Setup

1. **Set up ESP-IDF**
   ```bash
   # Source the ESP-IDF environment
   source /home/ties/project/esp32s3/.env
   
   # Verify the setup
   idf.py --version
   ```

2. **Get the source code**
   ```bash
   git clone <repository-url> kvml
   cd kvml
   ```

## Building and Flashing

1. **Configure the project**
   ```bash
   idf.py set-target esp32s3
   idf.py menuconfig  # Optional: Configure project settings
   ```

2. **Build the project**
   ```bash
   idf.py build
   ```

3. **Flash the firmware**
   ```bash
   # Flash to the first ESP32-S3 (Master)
   idf.py -p /dev/ttyUSB0 flash
   
   # Flash to the second ESP32-S3 (Slave)
   idf.py -p /dev/ttyUSB1 flash
   ```
   
   Replace `/dev/ttyUSB0` and `/dev/ttyUSB1` with the correct serial ports on your system.

4. **Monitor the output**
   ```bash
   idf.py monitor
   ```

## Project Structure

- `main/` - Main application entry point with app initialization
- `components/` - Modular project components
  - `ble_hid/` - BLE HID implementation for keyboard and mouse profiles
  - `usb_host/` - USB host drivers for keyboard and mouse input detection
  - `uart_protocol/` - UART communication protocol for inter-board communication
  - `web_ui/` - Web interface for configuration and status monitoring
  - `common/` - Shared utilities, configuration management, and error handling
  - `kvml/` - Core multiplexer logic and host switching functionality
  - `taskmaster_ai/` - AI-based task management and screen region handling
  - `kvml_main/` - Application-level initialization and coordination
- `partitions.csv` - Partition table with OTA slots and SPIFFS configurations
- `sdkconfig.defaults` - Default build configuration parameters

## Component Details

### Common
- Configuration management using NVS
- Error handling and logging utilities
- Shared data structures and constants

### BLE HID
- Bluetooth Low Energy HID device implementation
- Keyboard and mouse profiles
- Persistent BLE bonding with host computers

### USB Host
- USB HID host interface for keyboard and mouse
- Input event detection and processing

### UART Protocol
- Framing protocol for reliable communication between boards
- Command and response handling

### Web UI
- Configuration web server with RESTful API
- Status monitoring interface
- SPIFFS-based file storage for web assets

### KVML Core
- Host switching logic
- Device discovery and management
- Input routing between hosts

## Partition Table

The partition table is configured as follows:
- nvs: 24KB for BLE bonding keys and configuration
- ota_0: 1.5MB for firmware slot 1
- ota_1: 1.5MB for firmware slot 2
- spiffs: 512KB for Web UI and configuration files

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
