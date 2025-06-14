# KVML - Keyboard and Video Mouse Link

A dual ESP32-S3 based Bluetooth HID multiplexer that routes input from a single USB keyboard and mouse to two separate computers via Bluetooth.

## Features

- Dual ESP32-S3 board system with active/inactive architecture
- Bluetooth HID interface for keyboard and relative mouse movement
- USB host for keyboard and mouse input detection
- High-speed UART communication protocol (921600 baud) between boards
- Event-driven UART reception with duplicate report filtering
- Mouse movement accumulation and rate limiting for smooth BLE transmission
- Automatic BLE device naming with MAC address suffix (e.g., "KM740A")
- Ctrl+Ctrl double-tap hotkey for seamless host switching with CapsLock blink feedback
- LED state synchronization between ESP32s and connected computers
- OTA firmware updates with dual app partitions
- Persistent BLE bonding and configuration storage

## Hardware Requirements

- 2x ESP32-S3 development boards with USB-OTG support
- USB cables for programming and power
- 3 jumper wires for UART communication between boards:
  - Board A GPIO15 (TX) ↔ Board B GPIO40 (RX)
  - Board A GPIO40 (RX) ↔ Board B GPIO15 (TX)  
  - Board A GND ↔ Board B GND
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
   # Flash to the first ESP32-S3
   idf.py -p /dev/ttyACM0 flash
   
   # Flash to the second ESP32-S3
   idf.py -p /dev/ttyACM1 flash
   ```
   
   Replace `/dev/ttyACM0` and `/dev/ttyACM1` with the correct serial ports on your system.

4. **Monitor the output**
   ```bash
   idf.py monitor
   ```

## Project Structure

- `main/` - Main application entry point with app initialization
- `components/` - Modular project components
  - `ble_hid/` - BLE HID implementation for keyboard and mouse profiles
  - `usb_host/` - USB host drivers for keyboard and mouse input detection
  - `uart_protocol/` - High-speed UART communication protocol for inter-board communication
  - `common/` - Shared utilities, configuration management, and error handling
  - `kvml/` - Core multiplexer logic and host switching functionality
- `partitions.csv` - Partition table with OTA slots and NVS configurations
- `sdkconfig.defaults` - Default build configuration parameters

## Component Details

### Common
- Configuration management using NVS
- Error handling and logging utilities
- Shared data structures and constants

### BLE HID
- Bluetooth Low Energy HID device implementation
- Combined keyboard and mouse profiles for Ubuntu compatibility
- Automatic device naming with MAC address suffix
- Persistent BLE bonding with host computers
- LED state output report handling for keyboard indicators

### USB Host
- USB HID host interface for keyboard and mouse
- Hotkey detection (Ctrl+Ctrl double-tap) for host switching
- Input event detection and processing with duplicate filtering
- Mouse movement accumulation and rate limiting

### UART Protocol
- High-speed framing protocol (921600 baud) for reliable communication
- Event-driven reception with CRC verification
- Active/inactive board coordination to prevent input conflicts
- Keyboard, mouse, and LED state forwarding between boards
- Auto-response system for connectivity testing

### KVML Core
- Active/inactive host switching logic with visual CapsLock feedback
- Device discovery and management
- Input routing coordination between ESP32-S3 boards

## Usage

1. **Initial Setup**
   - Flash both ESP32-S3 boards with the same firmware
   - Connect the UART wires between boards as shown in Hardware Requirements
   - Power on both boards - they will start in active/inactive mode automatically

2. **BLE Pairing**
   - Each ESP32-S3 will advertise as "KM" + last 2 digits of MAC (e.g., "KM740A")
   - Pair each device with a different computer
   - The combined keyboard+mouse device will appear in your Bluetooth settings

3. **Host Switching**
   - **Ctrl+Ctrl double-tap**: Press and release Left Ctrl twice within ~200ms
   - CapsLock LED will blink to indicate successful switch
   - The inactive board's USB input is forwarded to the active board's BLE connection
   - Only one board forwards input to BLE at a time to prevent conflicts

4. **LED Synchronization**
   - Keyboard LED states (NumLock, CapsLock, ScrollLock) sync between both systems
   - When switching hosts, LED states are preserved and applied correctly

## Partition Table

The partition table is configured as follows:
- nvs: 32KB for BLE bonding keys and configuration
- phy_init: 4KB for RF calibration data  
- factory: ~4MB for main firmware application

## Troubleshooting

### UART Communication Issues
- Verify GPIO15 ↔ GPIO40 cross-connection between boards
- Check GND connection between boards
- Monitor logs for "Frame loopback: no data received" warnings
- UART runs at 921600 baud with event-driven reception

### BLE Connection Problems
- Ensure both computers support Bluetooth LE
- Clear old pairings if device names changed
- Each ESP32-S3 creates a unique device name based on its MAC address
- Ubuntu/Linux: Check `bluetoothctl` for pairing status

### Host Switching Not Working
- Ctrl+Ctrl timing is critical (~200ms between presses)
- Look for CapsLock blink feedback on successful switch
- Check UART communication between boards
- Only Left Ctrl triggers the hotkey detection

### Performance Issues
- Mouse movement uses accumulation and rate limiting for smooth BLE transmission
- Event-driven UART prevents CPU hogging
- Duplicate report filtering reduces unnecessary transmissions

## Technical Notes

- **Mouse Movement**: Relative movement with int8_t range clamping
- **UART Protocol**: CRC-verified frames with automatic retry
- **BLE Security**: No-IO pairing for maximum host compatibility  
- **Memory Management**: NVS for persistent configuration and bonding keys
- **Logging**: Configurable ESP-IDF log levels for debugging

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
