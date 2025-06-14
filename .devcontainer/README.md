# KVML Development Container

This devcontainer provides a complete ESP32-S3 development environment for the KVML project.

## Features

- **ESP-IDF**: Pre-installed ESP-IDF framework with ESP32-S3 support
- **Development Tools**: GCC toolchain, CMake, Python, Git
- **VS Code Extensions**: 
  - ESP-IDF Extension
  - C/C++ IntelliSense
  - Python support
  - CMake Tools
- **USB Device Access**: Automatic mounting of ESP32-S3 serial devices
- **Claude Code**: Pre-installed for AI-assisted development

## Usage

### Opening in VS Code

1. Clone or download the KVML project
2. Open the `kvml/` directory in VS Code
3. When prompted, click "Reopen in Container"
4. Wait for the container to build (first time only)

### Manual Container Build

```bash
cd kvml/
docker build -t kvml-dev .devcontainer/
docker run -it --rm \
  --device=/dev/ttyACM0:/dev/ttyACM0 \
  --device=/dev/ttyACM1:/dev/ttyACM1 \
  -v $(pwd):/workspace \
  kvml-dev
```

### ESP32-S3 Development

Once in the container:

```bash
# Build the project
idf.py build

# Flash to master board
idf.py -p /dev/ttyACM0 flash

# Flash to slave board  
idf.py -p /dev/ttyACM1 flash
```

## Hardware Requirements

- Two ESP32-S3 boards connected via USB
- USB devices typically appear as `/dev/ttyACM0` and `/dev/ttyACM1`
- May also appear as `/dev/ttyUSB0` and `/dev/ttyUSB1` depending on the board

## Environment Variables

- `IDF_PATH`: Points to ESP-IDF installation
- `PATH`: Includes ESP-IDF tools and utilities

## Notes

- The container runs as user `node` (uid 1000)
- ESP-IDF environment is automatically sourced in shell sessions
- Claude Code is available for AI-assisted development
- Firewall rules restrict network access for security