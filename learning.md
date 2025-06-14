# Key Learnings from ESP32-S3 BLE HID Implementation

This document captures critical insights learned during the development of the KVML BLE HID multiplexer, particularly around USB-to-BLE HID bridging and the subtle but crucial differences between protocols.

## BLE HID vs USB HID Report Format Differences

### The Critical Report ID Issue

**Problem**: Ubuntu was misinterpreting mouse data, causing "mouse clicks count as X, X counts as Y and Y counts as wheel" behavior.

**Root Cause**: Fundamental difference between USB HID and BLE HID report formats:

- **USB HID**: Uses Report ID as the first byte: `[report_id, buttons, X, Y, wheel]` (5 bytes)
- **BLE HID**: GATT characteristic handle identifies report type, Report ID should NOT be included: `[buttons, X, Y, wheel]` (4 bytes)

**Fix**: Remove Report ID from all BLE HID payloads. The GATT characteristic handle (e.g., `HANDLE_MOUSE_INPUT_REPORT`) tells the host what type of report it is.

```c
// WRONG (causes 1-byte offset in Ubuntu)
uint8_t report[5] = {0x01, buttons, delta_x, delta_y, scroll_y};

// CORRECT for BLE HID
uint8_t report[4] = {buttons, delta_x, delta_y, scroll_y};
```

### Protocol Mode Considerations

- **Boot Protocol Mode**: Simplified, fixed format for basic compatibility
- **Report Protocol Mode**: Full HID descriptor support, more flexible
- **Key Insight**: BLE HID works better in Report Protocol Mode with proper GATT service structure

## USB Composite Device Complexity

### Mouse with "Keyboard" Interface

**Discovery**: Many USB mice report as composite devices with both mouse and keyboard interfaces, even when they're just mice.

**Challenge**: The "keyboard" interface on a mouse device actually reports:
- Mouse button presses as keyboard key codes
- Scroll wheel events as special key combinations
- NOT actual keyboard input

**Solution**: Device type detection based on actual data patterns:
```c
// Determine if this is actually a keyboard or a mouse with keyboard interface
hid_device.is_actual_keyboard = false;  // Default to mouse, determine by reports
```

**Key Mapping Discovered**:
- `keys[i] == 0x01` → Left mouse button
- `keys[i] == 0x02` → Right mouse button  
- `keys[i] == 0x04` → Middle mouse button
- `modifier == 0x04` + specific keys → Scroll wheel events

## BLE Performance and Rate Limiting

### BLE is Inherently Slower than USB

**USB**: Can handle 1000Hz+ polling rates easily
**BLE**: Limited by connection intervals (typically 7.5ms - 45ms), packet processing overhead

### Adaptive Rate Limiting Strategy

**Problem**: High-frequency USB mouse input overwhelms BLE stack, causing `BLE_HS_ENOMEM` errors.

**Solution**: Multi-layered approach:
1. **Time-based rate limiting**: Minimum interval between sends (1-50ms adaptive)
2. **Movement accumulation**: Combine multiple small movements into single reports
3. **Adaptive timer intervals**: Slow down when BLE stack is congested
4. **Priority handling**: Send button changes immediately, accumulate movement

```c
// Successful approach
static uint32_t min_interval_us = 1000;  // Start at 1ms, adapt up to 50ms
static uint32_t timer_interval_us = 8000; // 125Hz target, adapt based on congestion
```

### BLE Memory Management

**Key Insight**: `ble_hs_mbuf_from_flat()` failures indicate BLE stack congestion, not permanent failure.

**Strategy**: 
- Detect `BLE_HS_ENOMEM` errors specifically
- Increase rate limiting only for congestion errors
- Don't penalize for connection errors (`BLE_HS_ENOTCONN`)

## Ubuntu Bluez Compatibility Quirks

### GATT Service Structure Requirements

Ubuntu expects specific service characteristics for proper device recognition:

1. **Combined HID Service**: Mouse + Keyboard in single service works better than separate services
2. **Boot Protocol Characteristics**: Required even if using Report Protocol
3. **Device Information Service**: Critical for proper device categorization
4. **Battery Service**: Expected by most HID hosts

### Advertisement Data Optimization

- **Appearance**: `0x03c1` (HID keyboard+mouse combo) for proper Ubuntu categorization
- **Service UUIDs**: Must include HID (0x1812), Battery (0x180f), Device Info (0x180a)
- **Complete Local Name**: Keep short for space efficiency

## Security and Pairing Strategy

### Ubuntu Bluez Pairing

**Winning Configuration**:
- IO Capability: `BLE_SM_IO_CAP_NO_IO` (just-works pairing)
- Bonding: Enabled for persistent reconnection
- MITM Protection: Disabled (avoids PIN requirements)
- Key Distribution: Conservative (encryption key only)

## Debugging Techniques That Worked

### btmon for BLE Analysis
```bash
sudo btmon
```
Reveals actual BLE packet contents, connection parameters, and GATT operations.

### Cross-contamination Detection
Adding explicit error logging when unexpected functions are called:
```c
ESP_LOGE(TAG, "***** BLE KEYBOARD SEND CALLED - THIS SHOULD NOT HAPPEN WITH MOUSE! *****");
```

### USB Report Analysis
Logging raw USB report data to understand composite device behavior:
```c
ESP_LOGI(TAG, "Raw mouse report data[0-4]: %02x %02x %02x %02x %02x", 
         data[0], data[1], data[2], data[3], data[4]);
```

## Content-Based Routing: Device vs Report Analysis

### The Fundamental Challenge

**Problem**: Both USB keyboards and mice can present composite HID interfaces that report both keyboard AND mouse protocols, making device-level classification unreliable.

**Example**: A USB mouse may present two interfaces:
- Interface 0: Keyboard protocol (for mouse buttons/scroll via "key codes")
- Interface 1: Mouse protocol (for movement data)

### Failed Approach: Device-Level Classification

**Initial Strategy**: Try to classify entire USB devices as "keyboard" or "mouse" based on descriptors.

**Why It Failed**:
```c
// This approach doesn't work because BOTH devices report BOTH interfaces
bool is_actual_keyboard = /* some classification logic */;
if (is_actual_keyboard) {
    // Route to BLE keyboard
} else {
    // Route to BLE mouse
}
```

**Root Cause**: Modern HID devices are composite - a mouse needs keyboard interface for buttons, a keyboard might have media keys that look like consumer reports.

### Successful Approach: Content-Based Routing

**Key Insight**: Analyze the CONTENT of individual USB HID reports, not the device type.

**Implementation**:
```c
typedef enum {
    HID_REPORT_TYPE_KEYBOARD,       // Real keyboard input
    HID_REPORT_TYPE_MOUSE_BUTTONS,  // Mouse button events via keyboard interface  
    HID_REPORT_TYPE_MOUSE_SCROLL,   // Mouse scroll events via keyboard interface
    HID_REPORT_TYPE_EMPTY           // Empty/no-op report
} hid_report_content_type_t;

static hid_report_content_type_t analyze_keyboard_report_content(uint8_t modifier, const uint8_t *keys)
{
    // Analyze each report individually based on key codes used
    for (int i = 0; i < 6; i++) {
        if (keys[i] >= 0x01 && keys[i] <= 0x08) {
            // Key codes 0x01-0x08 are mouse buttons, not keyboard keys
            return HID_REPORT_TYPE_MOUSE_BUTTONS;
        } else if (keys[i] >= 0x04 && keys[i] <= 0x65) {
            // Standard HID keyboard scan codes
            return HID_REPORT_TYPE_KEYBOARD;
        }
    }
    // Check for scroll patterns...
}
```

### Mouse Button Mapping Discovery

**Critical Finding**: Mouse buttons from keyboard interface use specific key codes:
```c
// Mouse button mappings via "keyboard" interface
if (keys[i] == 0x01) mouse_buttons |= 0x01; // Left button
if (keys[i] == 0x02) mouse_buttons |= 0x02; // Right button  
if (keys[i] == 0x04) mouse_buttons |= 0x04; // Middle button
if (keys[i] == 0x05) mouse_buttons |= 0x08; // Button 4 (Back)
if (keys[i] == 0x06) mouse_buttons |= 0x10; // Button 5 (Forward)
```

### Scroll Wheel Detection

**Pattern**: Scroll events have specific modifier + key combinations:
```c
// Scroll wheel via keyboard interface
if (modifier == 0x04) {
    if (keys[i] == 0x05) scroll_value = 1;  // Scroll up
    if (keys[i] == 0x0F) scroll_value = -1; // Scroll down
}
```

### Extended Mouse Button Support

**Enhancement**: Support for additional mouse buttons (4 & 5) required:
1. **USB Detection**: Expand key code detection to 0x01-0x08
2. **BLE HID Descriptor**: Update from 3 to 5 supported buttons:
```c
0x29, 0x05,        //     Usage Maximum (5) - Support buttons 1-5
0x95, 0x05,        //     Report Count (5) - 5 buttons  
0x75, 0x03,        //     Report Size (3) - Only 3 padding bits needed
```

### Routing Strategy

**Per-Report Routing**:
```c
switch (analyze_keyboard_report_content(modifier, keys)) {
    case HID_REPORT_TYPE_MOUSE_BUTTONS:
        // Route to BLE mouse button report
        ble_hid_mouse_report_enhanced(0, 0, mouse_buttons, 0);
        break;
    case HID_REPORT_TYPE_MOUSE_SCROLL:
        // Route to BLE mouse scroll report  
        ble_hid_mouse_report_enhanced(0, 0, 0, scroll_value);
        break;
    case HID_REPORT_TYPE_KEYBOARD:
        // Route to BLE keyboard report
        ble_hid_keyboard_report(modifier, reserved, keys);
        break;
}
```

### Key Architectural Lessons

1. **Don't Trust Device Types**: USB HID device classification is unreliable for composite devices
2. **Analyze Report Content**: Each individual report must be analyzed for routing decisions  
3. **Support Extended Features**: Modern mice have 4+ buttons that need proper mapping
4. **Separate Interfaces Are Normal**: Expect mouse movement on one interface, buttons on another

**Bottom Line**: Content-based routing at the report level is essential for properly handling modern composite HID devices that blur the line between keyboards and mice.

## USB Enumeration Robustness Issues

### Device Already Connected During Boot

**Problem**: When USB HID devices (keyboard/mouse) are already connected during ESP32 boot, enumeration errors occur:
```
E (9442) USBH: Dev 1 EP 0 Error  
E (9452) ENUM: Bad transfer status 1: CHECK_FULL_CONFIG_DESC
E (9452) ENUM: [0:0] CHECK_FULL_CONFIG_DESC FAILED
```

**Analysis**: 
- Previously saw `CHECK_FULL_DEV_DESC` errors, now seeing `CHECK_FULL_CONFIG_DESC` 
- This indicates the enumeration is progressing further but still failing on configuration descriptor reads
- The system continues to function normally despite these errors - BLE HID works fine
- Hot-plugging devices after boot appears to work better than cold-plug scenarios

**Workarounds Implemented**:
1. **Extended timing delays**: 500ms wait after USB host initialization  
2. **USB enumeration filter**: Added callback to handle problematic devices gracefully
3. **Enhanced error logging**: Clear messages that enumeration errors are non-fatal
4. **USB Host configuration tuning**: Added `CONFIG_USB_HOST_CONTROL_TRANSFER_MAX_SIZE=512`

**Recommended Usage**: 
- For most reliable operation, start ESP32 without USB devices connected
- Plug in keyboard/mouse after system has fully booted and BLE is advertising
- If devices must be connected during boot, expect enumeration errors but normal operation afterwards

### The Breakthrough: Modifier-Based Routing

**Problem**: Complex content analysis with overlapping key code ranges was error-prone and missed the fundamental distinction.

**Solution**: Use the USB HID modifier byte + magic pattern as the routing filter:

```c
if (modifier == 0x04 && keys[5] == 0x01) {
    // Mouse interface data coming through keyboard HID interface
    // Handle mouse buttons (0x01, 0x02, 0x04, 0x08, 0x10) and scroll (0x05, 0x0F)  
    // Route to BLE mouse reports
} else {
    // Real keyboard input (including Left Alt when keys[5] != 0x01)
    // Route to BLE keyboard reports
}
```

**Key Discovery**: 
- **mod=0x04 + keys[5]=0x01**: Mouse events (buttons, scroll) via keyboard interface
- **mod=0x04 + keys[5]=0x00**: Left Alt key (normal keyboard input)
- **Any other modifier**: Normal keyboard input (Ctrl, Shift, etc.)
- **The Magic Pattern**: Mouse interface always has the specific combination of Left Alt modifier (0x04) + magic byte 0x01 in keys[5]

**Benefits**:
1. **Simpler Logic**: One check instead of complex key code analysis
2. **100% Reliable**: Clear protocol-level distinction
3. **Future-Proof**: Works regardless of mouse button count or key layouts

### State Tracking for Duplicate Report Elimination

**Problem**: USB keyboards continuously send reports even when no keys are pressed, causing BLE spam.

**Solution**: Shadow state tracking with change detection:

```c
typedef struct {
    uint8_t last_modifier;
    uint8_t last_keys[6];
} keyboard_state_t;

// Only send BLE report if state changed
bool report_changed = (modifier != last_keyboard_state.last_modifier);
if (!report_changed) {
    for (int i = 0; i < 6; i++) {
        if (keys[i] != last_keyboard_state.last_keys[i]) {
            report_changed = true;
            break;
        }
    }
}
```

**Benefits**:
1. **Reduced BLE Traffic**: Only send when state actually changes
2. **Better Performance**: Eliminates unnecessary packet processing
3. **Cleaner Logs**: No spam from empty/duplicate reports

## Architecture Insights

### Single Service vs Multiple Services

**Lesson**: Ubuntu works better with a single HID service containing multiple report characteristics than separate mouse/keyboard services.

### Handle Management

**Critical**: GATT characteristic handles must be properly managed and logged for debugging:
```c
ESP_LOGI(TAG, "Mouse Input Report handle: 0x%04x", ble_hid_svc_char_handles[HANDLE_MOUSE_INPUT_REPORT]);
```

### State Management

**Key Insight**: BLE HID requires careful state tracking because you can't query current state like with USB:
- Button state persistence across reports
- Connection state validation before sending
- Protocol mode awareness

## Performance Optimization

### Movement Accumulation Benefits

- Reduces BLE packet overhead
- Handles USB polling rate mismatches  
- Prevents BLE stack overload
- Maintains smooth cursor movement

### Error Recovery Strategy

**Successful Pattern**:
1. Detect specific error types
2. Apply backpressure only for congestion  
3. Retry with exponential backoff
4. Clear state on connection loss

## Future Considerations

### Keyboard Implementation

Based on mouse learnings:
- Use same 4-byte report format principle (no Report ID)
- Implement similar rate limiting for key repeat scenarios
- Consider modifier key handling for composite devices

### Multi-host Support

- Connection parameter optimization per host type
- Host-specific compatibility modes
- Connection priority management

## Multiple Mouse Type Support and Data Format Discovery

### The 8-Byte Mouse Format Challenge

**Problem**: Different mice use different USB HID report formats, breaking our 4-byte mouse handling.

**Discovery Process**:
1. **Comprehensive Raw Data Logging**: Added hex dumps and bit-by-bit analysis of all USB reports
2. **Pattern Recognition**: Identified 8-byte format: `[Report ID, Buttons, X_low, X_high, Y_low, Y_high, Scroll, Reserved]`
3. **Format-Specific Handling**: Movement data uses 16-bit little-endian encoding, needs clamping to 8-bit for BLE

**Key Format Insights**:
```c
// 8-byte format structure
data[0] = Report ID (0x01 for pure movement/scroll, other values for hybrid mice)
data[1] = Button state (when data[0] == 0x01) 
data[2] = X movement low byte
data[3] = X movement high byte  
data[4] = Y movement low byte
data[5] = Y movement high byte
data[6] = Scroll data (0x01 = up, 0xFF = down, 0x00 = none)
data[7] = Reserved/padding

// 16-bit movement extraction
int16_t x_16 = (int16_t)(data[2] | (data[3] << 8));
int16_t y_16 = (int16_t)(data[4] | (data[5] << 8));
// Clamp to 8-bit for BLE HID
int8_t delta_x = (x_16 > 127) ? 127 : (x_16 < -127) ? -127 : (int8_t)x_16;
```

### Button Overlay System for Hybrid Mice

**Critical Problem**: Some mice report buttons via keyboard interface but movement via mouse interface. Movement reports would override button state, causing button releases when moving the mouse.

**Solution**: Persistent button overlay system:
```c
// Global button state from keyboard interface  
static uint8_t button_overlay = 0;

// In keyboard callback - update overlay
if (modifier == 0x04 && keys[5] == 0x01) {
    button_overlay = mouse_buttons;  // Store button state
}

// In mouse callback - combine with overlay
if (data[0] == 1) {
    actual_buttons = data[1];  // Use buttons from pure movement reports
} else {
    actual_buttons = button_overlay;  // Use persistent overlay for hybrid mice
}
```

**Why This Works**:
1. **Persistent State**: Button state survives across movement reports
2. **Interface Bridging**: Combines data from keyboard and mouse interfaces seamlessly  
3. **Mouse Type Agnostic**: Works for both hybrid and pure mice
4. **No False Releases**: Movement can't accidentally clear button state

### Conditional Debug Compilation System

**Problem**: Need to debug different mice without affecting production code.

**Solution**: Centralized debug header with conditional compilation:
```c
// ble_hid_debug.h
#ifndef BLE_HID_DEBUG_DISABLE_REPORTING
#define BLE_HID_DEBUG_DISABLE_REPORTING 0  // 0 = normal, 1 = debug mode
#endif

// In implementation files
#if BLE_HID_DEBUG_DISABLE_REPORTING
    ESP_LOGI(TAG, "BLE reporting disabled for debugging - would send: buttons=0x%02x", buttons);
    return ESP_OK;  // Skip actual BLE send
#else
    return ble_gattc_notify_custom(conn_handle, char_handle, om);
#endif
```

**Benefits**:
1. **Zero Code Changes**: Switch modes by changing one define
2. **Complete Data Flow**: See USB parsing without BLE interference
3. **Production Ready**: No debug code in release builds
4. **Centralized Control**: One file controls all debug flags

### Unified Mouse Format Handling Strategy

**Architectural Decision**: Always parse as 8-byte format, with fallback to 4-byte for older mice.

```c
if (length == 8) {
    // Modern 8-byte format
    // Always extract 16-bit movement data
    // Conditional scroll processing based on data[0]
    // Button overlay handling for hybrid mice
} else if (length >= 4) {
    // Legacy 4-byte format  
    // Direct 8-bit movement data
    // Standard button/scroll handling
}
```

**Key Principles**:
1. **Length-Based Detection**: Determine format from report length
2. **16-bit Movement Processing**: Handle high-resolution mice properly
3. **Conditional Scroll Usage**: Only process scroll when appropriate (data[0] == 1)
4. **Backward Compatibility**: Still support older 4-byte format mice

### Raw USB Data Analysis Techniques

**Essential Debugging Tools**:
```c
// Comprehensive hex dump with ASCII
for (int i = 0; i < length; i += 8) {
    ESP_LOGD(TAG, "  [%02d-%02d]: %02x %02x %02x %02x %02x %02x %02x %02x | %c%c%c%c%c%c%c%c",
             i, i+7, data[i], data[i+1], ..., ascii[i], ascii[i+1], ...);
}

// Bit-by-bit analysis
ESP_LOGD(TAG, "  data[%d] = 0x%02x = %s%s%s%s%s%s%s%s", i, data[i],
         (data[i] & 0x80) ? "1" : "0", (data[i] & 0x40) ? "1" : "0", ...);
```

**Analysis Process**:
1. **Capture Everything**: Log all bytes, all formats, all interpretations
2. **Pattern Recognition**: Look for consistent data positions across actions
3. **Interactive Testing**: Press specific buttons/scroll while logging
4. **Format Hypothesis**: Test different interpretations until patterns emerge

### Scroll Data Interpretation

**Critical Finding**: Scroll data usage depends on report type:
- **data[0] == 1**: Pure movement/scroll reports → use scroll from data[6]  
- **data[0] != 1**: Hybrid movement reports → ignore scroll (comes via keyboard interface)

**Scroll Value Mapping**:
```c
if (data[6] == 0x01) scroll_y = 1;   // Scroll up
if (data[6] == 0xFF) scroll_y = -1;  // Scroll down  
else scroll_y = 0;                   // No scroll
```

### Mouse Type Detection and Compatibility

**Two Mouse Archetypes Identified**:

1. **Pure 8-Byte Format Mouse** (e.g., newer/gaming mice):
   - Single mouse interface
   - All data (buttons, movement, scroll) in mouse reports
   - data[0] = 1, data[1] = buttons, data[6] = scroll

2. **Hybrid Keyboard+Mouse Interface Mouse** (e.g., older composite mice):
   - Mouse interface: movement only (data[0] != 1)
   - Keyboard interface: buttons and scroll via key codes
   - Requires button overlay system

**Universal Compatibility Strategy**:
- Parse all mice as 8-byte format
- Use button overlay for all mice (pure mice just have overlay = 0)
- Conditional scroll processing based on data[0]
- Movement always from 16-bit fields with 8-bit clamping

### Performance Considerations for Multiple Format Support

**Efficient Processing**:
1. **Single Code Path**: Unified 8-byte parsing reduces branching
2. **Minimal Overhead**: Button overlay adds only one variable
3. **Format Detection**: Length-based detection is O(1)
4. **Future Proof**: Can easily add new format support

### Debugging Production Issues

**Log Level Management**:
- **DEBUG**: Raw data dumps, bit analysis, comprehensive parsing info
- **INFO**: Final BLE output, significant state changes, connection events  
- **WARN/ERROR**: Actual problems requiring attention

**State Visibility**:
```c
ESP_LOGD(TAG, "Final BLE output: USB_buttons=0x%02x -> BLE_buttons=0x%02x, delta=(%d,%d), scroll=%d",
         actual_buttons, ble_buttons, delta_x, delta_y, scroll_y);
```

## Key Architectural Lessons

### Multi-Format Device Support

1. **Expect Format Diversity**: Different manufacturers use different report formats
2. **Build Flexible Parsers**: Length-based format detection with unified processing
3. **Handle Interface Splitting**: Modern devices split functionality across interfaces
4. **Persistent State Management**: Critical for hybrid devices

### Debug Infrastructure

1. **Conditional Compilation**: Essential for testing without affecting production
2. **Comprehensive Logging**: Raw data analysis reveals format patterns
3. **Centralized Debug Control**: Single header for all debug configuration
4. **Production Debug Hooks**: Leave hooks for field debugging

### Interface Bridging

1. **State Persistence**: Some device types require bridging data across interfaces
2. **Overlay Systems**: Combine data from multiple sources seamlessly
3. **Interface-Agnostic Processing**: Design for various input combinations
4. **Type Detection**: Determine behavior from data patterns, not device descriptors

## UART Communication Protocol for Dual ESP32-S3 Boards

### Auto-Response System Architecture

**Problem**: Need reliable communication between two ESP32-S3 boards for USB input device sharing.

**Solution**: Frame-based UART protocol with automatic response capability for connectivity testing.

### Physical Layer Implementation

**GPIO Configuration**:
```c
#define UART_TX_PIN             GPIO_NUM_12             // UART1 TX pin
#define UART_RX_PIN             GPIO_NUM_13             // UART1 RX pin
#define UART_BAUD_RATE          921600                  // High-speed communication
```

**Physical Wiring**:
```
Board A (ESP32-S3 #1)         Board B (ESP32-S3 #2)
---------------------         ---------------------
GPIO12 (TX) ----------*---*---------- GPIO13 (RX)
GPIO13 (RX) ----------*---*---------- GPIO12 (TX)
GND        ----------*---*---------- GND
```

### Frame Protocol with CRC16 Validation

**Frame Structure**:
```c
typedef struct {
    uint8_t  start_byte;    // Frame start indicator (0xAA)
    uint8_t  command;       // Command identifier
    uint16_t length;        // Payload length (little-endian)
    uint8_t* payload;       // Payload data
    uint16_t crc;           // CRC16 for error detection (little-endian)
} uart_frame_t;
```

**Serialized Frame Layout**: `[0xAA][cmd][len_low][len_high][payload...][crc_low][crc_high]`

### Command Protocol Design

**Test Commands**:
```c
#define UART_CMD_TEST_LOOPBACK  0xAB    // Loopback test command
#define UART_CMD_TEST_RESPONSE  0xAC    // Automatic response to test command
#define UART_CMD_PING           0xA0    // Neighbor ping command
#define UART_CMD_PONG           0xA1    // Neighbor pong response
```

**USB HID Forwarding Commands**:
```c
#define UART_CMD_USB_MOUSE      0x10    // USB mouse HID report
#define UART_CMD_USB_KEYBOARD   0x11    // USB keyboard HID report
```

### Auto-Response System Benefits

**Connectivity Testing Without Physical Loopback**:
```c
// Board A sends test frame
uart_protocol_forward_test_frame() → UART TX

// Board B's listener automatically responds  
UART RX → Background listener task → Auto-response → UART TX

// Board A receives response, confirms connectivity
UART RX → "Neighbor connectivity PASSED"
```

**Key Features**:
1. **Background Listener Task**: Continuously monitors UART for incoming frames
2. **Automatic Responses**: Test commands trigger immediate responses
3. **Fire-and-Forget**: No blocking waits, works with single board or dual board
4. **Timing Considerations**: 5ms delay prevents transmission collisions

### USB HID Report Forwarding Architecture

**Design Philosophy**: Raw movement data forwarding with individual BLE accumulation per board.

**Keyboard Data Format**:
```c
// Forward raw USB HID report (8 bytes)
uint8_t keyboard_report[8] = {modifier, reserved, keys[0], keys[1], keys[2], keys[3], keys[4], keys[5]};
uart_protocol_forward_keyboard_report(keyboard_report, 8);
```

**Mouse Data Format**:
```c
// Forward processed movement data (4 bytes)
uint8_t mouse_report[4] = {
    (uint8_t)delta_x,      // Already processed and clamped to ±127
    (uint8_t)delta_y,      // Already processed and clamped to ±127
    ble_buttons,           // Already mapped to BLE button format
    (uint8_t)scroll_y      // Already processed scroll data
};
uart_protocol_forward_mouse_report(mouse_report, 4);
```

### Data Flow Architecture

**Complete System Flow**:
```
USB Device → USB Host Processing → Local BLE + UART Forward → Neighbor Board
                                 ↓                         ↓
Local Host ←  Local BLE     UART Listener ← Neighbor BLE ← Neighbor Host
```

**Benefits of This Architecture**:

1. **Individual BLE Optimization**: Each board's BLE accumulator adapts to its own host connection characteristics
2. **High-Frequency Support**: 921600 baud easily handles raw mouse movement at USB polling rates
3. **USB Complexity Handled Once**: Input board does format parsing, output board gets clean data
4. **No Accumulation Conflicts**: Each board accumulates independently based on its BLE stack conditions

### UART Bandwidth Analysis

**Mouse Movement**: ~10 reports/frame/second average, 4 bytes + 6-byte frame overhead = ~100 bytes/second
**Keyboard**: ~5 reports/frame/second average, 8 bytes + 6-byte frame overhead = ~70 bytes/second  
**Total Bandwidth**: ~170 bytes/second out of 115,200 bytes/second available (0.15% utilization)

**Conclusion**: UART bandwidth is not a limiting factor; BLE throttling provides natural flow control.

### Fire-and-Forget Design Benefits

**Single Board Operation**:
- No blocking waits for neighbor responses
- UART reports "not initialized" but continues normally
- All USB input still processed through local BLE

**Dual Board Operation**:
- Seamless USB input sharing between both hosts
- Independent BLE accumulation optimizes for each host connection
- Robust frame validation ensures data integrity

### Integration Points

**USB Host Integration** (`usb_host.c`):
```c
// Added after successful BLE report send
esp_err_t uart_ret = uart_protocol_forward_keyboard_report(keyboard_report, 8);
esp_err_t uart_ret = uart_protocol_forward_mouse_report(mouse_report, 4);
```

**UART Listener Integration** (`uart_protocol.c`):
```c
// Background task automatically handles incoming USB reports
if (frame.command == UART_CMD_USB_MOUSE) {
    ble_hid_mouse_report_enhanced(delta_x, delta_y, ble_buttons, scroll_y);
} else if (frame.command == UART_CMD_USB_KEYBOARD) {
    ble_hid_keyboard_report(modifier, reserved, keys);
}
```

### Component Dependencies

**Build System Integration**:
```cmake
# uart_protocol component needs ble_hid for forwarding received reports
REQUIRES ble_hid

# usb_host component needs uart_protocol for forwarding outgoing reports  
REQUIRES uart_protocol
```

### Code Quality Practices

**GPIO Pin Abstraction**:
```c
// Good: Uses defines for maintainability
ESP_LOGI(TAG, "GPIO%d <-> GPIO%d", UART_TX_PIN, UART_RX_PIN);

// Bad: Hardcoded pin numbers  
ESP_LOGI(TAG, "GPIO12 <-> GPIO13");
```

**Error Handling Strategy**:
- **DEBUG Level**: UART forwarding failures (non-critical)
- **WARN Level**: Frame parsing errors (data integrity issues)
- **ERROR Level**: UART initialization failures (system issues)

### Key Architectural Lessons

#### Dual-Board Communication

1. **Auto-Response for Testing**: Essential for validating connectivity without requiring physical loopback
2. **Fire-and-Forget Protocol**: Prevents blocking when only one board is present
3. **Frame-Based Reliability**: CRC validation ensures data integrity over UART
4. **Background Processing**: Listener task handles incoming data without blocking main flow

#### Data Format Strategy

1. **Keyboard**: Raw USB HID format preserves compatibility and requires no processing on receiving board
2. **Mouse**: Processed movement data avoids duplicating complex USB parsing logic on receiving board  
3. **Individual Accumulation**: Let each board's BLE stack optimize independently rather than central accumulation
4. **High-Frequency Support**: UART bandwidth sufficient for raw USB polling rates

#### Component Architecture

1. **Layered Integration**: Clear separation between USB processing, UART forwarding, and BLE output
2. **Dependency Management**: Proper CMake dependencies ensure build system reliability
3. **Error Isolation**: UART communication failures don't affect local USB-to-BLE functionality
4. **Maintainable Configuration**: Use defines for pin assignments and protocol constants

**Bottom Line**: The dual ESP32-S3 UART communication system provides reliable, high-performance USB input device sharing with robust error handling and graceful single-board operation.

## Component Architecture Decision: Hotkey Detection Placement

### The Question: New Component vs. Existing Component Integration

**Problem**: Where to implement Ctrl+Ctrl hotkey detection for host switching?

**Options Considered**:
1. **New hotkey_switch component**: Separate abstraction layer for hotkey detection
2. **Direct integration**: Add hotkey logic to existing usb_host component

### Decision: Direct Integration in USB Host Component

**Reasoning**:

**Against New Component**:
- **Overkill for single use case**: Only detecting one specific hotkey pattern (Ctrl+Ctrl)
- **Tight coupling**: Hotkey detection is inherently tied to USB keyboard data processing
- **Simple logic**: Timing + key state tracking doesn't warrant component abstraction
- **Build complexity**: Additional CMakeLists.txt, dependencies, and interface definitions
- **Performance overhead**: Function calls across component boundaries for hot-path code

**For Direct Integration**:
- ✅ **Co-location principle**: Hotkey logic lives where keyboard data naturally flows
- ✅ **Simplicity**: Leverage existing keyboard callback infrastructure  
- ✅ **Performance**: No cross-component function calls in USB event processing
- ✅ **Implementation speed**: Build on established USB host patterns
- ✅ **Maintenance**: Single location for all keyboard-related logic

### Component Design Guidelines Learned

**Create separate components when**:
- Logic is **reusable** across multiple contexts
- Clear **abstraction boundaries** exist
- **Well-defined interfaces** can be established
- **Independent testing** is beneficial

**Integrate directly when**:
- Logic is **tightly coupled** to existing data flow
- **Single use case** with no reusability requirements
- **Simple functionality** that doesn't warrant abstraction overhead
- **Performance-critical** hot-path processing

### Implementation Strategy

**Hotkey Detection in USB Host**:
```c
// In usb_host.c - add state tracking
static struct {
    uint64_t last_left_ctrl_time;
    bool left_ctrl_pressed;
    bool hotkey_detected;
} hotkey_state = {0};

// In keyboard callback - detect Ctrl+Ctrl pattern
static void detect_ctrl_hotkey(uint8_t modifier, const uint8_t* keys) {
    // Left Ctrl detection and timing logic
    // Trigger host switching when pattern detected
}
```

**Benefits Realized**:
- **Immediate access** to keyboard modifier and key data
- **Natural integration** with existing change detection logic
- **Simple state management** using static variables
- **Direct triggering** of UART coordination and LED feedback

### Key Architectural Lesson

**Favor simplicity over abstraction** when building application-specific functionality. Components should solve **real architectural problems**, not create artificial boundaries that add complexity without benefit.

The hotkey detection represents a **feature enhancement** to USB keyboard processing, not a **separate concern** requiring component isolation.

## BLE HID Output Report Implementation for LED Control

### Keyboard LED State Management Challenge

**Problem**: Need to support keyboard LED control (CapsLock, NumLock, ScrollLock) from BLE host operating systems while coordinating LED state between dual ESP32-S3 boards sharing a single USB keyboard.

**Solution**: Implement BLE HID output report handling with host-specific LED state caching and intelligent forwarding to USB keyboard based on active host.

### BLE HID Output Report Architecture

**Key Design Principles**:
1. **Host-Specific State Caching**: Each board tracks its own local BLE host LED state
2. **Active Host LED Forwarding**: Only the currently active host's LED commands affect the USB keyboard
3. **LED State Synchronization**: Apply cached LED state when switching between hosts
4. **Deduplication Logic**: Avoid redundant USB commands when LED state unchanged

**BLE Output Report Handler Implementation**:
```c
// In ble_hid_keyboard.c - local LED state tracking
static uint8_t local_ble_led_state = 0;

static int ble_hid_keyboard_output_report_handler(struct ble_gatt_access_ctxt *ctxt)
{
    if (ctxt->om->om_len != 1) {
        ESP_LOGW(TAG, "Invalid LED output report length: %d", ctxt->om->om_len);
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    
    uint8_t led_state = ctxt->om->om_data[0];
    local_ble_led_state = led_state;  // Cache the BLE host LED state
    
    ESP_LOGI(TAG, "BLE HID output report received: 0x%02x (NumLock: %s, CapsLock: %s, ScrollLock: %s)",
             led_state,
             (led_state & 0x01) ? "ON" : "OFF",
             (led_state & 0x02) ? "ON" : "OFF", 
             (led_state & 0x04) ? "ON" : "OFF");
    
    // Only forward to USB if this board is currently active for input routing
    bool is_input_active = uart_protocol_is_input_active();
    if (is_input_active) {
        ESP_LOGI(TAG, "Board is active - forwarding LED state to USB keyboard");
        esp_err_t ret = usb_host_send_keyboard_output_report(led_state);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to send LED state to USB keyboard: %s", esp_err_to_name(ret));
        }
    } else {
        ESP_LOGI(TAG, "Board is inactive - LED state cached but not forwarded to USB");
    }
    
    return 0;
}
```

### Host Switching LED State Application

**Challenge**: When switching between hosts, the USB keyboard LEDs should reflect the cached LED state of the newly active host.

**Solution**: Apply local BLE LED state to USB keyboard when becoming active host:

```c
// In host_switching.c - apply cached LED state when becoming active
if (current_host == 0) {
    ESP_LOGI(TAG, "Becoming active host - applying local BLE LED state to USB keyboard");
    
    esp_err_t usb_err = ble_hid_keyboard_apply_local_led_state_to_usb();
    if (usb_err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to apply local LED state to USB: %s", esp_err_to_name(usb_err));
    }
    
    // Forward LED state to neighbor for coordination
    uint8_t led_state = ble_hid_keyboard_get_local_led_state();
    esp_err_t uart_err = uart_protocol_forward_led_state(led_state);
}
```

### USB Control Transfer Robustness

**Problem**: USB control transfers for LED output reports frequently fail with `ESP_ERR_NOT_FINISHED` during busy USB conditions or concurrent operations.

**Solution**: Implement retry logic with exponential backoff for USB LED commands:

```c
// In usb_host.c - robust USB LED state setting with retry logic
static esp_err_t usb_keyboard_set_led_state(uint8_t led_state)
{
    const int MAX_RETRIES = 3;
    const int RETRY_DELAY_MS = 10;
    
    for (int attempt = 0; attempt < MAX_RETRIES; attempt++) {
        esp_err_t ret = hid_class_request_set_report(
            hid_device.handle,
            HID_REPORT_TYPE_OUTPUT,
            0,
            &led_state,
            sizeof(led_state)
        );
        
        if (ret == ESP_OK) {
            usb_led_state.current_led_state = led_state;
            usb_led_state.led_state_known = true;
            return ESP_OK;
        } else if (ret == ESP_ERR_NOT_FINISHED && attempt < MAX_RETRIES - 1) {
            ESP_LOGD(TAG, "USB busy (attempt %d/%d), retrying LED command in %dms", 
                     attempt + 1, MAX_RETRIES, RETRY_DELAY_MS);
            vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
        } else {
            ESP_LOGE(TAG, "Failed to send LED output report (attempt %d/%d): %s", 
                     attempt + 1, MAX_RETRIES, esp_err_to_name(ret));
        }
    }
    
    return ESP_FAIL;
}
```

### LED State Masking During CapsLock Blink

**Problem**: Host switching triggers a CapsLock LED blink for visual feedback, but concurrent LED state changes from host switching commands could interfere with the blink.

**Solution**: Implement LED state masking during blink timer to prevent interference:

```c
// In usb_host.c - LED state masking during blink
static bool blink_timer_pending = false;

static esp_err_t usb_keyboard_set_led_state(uint8_t led_state)
{
    // Block LED state changes during blink to avoid interference
    if (blink_timer_pending) {
        ESP_LOGI(TAG, "Blink timer pending - deferring LED state change to 0x%02x", led_state);
        // Update our tracking but don't send to hardware
        usb_led_state.current_led_state = led_state;
        usb_led_state.led_state_known = true;
        return ESP_OK;
    }
    
    // ... proceed with USB command ...
}

// Force-restore LED state after blink completes
static void led_restore_timer_callback(void* arg)
{
    // Determine correct LED state based on active host
    uint8_t current_state = uart_protocol_is_input_active() ? 
        ble_hid_keyboard_get_local_led_state() : usb_led_state.current_led_state;
    
    // Force-send LED state bypassing deduplication to ensure hardware matches
    ESP_LOGI(TAG, "Force-sending LED state to hardware (bypassing deduplication): 0x%02x", current_state);
    hid_class_request_set_report(hid_device.handle, HID_REPORT_TYPE_OUTPUT, 0, &current_state, sizeof(current_state));
    
    blink_timer_pending = false;  // Unblock LED state changes
}
```

### Robust Hotkey Detection Enhancement

**Problem**: Ctrl+C was sometimes triggering the hotkey sequence instead of pure Ctrl+Ctrl, causing unintended host switching.

**Solution**: Enhanced hotkey validation to check for other keys and modifiers:

```c
// In usb_host.c - robust hotkey detection with validation
static bool detect_ctrl_hotkey(uint8_t modifier, const uint8_t keys[6])
{
    bool left_ctrl_pressed = (modifier & 0x01) != 0;  // Left Ctrl is bit 0
    
    // Check for other modifier keys (Shift, Alt, Right Ctrl, etc.)
    bool other_modifiers_pressed = (modifier & 0xFE) != 0;
    
    // Check if any non-Ctrl keys are pressed
    bool other_keys_pressed = false;
    for (int i = 0; i < 6; i++) {
        if (keys[i] != 0) {
            other_keys_pressed = true;
            break;
        }
    }
    
    // Invalidate sequence if other keys/modifiers detected during Ctrl sequence
    if ((other_keys_pressed || other_modifiers_pressed) && hotkey_state.waiting_for_second_ctrl) {
        ESP_LOGD(TAG, "Other keys/modifiers pressed during Ctrl sequence - invalidating hotkey detection");
        hotkey_state.waiting_for_second_ctrl = false;
        return false;
    }
    
    // ... continue with Ctrl+Ctrl timing validation ...
}
```

### Architecture Insights from LED Implementation

**State Synchronization Challenges**:
1. **Multiple State Tracking Systems**: BLE output handler, host switching module, UART protocol, and USB host component all needed to coordinate LED state
2. **Deduplication Complexity**: Preventing redundant USB commands while ensuring state accuracy required careful tracking
3. **Timing Conflicts**: LED blink functionality could be interfered with by concurrent host switching LED commands

**Solutions Implemented**:
1. **Single Source of Truth**: Use `uart_protocol_is_input_active()` consistently across all modules for input routing decisions
2. **State Variable Consolidation**: Centralize USB LED state tracking in usb_host component
3. **Force-Restoration**: After timing conflicts, unconditionally restore LED state to hardware

### BLE HID Output Report Specification Compliance

**GATT Characteristic Configuration**:
```c
// HID Keyboard Output Report characteristic for LED control
{
    .uuid = BLE_UUID16_DECLARE(BLE_GATT_CHR_CHARACTERISTIC),
    .access_cb = ble_hid_keyboard_access_cb,
    .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
    .val_handle = &ble_hid_svc_char_handles[HANDLE_KEYBOARD_OUTPUT_REPORT],
},
```

**HID Report Descriptor Integration**:
```c
// Output report in combined mouse+keyboard HID descriptor
0x95, 0x05,        //     Report Count (5) - 5 LEDs: NumLock, CapsLock, ScrollLock, Compose, Kana
0x75, 0x01,        //     Report Size (1) - 1 bit per LED
0x05, 0x08,        //     Usage Page (LEDs)
0x19, 0x01,        //     Usage Minimum (Num Lock)
0x29, 0x05,        //     Usage Maximum (Kana)
0x91, 0x02,        //     Output (Data,Var,Abs) - LED state output report
```

### Key Learnings from LED State Management

**Component Communication Patterns**:
1. **Cross-Component Function Calls**: BLE → USB host for LED forwarding required proper dependency management in CMakeLists.txt
2. **State Sharing**: Multiple components needed access to input routing state for coordination
3. **Event Coordination**: LED blink, host switching, and BLE output reports needed careful orchestration

**USB HID Control Transfer Reliability**:
1. **Retry Logic Essential**: USB control transfers can fail transiently, requiring retry mechanisms
2. **Error Code Specificity**: `ESP_ERR_NOT_FINISHED` indicates busy conditions, not permanent failures
3. **Timing Considerations**: Small delays (10ms) between retries often resolve busy conditions

**LED State Consistency**:
1. **Force-Restoration Required**: After timing conflicts, hardware state may not match software tracking
2. **Deduplication vs. Correctness**: Balance avoiding redundant commands with ensuring state accuracy
3. **Multiple State Sources**: BLE hosts, UART neighbors, and local tracking all contribute to final LED state

### Build System Dependencies for Cross-Component Integration

**Component Dependency Updates Required**:
```cmake
# ble_hid component CMakeLists.txt
REQUIRES 
    uart_protocol  # For input routing state checks
    usb_host      # For LED forwarding to USB keyboard

# uart_protocol component CMakeLists.txt  
REQUIRES
    ble_hid       # For LED state forwarding commands

# usb_host component CMakeLists.txt
REQUIRES
    ble_hid       # For applying local LED state
    kvml          # For host switching integration
```

**Key Architectural Lessons**:

1. **LED State Management**: Host-specific caching with active host forwarding provides seamless LED control across dual boards
2. **USB Robustness**: Retry logic with exponential backoff essential for reliable USB control transfers
3. **State Masking**: Prevent interference between LED blink visual feedback and host switching LED commands  
4. **Hotkey Validation**: Robust key and modifier validation prevents false hotkey triggers
5. **Component Integration**: Cross-component LED state coordination requires careful dependency management and consistent state variables

**Bottom Line**: BLE HID output report implementation provides complete keyboard LED control functionality with robust error handling, state synchronization, and visual feedback integration.

---

*These learnings represent real debugging sessions and working solutions for ESP32-S3 BLE HID implementation with Ubuntu hosts.*