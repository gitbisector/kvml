#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"
#include "ble_hid_debug.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_HID_MOUSE_REPORT_LEN 5

// Handle enumeration for combined mouse+keyboard service - Ubuntu compatibility
enum ble_hid_attr_handles {
    // HID Service handles (shared)
    HANDLE_HID_REPORT_MAP,              // 0 - Combined mouse+keyboard report map
    HANDLE_MOUSE_INPUT_REPORT,          // 1 - Mouse input report
    HANDLE_KEYBOARD_INPUT_REPORT,       // 2 - Keyboard input report
    HANDLE_HID_PROTOCOL_MODE,           // 3 - HID protocol mode
    HANDLE_HID_CONTROL_POINT,           // 4 - HID control point
    HANDLE_HID_INFO,                    // 5 - HID info
    HANDLE_BOOT_MOUSE_INPUT_REPORT,     // 6 - Boot Protocol mouse input
    HANDLE_BOOT_KEYBOARD_INPUT_REPORT,  // 7 - Boot Protocol keyboard input

    // Battery Service handles
    HANDLE_BATTERY_LEVEL,               // 8 - Battery level

    // Device Information Service handles
    HANDLE_DIS_MANUFACTURER_NAME,       // 9 - Manufacturer name
    HANDLE_DIS_MODEL_NUMBER,            // 10 - Model number
    HANDLE_DIS_PNP_ID,                  // 11 - PnP ID

    HANDLE_KEYBOARD_OUTPUT_REPORT,      // 12 - Keyboard output report (LED control)
    
    HANDLE_CONSUMER_INPUT_REPORT,       // 13 - Consumer control input report

    HANDLE_BLE_HID_COUNT                // 14 - Total handle count
};

// Global handle array - must be externally accessible for NimBLE
extern uint16_t ble_hid_svc_char_handles[];

/**
 * @brief Initialize the BLE HID mouse functionality
 * @return ESP_OK if successful, otherwise an error code
 */
esp_err_t ble_hid_mouse_init(void);

/**
 * @brief Send a mouse HID report over BLE
 * @param conn_idx Connection index (0 to BLE_HID_MAX_CONN-1)
 * @param report Pointer to the mouse report data
 * @param len Length of the report data in bytes
 */
void ble_hid_mouse_send_report(uint8_t conn_idx, const uint8_t *report, size_t len);

/**
 * @brief Send a keyboard HID report to a connected host
 * @param conn_idx Connection index (0 or 1)
 * @param report Pointer to keyboard report (8 bytes)
 * @param len Length of report (should be 8)
 */
void ble_hid_keyboard_send_report(uint8_t conn_idx, const uint8_t *report, size_t len);

// Mouse button definitions
// Protocol mode definitions
#define PROTOCOL_MODE_BOOT   0x00
#define PROTOCOL_MODE_REPORT 0x01

// Mouse button definitions
#define BLE_HID_MOUSE_LEFT_BUTTON   0x01
#define BLE_HID_MOUSE_RIGHT_BUTTON  0x02
#define BLE_HID_MOUSE_MIDDLE_BUTTON 0x04

/**
 * @brief Move mouse cursor relatively to current position
 * @param delta_x Horizontal movement (-127 to 127)
 * @param delta_y Vertical movement (-127 to 127)
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t ble_hid_mouse_move(int8_t delta_x, int8_t delta_y);

/**
 * @brief Send complete mouse report with movement, buttons, and scroll
 * @param delta_x Horizontal movement (-127 to 127)
 * @param delta_y Vertical movement (-127 to 127)
 * @param buttons Button mask (BLE_HID_MOUSE_*_BUTTON)
 * @param scroll_y Vertical scroll (-127 to 127, positive = up)
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t ble_hid_mouse_report(int8_t delta_x, int8_t delta_y, uint8_t buttons, int8_t scroll_y);

/**
 * @brief Enhanced mouse report function with movement accumulation and flow control
 * @param delta_x Horizontal movement (-127 to 127)
 * @param delta_y Vertical movement (-127 to 127)
 * @param buttons Button mask (BLE_HID_MOUSE_*_BUTTON)
 * @param scroll_y Vertical scroll (-127 to 127, positive = up)
 * @return ESP_OK if successful, error code otherwise
 * 
 * This function handles BLE congestion by accumulating movement when the BLE stack
 * is busy and sending accumulated movement using an adaptive timer. Use this instead
 * of ble_hid_mouse_report() for better performance with high-frequency mouse input.
 */
esp_err_t ble_hid_mouse_report_enhanced(int8_t delta_x, int8_t delta_y, uint8_t buttons, int8_t scroll_y);

/**
 * @brief Called when a BLE notification completes (internal function)
 * This function should be called from BLE event handlers when notifications complete
 */
void ble_hid_mouse_on_notify_complete(void);

/**
 * @brief Log comprehensive mouse report statistics for debugging
 * Shows sent, freed, failed, outstanding, and peak outstanding report counts
 */
void ble_hid_mouse_log_stats(void);

/**
 * @brief Get current mouse button state
 * @return Current button mask
 */
uint8_t ble_hid_mouse_get_button_state(void);

/**
 * @brief Send Boot Protocol mouse report (for Ubuntu compatibility)
 * @param conn_idx Connection index (0 to BLE_HID_MAX_CONN-1)
 * @param report Pointer to boot mouse report (4 bytes: buttons, X, Y, wheel)
 * @param len Length of the report data in bytes
 */
void ble_hid_boot_mouse_send_report(uint8_t conn_idx, const uint8_t *report, size_t len);

/**
 * @brief Send Boot Protocol keyboard report (for Ubuntu compatibility)
 * @param conn_idx Connection index (0 to BLE_HID_MAX_CONN-1)
 * @param report Pointer to boot keyboard report (8 bytes: modifier, reserved, 6 keys)
 * @param len Length of the report data in bytes
 */
void ble_hid_boot_keyboard_send_report(uint8_t conn_idx, const uint8_t *report, size_t len);

/**
 * @brief Send a consumer control report over BLE HID
 * 
 * @param usage_code The 16-bit HID consumer control usage code
 * @return ESP_OK on success or an error code
 */
esp_err_t ble_hid_consumer_control(uint16_t usage_code);

#ifdef __cplusplus
}
#endif
