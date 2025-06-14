#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"
#include "ble_hid_debug.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_HID_KEYBOARD_REPORT_LEN 8

// Keyboard modifier bits
#define HID_KEYBOARD_MODIFIER_LEFT_CTRL   0x01
#define HID_KEYBOARD_MODIFIER_LEFT_SHIFT  0x02
#define HID_KEYBOARD_MODIFIER_LEFT_ALT    0x04
#define HID_KEYBOARD_MODIFIER_LEFT_GUI    0x08
#define HID_KEYBOARD_MODIFIER_RIGHT_CTRL  0x10
#define HID_KEYBOARD_MODIFIER_RIGHT_SHIFT 0x20
#define HID_KEYBOARD_MODIFIER_RIGHT_ALT   0x40
#define HID_KEYBOARD_MODIFIER_RIGHT_GUI   0x80

// Keyboard HID usage codes for common keys
#define HID_KEY_A 0x04
#define HID_KEY_B 0x05
#define HID_KEY_C 0x06
#define HID_KEY_D 0x07
#define HID_KEY_E 0x08
#define HID_KEY_F 0x09
#define HID_KEY_G 0x0A
#define HID_KEY_H 0x0B
#define HID_KEY_I 0x0C
#define HID_KEY_J 0x0D
#define HID_KEY_K 0x0E
#define HID_KEY_L 0x0F
#define HID_KEY_M 0x10
#define HID_KEY_N 0x11
#define HID_KEY_O 0x12
#define HID_KEY_P 0x13
#define HID_KEY_Q 0x14
#define HID_KEY_R 0x15
#define HID_KEY_S 0x16
#define HID_KEY_T 0x17
#define HID_KEY_U 0x18
#define HID_KEY_V 0x19
#define HID_KEY_W 0x1A
#define HID_KEY_X 0x1B
#define HID_KEY_Y 0x1C
#define HID_KEY_Z 0x1D

#define HID_KEY_1 0x1E
#define HID_KEY_2 0x1F
#define HID_KEY_3 0x20
#define HID_KEY_4 0x21
#define HID_KEY_5 0x22
#define HID_KEY_6 0x23
#define HID_KEY_7 0x24
#define HID_KEY_8 0x25
#define HID_KEY_9 0x26
#define HID_KEY_0 0x27

#define HID_KEY_ENTER     0x28
#define HID_KEY_ESCAPE    0x29
#define HID_KEY_BACKSPACE 0x2A
#define HID_KEY_TAB       0x2B
#define HID_KEY_SPACE     0x2C
#define HID_KEY_CAPS_LOCK 0x39

#define HID_KEY_F1  0x3A
#define HID_KEY_F2  0x3B
#define HID_KEY_F3  0x3C
#define HID_KEY_F4  0x3D
#define HID_KEY_F5  0x3E
#define HID_KEY_F6  0x3F
#define HID_KEY_F7  0x40
#define HID_KEY_F8  0x41
#define HID_KEY_F9  0x42
#define HID_KEY_F10 0x43
#define HID_KEY_F11 0x44
#define HID_KEY_F12 0x45

#define HID_KEY_PRINT_SCREEN 0x46
#define HID_KEY_SCROLL_LOCK  0x47
#define HID_KEY_PAUSE        0x48
#define HID_KEY_INSERT       0x49
#define HID_KEY_HOME         0x4A
#define HID_KEY_PAGE_UP      0x4B
#define HID_KEY_DELETE       0x4C
#define HID_KEY_END          0x4D
#define HID_KEY_PAGE_DOWN    0x4E
#define HID_KEY_RIGHT_ARROW  0x4F
#define HID_KEY_LEFT_ARROW   0x50
#define HID_KEY_DOWN_ARROW   0x51
#define HID_KEY_UP_ARROW     0x52

// Keyboard LED bit masks (for output reports)
#define HID_LED_NUM_LOCK     0x01
#define HID_LED_CAPS_LOCK    0x02
#define HID_LED_SCROLL_LOCK  0x04
#define HID_LED_COMPOSE      0x08
#define HID_LED_KANA         0x10

// HID Report Types
#define HID_REPORT_TYPE_INPUT   0x01
#define HID_REPORT_TYPE_OUTPUT  0x02
#define HID_REPORT_TYPE_FEATURE 0x03

// Maximum number of simultaneous BLE host connections for LED state caching
#define BLE_HID_MAX_HOSTS 2

/**
 * @brief Initialize BLE keyboard HID functionality
 *
 * @return ESP_OK if successful, otherwise an error code
 */
esp_err_t ble_hid_keyboard_init(void);

/**
 * @brief Send a keyboard HID report over BLE
 * @param conn_idx Connection index (0 to BLE_HID_MAX_CONN-1)
 * @param report Pointer to the keyboard report data (8 bytes)
 * @param len Length of the report data in bytes (should be 8)
 */
void ble_hid_keyboard_send_report(uint8_t conn_idx, const uint8_t *report, size_t len);

/**
 * @brief Press a key with optional modifiers
 * @param key_code HID key code (e.g., HID_KEY_A)
 * @param modifiers Modifier mask (e.g., HID_KEYBOARD_MODIFIER_LEFT_CTRL)
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t ble_hid_keyboard_key_press(uint8_t key_code, uint8_t modifiers);

/**
 * @brief Release a key
 * @param key_code HID key code to release
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t ble_hid_keyboard_key_release(uint8_t key_code);

/**
 * @brief Release all keys and modifiers
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t ble_hid_keyboard_release_all(void);

/**
 * @brief Send a complete keyboard report
 * @param modifiers Modifier byte
 * @param reserved Reserved byte (should be 0)
 * @param keys Array of 6 key codes
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t ble_hid_keyboard_report(uint8_t modifiers, uint8_t reserved, const uint8_t keys[6]);

/**
 * @brief Type a string of characters
 * @param text Text to type
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t ble_hid_keyboard_type_string(const char *text);

/**
 * @brief Handle BLE HID output report for keyboard LED control
 * @param conn_handle BLE connection handle identifying the host
 * @param report_data Output report data (single byte with LED bit flags)
 * @param len Length of report data (should be 1)
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t ble_hid_keyboard_handle_output_report(uint16_t conn_handle, const uint8_t *report_data, size_t len);

/**
 * @brief Get the current local BLE LED state
 * @return LED state byte with bit flags
 */
uint8_t ble_hid_keyboard_get_local_led_state(void);

/**
 * @brief Apply local BLE LED state to USB keyboard
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t ble_hid_keyboard_apply_local_led_state_to_usb(void);

#ifdef __cplusplus
}
#endif
