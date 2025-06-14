#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "host/ble_hs.h"
#include "host/ble_gatt.h"
#include "ble_hid_keyboard.h"
#include "ble_hid_device.h"
#include "ble_hid_mouse.h"  // For handle access
#include "usb_host.h"        // For USB LED forwarding
#include "uart_protocol.h"   // For neighbor LED forwarding and checking if this board is active

static const char *TAG = "BLE_HID_KB";

// Keyboard state tracking
typedef struct {
    uint8_t modifiers;        // Current modifier state
    uint8_t keys[6];          // Currently pressed keys (up to 6)
    uint8_t key_count;        // Number of keys currently pressed
} keyboard_state_t;

static keyboard_state_t keyboard_state = {0};

// LED state cache for local BLE host only
static uint8_t local_ble_led_state = 0;

// Standard Keyboard HID Report Descriptor (TODO: implement keyboard service)
/*
static const uint8_t keyboard_hid_report_map[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
    0x09, 0x06,        // Usage (Keyboard)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x02,        //   Report ID (2)
    0x05, 0x07,        //   Usage Page (Kbrd/Keypad)
    0x19, 0xE0,        //   Usage Minimum (0xE0)
    0x29, 0xE7,        //   Usage Maximum (0xE7)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x08,        //   Report Count (8)
    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x95, 0x01,        //   Report Count (1)
    0x75, 0x08,        //   Report Size (8)
    0x81, 0x01,        //   Input (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x95, 0x06,        //   Report Count (6)
    0x75, 0x08,        //   Report Size (8)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x65,        //   Logical Maximum (101)
    0x05, 0x07,        //   Usage Page (Kbrd/Keypad)
    0x19, 0x00,        //   Usage Minimum (0x00)
    0x29, 0x65,        //   Usage Maximum (0x65)
    0x81, 0x00,        //   Input (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,              // End Collection
};
*/

esp_err_t ble_hid_keyboard_init(void)
{
    ESP_LOGI(TAG, "Initializing BLE keyboard HID");

    // Clear keyboard state
    memset(&keyboard_state, 0, sizeof(keyboard_state));

    ESP_LOGI(TAG, "BLE keyboard HID initialized successfully");
    return ESP_OK;
}

void ble_hid_keyboard_send_report(uint8_t conn_idx, const uint8_t *report, size_t len)
{
    if (!ble_hid_is_connected()) {
        ESP_LOGD(TAG, "Keyboard: Not connected");
        return;
    }

    if (len != BLE_HID_KEYBOARD_REPORT_LEN) {
        ESP_LOGE(TAG, "Invalid keyboard report length: %d (expected %d)", (int)len, BLE_HID_KEYBOARD_REPORT_LEN);
        return;
    }

    // Check if keyboard input report handle is initialized
    if (ble_hid_svc_char_handles[HANDLE_KEYBOARD_INPUT_REPORT] == 0) {
        ESP_LOGW(TAG, "Keyboard input report handle not initialized");
        return;
    }

    ESP_LOGD(TAG, "Sending keyboard report: mod=0x%02x keys=%02x,%02x,%02x,%02x,%02x,%02x",
             report[0], report[2], report[3], report[4], report[5], report[6], report[7]);

    // BLE HID: Send report data directly without Report ID
    // (GATT characteristic handle identifies the report type)
    struct os_mbuf *om = ble_hs_mbuf_from_flat(report, len);
    if (om != NULL) {
        int rc = ble_gattc_notify_custom(ble_hid_dev_state.conn_handle,
                                       ble_hid_svc_char_handles[HANDLE_KEYBOARD_INPUT_REPORT], om);
        if (rc != 0) {
            // Decode common error codes for better diagnostics
            const char *error_name = "UNKNOWN";
            switch (rc) {
                case 3: error_name = "BLE_HS_ENOTCONN"; break;
                case 8: error_name = "BLE_HS_ENOMEM"; break;
                case 30: error_name = "BLE_HS_ENOTCONNFAIL"; break;
                case 257: error_name = "BLE_HS_ATT_ERR_INVALID_HANDLE"; break;
            }
            ESP_LOGW(TAG, "Failed to send keyboard HID notification: %d (%s), handle=0x%04x",
                     rc, error_name, ble_hid_svc_char_handles[HANDLE_KEYBOARD_INPUT_REPORT]);
        } else {
            ESP_LOGD(TAG, "Sent keyboard HID report notification to conn %d", conn_idx);
        }
    } else {
        ESP_LOGE(TAG, "Failed to allocate mbuf for keyboard report");
    }
}

esp_err_t ble_hid_keyboard_key_press(uint8_t key_code, uint8_t modifiers)
{
    if (!ble_hid_is_connected()) {
        return ESP_ERR_INVALID_STATE;
    }

    // Update modifiers
    keyboard_state.modifiers = modifiers;

    // Add key to the list if not already present and there's space
    bool key_found = false;
    for (int i = 0; i < 6; i++) {
        if (keyboard_state.keys[i] == key_code) {
            key_found = true;
            break;
        }
    }

    if (!key_found && keyboard_state.key_count < 6) {
        keyboard_state.keys[keyboard_state.key_count] = key_code;
        keyboard_state.key_count++;
    }

    // Send the updated report
    return ble_hid_keyboard_report(keyboard_state.modifiers, 0, keyboard_state.keys);
}

esp_err_t ble_hid_keyboard_key_release(uint8_t key_code)
{
    if (!ble_hid_is_connected()) {
        return ESP_ERR_INVALID_STATE;
    }

    // Remove key from the list
    for (int i = 0; i < 6; i++) {
        if (keyboard_state.keys[i] == key_code) {
            // Shift remaining keys down
            for (int j = i; j < 5; j++) {
                keyboard_state.keys[j] = keyboard_state.keys[j + 1];
            }
            keyboard_state.keys[5] = 0;
            keyboard_state.key_count--;
            break;
        }
    }

    // Send the updated report
    return ble_hid_keyboard_report(keyboard_state.modifiers, 0, keyboard_state.keys);
}

esp_err_t ble_hid_keyboard_release_all(void)
{
    // Clear all state
    memset(&keyboard_state, 0, sizeof(keyboard_state));

    // Send empty report
    uint8_t empty_keys[6] = {0};
    return ble_hid_keyboard_report(0, 0, empty_keys);
}

esp_err_t ble_hid_keyboard_report(uint8_t modifiers, uint8_t reserved, const uint8_t keys[6])
{
#if BLE_HID_DEBUG_DISABLE_REPORTING
    ESP_LOGI(TAG, "[DEBUG] BLE DISABLED - Keyboard report: mod=0x%02x keys=%02x,%02x,%02x,%02x,%02x,%02x",
             modifiers, keys[0], keys[1], keys[2], keys[3], keys[4], keys[5]);
    return ESP_OK;
#else
    if (!ble_hid_is_connected()) {
        return ESP_ERR_INVALID_STATE;
    }

    // Create keyboard report: [modifiers, reserved, key1, key2, key3, key4, key5, key6]
    uint8_t report[BLE_HID_KEYBOARD_REPORT_LEN] = {
        modifiers,  // Modifier byte
        reserved,   // Reserved byte
        keys[0],    // Key 1
        keys[1],    // Key 2
        keys[2],    // Key 3
        keys[3],    // Key 4
        keys[4],    // Key 5
        keys[5]     // Key 6
    };

    ESP_LOGD(TAG, "Keyboard report: mod=0x%02x keys=%02x,%02x,%02x,%02x,%02x,%02x",
             modifiers, keys[0], keys[1], keys[2], keys[3], keys[4], keys[5]);

    // Send the keyboard report
    ble_hid_keyboard_send_report(0, report, BLE_HID_KEYBOARD_REPORT_LEN);

    return ESP_OK;
#endif
}

esp_err_t ble_hid_keyboard_type_string(const char *text)
{
    if (!text) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Typing string: %s", text);

    for (int i = 0; text[i] != '\0'; i++) {
        char c = text[i];
        uint8_t key_code = 0;
        uint8_t modifiers = 0;

        // Simple character to HID key mapping
        if (c >= 'a' && c <= 'z') {
            key_code = HID_KEY_A + (c - 'a');
        } else if (c >= 'A' && c <= 'Z') {
            key_code = HID_KEY_A + (c - 'A');
            modifiers = HID_KEYBOARD_MODIFIER_LEFT_SHIFT;
        } else if (c >= '1' && c <= '9') {
            key_code = HID_KEY_1 + (c - '1');
        } else if (c == '0') {
            key_code = HID_KEY_0;
        } else if (c == ' ') {
            key_code = HID_KEY_SPACE;
        } else if (c == '\n') {
            key_code = HID_KEY_ENTER;
        } else {
            ESP_LOGW(TAG, "Unsupported character: '%c'", c);
            continue;
        }

        // Press key
        ble_hid_keyboard_key_press(key_code, modifiers);
        vTaskDelay(pdMS_TO_TICKS(50)); // Small delay between keypresses

        // Release key
        ble_hid_keyboard_key_release(key_code);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    return ESP_OK;
}

/**
 * Get the current local BLE LED state
 */
uint8_t ble_hid_keyboard_get_local_led_state(void) {
    return local_ble_led_state;
}

esp_err_t ble_hid_keyboard_handle_output_report(uint16_t conn_handle, const uint8_t *report_data, size_t len) {
    if (!report_data || len != 1) {
        ESP_LOGE(TAG, "Invalid output report: data=%p, len=%d", report_data, (int)len);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t led_state = report_data[0];

    // Update local BLE LED state cache
    uint8_t old_state = local_ble_led_state;
    local_ble_led_state = led_state;

    ESP_LOGI(TAG, "Local BLE LED state update (conn=0x%04x): 0x%02x -> 0x%02x",
             conn_handle, old_state, led_state);
    ESP_LOGI(TAG, "  NumLock: %s, CapsLock: %s, ScrollLock: %s",
             (led_state & HID_LED_NUM_LOCK) ? "ON" : "OFF",
             (led_state & HID_LED_CAPS_LOCK) ? "ON" : "OFF",
             (led_state & HID_LED_SCROLL_LOCK) ? "ON" : "OFF");

    // If this board is currently active for input routing, apply LED state to USB keyboard immediately
    bool is_input_active = uart_protocol_is_input_active();
    if (is_input_active) {
        ESP_LOGI(TAG, "This board is active - applying LED state to USB keyboard");
        esp_err_t usb_ret = usb_host_send_keyboard_output_report(led_state);
        if (usb_ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to apply LED state to USB keyboard: %s", esp_err_to_name(usb_ret));
        } else {
            ESP_LOGI(TAG, "LED state applied to USB keyboard: 0x%02x", led_state);
        }

        // Also send to neighbor so they have our current state
        esp_err_t uart_ret = uart_protocol_forward_led_state(led_state);
        if (uart_ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to send LED state to neighbor: %s", esp_err_to_name(uart_ret));
        } else {
            ESP_LOGD(TAG, "LED state forwarded to neighbor: 0x%02x", led_state);
        }
    } else {
        ESP_LOGI(TAG, "This board is inactive (neighbor is active) - LED state only cached");
    }

    return ESP_OK;
}


esp_err_t ble_hid_keyboard_apply_local_led_state_to_usb(void) {
    ESP_LOGI(TAG, "Applying local BLE LED state to USB keyboard: 0x%02x", local_ble_led_state);
    ESP_LOGI(TAG, "  NumLock: %s, CapsLock: %s, ScrollLock: %s",
             (local_ble_led_state & HID_LED_NUM_LOCK) ? "ON" : "OFF",
             (local_ble_led_state & HID_LED_CAPS_LOCK) ? "ON" : "OFF",
             (local_ble_led_state & HID_LED_SCROLL_LOCK) ? "ON" : "OFF");

    esp_err_t ret = usb_host_send_keyboard_output_report(local_ble_led_state);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to apply LED state to USB keyboard: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "LED state applied to USB keyboard: 0x%02x", local_ble_led_state);
    }

    return ret;
}
