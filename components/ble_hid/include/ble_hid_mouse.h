#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_HID_MAX_CONN 2
#define BLE_HID_MOUSE_REPORT_LEN 5

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

#ifdef __cplusplus
}
#endif
