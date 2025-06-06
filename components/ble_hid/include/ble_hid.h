#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the BLE HID component
 * @details This function initializes the NimBLE stack, BLE HID device functionality,
 *          and individual HID device types (mouse, keyboard, etc)
 * @return ESP_OK if successful, otherwise an error code
 */
esp_err_t ble_hid_init(void);

#ifdef __cplusplus
}
#endif
