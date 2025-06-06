#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize BLE keyboard HID functionality
 *
 * @return ESP_OK if successful, otherwise an error code
 */
esp_err_t ble_hid_keyboard_init(void);

#ifdef __cplusplus
}
#endif
