#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize USB HID host functionality
 *
 * @return ESP_OK if successful, otherwise an error code
 */
esp_err_t usb_hid_host_init(void);

#ifdef __cplusplus
}
#endif
