#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the device discovery module
 * 
 * @return ESP_OK if successful, otherwise an error code
 */
esp_err_t device_discovery_init(void);

/**
 * @brief Start device scanning
 * 
 * @return ESP_OK if successful, otherwise an error code
 */
esp_err_t device_discovery_start_scan(void);

/**
 * @brief Stop device scanning
 * 
 * @return ESP_OK if successful, otherwise an error code
 */
esp_err_t device_discovery_stop_scan(void);

#ifdef __cplusplus
}
#endif
