#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the configuration manager
 * 
 * @return ESP_OK if successful, otherwise an error code
 */
esp_err_t config_manager_init(void);

/**
 * @brief Save configuration to NVS
 * 
 * @return ESP_OK if successful, otherwise an error code
 */
esp_err_t config_manager_save(void);

/**
 * @brief Load configuration from NVS
 * 
 * @return ESP_OK if successful, otherwise an error code
 */
esp_err_t config_manager_load(void);

#ifdef __cplusplus
}
#endif
