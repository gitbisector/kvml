#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the KVML core system
 * 
 * @return ESP_OK if successful, otherwise an error code
 */
esp_err_t kvml_core_init(void);

/**
 * @brief Start the KVML system
 * 
 * @return ESP_OK if successful, otherwise an error code
 */
esp_err_t kvml_core_start(void);

/**
 * @brief Stop the KVML system
 * 
 * @return ESP_OK if successful, otherwise an error code
 */
esp_err_t kvml_core_stop(void);

#ifdef __cplusplus
}
#endif
