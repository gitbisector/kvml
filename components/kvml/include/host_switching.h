#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the host switching module
 * 
 * @return ESP_OK if successful, otherwise an error code
 */
esp_err_t host_switching_init(void);

/**
 * @brief Switch active host to the specified target
 * 
 * @param target_host Host ID (0 or 1)
 * @return ESP_OK if successful, otherwise an error code
 */
esp_err_t host_switching_select(uint8_t target_host);

/**
 * @brief Get the current active host
 * 
 * @return Current host ID (0 or 1)
 */
uint8_t host_switching_get_current(void);

#ifdef __cplusplus
}
#endif
