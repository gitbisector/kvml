#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize error handling system
 * 
 * @return ESP_OK if successful, otherwise an error code
 */
esp_err_t error_handling_init(void);

/**
 * @brief Register and handle a system error
 * 
 * @param error ESP error code
 * @param file Source file where error occurred
 * @param line Line number where error occurred
 * @param function Function where error occurred
 * @param description Additional error description
 * 
 * @return ESP_OK if successfully handled, otherwise the original error code
 */
esp_err_t handle_error(esp_err_t error, const char *file, int line, 
                       const char *function, const char *description);

// Convenience macro for error handling
#define HANDLE_ERROR(err, desc) \
    handle_error(err, __FILE__, __LINE__, __FUNCTION__, desc)

#ifdef __cplusplus
}
#endif
