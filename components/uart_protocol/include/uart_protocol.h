#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize UART protocol
 *
 * @return ESP_OK if successful, otherwise an error code
 */
esp_err_t uart_protocol_init(void);

#ifdef __cplusplus
}
#endif
