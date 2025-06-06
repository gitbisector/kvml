#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "error_handling.h"

static const char *TAG = "ERROR_HANDLER";

esp_err_t error_handling_init(void)
{
    ESP_LOGI(TAG, "Initializing error handling system");
    return ESP_OK;
}

esp_err_t handle_error(esp_err_t error, const char *file, int line, 
                       const char *function, const char *description)
{
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "[%s:%d] %s: %s - %s", 
                 file, line, function, 
                 esp_err_to_name(error), description);
    }
    return error;
}
