#include <stdio.h>
#include "esp_log.h"
#include "uart_protocol.h"

static const char *TAG = "UART_PROTOCOL";

esp_err_t uart_protocol_init(void)
{
    ESP_LOGI(TAG, "Initializing UART protocol");
    return ESP_OK;
}
