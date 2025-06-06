#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "kvml_core.h"
#include "host_switching.h"
#include "device_discovery.h"

static const char *TAG = "KVML_CORE";
static TaskHandle_t kvml_task_handle = NULL;

// KVML core task function
static void kvml_core_task(void *pvParameters)
{
    ESP_LOGI(TAG, "KVML core task started");
    
    while (true) {
        // Main processing loop
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

esp_err_t kvml_core_init(void)
{
    ESP_LOGI(TAG, "Initializing KVML core");
    
    // Initialize host switching module
    esp_err_t ret = host_switching_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize host switching: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize device discovery module
    ret = device_discovery_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize device discovery: %s", esp_err_to_name(ret));
        return ret;
    }
    
    return ESP_OK;
}

esp_err_t kvml_core_start(void)
{
    ESP_LOGI(TAG, "Starting KVML core");
    
    // Create KVML core task
    BaseType_t task_created = xTaskCreate(
        kvml_core_task,
        "kvml_core_task",
        4096,
        NULL,
        5,
        &kvml_task_handle);
        
    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create KVML core task");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t kvml_core_stop(void)
{
    ESP_LOGI(TAG, "Stopping KVML core");
    
    if (kvml_task_handle != NULL) {
        vTaskDelete(kvml_task_handle);
        kvml_task_handle = NULL;
    }
    
    return ESP_OK;
}
