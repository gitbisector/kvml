#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "device_discovery.h"

static const char *TAG = "DEVICE_DISCOVERY";
static TaskHandle_t discovery_task_handle = NULL;
static EventGroupHandle_t discovery_event_group = NULL;

// Event bits
#define DISCOVERY_SCAN_BIT BIT0
#define DISCOVERY_STOP_BIT BIT1

// Discovery task function
static void device_discovery_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Device discovery task started");
    
    while (true) {
        // Wait for scan bit to be set
        EventBits_t bits = xEventGroupWaitBits(
            discovery_event_group,
            DISCOVERY_SCAN_BIT | DISCOVERY_STOP_BIT,
            pdFALSE,  // Don't clear bits on exit
            pdFALSE,  // Wait for any bit, not all
            portMAX_DELAY);
            
        // Check if stop bit is set
        if (bits & DISCOVERY_STOP_BIT) {
            ESP_LOGI(TAG, "Device discovery task stopping");
            break;
        }
        
        // Check if scan bit is set
        if (bits & DISCOVERY_SCAN_BIT) {
            ESP_LOGI(TAG, "Scanning for devices...");
            
            // Perform device scanning
            // This would include scanning for USB and BLE devices
            
            // Clear scan bit when done
            xEventGroupClearBits(discovery_event_group, DISCOVERY_SCAN_BIT);
        }
        
        // Sleep for a short time
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Clean up and exit task
    discovery_task_handle = NULL;
    vTaskDelete(NULL);
}

esp_err_t device_discovery_init(void)
{
    ESP_LOGI(TAG, "Initializing device discovery module");
    
    // Create event group
    discovery_event_group = xEventGroupCreate();
    if (discovery_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create discovery event group");
        return ESP_FAIL;
    }
    
    // Create discovery task
    BaseType_t task_created = xTaskCreate(
        device_discovery_task,
        "device_discovery",
        4096,
        NULL,
        5,
        &discovery_task_handle);
        
    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create device discovery task");
        vEventGroupDelete(discovery_event_group);
        discovery_event_group = NULL;
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t device_discovery_start_scan(void)
{
    ESP_LOGI(TAG, "Starting device scan");
    
    if (discovery_event_group == NULL) {
        ESP_LOGE(TAG, "Device discovery not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Set scan bit
    xEventGroupSetBits(discovery_event_group, DISCOVERY_SCAN_BIT);
    
    return ESP_OK;
}

esp_err_t device_discovery_stop_scan(void)
{
    ESP_LOGI(TAG, "Stopping device scan");
    
    if (discovery_event_group == NULL) {
        ESP_LOGE(TAG, "Device discovery not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Set stop bit to terminate task
    xEventGroupSetBits(discovery_event_group, DISCOVERY_STOP_BIT);
    
    // Wait for task to exit
    while (discovery_task_handle != NULL) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Clean up
    vEventGroupDelete(discovery_event_group);
    discovery_event_group = NULL;
    
    return ESP_OK;
}
