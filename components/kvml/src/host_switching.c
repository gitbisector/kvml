#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "host_switching.h"
#include "ble_hid_keyboard.h"  // For LED state management
#include "uart_protocol.h"     // For neighbor communication

static const char *TAG = "HOST_SWITCHING";
static const char *NVS_NAMESPACE = "kvml";
static const char *NVS_HOST_KEY = "active_host";

static uint8_t current_host = 0;
static SemaphoreHandle_t host_mutex = NULL;

esp_err_t host_switching_init(void)
{
    ESP_LOGI(TAG, "Initializing host switching module");
    
    // Create mutex for thread safety
    host_mutex = xSemaphoreCreateMutex();
    if (host_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create host switching mutex");
        return ESP_FAIL;
    }
    
    // Try to read last active host from NVS
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err == ESP_OK) {
        uint8_t saved_host = 0;
        err = nvs_get_u8(nvs_handle, NVS_HOST_KEY, &saved_host);
        
        if (err == ESP_OK) {
            current_host = (saved_host > 1) ? 0 : saved_host;
            ESP_LOGI(TAG, "Loaded active host from NVS: %d", current_host);
        } else if (err == ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGI(TAG, "No saved host found in NVS, using default: %d", current_host);
        } else {
            ESP_LOGW(TAG, "Error reading host from NVS: %s", esp_err_to_name(err));
        }
        
        nvs_close(nvs_handle);
    } else {
        ESP_LOGW(TAG, "Error opening NVS namespace: %s", esp_err_to_name(err));
    }
    
    return ESP_OK;
}

esp_err_t host_switching_select(uint8_t target_host)
{
    // Validate target host
    if (target_host > 1) {
        ESP_LOGE(TAG, "Invalid host ID: %d (must be 0 or 1)", target_host);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Take mutex
    if (xSemaphoreTake(host_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take host mutex");
        return ESP_FAIL;
    }
    
    // Check if already on target host
    if (current_host == target_host) {
        ESP_LOGI(TAG, "Already on host %d", target_host);
        xSemaphoreGive(host_mutex);
        return ESP_OK;
    }
    
    // Switch to new host
    ESP_LOGI(TAG, "Switching from host %d to host %d", current_host, target_host);
    current_host = target_host;
    
    // Save new host to NVS
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK) {
        err = nvs_set_u8(nvs_handle, NVS_HOST_KEY, current_host);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Error saving host to NVS: %s", esp_err_to_name(err));
        }
        
        err = nvs_commit(nvs_handle);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Error committing NVS: %s", esp_err_to_name(err));
        }
        
        nvs_close(nvs_handle);
    } else {
        ESP_LOGW(TAG, "Error opening NVS namespace: %s", esp_err_to_name(err));
    }
    
    // If we're becoming the active host (host 0), apply our local BLE LED state
    if (current_host == 0) {
        ESP_LOGI(TAG, "Becoming active host - applying local BLE LED state to USB keyboard");
        
        // Apply local BLE LED state to USB keyboard
        esp_err_t usb_err = ble_hid_keyboard_apply_local_led_state_to_usb();
        if (usb_err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to apply local LED state to USB: %s", esp_err_to_name(usb_err));
        }
        
        // Send local BLE LED state to neighbor
        uint8_t led_state = ble_hid_keyboard_get_local_led_state();
        esp_err_t uart_err = uart_protocol_forward_led_state(led_state);
        if (uart_err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to send LED state to neighbor: %s", esp_err_to_name(uart_err));
        } else {
            ESP_LOGI(TAG, "Sent local LED state 0x%02x to neighbor", led_state);
        }
    } else {
        ESP_LOGI(TAG, "Becoming inactive host (neighbor is active)");
    }
    
    // Release mutex
    xSemaphoreGive(host_mutex);
    
    return ESP_OK;
}

uint8_t host_switching_get_current(void)
{
    uint8_t host;
    
    // Take mutex
    if (xSemaphoreTake(host_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take host mutex");
        return 0; // Return default in case of failure
    }
    
    host = current_host;
    
    // Release mutex
    xSemaphoreGive(host_mutex);
    
    return host;
}
