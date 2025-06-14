#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

// Include component headers
#include "common_utils.h"
#include "config_manager.h"
#include "error_handling.h"
#include "ble_hid.h"
#include "ble_hid_mouse.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "usb_host.h"
#include "uart_protocol.h"
#include "kvml_core.h"

static const char *TAG = "KVML_MAIN";

/**
 * @brief Initialize NVS (Non-Volatile Storage)
 */
static esp_err_t init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    
    return ret;
}

/**
 * @brief Initialize all required subsystems
 */

static esp_err_t init_subsystems(void)
{
    esp_err_t ret;
    
    ESP_LOGI(TAG, "4.1 Initializing common utilities...");
    common_init();
    ESP_LOGI(TAG, "Common utilities initialized");
    
    ESP_LOGI(TAG, "4.2 Initializing error handling...");
    ret = error_handling_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize error handling: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Error handling initialized");
    
    ESP_LOGI(TAG, "4.3 Initializing configuration manager...");
    ret = config_manager_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize configuration manager: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Configuration manager initialized");

    ESP_LOGI(TAG, "4.4 Loading configuration...");
    ret = config_manager_load();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to load configuration, using defaults: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Configuration loaded successfully");
    }
    
    ESP_LOGI(TAG, "4.5 Initializing USB host...");
    ret = usb_host_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize USB host: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "USB host initialized");
    
    ESP_LOGI(TAG, "4.6 Initializing UART protocol...");
    ret = uart_protocol_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize UART protocol: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "UART protocol initialized");
    
    ESP_LOGI(TAG, "4.6.1 Starting UART background listener for auto-response...");
    ret = uart_protocol_start_listener();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start UART listener: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "UART listener started - auto-response enabled for connectivity testing");
    
    ESP_LOGI(TAG, "4.6.2 Testing UART frame protocol with CRC (GPIO%d <-> GPIO%d)...", UART_TX_PIN, UART_RX_PIN);
    ret = uart_protocol_test_loopback();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "UART frame protocol test PASSED - framing and CRC working correctly");
    } else if (ret == ESP_ERR_NOT_FOUND) {
        ESP_LOGW(TAG, "UART frame test: no data received (check GPIO%d-GPIO%d connection)", UART_TX_PIN, UART_RX_PIN);
    } else {
        ESP_LOGE(TAG, "UART frame protocol test FAILED: %s", esp_err_to_name(ret));
    }
    
    ESP_LOGI(TAG, "4.7 Initializing KVML core...");
    ret = kvml_core_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize KVML core: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "KVML core initialized");
    
    ESP_LOGI(TAG, "4.8 Initializing BLE HID (final step)...");
    ret = ble_hid_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BLE HID: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "BLE HID initialized");
    
    ESP_LOGI(TAG, "All subsystems initialized successfully");
    return ESP_OK;
}

/**
 * @brief Start all subsystems
 */
static esp_err_t start_subsystems(void)
{
    esp_err_t ret;
    
    ESP_LOGI(TAG, "5.1 Starting KVML core...");
    ret = kvml_core_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start KVML core: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "KVML core started successfully");
    
    // Add any additional subsystem starts here with similar logging
    
    // All subsystems started successfully
    
    return ESP_OK;
}

/**
 * @brief Main application entry point
 */
void app_main(void)
{
    ESP_LOGI(TAG, "app_main() started");
    
    // Initialize NVS
    ESP_LOGI(TAG, "1. Initializing NVS...");
    esp_err_t ret = init_nvs();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS initialization failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "NVS initialized successfully");
    
    // Initialize all subsystems
    ESP_LOGI(TAG, "2. Initializing subsystems...");
    ret = init_subsystems();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Subsystem initialization failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "All subsystems initialized successfully");
    
    // Start all subsystems
    ESP_LOGI(TAG, "3. Starting subsystems...");
    ret = start_subsystems();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start subsystems: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "All subsystems started successfully");
    
    ESP_LOGI(TAG, "System initialization complete");

    // Main application loop
    while (1) {
        ESP_LOGI(TAG, "KVML application running...");
        vTaskDelay(pdMS_TO_TICKS(10000)); // Log status every 10 seconds
    }
}
