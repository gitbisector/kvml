#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"

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
#include "web_ui.h"
#include "kvml_core.h"
#include "wifi_config.h"

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
 * @brief Initialize WiFi in station mode
 */
static esp_err_t init_wifi(void)
{
    ESP_LOGI(TAG, "Initializing WiFi...");
    
    // Initialize TCP/IP network interface (required for WiFi)
    ESP_LOGI(TAG, "Initializing TCP/IP");
    ESP_ERROR_CHECK(esp_netif_init());
    
    // Create default event loop
    ESP_LOGI(TAG, "Creating event loop");
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // Create default WiFi STA interface
    ESP_LOGI(TAG, "Creating WiFi station");
    esp_netif_create_default_wifi_sta();
    
    // Initialize WiFi with default config
    ESP_LOGI(TAG, "Initializing WiFi with default config");
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    // Set WiFi storage to RAM (faster, but won't persist across reboots)
    ESP_LOGI(TAG, "Setting WiFi storage to RAM");
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    
    // Set WiFi mode to Station mode
    ESP_LOGI(TAG, "Setting WiFi mode to STA");
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    
    // Configure WiFi station
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .scan_method = WIFI_FAST_SCAN,  // Faster connection
            .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,  // Connect to strongest AP
            .threshold.rssi = -127,  // Minimum signal strength
            .threshold.authmode = WIFI_AUTH_OPEN,  // Most permissive auth mode
        },
    };
    
    ESP_LOGI(TAG, "Setting WiFi configuration for SSID: %s", WIFI_SSID);
    
    // Set the WiFi configuration
    esp_err_t config_ret = esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    if (config_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi configuration: %s", esp_err_to_name(config_ret));
        return config_ret;
    }
    
    // Start WiFi
    ESP_LOGI(TAG, "Starting WiFi...");
    esp_err_t start_ret = esp_wifi_start();
    if (start_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start WiFi: %s", esp_err_to_name(start_ret));
        return start_ret;
    }
    
    // Wait for WiFi to connect
    ESP_LOGI(TAG, "Connecting to WiFi...");
    
    // Simple connection wait with retries
    int retry_count = 0;
    while (retry_count < MAXIMUM_RETRY) {
        ESP_LOGI(TAG, "WiFi connection attempt %d/%d...", retry_count + 1, MAXIMUM_RETRY);
        
        esp_err_t err = esp_wifi_connect();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "esp_wifi_connect failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(1000));
            retry_count++;
            continue;
        }
        
        // Wait for IP address with a small delay
        for (int i = 0; i < 10; i++) {
            esp_netif_ip_info_t ip_info;
            esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
            
            if (netif == NULL) {
                ESP_LOGW(TAG, "Network interface not found, retrying...");
                vTaskDelay(pdMS_TO_TICKS(500));
                continue;
            }
            
            if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
                if (ip_info.ip.addr != 0) {
                    uint8_t *ip = (uint8_t *)&ip_info.ip.addr;
                    ESP_LOGI(TAG, "Connected to WiFi. IP: %d.%d.%d.%d", 
                            ip[0], ip[1], ip[2], ip[3]);
                    return ESP_OK;
                } else {
                    ESP_LOGI(TAG, "Waiting for IP address...");
                }
            } else {
                ESP_LOGW(TAG, "Failed to get IP info, retrying...");
            }
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        
        // If we get here, connection or IP assignment failed
        esp_wifi_disconnect();
        ESP_LOGW(TAG, "WiFi connection attempt %d/%d failed, retrying...", 
                retry_count + 1, MAXIMUM_RETRY);
        vTaskDelay(pdMS_TO_TICKS(1000));
        retry_count++;
    }
    
    ESP_LOGE(TAG, "Failed to connect to WiFi after %d attempts", MAXIMUM_RETRY);
    return ESP_FAIL;
}

/**
 * @brief Initialize all required subsystems
 */
static void init_spiffs(void) {
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE("SPIFFS", "Failed to mount or format filesystem (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI("SPIFFS", "SPIFFS mounted successfully");
    }
}

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
    
    ESP_LOGI(TAG, "4.5 Initializing WiFi...");
    ret = init_wifi();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize WiFi: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "WiFi initialization completed");
    
    ESP_LOGI(TAG, "4.6 Initializing BLE HID...");
    ret = ble_hid_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BLE HID: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "BLE HID initialized");
    
    ESP_LOGI(TAG, "4.7 Initializing USB host...");
    ret = usb_host_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize USB host: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "USB host initialized");
    
    ESP_LOGI(TAG, "4.8 Initializing UART protocol...");
    ret = uart_protocol_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize UART protocol: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "UART protocol initialized");
    
    ESP_LOGI(TAG, "4.9 Mounting SPIFFS filesystem...");
    init_spiffs();
    ESP_LOGI(TAG, "4.10 Initializing Web UI...");
    ret = web_ui_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Web UI: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Web UI initialized");
    
    ESP_LOGI(TAG, "4.10 Initializing KVML core...");
    ret = kvml_core_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize KVML core: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "KVML core initialized");
    
    ESP_LOGI(TAG, "All subsystems initialized successfully");
    return ESP_OK;
}

/**
 * @brief Start all subsystems
 */
static esp_err_t start_subsystems(void)
{
    esp_err_t ret;
    
    ESP_LOGI(TAG, "5.1 Starting Web UI...");
    ret = web_ui_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start Web UI: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Web UI started successfully");
    
    ESP_LOGI(TAG, "5.2 Starting KVML core...");
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

    // Initialize network interface
    ESP_LOGI(TAG, "2. Initializing network stack...");
    ret = esp_netif_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Network stack initialization failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Network stack initialized");
    
    ESP_LOGI(TAG, "3. Creating default event loop...");
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {  // Allow already created
        ESP_LOGE(TAG, "Event loop creation failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Event loop created");
    
    // Initialize all subsystems
    ESP_LOGI(TAG, "4. Initializing subsystems...");
    ret = init_subsystems();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Subsystem initialization failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "All subsystems initialized successfully");
    
    // Start all subsystems
    ESP_LOGI(TAG, "5. Starting subsystems...");
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
