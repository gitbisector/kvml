#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "config_manager.h"

static const char *TAG = "CONFIG_MANAGER";
static const char *NVS_NAMESPACE = "kvml_config";

esp_err_t config_manager_init(void)
{
    ESP_LOGI(TAG, "Initializing configuration manager");
    return ESP_OK;
}

esp_err_t config_manager_save(void)
{
    ESP_LOGI(TAG, "Saving configuration to NVS");
    nvs_handle_t nvs_handle;
    esp_err_t err;

    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS namespace: %s", esp_err_to_name(err));
        return err;
    }

    // Save configuration here

    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error committing NVS: %s", esp_err_to_name(err));
    }

    nvs_close(nvs_handle);
    return err;
}

esp_err_t config_manager_load(void)
{
    ESP_LOGI(TAG, "Loading configuration from NVS");
    nvs_handle_t nvs_handle;
    esp_err_t err;

    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGW(TAG, "Configuration not found in NVS, using defaults");
            return ESP_OK;
        }
        ESP_LOGE(TAG, "Error opening NVS namespace: %s", esp_err_to_name(err));
        return err;
    }

    // Load configuration here

    nvs_close(nvs_handle);
    return ESP_OK;
}
