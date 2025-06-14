#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ble_hid.h"
#include "ble_hid_device.h"
#include "ble_hid_mouse.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "esp_bt.h"

static const char *TAG = "BLE_HID";
static bool ble_hid_initialized = false;

/**
 * The NimBLE host task.
 *
 * This function is executed by the FreeRTOS scheduler when NimBLE host events
 * are detected. It runs the NimBLE host in a task context.
 *
 * @param param Unused parameter required by the FreeRTOS task definition.
 */
static void ble_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE Host Task Started");

    // This function will return only when nimble_port_stop() is executed
    nimble_port_run();

    // If we get here, the NimBLE host has been stopped
    ESP_LOGI(TAG, "BLE Host Task Stopped");
    nimble_port_freertos_deinit();
}

esp_err_t ble_hid_init(void)
{
    if (ble_hid_initialized) {
        ESP_LOGW(TAG, "BLE HID already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing BLE HID component");

    // Check if BT controller is already enabled
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) {
        ESP_LOGI(TAG, "BT controller already enabled");
    } else {
        ESP_LOGI(TAG, "BT controller not enabled yet");
    }

    // NimBLE GATT caching is disabled in menuconfig to avoid type conflicts
    ESP_LOGI(TAG, "Using NimBLE with GATT caching disabled");

    // Initialize NimBLE controller
    ESP_LOGI(TAG, "Initializing NimBLE controller...");
    esp_err_t err = nimble_port_init();
    if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "NimBLE controller already initialized, continuing...");
        err = ESP_OK;
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init NimBLE port: %s", esp_err_to_name(err));
        return err;
    }

    // Initialize common HID device functionality BEFORE starting NimBLE host
    // This ensures sync callback is registered before the host starts
    ESP_LOGI(TAG, "Initializing common HID device functionality...");
    err = ble_hid_device_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init BLE HID device: %s", esp_err_to_name(err));
        return err;
    }

    // Initialize NimBLE host stack in a separate task AFTER setting up callbacks
    ESP_LOGI(TAG, "Initializing NimBLE host stack...");
    nimble_port_freertos_init(ble_host_task);

    // Note: BLE HID Mouse service registration will be done in the sync callback
    // when the NimBLE stack is fully ready

    ble_hid_initialized = true;
    ESP_LOGI(TAG, "BLE HID component initialized successfully");
    return ESP_OK;
}
