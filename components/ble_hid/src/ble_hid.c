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

static const char *TAG = "BLE_HID";

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
    ESP_LOGI(TAG, "Initializing BLE HID component");

    // NimBLE GATT caching is disabled in menuconfig to avoid type conflicts
    ESP_LOGI(TAG, "Using NimBLE with GATT caching disabled");

    // Initialize NimBLE controller
    ESP_LOGI(TAG, "Initializing NimBLE controller...");
    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init NimBLE port: %s", esp_err_to_name(err));
        return err;
    }

    // Initialize NimBLE host stack in a separate task
    ESP_LOGI(TAG, "Initializing NimBLE host stack...");
    nimble_port_freertos_init(ble_host_task);

    // Register essential BLE services
    ble_svc_gap_init();
    ble_svc_gatt_init();

    // Initialize common HID device functionality
    ESP_LOGI(TAG, "Initializing common HID device functionality...");
    err = ble_hid_device_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init BLE HID device: %s", esp_err_to_name(err));
        return err;
    }

    // Initialize the BLE HID mouse functionality
    ESP_LOGI(TAG, "Initializing BLE HID Mouse...");
    err = ble_hid_mouse_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init BLE HID Mouse: %s", esp_err_to_name(err));
        return err;
    }

    // Start advertising
    ble_hid_device_start_advertising();

    ESP_LOGI(TAG, "BLE HID component initialized successfully");
    return ESP_OK;
}
