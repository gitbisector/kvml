#include <stdio.h>
#include "esp_log.h"
#include "ble_hid_keyboard.h"

static const char *TAG = "BLE_HID_KB";

esp_err_t ble_hid_keyboard_init(void)
{
    ESP_LOGI(TAG, "Initializing BLE keyboard HID");
    return ESP_OK;
}
