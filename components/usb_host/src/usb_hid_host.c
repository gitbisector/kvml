#include <stdio.h>
#include "esp_log.h"
#include "usb_hid_host.h"

static const char *TAG = "USB_HID_HOST";

esp_err_t usb_hid_host_init(void)
{
    ESP_LOGI(TAG, "Initializing USB HID host");
    return ESP_OK;
}
