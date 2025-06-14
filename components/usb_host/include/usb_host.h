#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "usb/usb_host.h"

#ifdef __cplusplus
extern "C" {
#endif

// Task configuration
#define USB_HOST_TASK_STACK_SIZE (8192)
#define USB_HOST_TASK_PRIORITY (5)
#define USB_EVENT_TASK_STACK_SIZE (4096)
#define USB_EVENT_TASK_PRIORITY (5)

/**
 * @brief Initialize USB host functionality
 *
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t usb_host_init(void);

/**
 * @brief Deinitialize USB host and free resources
 *
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t usb_host_deinit(void);

/**
 * @brief Check if USB host is initialized
 *
 * @return true if initialized, false otherwise
 */
bool usb_host_is_initialized(void);

/**
 * @brief Send LED output report to connected USB keyboard
 *
 * @param led_state LED state byte (bit 0=NumLock, bit 1=CapsLock, bit 2=ScrollLock)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t usb_host_send_keyboard_output_report(uint8_t led_state);

#ifdef __cplusplus
}
#endif
