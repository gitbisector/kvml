#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "usb/hid_host.h"
#include "usb/hid_usage_keyboard.h"
#include "usb/hid_usage_mouse.h"
#include "usb_host.h"

static const char *TAG = "USB_HOST";

/**
 * @brief USB Host Event Types
 */
typedef enum {
    USB_HOST_DEVICE_EVENT    // Device connection/disconnection event
} usb_host_event_type_t;

/**
 * @brief USB Host Event Data
 */
typedef union {
    struct {
        hid_host_device_handle_t handle;   // HID device handle
        hid_host_driver_event_t event;     // HID device event
    } device;
} usb_host_event_data_t;

/**
 * @brief USB Host Event
 */
typedef struct {
    usb_host_event_type_t type;    // Event type
    usb_host_event_data_t data;    // Event data
} usb_host_event_t;

/**
 * @brief HID Protocol string names
 */
static const char *hid_proto_name_str[] = {
    "NONE",
    "KEYBOARD",
    "MOUSE"
};

// Event queue and HID device callback reference
static QueueHandle_t usb_host_event_queue = NULL;
static TaskHandle_t usb_host_task_handle = NULL;
static bool usb_host_initialized = false;

// Structure to hold HID device context
typedef struct {
    bool connected;
    hid_host_device_handle_t handle;
    hid_host_dev_params_t params;
} hid_device_state_t;

static hid_device_state_t hid_device = {0};

/**
 * @brief APP event group
 *
 * Application logic can be different. There is a one among other ways to distinguish the
 * event by application event group.
 */
typedef enum {
    USB_HOST_EVENT_QUIT = 0,    // Request to quit the USB host task
    USB_HOST_EVENT_DEVICE,       // HID device event
} usb_host_event_group_t;

/**
 * @brief USB Host event queue
 *
 * This event is used for delivering events to the main task.
 */
typedef struct {
    usb_host_event_group_t event_group;
    union {
        // HID Host - Device related info
        struct {
            hid_host_device_handle_t handle;
            hid_host_driver_event_t event;
        } hid_device;
    } data;
} usb_host_event_queue_t;

// Mouse position tracking
static int mouse_x = 0;
static int mouse_y = 0;
static bool mouse_left_button = false;
static bool mouse_right_button = false;
static bool mouse_middle_button = false;

/**
 * @brief USB HID Host Keyboard Interface report callback handler
 *
 * Process received keyboard reports and print keyboard actions
 *
 * @param[in] data    Pointer to input report data buffer
 * @param[in] length  Length of input report data buffer
 */
static void hid_host_keyboard_report_callback(const uint8_t *const data, const int length)
{
    if (length < 2) {
        return;  // Invalid report
    }

    ESP_LOGD(TAG, "HID Keyboard report: modifier=0x%02x, keys=%02x,%02x,%02x,%02x,%02x,%02x",
        data[0], data[2], data[3], data[4], data[5], data[6], data[7]);

    // Process the keyboard report - here you can implement your keyboard handling logic
    uint8_t modifier = data[0];

    // Example: detect if any key is pressed
    for (int i = 2; i < 8; i++) {
        if (data[i] != 0) {
            ESP_LOGI(TAG, "Key pressed: 0x%02x, modifier: 0x%02x", data[i], modifier);
            // Here you would add your keyboard handling logic
            break;
        }
    }
}

/**
 * @brief USB HID Host Mouse Interface report callback handler
 *
 * Process received mouse reports and update mouse state
 *
 * @param[in] data    Pointer to input report data buffer
 * @param[in] length  Length of input report data buffer
 */
static void hid_host_mouse_report_callback(const uint8_t *const data, const int length)
{
    if (length < 5) {  // We need at least 5 bytes for this mouse
        ESP_LOGI(TAG, "Mouse report too short: %d bytes", length);
        return;  // Invalid report
    }

    // Log raw report bytes for debugging
    ESP_LOGI(TAG, "Raw mouse report data[0-2]: %02x %02x %02x", data[0], data[1], data[2]);
    ESP_LOGI(TAG, "Extra mouse report data[3+]: %02x %02x", data[3], data[4]);

    // Update mouse button states
    mouse_left_button = (data[0] & 0x01) ? true : false;
    mouse_right_button = (data[0] & 0x02) ? true : false;
    mouse_middle_button = (data[0] & 0x04) ? true : false;

    // Based on user testing with specific movements, the correct mapping is:
    // - data[0] = button states
    // - data[2] = X movement (Left = 0xFF, Right = 0x01)
    // - data[4] = Y movement (Up = 0xFF, Down = 0x01)

    // Extract X movement from data[2] (left/right)
    int8_t delta_x = (int8_t)data[2];

    // Extract Y movement from data[4] (up/down)
    int8_t delta_y = (int8_t)data[4];

    // Update virtual mouse position
    mouse_x += delta_x;
    mouse_y += delta_y;

    // Bound the position to avoid overflow (adjust limits as needed)
    if (mouse_x < 0) mouse_x = 0;
    if (mouse_y < 0) mouse_y = 0;
    if (mouse_x > 1000) mouse_x = 1000;
    if (mouse_y > 1000) mouse_y = 1000;

    ESP_LOGI(TAG, "Mouse: pos=(%d,%d) delta=(%d,%d) buttons: L:%d R:%d M:%d",
        mouse_x, mouse_y, delta_x, delta_y,
        mouse_left_button, mouse_right_button, mouse_middle_button);

    // Here you would implement your mouse handling logic
}

/**
 * @brief USB HID Host Generic Interface report callback handler
 *
 * For generic devices, just print the report data
 *
 * @param[in] data    Pointer to input report data buffer
 * @param[in] length  Length of input report data buffer
 */
static void hid_host_generic_report_callback(const uint8_t *const data, const int length)
{
    // Log the raw report data for debugging
    ESP_LOGD(TAG, "Generic HID report received, length: %d bytes", length);
    // You could add your own processing for specific generic HID devices here
}

/**
 * @brief USB HID Host Interface callback
 *
 * Handles interface events and routes reports to appropriate callbacks
 *
 * @param[in] hid_device_handle  HID Device handle
 * @param[in] event              HID Host interface event
 * @param[in] arg                Pointer to arguments, not used
 */
static void hid_host_interface_callback(hid_host_device_handle_t hid_device_handle,
                                       const hid_host_interface_event_t event,
                                       void *arg)
{
    uint8_t data[64] = { 0 };
    size_t data_length = 0;
    hid_host_dev_params_t dev_params;
    esp_err_t ret = hid_host_device_get_params(hid_device_handle, &dev_params);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get device params: %s", esp_err_to_name(ret));
        return;
    }

    switch (event) {
    case HID_HOST_INTERFACE_EVENT_INPUT_REPORT:
        // Get the input report data directly from the device
        ret = hid_host_device_get_raw_input_report_data(hid_device_handle,
                                                       data,
                                                       sizeof(data),
                                                       &data_length);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to get input report data: %s", esp_err_to_name(ret));
            break;
        }

        // Call the appropriate callback based on the device protocol
        if (HID_SUBCLASS_BOOT_INTERFACE == dev_params.sub_class) {
            if (HID_PROTOCOL_KEYBOARD == dev_params.proto) {
                hid_host_keyboard_report_callback(data, data_length);
            } else if (HID_PROTOCOL_MOUSE == dev_params.proto) {
                hid_host_mouse_report_callback(data, data_length);
            }
        } else {
            hid_host_generic_report_callback(data, data_length);
        }
        break;

    case HID_HOST_INTERFACE_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "HID Device, protocol '%s' DISCONNECTED",
                 hid_proto_name_str[dev_params.proto]);
        ESP_ERROR_CHECK(hid_host_device_close(hid_device_handle));
        break;

    case HID_HOST_INTERFACE_EVENT_TRANSFER_ERROR:
        ESP_LOGI(TAG, "HID Device, protocol '%s' TRANSFER_ERROR",
                 hid_proto_name_str[dev_params.proto]);
        break;

    default:
        ESP_LOGE(TAG, "HID Device, protocol '%s' Unhandled event",
                 hid_proto_name_str[dev_params.proto]);
        break;
    }

    // Update device context
    hid_device.handle = hid_device_handle;
    hid_device.params = dev_params;
    if (event != HID_HOST_INTERFACE_EVENT_DISCONNECTED) {
        hid_device.connected = true;
    }
}

/**
 * @brief HID Host Device callback
 *
 * Puts new HID Device event to the queue
 *
 * @param[in] hid_device_handle HID Device handle
 * @param[in] event             HID Device event
 * @param[in] arg               Not used
 */
static void hid_host_device_callback(hid_host_device_handle_t hid_device_handle,
                                      const hid_host_driver_event_t event,
                                      void *arg)
{
    if (hid_device_handle == NULL || !usb_host_event_queue) {
        return;
    }
    // Only queue device events, not interface events
    usb_host_event_queue_t evt = {
        .event_group = USB_HOST_EVENT_DEVICE,
        .data.hid_device = {
            .handle = hid_device_handle,
            .event = event
        }
    };
    // Send event to queue
    if (xQueueSend(usb_host_event_queue, &evt, pdMS_TO_TICKS(100)) != pdPASS) {
        ESP_LOGE(TAG, "Failed to send HID device event to queue");
    }
}

/**
 * @brief Handle a connected HID device
 *
 * Configures and starts a connected HID device
 *
 * @param[in] device_handle Handle to the connected HID device
 */
static void handle_connected_device(hid_host_device_handle_t device_handle)
{
    hid_host_dev_params_t dev_params;
    ESP_ERROR_CHECK(hid_host_device_get_params(device_handle, &dev_params));

    ESP_LOGI(TAG, "HID Device connected, protocol: %s",
             hid_proto_name_str[dev_params.proto]);

    // Configure the HID device
    const hid_host_device_config_t hid_config = {
        .callback = hid_host_interface_callback,
        .callback_arg = NULL
    };

    // Open the HID device
    ESP_LOGI(TAG, "Opening HID device");
    esp_err_t err = hid_host_device_open(device_handle, &hid_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open HID device: %s", esp_err_to_name(err));
        return;
    }

    // Set the HID protocol to report mode
    ESP_LOGI(TAG, "Setting HID protocol to report mode");
    err = hid_class_request_set_protocol(device_handle, HID_REPORT_PROTOCOL_BOOT);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set HID protocol to report mode: %s", esp_err_to_name(err));
        // Continue anyway as this is not critical
    }

    // Set the HID idle rate to 0 (infinite)
    ESP_LOGI(TAG, "Setting HID idle rate");
    err = hid_class_request_set_idle(device_handle, 0, 0);
    if (err != ESP_OK && err != ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "Failed to set HID idle rate: %s", esp_err_to_name(err));
        // Continue anyway as this is not critical
    }

    // Start the HID device
    ESP_LOGI(TAG, "Starting HID device");
    err = hid_host_device_start(device_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HID device: %s", esp_err_to_name(err));
        hid_host_device_close(device_handle);
        return;
    }

    // Store the device handle
    hid_device.handle = device_handle;
    hid_device.connected = true;

    ESP_LOGI(TAG, "HID device connected and ready");
}

/**
 * @brief Handle a disconnected HID device
 */
static void handle_disconnected_device(hid_host_device_handle_t device_handle)
{
    // Close the device
    ESP_ERROR_CHECK(hid_host_device_close(device_handle));

    // Update device state
    hid_device.connected = false;
    ESP_LOGI(TAG, "HID Device disconnected");
}

/**
 * @brief USB HID Host Device event
 *
 * @param[in] hid_device_handle  HID Device handle
 * @param[in] event              HID Host Device event
 * @param[in] arg                Pointer to arguments, not used
 */
static void hid_host_device_event(hid_host_device_handle_t hid_device_handle,
                                  const hid_host_driver_event_t event,
                                  void *arg)
{
    if (hid_device_handle == NULL) {
        return;
    }

    hid_host_dev_params_t dev_params;
    esp_err_t ret = hid_host_device_get_params(hid_device_handle, &dev_params);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get device params: %s", esp_err_to_name(ret));
        return;
    }

    switch (event) {
    case HID_HOST_DRIVER_EVENT_CONNECTED:
        ESP_LOGI(TAG, "HID Device, protocol '%s' CONNECTED",
                 hid_proto_name_str[dev_params.proto]);

        // Configure the device with the interface callback
        const hid_host_device_config_t dev_config = {
            .callback = hid_host_interface_callback,
            .callback_arg = NULL
        };

        // Open the device
        ret = hid_host_device_open(hid_device_handle, &dev_config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to open HID device: %s", esp_err_to_name(ret));
            return;
        }

        // Just skip the protocol and idle set steps
        // These are causing control transfer errors but the device works anyway

        // Start the device directly
        ret = hid_host_device_start(hid_device_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start HID device: %s", esp_err_to_name(ret));
            hid_host_device_close(hid_device_handle);
            return;
        }

        // Store the device handle
        hid_device.handle = hid_device_handle;
        hid_device.connected = true;
        memcpy(&hid_device.params, &dev_params, sizeof(hid_host_dev_params_t));
        ESP_LOGI(TAG, "HID device connected and ready");
        break;

    default:
        break;
    }
}

/**
 * @brief USB host task that handles USB events
 *
 * @param[in] arg Not used
 */
static void usb_host_task(void *arg)
{
    // Timeout for USB host library events handling
    const TickType_t event_timeout = 50 / portTICK_PERIOD_MS;

    while (1) {
        // Handle USB host library events
        uint32_t event_flags;
        esp_err_t ret = usb_host_lib_handle_events(event_timeout, &event_flags);

        if (ret == ESP_OK) {
            // Process USB host events
            if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
                ESP_LOGI(TAG, "No more clients");
            }
            if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
                ESP_LOGI(TAG, "All resources free");

                // If all devices are disconnected, we can shutdown the USB host
                // if needed, or just wait for new devices
                // usb_host_uninstall();
                // break;
            }
        } else if (ret != ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "USB host lib handle events failed: %s", esp_err_to_name(ret));
            // Add a delay to prevent busy loop on error
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Handle queued HID events
        usb_host_event_queue_t event;
        while (xQueueReceive(usb_host_event_queue, &event, 0) == pdTRUE) {
            switch (event.event_group) {
                case USB_HOST_EVENT_DEVICE:
                    // Handle HID device events (connect/disconnect)
                    hid_host_device_event(event.data.hid_device.handle,
                                         event.data.hid_device.event,
                                         NULL);
                    break;

                case USB_HOST_EVENT_QUIT:
                    // Exit the task
                    ESP_LOGI(TAG, "USB host task exiting");
                    vTaskDelete(NULL);
                    return;

                default:
                    ESP_LOGE(TAG, "Unknown event group: %d", event.event_group);
                    break;
            }
        }

        // Add a small delay to prevent CPU spinning
        vTaskDelay(1);
    }
}

esp_err_t usb_host_init(void)
{
    if (usb_host_initialized) {
        ESP_LOGW(TAG, "USB host already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing USB host");

    // Create event queue
    usb_host_event_queue = xQueueCreate(10, sizeof(usb_host_event_queue_t));
    if (usb_host_event_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create USB host event queue");
        return ESP_ERR_NO_MEM;
    }

    // Initialize the USB Host Library
    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };

    esp_err_t ret = usb_host_install(&host_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install USB host: %s", esp_err_to_name(ret));
        vQueueDelete(usb_host_event_queue);
        usb_host_event_queue = NULL;
        return ret;
    }

    // Create the USB host task
    BaseType_t task_ret = xTaskCreate(
        usb_host_task,
        "usb_host",
        4096,
        NULL,
        5, // Priority
        &usb_host_task_handle
    );

    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create USB host task");
        usb_host_uninstall();
        vQueueDelete(usb_host_event_queue);
        usb_host_event_queue = NULL;
        return ESP_FAIL;
    }

    // Install HID Host driver
    const hid_host_driver_config_t hid_host_driver_config = {
        .create_background_task = true,  // Let the HID driver handle its own task
        .task_priority = 5,
        .stack_size = 4096,
        .core_id = 0,
        .callback = hid_host_device_callback,
        .callback_arg = NULL
    };

    ret = hid_host_install(&hid_host_driver_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install HID host driver: %s", esp_err_to_name(ret));
        // Signal the task to exit
        usb_host_event_queue_t evt = {
            .event_group = USB_HOST_EVENT_QUIT
        };
        xQueueSend(usb_host_event_queue, &evt, portMAX_DELAY);

        // Wait for task to exit
        vTaskDelay(100 / portTICK_PERIOD_MS);

        // Clean up
        usb_host_uninstall();
        vQueueDelete(usb_host_event_queue);
        usb_host_event_queue = NULL;
        return ret;
    }

    // Initialize device state
    hid_device.connected = false;
    hid_device.handle = NULL;

    ESP_LOGI(TAG, "USB host initialized successfully");
    usb_host_initialized = true;
    return ESP_OK;
}

esp_err_t usb_host_deinit(void)
{
    if (!usb_host_initialized) {
        ESP_LOGW(TAG, "USB host not initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Deinitializing USB host");

    // Clean up HID host driver
    esp_err_t ret = hid_host_uninstall();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to uninstall HID host driver: %s", esp_err_to_name(ret));
        return ret;
    }

    // Signal the USB host task to exit
    if (usb_host_task_handle != NULL) {
        // Send quit event to the task
        usb_host_event_queue_t evt = {
            .event_group = USB_HOST_EVENT_QUIT
        };
        xQueueSend(usb_host_event_queue, &evt, portMAX_DELAY);

        // Wait for task to exit (with timeout)
        vTaskDelay(100 / portTICK_PERIOD_MS);

        // If task is still running, delete it
        if (eTaskGetState(usb_host_task_handle) != eDeleted) {
            vTaskDelete(usb_host_task_handle);
        }
        usb_host_task_handle = NULL;
    }

    // Delete the queue if it exists
    if (usb_host_event_queue != NULL) {
        vQueueDelete(usb_host_event_queue);
        usb_host_event_queue = NULL;
    }

    ESP_LOGI(TAG, "USB host deinitialized");
    return ESP_OK;
}

bool usb_host_is_initialized(void)
{
    return usb_host_initialized;
}
