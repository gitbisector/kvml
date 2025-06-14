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
#include "esp_timer.h"
#include "driver/gpio.h"
#include "usb/hid_host.h"
#include "usb/hid.h"
#include "usb/hid_usage_keyboard.h"
#include "usb/hid_usage_mouse.h"
#include "usb_host.h"
#include "ble_hid_mouse.h"
#include "ble_hid_keyboard.h"
#include "uart_protocol.h"
#include "host_switching.h"

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

// Forward declarations
static esp_err_t handle_consumer_control(uint16_t usage_code);

// External function declarations
extern bool uart_protocol_is_input_active(void);

/**
 * @brief USB enumeration filter callback to handle problematic devices
 *
 * @param[in] device_desc Device descriptor from enumeration
 * @param[out] bConfigurationValue Configuration value to use
 * @return true to accept device, false to reject
 */
static bool usb_enum_filter_cb(const usb_device_desc_t *device_desc, uint8_t *bConfigurationValue)
{
    if (device_desc == NULL) {
        ESP_LOGW(TAG, "Enumeration filter: NULL device descriptor, accepting anyway");
        if (bConfigurationValue) *bConfigurationValue = 1;  // Use default config
        return true;  // Accept unknown devices
    }

    ESP_LOGI(TAG, "Enumeration filter: VID=0x%04X, PID=0x%04X, Class=0x%02X",
             device_desc->idVendor, device_desc->idProduct, device_desc->bDeviceClass);

    // Use the first configuration (most common)
    if (bConfigurationValue) {
        *bConfigurationValue = 1;
    }

    // Accept HID devices (class 0x03) and composite devices (class 0x00)
    if (device_desc->bDeviceClass == 0x03 || device_desc->bDeviceClass == 0x00) {
        ESP_LOGI(TAG, "Accepting HID/Composite device");
        return true;
    }

    // For now, accept all other devices too for maximum compatibility
    ESP_LOGI(TAG, "Accepting device (unknown class)");
    return true;
}

// Forward declarations
static esp_err_t reinitialize_usb_host(void);
static void usb_host_task(void *arg);
static void dump_usb_device_descriptors(hid_host_device_handle_t device_handle);

/**
 * @brief Force USB bus reset and re-enumeration
 *
 * This function attempts to recover from enumeration failures by:
 * 1. Performing a USB host re-initialization
 * 2. Waiting for device re-enumeration
 * 3. Retrying device detection
 */

/**
 * @brief Background task to monitor USB enumeration and retry if needed
 */
static void usb_enumeration_monitor_task(void *arg)
{
    ESP_LOGI(TAG, "USB enumeration monitor task started");

    // Wait for initial enumeration attempt
    vTaskDelay(pdMS_TO_TICKS(3000));

    // Wait for any initial enumeration to complete
    vTaskDelay(pdMS_TO_TICKS(5000));

    ESP_LOGI(TAG, "USB enumeration monitor: Starting long-term hot-plug monitoring");

    // Continue monitoring for hot-plug events
    // For now, this task just provides long-term background monitoring
    // The immediate retry logic in usb_host_init() handles the aggressive recovery
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(30000));  // Check every 30 seconds
        // This task monitors for hot-plug events
        // Future enhancement: Could add periodic USB bus health checks here
    }
}

// Structure to hold HID device context
typedef struct {
    bool connected;
    hid_host_device_handle_t handle;
    hid_host_dev_params_t params;
} hid_device_state_t;

static hid_device_state_t hid_device = {0};

/**
 * @brief HID report content types
 */
typedef enum {
    HID_REPORT_TYPE_KEYBOARD,       // Real keyboard input
    HID_REPORT_TYPE_MOUSE_BUTTONS,  // Mouse button events via keyboard interface
    HID_REPORT_TYPE_MOUSE_SCROLL,   // Mouse scroll events via keyboard interface
    HID_REPORT_TYPE_EMPTY           // Empty/no-op report
} hid_report_content_type_t;


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

// Button overlay for hybrid mice that report buttons via keyboard interface
static uint8_t button_overlay = 0;  // Current button state from keyboard interface

// Keyboard state tracking to avoid duplicate reports
typedef struct {
    uint8_t last_modifier;
    uint8_t last_keys[6];
} keyboard_state_t;

static keyboard_state_t last_keyboard_state = {0};

// Hotkey detection constants and state
#define LEFT_CTRL_KEY_CODE      0xE0        // Left Ctrl modifier bit
#define HOTKEY_TIMEOUT_US       500000      // 500ms timeout between Ctrl presses
#define CAPSLOCK_BLINK_DURATION_MS  200     // CapsLock LED blink duration

typedef struct {
    uint64_t last_left_ctrl_press_time;     // Timestamp of last left Ctrl press
    bool waiting_for_second_ctrl;           // Waiting for second Ctrl in sequence
    bool hotkey_detected;                   // Hotkey sequence completed
    uint8_t current_led_state;              // Current LED state byte (bit 1 = CapsLock)
    bool led_state_known;                   // Whether we know the current LED state
    esp_timer_handle_t led_restore_timer;   // Timer for LED restore after blink
    uint8_t led_restore_state;              // State to restore LED to
} hotkey_state_t;

// USB keyboard LED state tracking (single source of truth)
typedef struct {
    uint8_t current_led_state;              // Current USB keyboard LED state
    bool led_state_known;                   // Whether we know the current USB LED state
} usb_led_state_t;

static hotkey_state_t hotkey_state = {0};
static usb_led_state_t usb_led_state = {0};

// Blink state tracking
static bool blink_timer_pending = false;

// Forward declarations
static esp_err_t usb_keyboard_set_led_state(uint8_t led_state);
static void led_restore_timer_callback(void* arg);

/**
 * @brief Timer callback to restore LED state after blink
 */
static void led_restore_timer_callback(void* arg)
{
    ESP_LOGI(TAG, "LED restore timer triggered - restoring to current LED state");

    // Determine the correct LED state based on which host is currently active
    uint8_t current_state = 0x00;
    bool is_input_active = uart_protocol_is_input_active();

    if (is_input_active) {
        // This board is active - use local BLE LED state
        current_state = ble_hid_keyboard_get_local_led_state();
        ESP_LOGI(TAG, "This board is active - using local BLE LED state: 0x%02x", current_state);
    } else {
        // This board is inactive - use tracked USB state (from neighbor UART)
        current_state = usb_led_state.led_state_known ? usb_led_state.current_led_state : 0x00;
        ESP_LOGI(TAG, "This board is inactive - using tracked USB state: 0x%02x", current_state);
    }

    ESP_LOGI(TAG, "Restoring to current USB LED state: 0x%02x (known: %s)",
             current_state, usb_led_state.led_state_known ? "YES" : "NO");

    // Force send the LED state to hardware even if our tracking thinks it's already correct
    // The hardware might still be showing the blink state due to timing issues
    if (hid_device.connected) {
        esp_err_t ret = ESP_FAIL;
        const int MAX_RETRIES = 3;
        const int RETRY_DELAY_MS = 10;

        ESP_LOGI(TAG, "Force-sending LED state to hardware (bypassing deduplication): 0x%02x", current_state);

        for (int attempt = 0; attempt < MAX_RETRIES; attempt++) {
            ret = hid_class_request_set_report(
                hid_device.handle,
                HID_REPORT_TYPE_OUTPUT,
                0,
                &current_state,
                sizeof(current_state)
            );

            if (ret == ESP_OK) {
                if (attempt > 0) {
                    ESP_LOGI(TAG, "LED state force-applied on attempt %d: 0x%02x", attempt + 1, current_state);
                } else {
                    ESP_LOGI(TAG, "LED state force-applied: 0x%02x", current_state);
                }
                // Update tracking to reflect successful hardware state
                usb_led_state.current_led_state = current_state;
                usb_led_state.led_state_known = true;
                break;
            } else if (ret == ESP_ERR_NOT_FINISHED && attempt < MAX_RETRIES - 1) {
                ESP_LOGD(TAG, "USB busy during restore (attempt %d/%d), retrying in %dms",
                         attempt + 1, MAX_RETRIES, RETRY_DELAY_MS);
                vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
            } else {
                ESP_LOGW(TAG, "LED restore failed (attempt %d/%d): %s",
                         attempt + 1, MAX_RETRIES, esp_err_to_name(ret));
            }
        }

        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "All LED restore attempts failed, hardware state may be incorrect");
        }
    } else {
        ESP_LOGW(TAG, "No keyboard connected for LED restore");
    }

    // Clear the blink flag to allow LED state changes again
    blink_timer_pending = false;
    ESP_LOGI(TAG, "LED restore timer completed - LED state changes unblocked");
}

/**
 * @brief Initialize LED state by querying keyboard or using defaults
 */
static void initialize_led_state(void)
{
    if (!hid_device.connected) {
        return;
    }

    // Create LED restore timer if not already created
    if (hotkey_state.led_restore_timer == NULL) {
        esp_timer_create_args_t timer_args = {
            .callback = led_restore_timer_callback,
            .arg = NULL,
            .name = "led_restore"
        };
        esp_err_t timer_ret = esp_timer_create(&timer_args, &hotkey_state.led_restore_timer);
        if (timer_ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create LED restore timer: %s", esp_err_to_name(timer_ret));
        } else {
            ESP_LOGI(TAG, "LED restore timer created successfully");
        }
    }

    usb_led_state.current_led_state = 0x00;
    usb_led_state.led_state_known = false;
}

/**
 * @brief Send LED output report to keyboard
 *
 * @param led_state LED state byte (bit 1 = CapsLock)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
static esp_err_t usb_keyboard_set_led_state(uint8_t led_state)
{
    if (!hid_device.connected) {
        ESP_LOGD(TAG, "No keyboard connected, cannot set LED state");
        return ESP_ERR_INVALID_STATE;
    }

    // Block LED state changes during blink to avoid interference
    if (blink_timer_pending) {
        ESP_LOGI(TAG, "Blink timer pending - deferring LED state change to 0x%02x", led_state);
        // Update our tracking but don't send to hardware
        usb_led_state.current_led_state = led_state;
        usb_led_state.led_state_known = true;
        return ESP_OK;
    }

    // Check if LED state is already at the desired value to avoid double-toggle
    if (usb_led_state.led_state_known && usb_led_state.current_led_state == led_state) {
        ESP_LOGD(TAG, "USB LED state already at 0x%02x, skipping redundant USB command", led_state);
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Setting keyboard LED state: 0x%02x -> 0x%02x (NumLock: %s, CapsLock: %s, ScrollLock: %s)",
             usb_led_state.led_state_known ? usb_led_state.current_led_state : 0xFF,
             led_state,
             (led_state & 0x01) ? "ON" : "OFF",  // NumLock
             (led_state & 0x02) ? "ON" : "OFF",  // CapsLock
             (led_state & 0x04) ? "ON" : "OFF"); // ScrollLock

    // HID Output Report for keyboard LEDs using proper HID class request with retry for busy USB
    esp_err_t ret = ESP_FAIL;
    const int MAX_RETRIES = 3;
    const int RETRY_DELAY_MS = 10;

    for (int attempt = 0; attempt < MAX_RETRIES; attempt++) {
        ret = hid_class_request_set_report(
            hid_device.handle,        // hid_host_device_handle_t
            HID_REPORT_TYPE_OUTPUT,   // Report type (0x02)
            0,                        // Report ID (usually 0 for keyboards)
            &led_state,               // Pointer to report data
            sizeof(led_state)         // Length = 1 byte
        );

        if (ret == ESP_OK) {
            if (attempt > 0) {
                ESP_LOGI(TAG, "USB LED state set successfully on attempt %d: 0x%02x", attempt + 1, led_state);
            } else {
                ESP_LOGI(TAG, "USB LED state set successfully: 0x%02x", led_state);
            }
            // Update our tracked USB LED state on successful set
            usb_led_state.current_led_state = led_state;
            usb_led_state.led_state_known = true;
            break;
        } else if (ret == ESP_ERR_NOT_FINISHED && attempt < MAX_RETRIES - 1) {
            ESP_LOGD(TAG, "USB busy (attempt %d/%d), retrying LED command in %dms",
                     attempt + 1, MAX_RETRIES, RETRY_DELAY_MS);
            vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
        } else {
            ESP_LOGE(TAG, "Failed to send LED output report (attempt %d/%d): %s",
                     attempt + 1, MAX_RETRIES, esp_err_to_name(ret));
        }
    }

    return ret;
}

/**
 * @brief Trigger CapsLock LED blink for host switching feedback
 */
static void trigger_capslock_blink(void)
{
    // Get the current USB LED state as the baseline
    uint8_t current_state = 0x00;  // Default to all LEDs off

    if (usb_led_state.led_state_known) {
        current_state = usb_led_state.current_led_state;
        ESP_LOGI(TAG, "Using current USB CapsLock state: %s", (current_state & 0x02) ? "ON" : "OFF");
    } else {
        ESP_LOGI(TAG, "No known USB LED state, assuming OFF");
    }

    // Create blink by toggling CapsLock bit (XOR)
    uint8_t blink_state = current_state ^ 0x02;  // Toggle CapsLock bit (bit 1 = 0x02)

    ESP_LOGI(TAG, "Host switching detected! CapsLock blink: %s -> %s -> %s",
             (current_state & 0x02) ? "ON" : "OFF",
             (blink_state & 0x02) ? "ON" : "OFF",
             (current_state & 0x02) ? "ON" : "OFF");

    // Send blink state to hardware but preserve our tracked current state
    bool had_known_state = usb_led_state.led_state_known;

    if (hid_device.connected) {
        // Send blink state with retry logic for busy USB
        esp_err_t ret = ESP_FAIL;
        const int MAX_RETRIES = 3;
        const int RETRY_DELAY_MS = 10;

        for (int attempt = 0; attempt < MAX_RETRIES; attempt++) {
            ret = hid_class_request_set_report(
                hid_device.handle,
                HID_REPORT_TYPE_OUTPUT,
                0,
                &blink_state,
                sizeof(blink_state)
            );

            if (ret == ESP_OK) {
                if (attempt > 0) {
                    ESP_LOGI(TAG, "LED blink applied on attempt %d: 0x%02x (current state tracking preserved)",
                             attempt + 1, blink_state);
                } else {
                    ESP_LOGI(TAG, "LED blink applied: 0x%02x (current state tracking preserved)", blink_state);
                }
                // Restore the original tracking state (don't let blink overwrite it)
                usb_led_state.current_led_state = current_state;
                usb_led_state.led_state_known = had_known_state;
                break;
            } else if (ret == ESP_ERR_NOT_FINISHED && attempt < MAX_RETRIES - 1) {
                ESP_LOGD(TAG, "USB busy during blink (attempt %d/%d), retrying in %dms",
                         attempt + 1, MAX_RETRIES, RETRY_DELAY_MS);
                vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
            } else {
                ESP_LOGW(TAG, "LED blink failed (attempt %d/%d): %s",
                         attempt + 1, MAX_RETRIES, esp_err_to_name(ret));
            }
        }

        if (ret != ESP_OK) {
            return;
        }
    } else {
        ESP_LOGW(TAG, "No keyboard connected for blink");
        return;
    }

    // Set flag to block LED state changes during blink
    blink_timer_pending = true;

    // Start timer to restore LED state after blink duration
    if (hotkey_state.led_restore_timer != NULL) {
        esp_err_t timer_ret = esp_timer_start_once(hotkey_state.led_restore_timer,
                                                  CAPSLOCK_BLINK_DURATION_MS * 1000);  // Convert ms to us
        if (timer_ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to start LED restore timer: %s", esp_err_to_name(timer_ret));
            // Clear flag since timer won't fire
            blink_timer_pending = false;
            // Fallback: try immediate restore
            esp_err_t fallback_ret = usb_keyboard_set_led_state(current_state);
            if (fallback_ret != ESP_OK) {
                ESP_LOGW(TAG, "Immediate LED restore also failed, LED may remain in wrong state");
            }
        } else {
            ESP_LOGI(TAG, "LED restore timer started - will restore to current state in %dms",
                     CAPSLOCK_BLINK_DURATION_MS);
            ESP_LOGI(TAG, "LED state changes blocked during blink period");
        }
    } else {
        ESP_LOGW(TAG, "LED restore timer not available, trying immediate restore");
        // Clear flag since we're doing immediate restore
        blink_timer_pending = false;
        // Fallback to immediate restore
        vTaskDelay(pdMS_TO_TICKS(CAPSLOCK_BLINK_DURATION_MS));
        esp_err_t fallback_ret = usb_keyboard_set_led_state(current_state);
        if (fallback_ret != ESP_OK) {
            ESP_LOGW(TAG, "Immediate LED restore failed, LED may remain in wrong state");
        }
    }

    ESP_LOGI(TAG, "CapsLock LED blink initiated");
}

/**
 * @brief Detect Ctrl+Ctrl hotkey sequence for host switching
 *
 * @param modifier Current modifier state
 * @param keys Array of 6 key codes from keyboard report
 * @return true if hotkey sequence detected, false otherwise
 */
static bool detect_ctrl_hotkey(uint8_t modifier, const uint8_t keys[6])
{
    uint64_t current_time = esp_timer_get_time();
    bool left_ctrl_pressed = (modifier & 0x01) != 0;  // Left Ctrl is bit 0

    // Check for other modifier keys (Shift, Alt, Right Ctrl, etc.)
    bool other_modifiers_pressed = (modifier & 0xFE) != 0;  // Any modifier except Left Ctrl

    // Check if any non-Ctrl keys are pressed
    bool other_keys_pressed = false;
    for (int i = 0; i < 6; i++) {
        if (keys[i] != 0) {
            other_keys_pressed = true;
            break;
        }
    }

    // Track if Ctrl was used in combination with other keys/modifiers
    static bool ctrl_was_contaminated = false;

    // If Left Ctrl is pressed with other keys/modifiers, mark as contaminated
    if (left_ctrl_pressed && (other_keys_pressed || other_modifiers_pressed)) {
        ctrl_was_contaminated = true;
        if (hotkey_state.waiting_for_second_ctrl) {
            ESP_LOGD(TAG, "Ctrl+key combination detected - invalidating hotkey sequence");
            hotkey_state.waiting_for_second_ctrl = false;
        }
        return false;
    }

    // If other keys or modifiers are pressed during Ctrl sequence, invalidate it
    if ((other_keys_pressed || other_modifiers_pressed) && hotkey_state.waiting_for_second_ctrl) {
        if (other_keys_pressed) {
            ESP_LOGD(TAG, "Other keys pressed during Ctrl sequence (keys=%02x,%02x,%02x,%02x,%02x,%02x) - invalidating hotkey detection",
                     keys[0], keys[1], keys[2], keys[3], keys[4], keys[5]);
        }
        if (other_modifiers_pressed) {
            ESP_LOGD(TAG, "Other modifiers pressed during Ctrl sequence (mod=0x%02x) - invalidating hotkey detection", modifier);
        }
        hotkey_state.waiting_for_second_ctrl = false;
        return false;
    }

    if (left_ctrl_pressed && !other_keys_pressed && !other_modifiers_pressed) {
        // Pure Left Ctrl press - but check if it was contaminated
        if (ctrl_was_contaminated) {
            ESP_LOGD(TAG, "Left Ctrl pressed but was contaminated by previous combination - ignoring");
            return false;
        }

        if (!hotkey_state.waiting_for_second_ctrl) {
            // First Ctrl press detected
            hotkey_state.last_left_ctrl_press_time = current_time;
            hotkey_state.waiting_for_second_ctrl = true;
            ESP_LOGD(TAG, "First Ctrl press detected, waiting for second");
        } else {
            // Check if this is the second Ctrl press within timeout
            uint64_t time_diff = current_time - hotkey_state.last_left_ctrl_press_time;
            if (time_diff < HOTKEY_TIMEOUT_US) {
                // Valid Ctrl+Ctrl sequence detected!
                ESP_LOGI(TAG, "Ctrl+Ctrl hotkey detected! Time between presses: %llu us", time_diff);
                hotkey_state.hotkey_detected = true;
                hotkey_state.waiting_for_second_ctrl = false;
                ctrl_was_contaminated = false;  // Reset contamination on successful detection
                return true;
            } else {
                // Timeout exceeded, restart detection
                hotkey_state.last_left_ctrl_press_time = current_time;
                ESP_LOGD(TAG, "Ctrl timeout exceeded, restarting detection");
            }
        }
    } else if (!left_ctrl_pressed) {
        // Left Ctrl released - clear contamination and handle timeout
        ctrl_was_contaminated = false;

        if (hotkey_state.waiting_for_second_ctrl) {
            uint64_t time_diff = current_time - hotkey_state.last_left_ctrl_press_time;
            if (time_diff > HOTKEY_TIMEOUT_US) {
                hotkey_state.waiting_for_second_ctrl = false;
                ESP_LOGD(TAG, "Ctrl detection timeout, reset state");
            }
        }
    }

    return false;
}

/**
 * @brief Process host switching request
 */
static void process_host_switch(void)
{
    ESP_LOGI(TAG, "Processing host switch request...");

    // Trigger visual feedback
    trigger_capslock_blink();

    // Toggle roles: if we're currently active, make neighbor active (and us inactive)
    // If we're currently inactive, make ourselves active (and neighbor inactive)
    bool current_state = uart_protocol_is_input_active();
    bool make_neighbor_active = current_state;  // If we're active, make neighbor active

    esp_err_t uart_ret = uart_protocol_send_input_routing_command(make_neighbor_active);
    if (uart_ret != ESP_OK && uart_ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "Failed to send input routing command to neighbor: %s", esp_err_to_name(uart_ret));
    }

    if (make_neighbor_active) {
        ESP_LOGI(TAG, "Host switch: neighbor is now ACTIVE, this board is now INACTIVE");
        // Call host switching module to handle the switch to host 1 (neighbor active)
        esp_err_t host_switch_ret = host_switching_select(1);
        if (host_switch_ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to update host switching module state: %s", esp_err_to_name(host_switch_ret));
        }
    } else {
        ESP_LOGI(TAG, "Host switch: this board is now ACTIVE, neighbor is now INACTIVE");
        // Call host switching module to handle the switch to host 0 (this board active)
        esp_err_t host_switch_ret = host_switching_select(0);
        if (host_switch_ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to update host switching module state: %s", esp_err_to_name(host_switch_ret));
        }
    }

    // Reset hotkey state
    hotkey_state.hotkey_detected = false;
}

/**
 * @brief USB HID Host Keyboard Interface report callback handler
 *
 * Process received keyboard reports and route based on content analysis
 *
 * @param[in] data    Pointer to input report data buffer
 * @param[in] length  Length of input report data buffer
 */
static void hid_host_keyboard_report_callback(const uint8_t *const data, const int length)
{
    // COMPREHENSIVE RAW DATA LOGGING - Print ALL bytes received
    ESP_LOGD(TAG, "=== RAW KEYBOARD REPORT (length=%d) ===", length);
    char hex_buf[256] = {0};
    char ascii_buf[64] = {0};
    char *hex_ptr = hex_buf;
    char *ascii_ptr = ascii_buf;

    for (int i = 0; i < length; i++) {
        hex_ptr += sprintf(hex_ptr, "%02x ", data[i]);
        ascii_ptr += sprintf(ascii_ptr, "%c", (data[i] >= 32 && data[i] <= 126) ? data[i] : '.');

        // Print in groups of 8 bytes for readability
        if ((i + 1) % 8 == 0 || i == length - 1) {
            ESP_LOGD(TAG, "  [%02d-%02d]: %s | %s",
                     i - (i % 8), i, hex_buf, ascii_buf);
            hex_ptr = hex_buf;
            ascii_ptr = ascii_buf;
            hex_buf[0] = 0;
            ascii_buf[0] = 0;
        }
    }

    // Bit-by-bit analysis of first few bytes
    ESP_LOGD(TAG, "Bit analysis:");
    for (int i = 0; i < (length > 8 ? 8 : length); i++) {
        ESP_LOGD(TAG, "  data[%d] = 0x%02x = %s%s%s%s%s%s%s%s",
                 i, data[i],
                 (data[i] & 0x80) ? "1" : "0",
                 (data[i] & 0x40) ? "1" : "0",
                 (data[i] & 0x20) ? "1" : "0",
                 (data[i] & 0x10) ? "1" : "0",
                 (data[i] & 0x08) ? "1" : "0",
                 (data[i] & 0x04) ? "1" : "0",
                 (data[i] & 0x02) ? "1" : "0",
                 (data[i] & 0x01) ? "1" : "0");
    }

    if (length < 8) {
        ESP_LOGW(TAG, "Keyboard report too short: %d bytes (expected 8)", length);
        // Continue processing even if short, might still be useful data
    }

    // Standard USB HID keyboard report format:
    // data[0] = modifier byte
    // data[1] = reserved byte
    // data[2-7] = up to 6 simultaneous key codes

    uint8_t modifier = (length >= 1) ? data[0] : 0;
    uint8_t reserved = (length >= 2) ? data[1] : 0;

    // Extract the 6 key codes (with bounds checking)
    uint8_t keys[6] = {0};
    for (int i = 0; i < 6 && (i + 2) < length; i++) {
        keys[i] = data[i + 2];
    }

    ESP_LOGD(TAG, "Standard interpretation: modifier=0x%02x, reserved=0x%02x, keys=%02x,%02x,%02x,%02x,%02x,%02x",
        modifier, reserved, keys[0], keys[1], keys[2], keys[3], keys[4], keys[5]);

    // Use modifier + mysterious 0x01 in keys[5] to distinguish mouse interface from Alt key
    if (modifier == 0x04 && keys[5] == 0x01) {
        // Mouse interface data coming through keyboard HID interface (Alt key + magic 0x01)
        ESP_LOGD(TAG, "Mouse event via keyboard interface: mod=0x%02x keys=%02x,%02x,%02x,%02x,%02x,%02x",
            modifier, keys[0], keys[1], keys[2], keys[3], keys[4], keys[5]);

        // Check for scroll events (key 0x05 = scroll up, 0x0F = scroll down)
        for (int i = 0; i < 6; i++) {
            if (keys[i] == 0x05) {
                ESP_LOGD(TAG, "Mouse scroll UP");
                esp_err_t ret = ble_hid_mouse_report_enhanced(0, 0, 0, 1);
                if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
                    ESP_LOGW(TAG, "Failed to send mouse scroll to BLE: %s", esp_err_to_name(ret));
                }
                return;
            } else if (keys[i] == 0x0F) {
                ESP_LOGD(TAG, "Mouse scroll DOWN");
                esp_err_t ret = ble_hid_mouse_report_enhanced(0, 0, 0, -1);
                if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
                    ESP_LOGW(TAG, "Failed to send mouse scroll to BLE: %s", esp_err_to_name(ret));
                }
                return;
            }
        }

        // Handle mouse buttons (ignore the 0x01 in keys[5] - seems to be always present)
        uint8_t mouse_buttons = 0;
        for (int i = 0; i < 5; i++) { // Only check first 5 keys, ignore keys[5]
            if (keys[i] == 0x01) mouse_buttons |= 0x01; // Left button
            if (keys[i] == 0x02) mouse_buttons |= 0x02; // Right button
            if (keys[i] == 0x04) mouse_buttons |= 0x04; // Middle button
            if (keys[i] == 0x08) mouse_buttons |= 0x08; // Button 4 (Back)
            if (keys[i] == 0x10) mouse_buttons |= 0x10; // Button 5 (Forward)
        }

        // Update button overlay for hybrid mouse
        button_overlay = mouse_buttons;

        ESP_LOGD(TAG, "Mouse buttons from keyboard: 0x%02x (overlay updated)", mouse_buttons);
        esp_err_t ret = ble_hid_mouse_report_enhanced(0, 0, mouse_buttons, 0);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "Failed to send mouse buttons to BLE: %s", esp_err_to_name(ret));
        }

    } else {
        // Real keyboard input (any modifier that's not the mouse interface pattern)

        // Check if this report is different from the last one
        bool report_changed = (modifier != last_keyboard_state.last_modifier);
        if (!report_changed) {
            for (int i = 0; i < 6; i++) {
                if (keys[i] != last_keyboard_state.last_keys[i]) {
                    report_changed = true;
                    break;
                }
            }
        }

        // Only send BLE report if state changed
        if (report_changed) {
            ESP_LOGD(TAG, "Keyboard state changed: mod=0x%02x keys=%02x,%02x,%02x,%02x,%02x,%02x",
                modifier, keys[0], keys[1], keys[2], keys[3], keys[4], keys[5]);

            // Check for Ctrl+Ctrl hotkey sequence (keyboard board always handles hotkey detection)
            if (detect_ctrl_hotkey(modifier, keys)) {
                process_host_switch();
                // Continue processing the keyboard report normally
            }

            // Only forward to BLE if this board is active for input routing
            if (uart_protocol_is_input_active()) {
                esp_err_t ret = ble_hid_keyboard_report(modifier, reserved, keys);
                if (ret != ESP_OK) {
                    if (ret != ESP_ERR_INVALID_STATE) {
                        ESP_LOGW(TAG, "Failed to forward keyboard report to BLE: %s", esp_err_to_name(ret));
                    } else {
                        ESP_LOGD(TAG, "No BLE host connected, keyboard report discarded");
                    }
                }
            } else {
                ESP_LOGD(TAG, "Input routing INACTIVE - keyboard report not sent to BLE");
            }

            // Update last state
            last_keyboard_state.last_modifier = modifier;
            memcpy(last_keyboard_state.last_keys, keys, 6);
        } else {
            ESP_LOGD(TAG, "Keyboard report unchanged, skipping BLE send");
        }

        // Forward ALL keyboard HID reports to neighbor board via UART (including unchanged ones)
        // This ensures the neighbor gets every USB report for proper key repeat behavior
        uint8_t keyboard_report[8] = {modifier, reserved, keys[0], keys[1], keys[2], keys[3], keys[4], keys[5]};
        esp_err_t uart_ret = uart_protocol_forward_keyboard_report(keyboard_report, sizeof(keyboard_report));
        if (uart_ret != ESP_OK && uart_ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGI(TAG, "Failed to forward keyboard report to UART: %s", esp_err_to_name(uart_ret));
        }
    }
}

/**
 * @brief USB HID Host Consumer Control Interface report callback handler
 *
 * Process received consumer control reports (multimedia keys, etc)
 *
 * @param[in] data    Pointer to input report data buffer
 * @param[in] length  Length of input report data buffer
 */
static void hid_host_consumer_report_callback(const uint8_t *const data, const int length)
{
    ESP_LOGD(TAG, "Consumer control report received (length=%d)", length);

    // Extract usage code from the report data
    uint16_t usage_code = 0;

    // Handle different report formats
    if (length == 3) {
        // Standard 3-byte consumer control report
        if (data[0] == 0x02) {
            // Standard format with 0x02 prefix
            usage_code = (data[1] | (data[2] << 8));
            ESP_LOGD(TAG, "Standard consumer control format: prefix=0x02, usage=0x%04x", usage_code);
        } else {
            // Other formats that might be function keys
            // Some keyboards use different report formats for Fn+F keys
            if (data[1] == 0x0C) {
                // Usage page 0x0C is Consumer Page
                usage_code = data[2];
                ESP_LOGD(TAG, "Function key with consumer usage page: usage=0x%04x", usage_code);
            } else {
                // Try to interpret directly
                usage_code = (data[1] | (data[2] << 8));
                ESP_LOGD(TAG, "Alternative Fn key format detected: usage=0x%04x", usage_code);
            }
        }
    } else if (length == 2) {
        // Some keyboards use 2-byte format for consumer controls
        usage_code = data[1];
        ESP_LOGD(TAG, "Short consumer control format: usage=0x%04x", usage_code);
    } else {
        ESP_LOGW(TAG, "Unexpected consumer control report length: %d", length);
        return;
    }

    // Log the detected usage code
    ESP_LOGD(TAG, "Consumer control usage code: 0x%04x", usage_code);
    handle_consumer_control(usage_code);
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
    // COMPREHENSIVE RAW DATA LOGGING - Print ALL bytes received
    ESP_LOGD(TAG, "=== RAW MOUSE REPORT (length=%d) ===", length);
    char hex_buf[256] = {0};
    char ascii_buf[64] = {0};
    char *hex_ptr = hex_buf;
    char *ascii_ptr = ascii_buf;

    for (int i = 0; i < length; i++) {
        hex_ptr += sprintf(hex_ptr, "%02x ", data[i]);
        ascii_ptr += sprintf(ascii_ptr, "%c", (data[i] >= 32 && data[i] <= 126) ? data[i] : '.');

        // Print in groups of 8 bytes for readability
        if ((i + 1) % 8 == 0 || i == length - 1) {
            ESP_LOGD(TAG, "  [%02d-%02d]: %s | %s",
                     i - (i % 8), i, hex_buf, ascii_buf);
            hex_ptr = hex_buf;
            ascii_ptr = ascii_buf;
            hex_buf[0] = 0;
            ascii_buf[0] = 0;
        }
    }

    // Check for consumer control reports (multimedia keys) and function keys
    if (length == 3) {
        ESP_LOGD(TAG, "Analyzing 3-byte report with first byte: 0x%02x", data[0]);

        // Pattern 0: Standard mouse reports usually have first byte with only button bits set (0-7)
        // So any report with the first byte > 0x07 is likely not a standard mouse report
        bool is_standard_mouse = (data[0] <= 0x07);

        // Pattern 1: First byte is 0x02 for common consumer control (multimedia) keys
        if (data[0] == 0x02) {
            ESP_LOGD(TAG, "Detected consumer control report (length=3, 0x02), routing to consumer handler");
            hid_host_consumer_report_callback(data, length);
            return;
        }

        // Pattern 2: Any report ID that's not a standard mouse report ID
        // Only 0x00 and 0x01 are commonly used for mouse buttons
        if (!is_standard_mouse) {
            ESP_LOGD(TAG, "Non-standard mouse report ID: 0x%02x, likely function key", data[0]);

            // Route all these to consumer control handler to let it try to interpret
            ESP_LOGD(TAG, "Routing possible function key to consumer handler: %02x %02x %02x",
                    data[0], data[1], data[2]);
            hid_host_consumer_report_callback(data, length);
            return;
        }

        // Pattern 3: Look for consumer control usage page (0x0C)
        if (data[1] == 0x0C || ((data[1] | (data[2] << 8)) >= 0x00B0 && (data[1] | (data[2] << 8)) <= 0x03FF)) {
            ESP_LOGD(TAG, "Consumer control signature detected: %02x %02x %02x", data[0], data[1], data[2]);
            hid_host_consumer_report_callback(data, length);
            return;
        }

        // Pattern 4: Look for common function key patterns
        // Common function key usually have non-zero values in data[2]
        if (data[2] != 0x00) {
            ESP_LOGD(TAG, "Possible function key (non-zero data[2]): %02x %02x %02x", data[0], data[1], data[2]);
            hid_host_consumer_report_callback(data, length);
            return;
        }

        // Pattern 5: Standard mouse reports usually have movement data
        // So if there's button data but no movement, it might be a special key
        if (data[0] != 0x00 && data[1] == 0x00 && data[2] == 0x00) {
            ESP_LOGD(TAG, "Button press with no movement, might be special: %02x %02x %02x",
                    data[0], data[1], data[2]);
            hid_host_consumer_report_callback(data, length);
            return;
        }

        // If we get here, it's probably a genuine mouse report
        ESP_LOGD(TAG, "Treating as legitimate mouse report: %02x %02x %02x", data[0], data[1], data[2]);
    }

    // Bit-by-bit analysis of first few bytes
    ESP_LOGD(TAG, "Bit analysis:");
    for (int i = 0; i < (length > 8 ? 8 : length); i++) {
        ESP_LOGD(TAG, "  data[%d] = 0x%02x = %s%s%s%s%s%s%s%s",
                 i, data[i],
                 (data[i] & 0x80) ? "1" : "0",
                 (data[i] & 0x40) ? "1" : "0",
                 (data[i] & 0x20) ? "1" : "0",
                 (data[i] & 0x10) ? "1" : "0",
                 (data[i] & 0x08) ? "1" : "0",
                 (data[i] & 0x04) ? "1" : "0",
                 (data[i] & 0x02) ? "1" : "0",
                 (data[i] & 0x01) ? "1" : "0");
    }

    if (length < 3) {
        ESP_LOGW(TAG, "Mouse report too short: %d bytes", length);
        return;  // Invalid report
    }

    // Extract button states from data[0]
    uint8_t usb_buttons = data[0]; // Keep all 8 bits as the mouse may have more than 3 buttons

    // Update local button state tracking
    mouse_left_button = (usb_buttons & 0x01) ? true : false;
    mouse_right_button = (usb_buttons & 0x02) ? true : false;
    mouse_middle_button = (usb_buttons & 0x04) ? true : false;

    // Always use 8-byte format for modern mice
    int8_t delta_x = 0, delta_y = 0, scroll_y = 0;
    uint8_t actual_buttons = 0;

    if (length == 8) {
        ESP_LOGD(TAG, "Processing 8-byte mouse format");

        // Always extract movement from 16-bit fields
        // X movement is 16-bit little-endian: data[2] + (data[3] << 8)
        int16_t x_16 = (int16_t)(data[2] | (data[3] << 8));
        delta_x = (x_16 > 127) ? 127 : (x_16 < -127) ? -127 : (int8_t)x_16;

        // Y movement is 16-bit little-endian: data[4] + (data[5] << 8)
        int16_t y_16 = (int16_t)(data[4] | (data[5] << 8));
        delta_y = (y_16 > 127) ? 127 : (y_16 < -127) ? -127 : (int8_t)y_16;

        // Only use scroll when data[0] == 1 (pure movement/scroll reports)
        if (data[0] == 1) {
            actual_buttons = data[1];  // Buttons from data[1]

            // Scroll wheel in data[6]
            if (data[6] == 0x01) {
                scroll_y = 1;   // Scroll up
            } else if (data[6] == 0xFF) {
                scroll_y = -1;  // Scroll down
            } else {
                scroll_y = 0;   // No scroll
            }

            ESP_LOGD(TAG, "Pure mouse report: buttons=0x%02x, X16=%d->%d, Y16=%d->%d, scroll_raw=0x%02x->%d",
                     actual_buttons, x_16, delta_x, y_16, delta_y, data[6], scroll_y);
        } else {
            // For other data[0] values, ignore buttons and scroll (they come via keyboard interface)
            scroll_y = 0;

            ESP_LOGD(TAG, "Movement-only report (data[0]=0x%02x): X16=%d->%d, Y16=%d->%d, ignoring buttons/scroll",
                     data[0], x_16, delta_x, y_16, delta_y);
        }
        // overlay from keyboard events on some mice
        actual_buttons |= button_overlay;
    } else if (length >= 3) {
        // Fallback to standard format for older mice or 3-button mice with no scroll wheel
        ESP_LOGD(TAG, "Using standard %d-byte mouse format", length);

        actual_buttons = data[0];  // Buttons in data[0]
        delta_x = (int8_t)data[1];
        delta_y = (int8_t)data[2];
        scroll_y = (length >= 4) ? (int8_t)data[3] : 0;  // Only use scroll if 4+ bytes

        ESP_LOGD(TAG, "Standard parsed: buttons=0x%02x, delta=(%d,%d), scroll=%d",
                 actual_buttons, delta_x, delta_y, scroll_y);
    } else {
        ESP_LOGW(TAG, "Unknown mouse report format, length=%d", length);
        return;
    }

    // Update virtual mouse position for local tracking
    mouse_x += delta_x;
    mouse_y += delta_y;

    // Bound the position to avoid overflow (adjust limits as needed)
    if (mouse_x < 0) mouse_x = 0;
    if (mouse_y < 0) mouse_y = 0;
    if (mouse_x > 1000) mouse_x = 1000;
    if (mouse_y > 1000) mouse_y = 1000;

    // Map USB button bits to BLE HID button constants
    uint8_t ble_buttons = 0;
    if (actual_buttons & 0x01) ble_buttons |= BLE_HID_MOUSE_LEFT_BUTTON;
    if (actual_buttons & 0x02) ble_buttons |= BLE_HID_MOUSE_RIGHT_BUTTON;
    if (actual_buttons & 0x04) ble_buttons |= BLE_HID_MOUSE_MIDDLE_BUTTON;

    ESP_LOGD(TAG, "Final BLE output: USB_buttons=0x%02x -> BLE_buttons=0x%02x, delta=(%d,%d), scroll=%d",
             actual_buttons, ble_buttons, delta_x, delta_y, scroll_y);

    // Send mouse movement and button data to BLE (only if this board is active)
    if (uart_protocol_is_input_active()) {
        esp_err_t ret = ble_hid_mouse_report_enhanced(delta_x, delta_y, ble_buttons, scroll_y);
        if (ret != ESP_OK) {
            // Only log warning if it's not just a missing active host
            if (ret != ESP_ERR_INVALID_STATE) {
                ESP_LOGW(TAG, "Failed to forward mouse report to BLE: %s", esp_err_to_name(ret));
            } else {
                ESP_LOGD(TAG, "No BLE host connected, mouse report discarded");
            }
        } else {
            ESP_LOGD(TAG, "USB->BLE Mouse: pos=(%d,%d) delta=(%d,%d) USB_buttons=0x%02x BLE_buttons=0x%02x",
                mouse_x, mouse_y, delta_x, delta_y, usb_buttons, ble_buttons);
        }
    } else {
        ESP_LOGD(TAG, "Input routing INACTIVE - mouse report not sent to BLE");
    }

    // Forward processed mouse data to neighbor board via UART (fire-and-forget)
    uint8_t mouse_report[4] = {
        (uint8_t)delta_x,      // Raw movement delta (int8_t cast to uint8_t)
        (uint8_t)delta_y,      // Raw movement delta (int8_t cast to uint8_t)
        ble_buttons,           // BLE-mapped button state
        (uint8_t)scroll_y      // Scroll delta (int8_t cast to uint8_t)
    };
    esp_err_t uart_ret = uart_protocol_forward_mouse_report(mouse_report, sizeof(mouse_report));
    if (uart_ret != ESP_OK && uart_ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGD(TAG, "Failed to forward mouse report to UART: %s", esp_err_to_name(uart_ret));
    }
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
        ESP_LOGD(TAG, "USB HID report: sub_class=%d, proto=%d, length=%d", dev_params.sub_class, dev_params.proto, data_length);

        // Log all raw HID report data at the entry point for better debugging
        if (data_length <= 16) { // Only log reasonably sized reports
            char hex_buf[64] = {0};
            char *hex_ptr = hex_buf;
            for (int i = 0; i < data_length; i++) {
                hex_ptr += sprintf(hex_ptr, "%02x ", data[i]);
            }
            ESP_LOGD(TAG, "HID RAW DATA: %s", hex_buf);
        }

        // Special filtering for 3-byte reports (consumer control, function keys, etc.)
        if (data_length == 3) {
            ESP_LOGD(TAG, "INTERFACE: Analyzing 3-byte report with first byte: 0x%02x", data[0]);

            // Pattern 0: Standard mouse reports usually have first byte with mouse button bits (0-7)
            // First bytes 0x00-0x07 are standard mouse buttons (combinations of left, right, middle)
            bool is_standard_mouse = (data[0] <= 0x07);

            // Mouse button presses (0x01-0x07) with no movement should go to mouse handler, not consumer control
            if (is_standard_mouse && data[0] > 0) {
                ESP_LOGD(TAG, "INTERFACE: Button press detected in 3-byte report: %02x %02x %02x - routing to mouse handler",
                        data[0], data[1], data[2]);
                hid_host_mouse_report_callback(data, data_length);
                break;
            }

            // Pattern 1: First byte is 0x02 for common consumer control (multimedia) keys
            // Now only matched for non-mouse-button reports (first byte > 0x07)
            if (data[0] == 0x02 && !is_standard_mouse) {
                ESP_LOGD(TAG, "INTERFACE: Detected consumer control report (0x02), routing directly");
                hid_host_consumer_report_callback(data, data_length);
                break;
            }

            // Pattern 2: Any report ID that's not a standard mouse report ID
            if (!is_standard_mouse) {
                ESP_LOGD(TAG, "INTERFACE: Non-standard mouse report ID: 0x%02x, likely function key", data[0]);

                // Route all these to consumer control handler to let it try to interpret
                ESP_LOGD(TAG, "INTERFACE: Routing possible function key to consumer handler: %02x %02x %02x",
                        data[0], data[1], data[2]);
                hid_host_consumer_report_callback(data, data_length);
                break;
            }

            // Pattern 3: Look for consumer control usage page (0x0C)
            if (data[1] == 0x0C || ((data[1] | (data[2] << 8)) >= 0x00B0 && (data[1] | (data[2] << 8)) <= 0x03FF)) {
                ESP_LOGD(TAG, "INTERFACE: Consumer control signature detected: %02x %02x %02x", data[0], data[1], data[2]);
                hid_host_consumer_report_callback(data, data_length);
                break;
            }

            // Pattern 4: Common function key patterns
            // Common function key usually have non-zero values in data[2]
            if (data[2] != 0x00) {
                ESP_LOGD(TAG, "INTERFACE: Possible function key (non-zero data[2]): %02x %02x %02x", data[0], data[1], data[2]);
                hid_host_consumer_report_callback(data, data_length);
                break;
            }

            // Pattern 5: Standard mouse reports usually have movement data
            // So if there's button data but no movement, it might be a special key
            if (data[0] != 0x00 && data[1] == 0x00 && data[2] == 0x00) {
                ESP_LOGD(TAG, "INTERFACE: Button press with no movement, might be special: %02x %02x %02x",
                        data[0], data[1], data[2]);
                hid_host_consumer_report_callback(data, data_length);
                break;
            }

            // If we get here, it's probably a legitimate mouse report
            ESP_LOGD(TAG, "INTERFACE: Treating as legitimate mouse report: %02x %02x %02x", data[0], data[1], data[2]);
            goto normal_routing;
        }

        normal_routing:

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
        ESP_LOGW(TAG, "HID Device, protocol '%s' TRANSFER_ERROR - continuing operation",
                 hid_proto_name_str[dev_params.proto]);

        // Transfer errors can be transient - log but don't trigger aggressive recovery
        // This preserves hot-plug functionality while noting the error
        ESP_LOGI(TAG, "Transfer error logged, device may recover on its own");
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
 * @brief Reinitialize USB host to recover from enumeration failures
 */
static esp_err_t reinitialize_usb_host(void)
{
    ESP_LOGI(TAG, "Reinitializing USB host to recover from enumeration failures");

    // Uninstall HID host driver
    esp_err_t ret = hid_host_uninstall();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to uninstall HID host driver during reinit: %s", esp_err_to_name(ret));
    }

    // Wait for cleanup
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Reinstall HID host driver with higher priority
    const hid_host_driver_config_t hid_host_driver_config = {
        .create_background_task = true,
        .task_priority = 7,  // Higher priority for better responsiveness
        .stack_size = 6144,
        .core_id = 0,
        .callback = hid_host_device_callback,
        .callback_arg = NULL
    };

    ret = hid_host_install(&hid_host_driver_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reinstall HID host driver: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "USB host reinitialized, waiting for device enumeration");
    vTaskDelay(pdMS_TO_TICKS(2000));

    return ESP_OK;
}

/**
 * @brief Dump detailed USB device descriptor information
 *
 * @param[in] device_handle Handle to the HID device
 */
static void dump_usb_device_descriptors(hid_host_device_handle_t device_handle)
{
    ESP_LOGI(TAG, "=== USB Device Descriptor Information ===");

    // Get basic device parameters
    hid_host_dev_params_t dev_params;
    esp_err_t ret = hid_host_device_get_params(device_handle, &dev_params);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get device params: %s", esp_err_to_name(ret));
        return;
    }

    // Get device info (VID/PID, strings)
    hid_host_dev_info_t dev_info;
    ret = hid_host_get_device_info(device_handle, &dev_info);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get device info: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "Device Parameters:");
    ESP_LOGI(TAG, "  VID: 0x%04X", dev_info.VID);
    ESP_LOGI(TAG, "  PID: 0x%04X", dev_info.PID);
    ESP_LOGI(TAG, "  Protocol: %d (%s)", dev_params.proto, hid_proto_name_str[dev_params.proto]);
    ESP_LOGI(TAG, "  Sub-class: %d", dev_params.sub_class);
    ESP_LOGI(TAG, "  Address: %d", dev_params.addr);
    ESP_LOGI(TAG, "  Interface: %d", dev_params.iface_num);

    // Convert wide char strings to char for logging
    char manufacturer[HID_STR_DESC_MAX_LENGTH + 1] = {0};
    char product[HID_STR_DESC_MAX_LENGTH + 1] = {0};
    char serial[HID_STR_DESC_MAX_LENGTH + 1] = {0};

    // Convert wide strings to regular strings
    for (int i = 0; i < HID_STR_DESC_MAX_LENGTH && dev_info.iManufacturer[i] != 0; i++) {
        manufacturer[i] = (char)dev_info.iManufacturer[i];
    }
    for (int i = 0; i < HID_STR_DESC_MAX_LENGTH && dev_info.iProduct[i] != 0; i++) {
        product[i] = (char)dev_info.iProduct[i];
    }
    for (int i = 0; i < HID_STR_DESC_MAX_LENGTH && dev_info.iSerialNumber[i] != 0; i++) {
        serial[i] = (char)dev_info.iSerialNumber[i];
    }

    ESP_LOGI(TAG, "  Manufacturer: %s", manufacturer);
    ESP_LOGI(TAG, "  Product: %s", product);
    ESP_LOGI(TAG, "  Serial: %s", serial);

    // Try to get HID report descriptor for analysis
    size_t report_desc_len = 0;
    uint8_t *report_desc = hid_host_get_report_descriptor(device_handle, &report_desc_len);
    if (report_desc != NULL && report_desc_len > 0) {
        ESP_LOGI(TAG, "HID Report Descriptor (%d bytes):", report_desc_len);

        // Print the raw descriptor bytes for analysis
        for (size_t i = 0; i < report_desc_len; i += 16) {
            char line_buf[128];
            char *ptr = line_buf;
            ptr += sprintf(ptr, "  %04x: ", i);

            // Print hex bytes
            for (size_t j = 0; j < 16 && (i + j) < report_desc_len; j++) {
                ptr += sprintf(ptr, "%02x ", report_desc[i + j]);
            }

            // Pad to align ASCII
            while (ptr - line_buf < 54) {
                *ptr++ = ' ';
            }

            // Print ASCII representation
            *ptr++ = '|';
            for (size_t j = 0; j < 16 && (i + j) < report_desc_len; j++) {
                uint8_t byte = report_desc[i + j];
                *ptr++ = (byte >= 32 && byte <= 126) ? byte : '.';
            }
            *ptr++ = '|';
            *ptr = '\0';

            ESP_LOGI(TAG, "%s", line_buf);
        }

        // Analyze key HID usage pages in the descriptor
        ESP_LOGI(TAG, "HID Usage Analysis:");
        bool has_keyboard_usage = false;
        bool has_mouse_usage = false;
        bool has_consumer_usage = false;

        for (size_t i = 0; i < report_desc_len - 1; i++) {
            // Look for Usage Page items (0x05)
            if (report_desc[i] == 0x05) {
                uint8_t usage_page = report_desc[i + 1];
                switch (usage_page) {
                    case 0x01:
                        ESP_LOGI(TAG, "  - Generic Desktop usage page found");
                        break;
                    case 0x07:
                        has_keyboard_usage = true;
                        ESP_LOGI(TAG, "  - Keyboard/Keypad usage page found");
                        break;
                    case 0x02:
                        has_mouse_usage = true;
                        ESP_LOGI(TAG, "  - Simulation Controls usage page found");
                        break;
                    case 0x0C:
                        has_consumer_usage = true;
                        ESP_LOGI(TAG, "  - Consumer usage page found");
                        break;
                    default:
                        ESP_LOGI(TAG, "  - Usage page 0x%02X found", usage_page);
                        break;
                }
            }

            // Look for Usage items (0x09) following Generic Desktop (0x01)
            if (i > 0 && report_desc[i-2] == 0x05 && report_desc[i-1] == 0x01 && report_desc[i] == 0x09) {
                uint8_t usage = report_desc[i + 1];
                switch (usage) {
                    case 0x02:
                        has_mouse_usage = true;
                        ESP_LOGI(TAG, "  - Mouse usage found");
                        break;
                    case 0x06:
                        has_keyboard_usage = true;
                        ESP_LOGI(TAG, "  - Keyboard usage found");
                        break;
                    case 0x30:
                        ESP_LOGI(TAG, "  - X axis usage found");
                        break;
                    case 0x31:
                        ESP_LOGI(TAG, "  - Y axis usage found");
                        break;
                    case 0x38:
                        ESP_LOGI(TAG, "  - Wheel usage found");
                        break;
                    default:
                        ESP_LOGI(TAG, "  - Generic Desktop usage 0x%02X found", usage);
                        break;
                }
            }
        }

        ESP_LOGI(TAG, "Device Classification:");
        ESP_LOGI(TAG, "  Has Keyboard Usage: %s", has_keyboard_usage ? "YES" : "NO");
        ESP_LOGI(TAG, "  Has Mouse Usage: %s", has_mouse_usage ? "YES" : "NO");
        ESP_LOGI(TAG, "  Has Consumer Usage: %s", has_consumer_usage ? "YES" : "NO");

    } else {
        ESP_LOGI(TAG, "Failed to get HID report descriptor: %s", esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "==========================================");
}




/**
 * @brief Release all input reports (keyboard, mouse, consumer control)
 *
 * Called when USB device disconnects or has errors to prevent stuck keys
 */
static void release_all_inputs(void) {
    ESP_LOGI(TAG, "Releasing all inputs due to USB device disconnect or error");

    // 1. Release all keyboard keys by sending empty keyboard report
    uint8_t empty_keyboard_report[8] = {0}; // Report ID + modifier + 6 keys
    hid_host_keyboard_report_callback(empty_keyboard_report, sizeof(empty_keyboard_report));

    // 2. Release mouse buttons with empty mouse report
    uint8_t empty_mouse_report[4] = {0}; // Report ID + buttons + X + Y
    hid_host_mouse_report_callback(empty_mouse_report, sizeof(empty_mouse_report));

    // 3. Release any multimedia/consumer control keys
    uint8_t empty_consumer_report[3] = {0}; // Report ID + 16-bit usage code
    hid_host_consumer_report_callback(empty_consumer_report, sizeof(empty_consumer_report));

    ESP_LOGI(TAG, "All inputs released");
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

        ESP_LOGI(TAG, "HID interface ready - will route reports based on content analysis");

        // Initialize LED state for keyboards
        if (dev_params.proto == HID_PROTOCOL_KEYBOARD) {
            ESP_LOGI(TAG, "Initializing keyboard LED state...");
            initialize_led_state();
        }
        ESP_LOGI(TAG, "HID device connected and ready");
        break;

    default:
        // For any other event (assumed to be disconnection or errors), release all inputs
        if (hid_device.connected && hid_device_handle == hid_device.handle) {
            ESP_LOGW(TAG, "HID Device event %d - assuming device disconnection", event);

            // Release all pressed inputs to prevent stuck keys/buttons
            release_all_inputs();

            // Mark device as disconnected
            hid_device.connected = false;
            hid_device.handle = NULL;
        }
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
    // Descriptor dumps have been disabled to improve stability
    // We don't need the descriptors for normal operation

    // Timeout for USB host library events handling - increased for robustness
    const TickType_t event_timeout = 100 / portTICK_PERIOD_MS;

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
            ESP_LOGW(TAG, "USB host lib handle events failed: %s", esp_err_to_name(ret));

            // If we have a connected device and encounter USB host errors, release all inputs
            if (hid_device.connected) {
                ESP_LOGW(TAG, "USB host error while device connected - releasing all inputs");
                release_all_inputs();
            }

            // Add a longer delay to prevent busy loop on error and allow recovery
            vTaskDelay(pdMS_TO_TICKS(500));
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

        // Device timeout detection - if we have a device but haven't received data in a while,
        // assume it's unresponsive and release all inputs
        static uint32_t inactive_count = 0;
        static const uint32_t DEVICE_INACTIVITY_THRESHOLD = 500; // ~0.5 seconds

        if (hid_device.connected) {
            inactive_count++;

            if (inactive_count >= DEVICE_INACTIVITY_THRESHOLD) {
                // Check if device is still responsive by attempting a control transfer
                // for keyboard LED state (harmless if it succeeds, will error if device is gone)
                if (hid_device.params.proto == HID_PROTOCOL_KEYBOARD) {
                    esp_err_t dev_check = usb_keyboard_set_led_state(0); // Send dummy LED state command

                    if (dev_check != ESP_OK) {
                        ESP_LOGW(TAG, "USB device timed out or unresponsive - releasing all inputs");
                        release_all_inputs();

                        // Don't mark device as disconnected yet, let the normal error handling do that
                        // when disconnect is actually detected
                    }
                }

                inactive_count = 0;  // Reset counter after check
            }
        } else {
            inactive_count = 0;  // Reset when no device is connected
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

    // Add delay to allow USB devices to settle after power-on
    vTaskDelay(pdMS_TO_TICKS(100));

    // Create event queue
    usb_host_event_queue = xQueueCreate(10, sizeof(usb_host_event_queue_t));
    if (usb_host_event_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create USB host event queue");
        return ESP_ERR_NO_MEM;
    }

    // Initialize the USB Host Library with robust configuration
    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
        .enum_filter_cb = usb_enum_filter_cb,  // Use our enumeration filter
    };

    esp_err_t ret = usb_host_install(&host_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install USB host: %s", esp_err_to_name(ret));
        vQueueDelete(usb_host_event_queue);
        usb_host_event_queue = NULL;
        return ret;
    }

    // Create the USB host task with higher priority for better responsiveness
    BaseType_t task_ret = xTaskCreate(
        usb_host_task,
        "usb_host",
        4096,
        NULL,
        8, // Higher priority (was 5) for better USB responsiveness
        &usb_host_task_handle
    );

    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create USB host task");
        usb_host_uninstall();
        vQueueDelete(usb_host_event_queue);
        usb_host_event_queue = NULL;
        return ESP_FAIL;
    }

    // Allow USB host stack to settle before installing HID driver
    vTaskDelay(pdMS_TO_TICKS(50));

    // Install HID Host driver with robust configuration
    const hid_host_driver_config_t hid_host_driver_config = {
        .create_background_task = true,  // Let the HID driver handle its own task
        .task_priority = 7,  // Higher priority (was 5) for better HID responsiveness
        .stack_size = 6144,  // Increase stack size for robustness
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

    // Allow extended time for initial device enumeration
    ESP_LOGI(TAG, "Waiting for initial device enumeration...");
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Check if enumeration succeeded, if not attempt immediate retry
    int immediate_retry_count = 0;
    const int MAX_IMMEDIATE_RETRIES = 2;

    while (immediate_retry_count < MAX_IMMEDIATE_RETRIES && !hid_device.connected) {
        ESP_LOGW(TAG, "No devices enumerated, attempting immediate retry %d/%d",
                 immediate_retry_count + 1, MAX_IMMEDIATE_RETRIES);

        // Only use soft retries to avoid breaking hot-plug functionality
        esp_err_t retry_ret = reinitialize_usb_host();

        if (retry_ret == ESP_OK) {
            // Wait to see if this helped
            vTaskDelay(pdMS_TO_TICKS(1500));
        }

        immediate_retry_count++;
    }

    // Start the USB enumeration monitor task for ongoing cold-plugged device recovery
    BaseType_t monitor_task_ret = xTaskCreate(
        usb_enumeration_monitor_task,
        "usb_enum_monitor",
        4096,
        NULL,
        3, // Lower priority than main USB task
        NULL
    );

    if (monitor_task_ret != pdPASS) {
        ESP_LOGW(TAG, "Failed to create USB enumeration monitor task, continuing without retry capability");
    } else {
        ESP_LOGI(TAG, "USB enumeration monitor task started for ongoing device recovery");
    }

    if (hid_device.connected) {
        ESP_LOGI(TAG, "USB host initialized successfully - device connected");
    } else {
        ESP_LOGI(TAG, "USB host initialized - no devices detected yet (monitor active for hot-plug)");
    }

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

esp_err_t usb_host_send_keyboard_output_report(uint8_t led_state)
{
    ESP_LOGI(TAG, "BLE->USB LED: Forwarding LED command from BLE host");
    // Simply forward to the unified USB LED state function
    return usb_keyboard_set_led_state(led_state);
}

/**
 * @brief Handle consumer control usage code
 *
 * Sends the consumer control event to the BLE HID stack using the consumer control
 * report descriptor (Report ID 3). Also forwards via UART protocol when
 * this board is active and input routing is on.
 *
 * @param usage_code Consumer control usage code from USB HID report
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
static esp_err_t handle_consumer_control(uint16_t usage_code)
{
    ESP_LOGI(TAG, "Consumer control code 0x%04x forwarding to BLE HID", usage_code);

    // 1. Forward to the BLE HID stack (will handle standard HID consumer control report)
    esp_err_t ble_ret = ble_hid_consumer_control(usage_code);
    if (ble_ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send consumer control to BLE: %s", esp_err_to_name(ble_ret));
    }

    // 2. Forward to UART for the neighbor ESP32 module
    esp_err_t uart_ret = uart_protocol_forward_consumer_control(usage_code);
    if (uart_ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to forward consumer control to UART: %s", esp_err_to_name(uart_ret));
    }

    return (ble_ret == ESP_OK) ? ESP_OK : ble_ret; // Prioritize BLE status for return
}
