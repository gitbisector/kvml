#include "ble_hid_mouse.h"
#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "host/ble_hs.h"
#include "host/ble_gatt.h"
#include "services/gatt/ble_svc_gatt.h"
#include "host/ble_uuid.h"
#include "ble_hid_device.h"
#include "ble_hid_keyboard.h"

// HID Service and characteristic UUIDs
#define BLE_SVC_HID_UUID16                           0x1812
#define BLE_SVC_HID_CHR_REPORT_MAP_UUID16           0x2A4B
#define BLE_SVC_HID_CHR_REPORT_UUID16               0x2A4D
#define BLE_SVC_HID_CHR_PROTOCOL_MODE_UUID16        0x2A4E
#define BLE_SVC_HID_CHR_CONTROL_POINT_UUID16        0x2A4C
#define BLE_SVC_HID_CHR_INFO_UUID16                 0x2A4A
#define BLE_SVC_HID_CHR_BOOT_MOUSE_INPUT_UUID16     0x2A33
#define BLE_SVC_HID_CHR_BOOT_KEYBOARD_INPUT_UUID16  0x2A22
#define BLE_SVC_HID_CHR_BOOT_KEYBOARD_OUTPUT_UUID16 0x2A32
#define BLE_SVC_HID_CHR_REPORT_REFERENCE_UUID16     0x2908


// HID Report IDs
#define HID_REPORT_ID_MOUSE                         0x01
#define HID_REPORT_ID_KEYBOARD                       0x02

// Battery Service UUIDs
#define BLE_SVC_BAS_UUID16                        0x180F
#define BLE_SVC_BAS_CHR_BATTERY_LEVEL_UUID16      0x2A19

// Device Information Service UUIDs
#define BLE_SVC_DIS_UUID16                        0x180A
#define BLE_SVC_DIS_CHR_MANUFACTURER_NAME_UUID16  0x2A29
#define BLE_SVC_DIS_CHR_MODEL_NUMBER_UUID16       0x2A24
#define BLE_SVC_DIS_CHR_PNP_ID_UUID16             0x2A50

#define TAG "BLE_HID_MOUSE"

// Rate limiting configuration
#define MIN_SEND_INTERVAL_US 1000  // Minimum interval between sends (1ms = 1000Hz max)

// Movement accumulator state
typedef struct {
    int16_t delta_x;          // Accumulated X movement (can overflow int8_t)
    int16_t delta_y;          // Accumulated Y movement (can overflow int8_t)
    int8_t scroll_y;          // Accumulated scroll movement
    uint8_t buttons;          // Current button state
    bool pending_report;      // True if a BLE notification is in flight
    bool has_accumulated;     // True if we have movement to send
    esp_timer_handle_t timer; // Timer for delayed sending
    uint32_t timer_interval_us; // Adaptive timer interval
    uint32_t failed_sends;    // Count of consecutive send failures

    // Rate limiting and tracking
    uint64_t last_send_time_us; // Timestamp of last successful send (microseconds)
    uint32_t min_interval_us;   // Minimum interval between sends (adaptive)
    uint32_t reports_sent;      // Total reports sent successfully
    uint32_t reports_failed;    // Total reports that failed to send
    uint32_t reports_rate_limited; // Reports dropped due to rate limiting
} mouse_accumulator_t;

// Global accumulator state
static mouse_accumulator_t mouse_acc = {
    .timer_interval_us = 8000,           // Start with 8ms (~125Hz) - twice as fast to learn quickly
    .min_interval_us = MIN_SEND_INTERVAL_US,  // Start with minimum interval
    .failed_sends = 0,
    .last_send_time_us = 0,
};

// Forward declarations for timer functions
static void movement_timer_callback(void* arg);
static esp_err_t init_movement_timer(void);
static void start_movement_timer(void);
static void stop_movement_timer(void);
static esp_err_t send_accumulated_movement(bool bypass_rate_limit);

// Rate limiting functions
static bool can_send_now(void);
static void update_rate_limit_on_success(void);
static void update_rate_limit_on_failure(void);

// Combined Mouse+Keyboard+Consumer Control HID Report Descriptor
static const uint8_t combined_hid_report_map[] = {
    // Mouse Collection - Report ID 1
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x02,        // Usage (Mouse)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x01,        //   Report ID (1)
    0x09, 0x01,        //   Usage (Pointer)
    0xA1, 0x00,        //   Collection (Physical)
    0x05, 0x09,        //     Usage Page (Button)
    0x19, 0x01,        //     Usage Minimum (1)
    0x29, 0x05,        //     Usage Maximum (5) - Support buttons 1-5
    0x15, 0x00,        //     Logical Minimum (0)
    0x25, 0x01,        //     Logical Maximum (1)
    0x95, 0x05,        //     Report Count (5) - 5 buttons
    0x75, 0x01,        //     Report Size (1)
    0x81, 0x02,        //     Input (Data, Variable, Absolute)
    0x95, 0x01,        //     Report Count (1)
    0x75, 0x03,        //     Report Size (3) - Only 3 padding bits needed
    0x81, 0x01,        //     Input (Constant)
    0x05, 0x01,        //     Usage Page (Generic Desktop)
    0x09, 0x30,        //     Usage (X)
    0x09, 0x31,        //     Usage (Y)
    0x09, 0x38,        //     Usage (Wheel)
    0x15, 0x81,        //     Logical Minimum (-127)
    0x25, 0x7F,        //     Logical Maximum (127)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x03,        //     Report Count (3)
    0x81, 0x06,        //     Input (Data, Variable, Relative)
    0xC0,              //   End Collection
    0xC0,              // End Collection

    // Keyboard Collection - Report ID 2
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x06,        // Usage (Keyboard)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x02,        //   Report ID (2)
    0x05, 0x07,        //   Usage Page (Kbrd/Keypad)
    0x19, 0xE0,        //   Usage Minimum (0xE0)
    0x29, 0xE7,        //   Usage Maximum (0xE7)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x08,        //   Report Count (8)
    0x81, 0x02,        //   Input (Data,Var,Abs)
    0x95, 0x01,        //   Report Count (1)
    0x75, 0x08,        //   Report Size (8)
    0x81, 0x01,        //   Input (Const,Array,Abs)
    0x95, 0x06,        //   Report Count (6)
    0x75, 0x08,        //   Report Size (8)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x65,        //   Logical Maximum (101)
    0x05, 0x07,        //   Usage Page (Kbrd/Keypad)
    0x19, 0x00,        //   Usage Minimum (0x00)
    0x29, 0x65,        //   Usage Maximum (0x65)
    0x81, 0x00,        //   Input (Data,Array,Abs)

    // Keyboard LED Output Report
    0x05, 0x08,        //   Usage Page (LEDs)
    0x19, 0x01,        //   Usage Minimum (Num Lock)
    0x29, 0x05,        //   Usage Maximum (Kana)
    0x95, 0x05,        //   Report Count (5)
    0x75, 0x01,        //   Report Size (1)
    0x91, 0x02,        //   Output (Data,Var,Abs)
    0x95, 0x01,        //   Report Count (1)
    0x75, 0x03,        //   Report Size (3) - Padding
    0x91, 0x01,        //   Output (Const,Array,Abs)
    0xC0,              // End Collection

    // Consumer Control Collection - Report ID 3
    0x05, 0x0C,        // Usage Page (Consumer)
    0x09, 0x01,        // Usage (Consumer Control)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x03,        //   Report ID (3)
    0x15, 0x00,        //   Logical Min 0
    0x26, 0xFF, 0x03,  //   Logical Max 0x03FF (10-bit)
    0x19, 0x00,        //   Usage Min 0
    0x2A, 0xFF, 0x03,  //   Usage Max 0x03FF
    0x75, 0x10,        //   Report Size 16 bits
    0x95, 0x01,        //   Report Count 1
    0x81, 0x00,        //   Input (Data,Array,Abs)
    0xC0,              // End Collection
};


// HID information characteristic value
static const uint8_t hid_info[] = {
    0x11, 0x01,     // bcdHID (HID version 1.11) - Note: little endian order
    0x00,           // bCountryCode (not localized)
    0x03,           // Flags (0x03 = normally connectable AND remote wake supported)
};

// HID Control Point characteristic (write only)
static uint8_t hid_control_point = 0;

// Protocol Mode Characteristic (read/write without response)
static uint8_t protocol_mode = 0x01;   // Report Protocol Mode (BLE HID report format fixed)

// Battery level value (read/notify) - Start with 100%
static uint8_t battery_level = 100;

// Device Information Service values
static const char *manufacturer_name = "KVML";
static const char *model_number = "KVML-ESP32S3";
static const uint8_t pnp_id[] = {
    0x02,        // Vendor ID Source (USB Implementer's Forum)
    0x5E, 0x04,  // Vendor ID (Microsoft Corporation - for compatibility)
    0x83, 0x00,  // Product ID (Mouse device)
    0x01, 0x00   // Product Version
};

// Global handle array definition - must be global for NimBLE handle assignment
uint16_t ble_hid_svc_char_handles[HANDLE_BLE_HID_COUNT] = {0};

// Mouse report data with Report ID
static uint8_t mouse_input_report_data[5] = {0, 0, 0, 0, 0};  // report_id, buttons, X, Y, wheel

// Keyboard report data with Report ID
static uint8_t keyboard_input_report_data[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};  // report_id, modifier, reserved, key1-6

// Keyboard output report data with Report ID (for LED control)
static uint8_t keyboard_output_report_data[2] = {0, 0};  // report_id, led_state

// Consumer control report data - 16-bit usage code
static uint8_t consumer_input_report_data[2] = {0x00, 0x00};  // report_id, usage_code (16-bit)

// Boot Protocol mouse report data (same format as regular mouse report)
static uint8_t boot_mouse_input_report[4] = {0, 0, 0, 0};   // buttons, X, Y, wheel

// Boot Protocol keyboard report data
static uint8_t boot_keyboard_input_report[8] = {0, 0, 0, 0, 0, 0, 0, 0};   // modifier, reserved, key1-6

// Report Reference Descriptors for Report characteristics
static const uint8_t mouse_report_reference[] = {0x01, 0x01};       // Report ID 1 (Input)
static const uint8_t keyboard_report_reference[] = {0x02, 0x01};   // Report ID 2 (Input)
static const uint8_t keyboard_output_report_reference[] = {0x02, 0x02};  // Report ID 2 (Output)
static const uint8_t consumer_report_reference[] = {0x03, 0x01};  // Report ID 3 (Input)

/**
 * Combined HID Report Map access callback
 */
static int combined_hid_report_map_access(uint16_t conn_handle, uint16_t attr_handle,
                                        struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            ESP_LOGD(TAG, "Read combined HID report map descriptor");
            rc = os_mbuf_append(ctxt->om, combined_hid_report_map, sizeof(combined_hid_report_map));
            return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}


/**
 * Mouse HID Report access callback
 */
static int mouse_hid_report_access(uint16_t conn_handle, uint16_t attr_handle,
                                 struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            ESP_LOGD(TAG, "Read mouse input report");
            rc = os_mbuf_append(ctxt->om, mouse_input_report_data, sizeof(mouse_input_report_data));
            return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}

/**
 * Keyboard HID Report access callback
 */
static int keyboard_hid_report_access(uint16_t conn_handle, uint16_t attr_handle,
                                    struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            ESP_LOGD(TAG, "Read keyboard input report");
            rc = os_mbuf_append(ctxt->om, keyboard_input_report_data, sizeof(keyboard_input_report_data));
            return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}

/**
 * Keyboard Output Report access callback (for LED control)
 */
static int keyboard_output_report_access(uint16_t conn_handle, uint16_t attr_handle,
                                       struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            ESP_LOGD(TAG, "Read keyboard output report");
            rc = os_mbuf_append(ctxt->om, keyboard_output_report_data, sizeof(keyboard_output_report_data));
            return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;

        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            ESP_LOGI(TAG, "Write keyboard output report from conn_handle=0x%04x", conn_handle);
            ESP_LOGI(TAG, "MBUF length: %d bytes", OS_MBUF_PKTLEN(ctxt->om));

            // Get the length of the incoming data
            int data_len = OS_MBUF_PKTLEN(ctxt->om);
            ESP_LOGI(TAG, "Incoming data length: %d bytes", data_len);

            if (data_len == 1) {
                // BLE HID style: no Report ID in data (implied by characteristic)
                uint8_t led_state;
                rc = ble_hs_mbuf_to_flat(ctxt->om, &led_state, 1, NULL);
                if (rc != 0) {
                    ESP_LOGE(TAG, "Failed to extract 1-byte LED state: %d", rc);
                    return BLE_ATT_ERR_INSUFFICIENT_RES;
                }

                ESP_LOGI(TAG, "BLE HID output (1 byte): led_state=0x%02x", led_state);
                ESP_LOGI(TAG, "LED state bits: 7:%d 6:%d 5:%d 4:%d 3:%d 2:%d 1:%d 0:%d",
                         (led_state >> 7) & 1, (led_state >> 6) & 1, (led_state >> 5) & 1, (led_state >> 4) & 1,
                         (led_state >> 3) & 1, (led_state >> 2) & 1, (led_state >> 1) & 1, (led_state >> 0) & 1);

                // Update our local copy (no Report ID)
                keyboard_output_report_data[0] = HID_REPORT_ID_KEYBOARD; // Set Report ID
                keyboard_output_report_data[1] = led_state;

                // Forward to keyboard module
                esp_err_t err = ble_hid_keyboard_handle_output_report(conn_handle, &led_state, 1);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to handle keyboard output report: %s", esp_err_to_name(err));
                    return BLE_ATT_ERR_UNLIKELY;
                }

            } else if (data_len == 2) {
                // USB HID style: Report ID + data
                uint8_t report_data[2];
                rc = ble_hs_mbuf_to_flat(ctxt->om, report_data, sizeof(report_data), NULL);
                if (rc != 0) {
                    ESP_LOGE(TAG, "Failed to extract 2-byte report data: %d", rc);
                    return BLE_ATT_ERR_INSUFFICIENT_RES;
                }

                ESP_LOGI(TAG, "USB HID output (2 bytes): report_id=%d, led_state=0x%02x",
                         report_data[0], report_data[1]);
                ESP_LOGI(TAG, "LED state bits: 7:%d 6:%d 5:%d 4:%d 3:%d 2:%d 1:%d 0:%d",
                         (report_data[1] >> 7) & 1, (report_data[1] >> 6) & 1, (report_data[1] >> 5) & 1, (report_data[1] >> 4) & 1,
                         (report_data[1] >> 3) & 1, (report_data[1] >> 2) & 1, (report_data[1] >> 1) & 1, (report_data[1] >> 0) & 1);

                // Update our local copy
                memcpy(keyboard_output_report_data, report_data, sizeof(report_data));

                // Forward to keyboard module
                esp_err_t err = ble_hid_keyboard_handle_output_report(conn_handle, &report_data[1], 1);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to handle keyboard output report: %s", esp_err_to_name(err));
                    return BLE_ATT_ERR_UNLIKELY;
                }

            } else {
                ESP_LOGE(TAG, "Invalid output report length: %d (expected 1 or 2)", data_len);
                return BLE_ATT_ERR_INVALID_PDU;
            }

            return 0;

        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}

/**
 * HID Protocol Mode access callback
 */
static int hid_proto_mode_access(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;
    uint8_t old_mode;
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            rc = os_mbuf_append(ctxt->om, &protocol_mode, sizeof(protocol_mode));
            ESP_LOGI(TAG, "Protocol mode read: %s (%d)",
                     protocol_mode == PROTOCOL_MODE_BOOT ? "BOOT" : "REPORT", protocol_mode);
            return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            old_mode = protocol_mode;
            rc = ble_hs_mbuf_to_flat(ctxt->om, &protocol_mode, sizeof(protocol_mode), NULL);

            if (old_mode != protocol_mode) {
                ESP_LOGI(TAG, "Protocol mode changed from %s (%d) to %s (%d)",
                         old_mode == PROTOCOL_MODE_BOOT ? "BOOT" : "REPORT", old_mode,
                         protocol_mode == PROTOCOL_MODE_BOOT ? "BOOT" : "REPORT", protocol_mode);
            }

            return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}

/**
 * HID Control Point access callback
 */
static int hid_ctrl_point_access(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            rc = ble_hs_mbuf_to_flat(ctxt->om, &hid_control_point, sizeof(hid_control_point), NULL);
            return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}

/**
 * HID Information access callback
 */
static int hid_info_access(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            rc = os_mbuf_append(ctxt->om, hid_info, sizeof(hid_info));
            return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}

/**
 * Battery Level access callback
 */
static int battery_level_access(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            rc = os_mbuf_append(ctxt->om, &battery_level, sizeof(battery_level));
            return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}

/**
 * Boot Mouse Input Report access callback
 */
static int boot_mouse_input_access(uint16_t conn_handle, uint16_t attr_handle,
                                   struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            ESP_LOGD(TAG, "Read boot mouse input report");
            rc = os_mbuf_append(ctxt->om, boot_mouse_input_report, sizeof(boot_mouse_input_report));
            return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}

/**
 * Boot Keyboard Input Report access callback
 */
static int boot_keyboard_input_access(uint16_t conn_handle, uint16_t attr_handle,
                                     struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            ESP_LOGD(TAG, "Read boot keyboard input report");
            rc = os_mbuf_append(ctxt->om, boot_keyboard_input_report, sizeof(boot_keyboard_input_report));
            return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}

/**
 * Mouse Report Reference Descriptor access callback
 */
static int mouse_report_reference_access(uint16_t conn_handle, uint16_t attr_handle,
                                        struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_DSC:
            ESP_LOGD(TAG, "Read mouse report reference descriptor");
            rc = os_mbuf_append(ctxt->om, mouse_report_reference, sizeof(mouse_report_reference));
            return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}

/**
 * Keyboard Report Reference Descriptor access callback
 */
static int keyboard_report_reference_access(uint16_t conn_handle, uint16_t attr_handle,
                                           struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_DSC:
            ESP_LOGD(TAG, "Read keyboard report reference descriptor");
            rc = os_mbuf_append(ctxt->om, keyboard_report_reference, sizeof(keyboard_report_reference));
            return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}

/**
 * Keyboard Output Report Reference Descriptor access callback
 */
static int keyboard_output_report_reference_access(uint16_t conn_handle, uint16_t attr_handle,
                                                   struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_DSC:
            ESP_LOGD(TAG, "Read keyboard output report reference descriptor");
            rc = os_mbuf_append(ctxt->om, keyboard_output_report_reference,
                               sizeof(keyboard_output_report_reference));
            return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}

/**
 * Consumer Control Report Reference Descriptor access callback
 */
static int consumer_report_reference_access(uint16_t conn_handle, uint16_t attr_handle,
                                           struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_DSC:
            ESP_LOGD(TAG, "Read consumer control report reference descriptor");
            rc = os_mbuf_append(ctxt->om, consumer_report_reference,
                               sizeof(consumer_report_reference));
            return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}


/**
 * Device Information Service access callback
 */
static int dis_access(uint16_t conn_handle, uint16_t attr_handle,
                     struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    uint16_t uuid = ble_uuid_u16(ctxt->chr->uuid);
    int rc;

    switch (uuid) {
        case BLE_SVC_DIS_CHR_MANUFACTURER_NAME_UUID16:
            if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
                rc = os_mbuf_append(ctxt->om, manufacturer_name, strlen(manufacturer_name));
                return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            break;
        case BLE_SVC_DIS_CHR_MODEL_NUMBER_UUID16:
            if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
                rc = os_mbuf_append(ctxt->om, model_number, strlen(model_number));
                return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            break;
        case BLE_SVC_DIS_CHR_PNP_ID_UUID16:
            if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
                rc = os_mbuf_append(ctxt->om, pnp_id, sizeof(pnp_id));
                return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            break;
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
    return BLE_ATT_ERR_UNLIKELY;
}


/**
 * Define service UUIDs
 */
static const ble_uuid16_t hid_report_map_uuid = BLE_UUID16_INIT(BLE_SVC_HID_CHR_REPORT_MAP_UUID16);
static const ble_uuid16_t hid_report_uuid = BLE_UUID16_INIT(BLE_SVC_HID_CHR_REPORT_UUID16);
static const ble_uuid16_t hid_proto_mode_uuid = BLE_UUID16_INIT(BLE_SVC_HID_CHR_PROTOCOL_MODE_UUID16);
static const ble_uuid16_t hid_control_point_uuid = BLE_UUID16_INIT(BLE_SVC_HID_CHR_CONTROL_POINT_UUID16);
static const ble_uuid16_t hid_info_uuid = BLE_UUID16_INIT(BLE_SVC_HID_CHR_INFO_UUID16);
static const ble_uuid16_t hid_svc_uuid = BLE_UUID16_INIT(BLE_SVC_HID_UUID16);
// Boot Protocol UUIDs
static const ble_uuid16_t boot_mouse_input_uuid = BLE_UUID16_INIT(BLE_SVC_HID_CHR_BOOT_MOUSE_INPUT_UUID16);
static const ble_uuid16_t boot_keyboard_input_uuid = BLE_UUID16_INIT(BLE_SVC_HID_CHR_BOOT_KEYBOARD_INPUT_UUID16);

// Report Reference Descriptor UUID
static const ble_uuid16_t report_reference_uuid = BLE_UUID16_INIT(BLE_SVC_HID_CHR_REPORT_REFERENCE_UUID16);

// Battery Service UUIDs
static const ble_uuid16_t battery_svc_uuid = BLE_UUID16_INIT(BLE_SVC_BAS_UUID16);
static const ble_uuid16_t battery_level_uuid = BLE_UUID16_INIT(BLE_SVC_BAS_CHR_BATTERY_LEVEL_UUID16);

// Device Information Service UUIDs
#define BLE_SVC_DIS_UUID16                        0x180A
#define BLE_SVC_DIS_CHR_MANUFACTURER_NAME_UUID16  0x2A29
#define BLE_SVC_DIS_CHR_MODEL_NUMBER_UUID16       0x2A24
#define BLE_SVC_DIS_CHR_PNP_ID_UUID16             0x2A50

static const ble_uuid16_t dis_svc_uuid = BLE_UUID16_INIT(BLE_SVC_DIS_UUID16);
static const ble_uuid16_t dis_manufacturer_uuid = BLE_UUID16_INIT(BLE_SVC_DIS_CHR_MANUFACTURER_NAME_UUID16);
static const ble_uuid16_t dis_model_uuid = BLE_UUID16_INIT(BLE_SVC_DIS_CHR_MODEL_NUMBER_UUID16);
static const ble_uuid16_t dis_pnp_uuid = BLE_UUID16_INIT(BLE_SVC_DIS_CHR_PNP_ID_UUID16);


/**
 * Define Combined Mouse+Keyboard HID Service for Ubuntu compatibility
 */
static const struct ble_gatt_svc_def combined_hid_svcs[] = {
    {
        // HID Service - primary service for mouse and keyboard functionality
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &hid_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                // Combined HID Report Map characteristic
                .uuid = &hid_report_map_uuid.u,
                .access_cb = combined_hid_report_map_access,
                .flags = BLE_GATT_CHR_F_READ,
                .val_handle = &ble_hid_svc_char_handles[HANDLE_HID_REPORT_MAP],
            },
            {
                // Mouse Input Report characteristic
                .uuid = &hid_report_uuid.u,
                .access_cb = mouse_hid_report_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &ble_hid_svc_char_handles[HANDLE_MOUSE_INPUT_REPORT],
                .descriptors = (struct ble_gatt_dsc_def[]) {
                    {
                        // Report Reference Descriptor for Mouse
                        .uuid = &report_reference_uuid.u,
                        .access_cb = mouse_report_reference_access,
                        .att_flags = BLE_ATT_F_READ,
                    },
                    {
                        0, // No more descriptors
                    },
                },
            },
            {
                // Keyboard Input Report characteristic
                .uuid = &hid_report_uuid.u,
                .access_cb = keyboard_hid_report_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &ble_hid_svc_char_handles[HANDLE_KEYBOARD_INPUT_REPORT],
                .descriptors = (struct ble_gatt_dsc_def[]) {
                    {
                        // Report Reference Descriptor for Keyboard
                        .uuid = &report_reference_uuid.u,
                        .access_cb = keyboard_report_reference_access,
                        .att_flags = BLE_ATT_F_READ,
                    },
                    {
                        0, // No more descriptors
                    },
                },
            },
            {
                // HID Protocol Mode characteristic
                .uuid = &hid_proto_mode_uuid.u,
                .access_cb = hid_proto_mode_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE_NO_RSP,
                .val_handle = &ble_hid_svc_char_handles[HANDLE_HID_PROTOCOL_MODE],
            },
            {
                // HID Control Point characteristic
                .uuid = &hid_control_point_uuid.u,
                .access_cb = hid_ctrl_point_access,
                .flags = BLE_GATT_CHR_F_WRITE_NO_RSP,
                .val_handle = &ble_hid_svc_char_handles[HANDLE_HID_CONTROL_POINT],
            },
            {
                // HID Information characteristic
                .uuid = &hid_info_uuid.u,
                .access_cb = hid_info_access,
                .flags = BLE_GATT_CHR_F_READ,
                .val_handle = &ble_hid_svc_char_handles[HANDLE_HID_INFO],
            },
            {
                // Boot Mouse Input Report characteristic - critical for Ubuntu mouse device categorization
                .uuid = &boot_mouse_input_uuid.u,
                .access_cb = boot_mouse_input_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &ble_hid_svc_char_handles[HANDLE_BOOT_MOUSE_INPUT_REPORT],
            },
            {
                // Boot Keyboard Input Report characteristic - for Ubuntu keyboard device categorization
                .uuid = &boot_keyboard_input_uuid.u,
                .access_cb = boot_keyboard_input_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &ble_hid_svc_char_handles[HANDLE_BOOT_KEYBOARD_INPUT_REPORT],
            },
            {
                // Keyboard Output Report characteristic (for LED control)
                .uuid = &hid_report_uuid.u,
                .access_cb = keyboard_output_report_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
                .val_handle = &ble_hid_svc_char_handles[HANDLE_KEYBOARD_OUTPUT_REPORT],
                .descriptors = (struct ble_gatt_dsc_def[]) {
                    {
                        // Report Reference Descriptor for Keyboard Output
                        .uuid = &report_reference_uuid.u,
                        .access_cb = keyboard_output_report_reference_access,
                        .att_flags = BLE_ATT_F_READ,
                    },
                    {
                        0, // No more descriptors
                    },
                },
            },
            {
                // Consumer Control Input Report characteristic
                .uuid = &hid_report_uuid.u,
                .access_cb = mouse_hid_report_access, // Reuse access function since it's just a pass-through
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &ble_hid_svc_char_handles[HANDLE_CONSUMER_INPUT_REPORT],
                .descriptors = (struct ble_gatt_dsc_def[]) {
                    {
                        // Report Reference Descriptor for Consumer Control
                        .uuid = &report_reference_uuid.u,
                        .access_cb = consumer_report_reference_access,
                        .att_flags = BLE_ATT_F_READ,
                    },
                    {
                        0, // No more descriptors
                    },
                },
            },
            {
                0, // No more characteristics
            },
        },
    },
    {
        // Battery Service (required for many HID hosts)
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &battery_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                // Battery Level Characteristic
                .uuid = &battery_level_uuid.u,
                .access_cb = battery_level_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &ble_hid_svc_char_handles[HANDLE_BATTERY_LEVEL],
            },
            {
                0, // No more characteristics
            },
        },
    },
    {
        // Device Information Service (required for proper device categorization)
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &dis_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                // Manufacturer Name String
                .uuid = &dis_manufacturer_uuid.u,
                .access_cb = dis_access,
                .flags = BLE_GATT_CHR_F_READ,
                .val_handle = &ble_hid_svc_char_handles[HANDLE_DIS_MANUFACTURER_NAME],
            },
            {
                // Model Number String
                .uuid = &dis_model_uuid.u,
                .access_cb = dis_access,
                .flags = BLE_GATT_CHR_F_READ,
                .val_handle = &ble_hid_svc_char_handles[HANDLE_DIS_MODEL_NUMBER],
            },
            {
                // PnP ID
                .uuid = &dis_pnp_uuid.u,
                .access_cb = dis_access,
                .flags = BLE_GATT_CHR_F_READ,
                .val_handle = &ble_hid_svc_char_handles[HANDLE_DIS_PNP_ID],
            },
            {
                0, // No more characteristics
            },
        },
    },
    {
        0, // No more services
    },
};

esp_err_t ble_hid_mouse_init(void)
{
    ESP_LOGI(TAG, "Initializing BLE HID Combined Mouse+Keyboard device for Ubuntu compatibility");

    // Initialize handle array to 0 - matching working example pattern
    memset(&ble_hid_svc_char_handles, 0, sizeof(ble_hid_svc_char_handles[0]) * HANDLE_BLE_HID_COUNT);

    // Register the combined mouse+keyboard HID service
    int rc = ble_gatts_count_cfg(combined_hid_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to count GATT service attributes: %d", rc);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "GATT service count successful, adding combined mouse+keyboard services...");

    rc = ble_gatts_add_svcs(combined_hid_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to register GATT services: %d", rc);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "GATT services registration complete with return code: %d", rc);

    // Start the GATT server to assign handles
    rc = ble_gatts_start();
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to start GATT server: %d", rc);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "GATT server started successfully");

    // Log handle array address for debugging
    ESP_LOGI(TAG, "Handle array address: %p", ble_hid_svc_char_handles);
    ESP_LOGI(TAG, "Handle array size: %d", HANDLE_BLE_HID_COUNT);

    ESP_LOGI(TAG, "GATT Combined Mouse+Keyboard Service handles assigned:");
    ESP_LOGI(TAG, "  Combined Report Map handle: 0x%04x", ble_hid_svc_char_handles[HANDLE_HID_REPORT_MAP]);
    ESP_LOGI(TAG, "  Mouse Input Report handle: 0x%04x", ble_hid_svc_char_handles[HANDLE_MOUSE_INPUT_REPORT]);
    ESP_LOGI(TAG, "  Keyboard Input Report handle: 0x%04x", ble_hid_svc_char_handles[HANDLE_KEYBOARD_INPUT_REPORT]);
    ESP_LOGI(TAG, "  Consumer Control Input Report handle: 0x%04x", ble_hid_svc_char_handles[HANDLE_CONSUMER_INPUT_REPORT]);
    ESP_LOGI(TAG, "  Protocol Mode handle: 0x%04x", ble_hid_svc_char_handles[HANDLE_HID_PROTOCOL_MODE]);
    ESP_LOGI(TAG, "  Control Point handle: 0x%04x", ble_hid_svc_char_handles[HANDLE_HID_CONTROL_POINT]);
    ESP_LOGI(TAG, "  Keyboard Output Report handle: 0x%04x", ble_hid_svc_char_handles[HANDLE_KEYBOARD_OUTPUT_REPORT]);
    ESP_LOGI(TAG, "  HID Info handle: 0x%04x", ble_hid_svc_char_handles[HANDLE_HID_INFO]);
    ESP_LOGI(TAG, "  Boot Mouse Input handle: 0x%04x", ble_hid_svc_char_handles[HANDLE_BOOT_MOUSE_INPUT_REPORT]);
    ESP_LOGI(TAG, "  Boot Keyboard Input handle: 0x%04x", ble_hid_svc_char_handles[HANDLE_BOOT_KEYBOARD_INPUT_REPORT]);
    ESP_LOGI(TAG, "  Battery Level handle: 0x%04x", ble_hid_svc_char_handles[HANDLE_BATTERY_LEVEL]);
    ESP_LOGI(TAG, "  DIS Manufacturer handle: 0x%04x", ble_hid_svc_char_handles[HANDLE_DIS_MANUFACTURER_NAME]);
    ESP_LOGI(TAG, "  DIS Model Number handle: 0x%04x", ble_hid_svc_char_handles[HANDLE_DIS_MODEL_NUMBER]);
    ESP_LOGI(TAG, "  DIS PnP ID handle: 0x%04x", ble_hid_svc_char_handles[HANDLE_DIS_PNP_ID]);

    if (ble_hid_svc_char_handles[HANDLE_MOUSE_INPUT_REPORT] == 0) {
        ESP_LOGE(TAG, "Critical: Mouse input report handle not assigned - BLE HID will not work");
        return ESP_FAIL;
    } else {
        ESP_LOGI(TAG, "Mouse input report handle successfully assigned: 0x%04x", ble_hid_svc_char_handles[HANDLE_MOUSE_INPUT_REPORT]);
    }

    // Initialize the movement accumulator timer
    esp_err_t timer_err = init_movement_timer();
    if (timer_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize movement timer: %d", timer_err);
        return timer_err;
    }

    // Initialize rate limiting
    mouse_acc.last_send_time_us = esp_timer_get_time();

    ESP_LOGI(TAG, "BLE HID Combined Mouse+Keyboard initialization complete");
    return ESP_OK;
}

void ble_hid_mouse_send_report(uint8_t conn_idx, const uint8_t *report, size_t len)
{
    if (!ble_hid_dev_state.connected) {
        ESP_LOGD(TAG, "Mouse: Not connected");
        return;
    }

    if (len > sizeof(mouse_input_report_data)) {
        ESP_LOGE(TAG, "Mouse report too large: %d > %d", (int)len, (int)sizeof(mouse_input_report_data));
        return;
    }

    // Check current protocol mode
    if (protocol_mode == PROTOCOL_MODE_REPORT) {
        // Report Protocol Mode - Send to Report characteristic
        if (ble_hid_svc_char_handles[HANDLE_MOUSE_INPUT_REPORT] == 0) {
            ESP_LOGW(TAG, "Mouse input report handle not initialized");
            return;
        }
        // BLE HID Report Protocol: send report data directly without Report ID
        // (GATT handle identifies the report type)
        if (len > 4) {
            ESP_LOGW(TAG, "Report too large for BLE HID: %d bytes (max 4)", (int)len);
            return;
        }

        // Send notification to connected host via Report characteristic
        struct os_mbuf *om = ble_hs_mbuf_from_flat(report, len);
        if (om != NULL) {
            int rc = ble_gattc_notify_custom(ble_hid_dev_state.conn_handle,
                                           ble_hid_svc_char_handles[HANDLE_MOUSE_INPUT_REPORT], om);
            if (rc != 0) {
                // Decode common error codes for better diagnostics
                const char *error_name = "UNKNOWN";
                switch (rc) {
                    case 3: error_name = "BLE_HS_ENOTCONN"; break;
                    case 8: error_name = "BLE_HS_ENOMEM"; break;
                    case 30: error_name = "BLE_HS_ENOTCONNFAIL"; break;
                    case 257: error_name = "BLE_HS_ATT_ERR_INVALID_HANDLE"; break;
                }
                ESP_LOGW(TAG, "Failed to send mouse HID notification: %d (%s), handle=0x%04x",
                         rc, error_name, ble_hid_svc_char_handles[HANDLE_MOUSE_INPUT_REPORT]);
            } else {
                // Track successful send and update rate limiting
                mouse_acc.reports_sent++;
                update_rate_limit_on_success();
                ESP_LOGD(TAG, "Sent mouse HID report notification to conn %d", conn_idx);
            }
        } else {
            ESP_LOGE(TAG, "Failed to allocate mbuf for mouse report");
        }
    } else if (protocol_mode == PROTOCOL_MODE_BOOT) {
        // Boot Protocol Mode - Use the boot_mouse_input_report function instead
        ble_hid_boot_mouse_send_report(0, report, len);
    } else {
        ESP_LOGW(TAG, "Unknown protocol mode: %d", protocol_mode);
    }
}

// Keyboard functionality moved to ble_hid_keyboard.c

// Mouse state tracking for high-level API
static uint8_t current_mouse_buttons = 0;

// Helper function to send mouse report to the active host
static esp_err_t send_mouse_report_to_active_host(int8_t delta_x, int8_t delta_y, uint8_t buttons, int8_t scroll_y) {
#if BLE_HID_DEBUG_DISABLE_REPORTING
    ESP_LOGI(TAG, "[DEBUG] BLE DISABLED - Mouse report: buttons=0x%02x, delta=(%d,%d), scroll=%d",
             buttons, delta_x, delta_y, scroll_y);
    current_mouse_buttons = buttons;
    return ESP_OK;
#else
    if (!ble_hid_is_connected()) {
        ESP_LOGD(TAG, "Not connected, skipping mouse report");
        return ESP_ERR_INVALID_STATE;
    }

    // Only send report if there's actual movement or button change to avoid spam
    if (delta_x == 0 && delta_y == 0 && buttons == current_mouse_buttons && scroll_y == 0) {
        return ESP_OK; // No change, no need to send
    }

    // Create mouse report: [buttons, X, Y, wheel] - BLE HID format (no Report ID)
    uint8_t report[4] = {
        buttons,     // Button state
        delta_x,     // X movement (signed cast)
        delta_y,     // Y movement (signed cast)
        scroll_y     // Wheel scroll (signed cast)
    };

    ESP_LOGD(TAG, "Sending mouse report: buttons=0x%02x, delta=(%d,%d), scroll=%d, protocol_mode=%d",
             buttons, delta_x, delta_y, scroll_y, protocol_mode);

    current_mouse_buttons = buttons;
    // Send 4-byte report for both protocol modes (BLE HID uses GATT handle to identify report type)
    ble_hid_mouse_send_report(0, report, sizeof(report));

    return ESP_OK;
#endif
}

esp_err_t ble_hid_mouse_move(int8_t delta_x, int8_t delta_y) {
    return send_mouse_report_to_active_host(delta_x, delta_y, current_mouse_buttons, 0);
}

esp_err_t ble_hid_mouse_scroll(int8_t scroll_y) {
    return send_mouse_report_to_active_host(0, 0, current_mouse_buttons, scroll_y);
}

esp_err_t ble_hid_mouse_report(int8_t delta_x, int8_t delta_y, uint8_t buttons, int8_t scroll_y) {
    return send_mouse_report_to_active_host(delta_x, delta_y, buttons, scroll_y);
}

uint8_t ble_hid_mouse_get_button_state(void) {
    return current_mouse_buttons;
}

/**
 * Send Boot Protocol Mouse report
 */
void ble_hid_boot_mouse_send_report(uint8_t conn_idx, const uint8_t *report, size_t len)
{
    if (!ble_hid_dev_state.connected) {
        ESP_LOGD(TAG, "Boot mouse: Not connected");
        return;
    }

    if (len > sizeof(boot_mouse_input_report)) {
        ESP_LOGE(TAG, "Boot mouse report too large: %d > %d", (int)len, (int)sizeof(boot_mouse_input_report));
        return;
    }

    if (ble_hid_svc_char_handles[HANDLE_BOOT_MOUSE_INPUT_REPORT] == 0) {
        ESP_LOGW(TAG, "Boot mouse input report handle not initialized");
        return;
    }

    // Copy report data (without Report ID for boot protocol)
    memcpy(boot_mouse_input_report, report, len);

    // Send notification
    struct os_mbuf *om = ble_hs_mbuf_from_flat(boot_mouse_input_report, sizeof(boot_mouse_input_report));
    if (om != NULL) {
        int rc = ble_gattc_notify_custom(ble_hid_dev_state.conn_handle,
                                         ble_hid_svc_char_handles[HANDLE_BOOT_MOUSE_INPUT_REPORT], om);
        if (rc != 0) {
            // Decode common error codes for better diagnostics
            const char *error_name = "UNKNOWN";
            switch (rc) {
                case 3: error_name = "BLE_HS_ENOTCONN"; break;
                case 8: error_name = "BLE_HS_ENOMEM"; break;
                case 30: error_name = "BLE_HS_ENOTCONNFAIL"; break;
                case 257: error_name = "BLE_HS_ATT_ERR_INVALID_HANDLE"; break;
            }
            ESP_LOGW(TAG, "Failed to send boot mouse notification to conn %d: %d (%s), handle=0x%04x",
                     conn_idx, rc, error_name, ble_hid_svc_char_handles[HANDLE_BOOT_MOUSE_INPUT_REPORT]);
        } else {
            ESP_LOGD(TAG, "Sent boot mouse report notification");
        }
    } else {
        ESP_LOGE(TAG, "Failed to allocate mbuf for boot mouse report");
    }
}

/**
 * Send Boot Protocol Keyboard report
 */
void ble_hid_boot_keyboard_send_report(uint8_t conn_idx, const uint8_t *report, size_t len)
{
    if (!ble_hid_dev_state.connected) {
        ESP_LOGD(TAG, "Boot keyboard: Not connected");
        return;
    }

    if (len > sizeof(boot_keyboard_input_report)) {
        ESP_LOGE(TAG, "Boot keyboard report too large: %d > %d", (int)len, (int)sizeof(boot_keyboard_input_report));
        return;
    }

    if (ble_hid_svc_char_handles[HANDLE_BOOT_KEYBOARD_INPUT_REPORT] == 0) {
        ESP_LOGW(TAG, "Boot keyboard input report handle not initialized");
        return;
    }

    // Copy report data (without Report ID for boot protocol)
    memcpy(boot_keyboard_input_report, report, len);

    // Send notification
    struct os_mbuf *om = ble_hs_mbuf_from_flat(boot_keyboard_input_report, sizeof(boot_keyboard_input_report));
    if (om != NULL) {
        int rc = ble_gattc_notify_custom(ble_hid_dev_state.conn_handle,
                                         ble_hid_svc_char_handles[HANDLE_BOOT_KEYBOARD_INPUT_REPORT], om);
        if (rc != 0) {
            // Decode common error codes for better diagnostics
            const char *error_name = "UNKNOWN";
            switch (rc) {
                case 3: error_name = "BLE_HS_ENOTCONN"; break;
                case 8: error_name = "BLE_HS_ENOMEM"; break;
                case 30: error_name = "BLE_HS_ENOTCONNFAIL"; break;
                case 257: error_name = "BLE_HS_ATT_ERR_INVALID_HANDLE"; break;
            }
            ESP_LOGW(TAG, "Failed to send boot keyboard notification to conn %d: %d (%s), handle=0x%04x",
                     conn_idx, rc, error_name, ble_hid_svc_char_handles[HANDLE_BOOT_KEYBOARD_INPUT_REPORT]);
        } else {
            ESP_LOGD(TAG, "Sent boot keyboard report notification");
        }
    } else {
        ESP_LOGE(TAG, "Failed to allocate mbuf for boot keyboard report");
    }
}

// ===== MOVEMENT ACCUMULATOR IMPLEMENTATION =====

/**
 * Timer callback to send accumulated movement
 */
static void movement_timer_callback(void* arg) {
    // Only send if we still have accumulation (might have been sent already)
    if (mouse_acc.has_accumulated) {
        ESP_LOGD(TAG, "Timer fired: sending accumulated movement dx=%d, dy=%d, scroll=%d",
                mouse_acc.delta_x, mouse_acc.delta_y, mouse_acc.scroll_y);
        send_accumulated_movement(false);
    } else {
        ESP_LOGD(TAG, "Timer fired but no accumulation (already sent)");
    }
}

/**
 * Initialize the movement timer (called during mouse init)
 */
static esp_err_t init_movement_timer(void) {
    if (mouse_acc.timer == NULL) {
        const esp_timer_create_args_t timer_args = {
            .callback = &movement_timer_callback,
            .name = "mouse_accumulator"
        };
        esp_err_t err = esp_timer_create(&timer_args, &mouse_acc.timer);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create movement timer: %d", err);
            return err;
        }
        ESP_LOGI(TAG, "Movement accumulator timer created");
    }
    return ESP_OK;
}

/**
 * Start or restart the movement timer
 */
static void start_movement_timer(void) {
    if (mouse_acc.timer == NULL) {
        ESP_LOGW(TAG, "Timer not initialized, skipping start");
        return;
    }

    // Stop any existing timer
    esp_timer_stop(mouse_acc.timer);

    // Use shorter interval for accumulated movement to prevent long delays
    uint32_t send_interval = mouse_acc.has_accumulated ?
                            (mouse_acc.timer_interval_us / 2) : mouse_acc.timer_interval_us;
    if (send_interval < MIN_SEND_INTERVAL_US) send_interval = MIN_SEND_INTERVAL_US; // Enforce minimum interval

    // Start timer with the calculated interval
    esp_err_t err = esp_timer_start_once(mouse_acc.timer, send_interval);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to start movement timer: %d", err);
    }
}

/**
 * Stop the movement timer
 */
static void stop_movement_timer(void) {
    if (mouse_acc.timer) {
        esp_timer_stop(mouse_acc.timer);
    }
}

/**
 * Adjust timer interval based on send success/failure rate
 */
static void adjust_timer_interval(void) {
    // Increase interval if we have many failures
    if (mouse_acc.failed_sends > 3) {
        // Increase interval when congested (slower sending)
        mouse_acc.timer_interval_us = (mouse_acc.timer_interval_us * 150) / 100; // 1.5x
        if (mouse_acc.timer_interval_us > 50000) { // Cap at 50ms (20Hz) - typical BLE HID minimum
            mouse_acc.timer_interval_us = 50000;
        }
        ESP_LOGI(TAG, "tiv to %lu us (failures: %lu)",
                mouse_acc.timer_interval_us, mouse_acc.failed_sends);
	mouse_acc.failed_sends = 0;
    } else if (mouse_acc.failed_sends == 0 && mouse_acc.timer_interval_us > 8000) {
        // Decrease interval when flowing well (faster sending)
        mouse_acc.timer_interval_us = (mouse_acc.timer_interval_us * 90) / 100; // 0.9x
        if (mouse_acc.timer_interval_us < 8000) { // Floor at 8ms (125Hz) - faster target
            mouse_acc.timer_interval_us = 8000;
        }
        ESP_LOGI(TAG, "tiv to %lu us", mouse_acc.timer_interval_us);
    }
}

/**
 * Accumulate movement with saturation
 */
static void accumulate_movement(int8_t delta_x, int8_t delta_y, int8_t scroll_y) {
    // Accumulate with saturation to prevent overflow
    int32_t new_x = (int32_t)mouse_acc.delta_x + delta_x;
    int32_t new_y = (int32_t)mouse_acc.delta_y + delta_y;
    int32_t new_scroll = (int32_t)mouse_acc.scroll_y + scroll_y;

    // Clamp to int8_t range for transmission
    mouse_acc.delta_x = (new_x < -127) ? -127 : ((new_x > 127) ? 127 : (int16_t)new_x);
    mouse_acc.delta_y = (new_y < -127) ? -127 : ((new_y > 127) ? 127 : (int16_t)new_y);
    mouse_acc.scroll_y = (new_scroll < -127) ? -127 : ((new_scroll > 127) ? 127 : (int8_t)new_scroll);

    mouse_acc.has_accumulated = true;

    // Don't log during accumulation to reduce noise

    // If accumulation is getting large or we're near saturation, force send immediately
    int32_t total_movement = abs(mouse_acc.delta_x) + abs(mouse_acc.delta_y) + abs(mouse_acc.scroll_y);
    if (total_movement > 100) { // Force send when total movement is high (reasonable for BLE)
        ESP_LOGI(TAG, "Large accumulation detected (%ld), forcing immediate send", total_movement);
        send_accumulated_movement(true);  // Force send bypasses rate limiting
        return;
    }

    // Start or restart the timer
    start_movement_timer();
}

/**
 * Send the accumulated movement immediately
 */
static esp_err_t send_accumulated_movement(bool bypass_rate_limit) {
    if (!mouse_acc.has_accumulated) {
        return ESP_OK; // Nothing to send
    }

    // Skip pending report check for now - notification completion callback not hooked up
    // if (mouse_acc.pending_report) {
    //     ESP_LOGI(TAG, "Report in flight, keeping accumulation");
    //     return ESP_ERR_INVALID_STATE;
    // }

    // Get current active host
    if (!ble_hid_is_connected()) {
        ESP_LOGD(TAG, "No active host connected, clearing accumulation");
        mouse_acc.has_accumulated = false;
        mouse_acc.delta_x = 0;
        mouse_acc.delta_y = 0;
        mouse_acc.scroll_y = 0;
        return ESP_ERR_INVALID_STATE;
    }

    // Prepare report data
    int8_t send_x = (mouse_acc.delta_x < -127) ? -127 :
                   ((mouse_acc.delta_x > 127) ? 127 : (int8_t)mouse_acc.delta_x);
    int8_t send_y = (mouse_acc.delta_y < -127) ? -127 :
                   ((mouse_acc.delta_y > 127) ? 127 : (int8_t)mouse_acc.delta_y);

    // Create mouse report based on protocol mode
    uint8_t report[5]; // Max size for report with ID
    size_t report_len;

    if (protocol_mode == PROTOCOL_MODE_REPORT) {
        // Report Protocol Mode - BLE HID uses GATT handle to identify report type, no Report ID needed
        report[0] = mouse_acc.buttons;       // Button state
        report[1] = (uint8_t)send_x;         // X movement
        report[2] = (uint8_t)send_y;         // Y movement
        report[3] = (uint8_t)mouse_acc.scroll_y; // Wheel scroll
        report_len = 4;
    } else {
        // Boot Protocol Mode - no Report ID
        report[0] = mouse_acc.buttons;       // Button state
        report[1] = (uint8_t)send_x;         // X movement
        report[2] = (uint8_t)send_y;         // Y movement
        report[3] = (uint8_t)mouse_acc.scroll_y; // Wheel scroll
        report_len = 4;
    }

    // Try to allocate mbuf
    struct os_mbuf *om = ble_hs_mbuf_from_flat(report, report_len);
    if (om == NULL) {
        // BLE stack congested - increment failure count and update rate limiting
        mouse_acc.failed_sends++;
        update_rate_limit_on_failure();
        adjust_timer_interval();
        ESP_LOGI(TAG, "mbuf allocation failed (failure #%lu), will retry", mouse_acc.failed_sends);

        // Don't clear accumulation, let timer retry
        start_movement_timer();
        return ESP_ERR_NO_MEM;
    }

    // Send the notification
    uint16_t handle = (protocol_mode == PROTOCOL_MODE_REPORT) ?
                     ble_hid_svc_char_handles[HANDLE_MOUSE_INPUT_REPORT] :
                     ble_hid_svc_char_handles[HANDLE_BOOT_MOUSE_INPUT_REPORT];

    // Skip pending report tracking until notification completion is hooked up
    // mouse_acc.pending_report = true;
    int rc = ble_gattc_notify_custom(ble_hid_dev_state.conn_handle, handle, om);

    if (rc != 0) {
        // Send failed - track failure but only apply rate limiting for congestion errors
        // mouse_acc.pending_report = false;
        mouse_acc.reports_failed++;

        // Decode error codes to understand the failure type
        const char *error_name = "UNKNOWN";
        bool is_congestion_error = false;
        switch (rc) {
            case 3: error_name = "BLE_HS_ENOTCONN"; break;  // Not connected - don't rate limit
            case 8: error_name = "BLE_HS_ENOMEM"; is_congestion_error = true; break;  // No memory - rate limit
            case 30: error_name = "BLE_HS_ENOTCONNFAIL"; break;  // Connection failed - don't rate limit
            case 257: error_name = "BLE_HS_ATT_ERR_INVALID_HANDLE"; break;  // Invalid handle - don't rate limit
        }

        // Only apply rate limiting for actual congestion (memory) errors
        if (is_congestion_error) {
            mouse_acc.failed_sends++;
            update_rate_limit_on_failure();
            adjust_timer_interval();
            ESP_LOGI(TAG, "BLE congestion detected (rc=%d %s), increased rate limiting (failure #%lu)",
                    rc, error_name, mouse_acc.failed_sends);
        } else {
            ESP_LOGI(TAG, "Send failed with rc=%d (%s), but not applying rate limiting (total failed: %lu)",
                    rc, error_name, mouse_acc.reports_failed);
        }

        // Don't clear accumulation, let timer retry
        start_movement_timer();
        return ESP_FAIL;
    }

    // Send successful - track and clear accumulation
    mouse_acc.reports_sent++;
    update_rate_limit_on_success();
    mouse_acc.has_accumulated = false;
    mouse_acc.delta_x = 0;
    mouse_acc.delta_y = 0;
    mouse_acc.scroll_y = 0;
    mouse_acc.failed_sends = 0;
    adjust_timer_interval();

    ESP_LOGD(TAG, "Sent accumulated movement: dx=%d, dy=%d, scroll=%d",
            send_x, send_y, mouse_acc.scroll_y);

    return ESP_OK;
}

/**
 * Called when BLE notification completes (needs to be hooked into BLE events)
 */
void ble_hid_mouse_on_notify_complete(void) {
    mouse_acc.pending_report = false;

    // If we have more accumulated movement, send it immediately
    if (mouse_acc.has_accumulated) {
        ESP_LOGI(TAG, "Notification complete, sending next accumulated movement");
        send_accumulated_movement(false);
    }
}

/**
 * Enhanced mouse report function with accumulation
 */
esp_err_t ble_hid_mouse_report_enhanced(int8_t delta_x, int8_t delta_y, uint8_t buttons, int8_t scroll_y) {
#if BLE_HID_DEBUG_DISABLE_REPORTING
    ESP_LOGI(TAG, "[DEBUG] BLE DISABLED - Enhanced mouse report: buttons=0x%02x, delta=(%d,%d), scroll=%d",
             buttons, delta_x, delta_y, scroll_y);
    current_mouse_buttons = buttons;
    return ESP_OK;
#else
    // Check if button state has changed
    bool buttons_changed = (mouse_acc.buttons != buttons);

    // Always update button state
    mouse_acc.buttons = buttons;
    current_mouse_buttons = buttons;

    // If no movement, no scroll, and buttons haven't changed, do nothing
    if (delta_x == 0 && delta_y == 0 && scroll_y == 0 && !buttons_changed) {
        return ESP_OK;
    }

    // For button-only changes (no movement/scroll), send immediately (bypass rate limiting for buttons)
    if (delta_x == 0 && delta_y == 0 && scroll_y == 0 && buttons_changed) {
        ESP_LOGD(TAG, "Button-only change detected: 0x%02x, sending immediately", buttons);

        if (ble_hid_is_connected()) {
            // Create button-only report - BLE HID format (no Report ID)
            uint8_t report[4];
            size_t report_len;

            // BLE HID: GATT handle identifies report type, no Report ID needed
            report[0] = buttons;    // Button state
            report[1] = 0;          // No X movement
            report[2] = 0;          // No Y movement
            report[3] = 0;          // No scroll
            report_len = 4;

            struct os_mbuf *om = ble_hs_mbuf_from_flat(report, report_len);
            if (om != NULL) {
                uint16_t handle = (protocol_mode == PROTOCOL_MODE_REPORT) ?
                                 ble_hid_svc_char_handles[HANDLE_MOUSE_INPUT_REPORT] :
                                 ble_hid_svc_char_handles[HANDLE_BOOT_MOUSE_INPUT_REPORT];

                int rc = ble_gattc_notify_custom(ble_hid_dev_state.conn_handle, handle, om);
                if (rc == 0) {
                    // Track successful button-only send
                    mouse_acc.reports_sent++;
                    update_rate_limit_on_success();
                    ESP_LOGD(TAG, "Button-only report sent successfully: 0x%02x", buttons);
                    return ESP_OK;
                } else {
                    ESP_LOGW(TAG, "Failed to send button-only report: %d", rc);
                }
            } else {
                ESP_LOGW(TAG, "Failed to allocate mbuf for button-only report");
            }
        }
        return ESP_ERR_INVALID_STATE;
    }

    // If rate limiting allows it, try direct send first (even if we have accumulation)
    if (can_send_now()) {
        if (ble_hid_is_connected()) {

            // Try immediate send including any accumulated movement
            int16_t send_x = delta_x + mouse_acc.delta_x;
            int16_t send_y = delta_y + mouse_acc.delta_y;
            int16_t send_scroll = scroll_y + mouse_acc.scroll_y;

            // Clamp to valid range
            send_x = (send_x < -127) ? -127 : ((send_x > 127) ? 127 : send_x);
            send_y = (send_y < -127) ? -127 : ((send_y > 127) ? 127 : send_y);
            send_scroll = (send_scroll < -127) ? -127 : ((send_scroll > 127) ? 127 : send_scroll);

            uint8_t report[4];
            size_t report_len;

            // BLE HID: GATT handle identifies report type, no Report ID needed
            report[0] = buttons;
            report[1] = (uint8_t)send_x;
            report[2] = (uint8_t)send_y;
            report[3] = (uint8_t)send_scroll;
            report_len = 4;

            struct os_mbuf *om = ble_hs_mbuf_from_flat(report, report_len);
            if (om != NULL) {
                uint16_t handle = (protocol_mode == PROTOCOL_MODE_REPORT) ?
                                 ble_hid_svc_char_handles[HANDLE_MOUSE_INPUT_REPORT] :
                                 ble_hid_svc_char_handles[HANDLE_BOOT_MOUSE_INPUT_REPORT];

                // Skip pending report tracking until notification completion is hooked up
                // mouse_acc.pending_report = true;
                int rc = ble_gattc_notify_custom(ble_hid_dev_state.conn_handle, handle, om);

                if (rc == 0) {
                    // Track successful immediate send and clear all accumulation
                    mouse_acc.reports_sent++;
                    update_rate_limit_on_success();
                    mouse_acc.failed_sends = 0;
                    adjust_timer_interval();

                    // Clear accumulation since we sent it
                    mouse_acc.has_accumulated = false;
                    mouse_acc.delta_x = 0;
                    mouse_acc.delta_y = 0;
                    mouse_acc.scroll_y = 0;

                    // Cancel timer since we just sent
                    stop_movement_timer();

                    ESP_LOGD(TAG, "Immediate send successful (including accumulation): dx=%d, dy=%d, scroll=%d",
                            (int)send_x, (int)send_y, (int)send_scroll);
                    return ESP_OK;
                } else {
                    // Immediate send failed - check if it's a congestion error before rate limiting
                    const char *error_name = "UNKNOWN";
                    bool is_congestion_error = false;
                    switch (rc) {
                        case 8: error_name = "BLE_HS_ENOMEM"; is_congestion_error = true; break;
                        case 3: error_name = "BLE_HS_ENOTCONN"; break;
                        case 30: error_name = "BLE_HS_ENOTCONNFAIL"; break;
                        case 257: error_name = "BLE_HS_ATT_ERR_INVALID_HANDLE"; break;
                    }

                    if (is_congestion_error) {
                        update_rate_limit_on_failure();
                        ESP_LOGD(TAG, "Immediate send congestion (rc=%d %s), applying backpressure", rc, error_name);
                    } else {
                        ESP_LOGD(TAG, "Immediate send failed (rc=%d %s), but not rate limiting", rc, error_name);
                    }
                    // Fall through to accumulation
                    // mouse_acc.pending_report = false;
                }
            }
        }
    }

    // Accumulate the movement (either because send failed, we're already pending, or rate limited)
    if (!can_send_now()) {
        mouse_acc.reports_rate_limited++;
        // Don't adjust rate limiting here - we're already preventing the send, that's the point!
        ESP_LOGD(TAG, "(time-based), acc move(rl: %lu, mi: %lu us)",
                mouse_acc.reports_rate_limited, mouse_acc.min_interval_us);
    }
    accumulate_movement(delta_x, delta_y, scroll_y);
    return ESP_OK;
#endif
}

// ===== TIME-BASED RATE LIMITING IMPLEMENTATION =====

/**
 * Check if we can send a report now based on rate limiting
 */
static bool can_send_now(void) {
    uint64_t current_time_us = esp_timer_get_time();
    uint64_t time_since_last_send = current_time_us - mouse_acc.last_send_time_us;

    // Check if enough time has passed since last send
    bool can_send = (time_since_last_send >= mouse_acc.min_interval_us);

    if (!can_send) {
        ESP_LOGD(TAG, "Rate limited: %llu us since last send (min: %lu us)",
                time_since_last_send, mouse_acc.min_interval_us);
    }

    return can_send;
}

/**
 * Update rate limiting parameters after successful send
 */
static void update_rate_limit_on_success(void) {
    // Update last send time
    mouse_acc.last_send_time_us = esp_timer_get_time();

    // Gradually decrease minimum interval (allow faster sending) when successful
    if (mouse_acc.min_interval_us > MIN_SEND_INTERVAL_US) {
        mouse_acc.min_interval_us = (mouse_acc.min_interval_us * 98) / 100; // Reduce by 2%
        if (mouse_acc.min_interval_us < MIN_SEND_INTERVAL_US) {
            mouse_acc.min_interval_us = MIN_SEND_INTERVAL_US;
        }
    }

    ESP_LOGD(TAG, "Send success: min_interval now %lu us", mouse_acc.min_interval_us);
}

/**
 * Update rate limiting parameters after failed send (backpressure)
 */
static void update_rate_limit_on_failure(void) {
    // Increase minimum interval to slow down sending when we get backpressure
    mouse_acc.min_interval_us = (mouse_acc.min_interval_us * 150) / 100; // Increase by 50%
    if (mouse_acc.min_interval_us > 50000) { // Cap at 50ms (20Hz min) - typical BLE HID minimum
        mouse_acc.min_interval_us = 50000;
    }

    // Log message moved to actual failure sites for better context
}


/**
 * Log comprehensive report statistics
 */
void ble_hid_mouse_log_stats(void) {
    ESP_LOGI(TAG, "=== Mouse Report Statistics ===");
    ESP_LOGI(TAG, "Reports sent: %lu", mouse_acc.reports_sent);
    ESP_LOGI(TAG, "Reports failed: %lu", mouse_acc.reports_failed);
    ESP_LOGI(TAG, "Reports rate limited: %lu", mouse_acc.reports_rate_limited);
    ESP_LOGI(TAG, "Failed sends: %lu", mouse_acc.failed_sends);
    ESP_LOGI(TAG, "Timer interval: %lu us", mouse_acc.timer_interval_us);
    ESP_LOGI(TAG, "Min send interval: %lu us", mouse_acc.min_interval_us);

    uint64_t current_time_us = esp_timer_get_time();
    uint64_t time_since_last_send = current_time_us - mouse_acc.last_send_time_us;
    ESP_LOGI(TAG, "Time since last send: %llu us", time_since_last_send);
    ESP_LOGI(TAG, "Can send now: %s", can_send_now() ? "YES" : "NO");
    ESP_LOGI(TAG, "===============================");
}

/**
 * @brief Send a consumer control report over BLE HID
 *
 * @param usage_code The 16-bit HID consumer control usage code
 * @return ESP_OK on success or an error code
 */
esp_err_t ble_hid_consumer_control(uint16_t usage_code) {
#if BLE_HID_DEBUG_DISABLE_REPORTING
    ESP_LOGI(TAG, "[DEBUG] BLE DISABLED - Consumer control report: usage=0x%04x", usage_code);
    return ESP_OK;
#else
    ESP_LOGD(TAG, "Sending consumer control report: usage=0x%04x", usage_code);

    if (!ble_hid_is_connected()) {
        ESP_LOGW(TAG, "BLE not connected, cannot send consumer control report");
        return ESP_FAIL;
    }

    if (ble_hid_svc_char_handles[HANDLE_CONSUMER_INPUT_REPORT] == 0) {
        ESP_LOGW(TAG, "Consumer input report handle not initialized");
        return ESP_FAIL;
    }

    // Fill the consumer control report - no Report ID needed in BLE reports
    // as they are identified by the characteristic handle
    consumer_input_report_data[0] = usage_code & 0xFF;         // Low byte
    consumer_input_report_data[1] = (usage_code >> 8) & 0xFF;  // High byte

    struct os_mbuf *om = ble_hs_mbuf_from_flat(consumer_input_report_data, sizeof(consumer_input_report_data));
    if (om == NULL) {
        ESP_LOGE(TAG, "Failed to allocate mbuf for consumer control report");
        return ESP_ERR_NO_MEM;
    }

    // Send the report using the dedicated consumer control input report characteristic
    ESP_LOGD(TAG, "Sending consumer control report with handle 0x%04x", ble_hid_svc_char_handles[HANDLE_CONSUMER_INPUT_REPORT]);
    int rc = ble_gattc_notify_custom(ble_hid_dev_state.conn_handle,
                                    ble_hid_svc_char_handles[HANDLE_CONSUMER_INPUT_REPORT],
                                    om);

    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to send consumer control report: %d", rc);
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "Consumer control report sent successfully: usage=0x%04x", usage_code);
    return ESP_OK;
#endif
}
