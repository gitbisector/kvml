#include "ble_hid_mouse.h"
#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "host/ble_hs.h"
#include "host/ble_gatt.h"
#include "services/gatt/ble_svc_gatt.h"
#include "host/ble_uuid.h"
#include "ble_hid_device.h"

// HID Service and characteristic UUIDs
#define BLE_SVC_HID_UUID16                           0x1812
#define BLE_SVC_HID_CHR_REPORT_MAP_UUID16           0x2A4B
#define BLE_SVC_HID_CHR_REPORT_UUID16               0x2A4D
#define BLE_SVC_HID_CHR_PROTOCOL_MODE_UUID16        0x2A4E
#define BLE_SVC_HID_CHR_CONTROL_POINT_UUID16        0x2A4C
#define BLE_SVC_HID_CHR_INFO_UUID16                 0x2A4A

// Report descriptor: Report Reference Characteristic Descriptor
#define BLE_SVC_HID_DESCRIPTOR_REPORT_REF_UUID16    0x2908

// HID Report IDs
#define HID_REPORT_ID_MOUSE                         0x01
#define HID_REPORT_ID_KEYBOARD                       0x02

// Battery Service UUIDs
#define BLE_SVC_BAS_UUID16                        0x180F
#define BLE_SVC_BAS_CHR_BATTERY_LEVEL_UUID16      0x2A19

#define TAG "BLE_HID_MOUSE"

// Combined HID Report Descriptor for Mouse and Keyboard
static const uint8_t hid_report_map[] = {
    // Mouse with Report ID 1
    0x05, 0x01,       // Usage Page (Generic Desktop)
    0x09, 0x02,       // Usage (Mouse)
    0xA1, 0x01,       // Collection (Application)
    0x85, 0x01,       //   Report ID (1)
    0x09, 0x01,       //   Usage (Pointer)
    0xA1, 0x00,       //   Collection (Physical)
    0x05, 0x09,       //     Usage Page (Buttons)
    0x19, 0x01,       //     Usage Minimum (1)
    0x29, 0x03,       //     Usage Maximum (3)
    0x15, 0x00,       //     Logical Minimum (0)
    0x25, 0x01,       //     Logical Maximum (1)
    0x95, 0x03,       //     Report Count (3)
    0x75, 0x01,       //     Report Size (1)
    0x81, 0x02,       //     Input (Data, Variable, Absolute)
    0x95, 0x01,       //     Report Count (1)
    0x75, 0x05,       //     Report Size (5)
    0x81, 0x03,       //     Input (Constant, Variable, Absolute) ;5 bit padding
    0x05, 0x01,       //     Usage Page (Generic Desktop)
    0x09, 0x30,       //     Usage (X)
    0x09, 0x31,       //     Usage (Y)
    0x09, 0x38,       //     Usage (Wheel)
    0x15, 0x81,       //     Logical Minimum (-127)
    0x25, 0x7F,       //     Logical Maximum (127)
    0x75, 0x08,       //     Report Size (8)
    0x95, 0x03,       //     Report Count (3)
    0x81, 0x06,       //     Input (Data, Variable, Relative)
    0xC0,             //   End Collection
    0xC0,             // End Collection

    // Keyboard with Report ID 2
    0x05, 0x01,       // Usage Page (Generic Desktop)
    0x09, 0x06,       // Usage (Keyboard)
    0xA1, 0x01,       // Collection (Application)
    0x85, 0x02,       //   Report ID (2)

    // Modifier keys (shift, ctrl, alt, etc)
    0x05, 0x07,       //   Usage Page (Key Codes)
    0x19, 0xE0,       //   Usage Minimum (224)
    0x29, 0xE7,       //   Usage Maximum (231)
    0x15, 0x00,       //   Logical Minimum (0)
    0x25, 0x01,       //   Logical Maximum (1)
    0x75, 0x01,       //   Report Size (1)
    0x95, 0x08,       //   Report Count (8)
    0x81, 0x02,       //   Input (Data, Variable, Absolute) - Modifier byte

    // Reserved byte
    0x95, 0x01,       //   Report Count (1)
    0x75, 0x08,       //   Report Size (8)
    0x81, 0x03,       //   Input (Constant, Variable, Absolute) - Reserved byte

    // LED output report
    0x95, 0x05,       //   Report Count (5)
    0x75, 0x01,       //   Report Size (1)
    0x05, 0x08,       //   Usage Page (LEDs)
    0x19, 0x01,       //   Usage Minimum (1)
    0x29, 0x05,       //   Usage Maximum (5)
    0x91, 0x02,       //   Output (Data, Variable, Absolute) - LED report

    // LED padding
    0x95, 0x01,       //   Report Count (1)
    0x75, 0x03,       //   Report Size (3)
    0x91, 0x03,       //   Output (Constant, Variable, Absolute) - LED report padding

    // Key array (6 keys)
    0x95, 0x06,       //   Report Count (6)
    0x75, 0x08,       //   Report Size (8)
    0x15, 0x00,       //   Logical Minimum (0)
    0x26, 0xFF, 0x00, //   Logical Maximum (255)
    0x05, 0x07,       //   Usage Page (Key Codes)
    0x19, 0x00,       //   Usage Minimum (0)
    0x29, 0xFF,       //   Usage Maximum (255)
    0x81, 0x00,       //   Input (Data, Array) - Key array
    0xC0              // End Collection
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
static uint8_t protocol_mode = 0x01;   // Report Protocol Mode

// Battery level value (read/notify) - Start with 100%
static uint8_t battery_level = 100;

// HID characteristics handles
static uint16_t hid_report_map_handle;
static uint16_t mouse_input_report_handle;
static uint16_t keyboard_input_report_handle;
static uint16_t hid_protocol_mode_handle;
static uint16_t hid_control_point_handle;
static uint16_t hid_info_handle;

// Battery service characteristic handles
static uint16_t battery_level_handle;

// Report reference descriptor values
static uint8_t mouse_report_ref[2] = {HID_REPORT_ID_MOUSE, 0x01};     // Report ID 1, Input report
static uint8_t keyboard_report_ref[2] = {HID_REPORT_ID_KEYBOARD, 0x01}; // Report ID 2, Input report

// Mouse report data (report ID + buttons + X + Y + wheel)
static uint8_t mouse_input_report_data[5] = {HID_REPORT_ID_MOUSE, 0, 0, 0, 0};

// Keyboard report data (report ID + modifier + reserved + 6 key codes)
static uint8_t keyboard_input_report_data[9] = {HID_REPORT_ID_KEYBOARD, 0, 0, 0, 0, 0, 0, 0, 0};

/**
 * HID Report Map access callback
 */
static int hid_report_map_access(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            ESP_LOGD(TAG, "Read HID report map descriptor");
            rc = os_mbuf_append(ctxt->om, hid_report_map, sizeof(hid_report_map));
            return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}

/**
 * HID Report access callback
 */
static int hid_report_access(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;
    // Determine if this is the mouse or keyboard report characteristic
    bool is_mouse = (attr_handle == mouse_input_report_handle);
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            // Send mouse or keyboard report data
            if (is_mouse) {
                rc = os_mbuf_append(ctxt->om, mouse_input_report_data, sizeof(mouse_input_report_data));
                ESP_LOGD(TAG, "Read mouse input report");
            } else {
                rc = os_mbuf_append(ctxt->om, keyboard_input_report_data, sizeof(keyboard_input_report_data));
                ESP_LOGD(TAG, "Read keyboard input report");
            }
            return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;

        case BLE_GATT_ACCESS_OP_READ_DSC:
            // Read report reference descriptor
            if (is_mouse) {
                rc = os_mbuf_append(ctxt->om, mouse_report_ref, sizeof(mouse_report_ref));
            } else {
                rc = os_mbuf_append(ctxt->om, keyboard_report_ref, sizeof(keyboard_report_ref));
            }
            return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
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
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            rc = os_mbuf_append(ctxt->om, &protocol_mode, sizeof(protocol_mode));
            return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            rc = ble_hs_mbuf_to_flat(ctxt->om, &protocol_mode, sizeof(protocol_mode), NULL);
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
 * Define service UUIDs
 */
static const ble_uuid16_t hid_report_map_uuid = BLE_UUID16_INIT(BLE_SVC_HID_CHR_REPORT_MAP_UUID16);
static const ble_uuid16_t hid_report_uuid = BLE_UUID16_INIT(BLE_SVC_HID_CHR_REPORT_UUID16);
static const ble_uuid16_t hid_proto_mode_uuid = BLE_UUID16_INIT(BLE_SVC_HID_CHR_PROTOCOL_MODE_UUID16);
static const ble_uuid16_t hid_control_point_uuid = BLE_UUID16_INIT(BLE_SVC_HID_CHR_CONTROL_POINT_UUID16);
static const ble_uuid16_t hid_info_uuid = BLE_UUID16_INIT(BLE_SVC_HID_CHR_INFO_UUID16);
static const ble_uuid16_t hid_svc_uuid = BLE_UUID16_INIT(BLE_SVC_HID_UUID16);
static const ble_uuid16_t report_ref_uuid = BLE_UUID16_INIT(BLE_SVC_HID_DESCRIPTOR_REPORT_REF_UUID16);

// Battery Service UUIDs
static const ble_uuid16_t battery_svc_uuid = BLE_UUID16_INIT(BLE_SVC_BAS_UUID16);
static const ble_uuid16_t battery_level_uuid = BLE_UUID16_INIT(BLE_SVC_BAS_CHR_BATTERY_LEVEL_UUID16);

/**
 * Define HID Services
 */
static const struct ble_gatt_svc_def hid_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &hid_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                // HID Report Map characteristic
                .uuid = &hid_report_map_uuid.u,
                .access_cb = hid_report_map_access,
                .flags = BLE_GATT_CHR_F_READ,
                .val_handle = &hid_report_map_handle,
            },
            {
                // HID Mouse Input Report characteristic
                .uuid = &hid_report_uuid.u,
                .access_cb = hid_report_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &mouse_input_report_handle,
                .descriptors = (struct ble_gatt_dsc_def[]) {
                    {
                        .uuid = &report_ref_uuid.u,
                        .access_cb = hid_report_access,
                        .att_flags = BLE_ATT_F_READ,
                    },
                    {
                        0, // No more descriptors
                    },
                },
            },
            {
                // HID Keyboard Input Report characteristic
                .uuid = &hid_report_uuid.u,
                .access_cb = hid_report_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &keyboard_input_report_handle,
                .descriptors = (struct ble_gatt_dsc_def[]) {
                    {
                        .uuid = &report_ref_uuid.u,
                        .access_cb = hid_report_access,
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
                .val_handle = &hid_protocol_mode_handle,
            },
            {
                // HID Control Point characteristic
                .uuid = &hid_control_point_uuid.u,
                .access_cb = hid_ctrl_point_access,
                .flags = BLE_GATT_CHR_F_WRITE_NO_RSP,
                .val_handle = &hid_control_point_handle,
            },
            {
                // HID Information characteristic
                .uuid = &hid_info_uuid.u,
                .access_cb = hid_info_access,
                .flags = BLE_GATT_CHR_F_READ,
                .val_handle = &hid_info_handle,
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
                .val_handle = &battery_level_handle,
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
    ESP_LOGI(TAG, "Initializing BLE HID Mouse device");
    // Register the HID service with the combined mouse/keyboard report descriptor
    int rc = ble_gatts_count_cfg(hid_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to count GATT service attributes: %d", rc);
        return ESP_FAIL;
    }
    rc = ble_gatts_add_svcs(hid_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to register GATT services: %d", rc);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "BLE HID Mouse initialization complete");
    return ESP_OK;
}

void ble_hid_mouse_send_report(uint8_t conn_idx, const uint8_t *report, size_t len)
{
    if (conn_idx >= BLE_HID_MAX_CONN || !ble_hid_dev_state.connected[conn_idx]) {
        ESP_LOGW(TAG, "Invalid connection index or not connected");
        return;
    }

    // Validate report size
    if (len > sizeof(mouse_input_report_data)) {
        ESP_LOGE(TAG, "Mouse report too large: %d > %d",
                (int)len, (int)sizeof(mouse_input_report_data));
        return;
    }

    // Copy the report data
    memcpy(mouse_input_report_data, report, len);

    // Send notification to connected host
    struct os_mbuf *om = ble_hs_mbuf_from_flat(mouse_input_report_data, sizeof(mouse_input_report_data));
    if (om != NULL) {
        int rc = ble_gattc_notify_custom(ble_hid_dev_state.conn_handles[conn_idx],
                                         mouse_input_report_handle, om);
        if (rc != 0) {
            ESP_LOGW(TAG, "Failed to send mouse HID notification: %d", rc);
        } else {
            ESP_LOGD(TAG, "Sent mouse HID report notification");
        }
    }
}

void ble_hid_keyboard_send_report(uint8_t conn_idx, const uint8_t *report, size_t len)
{
    if (conn_idx >= BLE_HID_MAX_CONN || !ble_hid_dev_state.connected[conn_idx]) {
        ESP_LOGW(TAG, "Invalid connection index or not connected");
        return;
    }

    // Validate report size
    if (len > sizeof(keyboard_input_report_data)) {
        ESP_LOGE(TAG, "Keyboard report too large: %d > %d",
                (int)len, (int)sizeof(keyboard_input_report_data));
        return;
    }

    // Copy the report data
    memcpy(keyboard_input_report_data, report, len);

    // Send notification to connected host
    struct os_mbuf *om = ble_hs_mbuf_from_flat(keyboard_input_report_data, sizeof(keyboard_input_report_data));
    if (om != NULL) {
        int rc = ble_gattc_notify_custom(ble_hid_dev_state.conn_handles[conn_idx],
                                         keyboard_input_report_handle, om);
        if (rc != 0) {
            ESP_LOGW(TAG, "Failed to send keyboard HID notification: %d", rc);
        } else {
            ESP_LOGD(TAG, "Sent keyboard HID report notification");
        }
    }
}
