#pragma once

#include "esp_err.h"
#include "esp_timer.h"
#include "host/ble_gap.h"

// Maximum allowed BLE HID connections (single connection per ESP32 in master/slave architecture)
#define BLE_HID_MAX_CONN 1

// HID Service UUIDs
#define BLE_SVC_HID_UUID16            0x1812
#define BLE_SVC_DIS_UUID16            0x180A
#define BLE_SVC_BAS_UUID16            0x180F

// BLE Appearances
#define BLE_APPEARANCE_HID_KEYBOARD   0x03C1
#define BLE_APPEARANCE_HID_MOUSE      0x03C2

// Connection parameters
#define BLE_HID_CONN_MIN_INTERVAL     0x0008  // 10 ms
#define BLE_HID_CONN_MAX_INTERVAL     0x0010  // 20 ms
#define BLE_HID_CONN_LATENCY          0
#define BLE_HID_CONN_SUPERVISION_TIMEOUT  0x0C80  // 3.2 seconds

// Default advertisement parameters
#define BLE_HID_ADV_FAST_INTERVAL_MIN    0x0030  // 30 ms
#define BLE_HID_ADV_FAST_INTERVAL_MAX    0x0060  // 60 ms

// Device state tracking structure
typedef struct {
    bool connected;                       // Single connection state
    uint16_t conn_handle;                 // Single connection handle
    esp_timer_handle_t adv_restart_timer; // Timer for delayed advertising restart
    esp_timer_handle_t adv_retry_timer;   // Timer for retry when BLE stack is busy
    int adv_retry_count;                  // Count of advertisement retry attempts
    bool pairing_in_progress;             // Flag to indicate pairing is in progress
} ble_hid_dev_state_t;

// Global device state for tracking connections
extern ble_hid_dev_state_t ble_hid_dev_state;

// Common BLE HID service UUIDs
extern const ble_uuid16_t gatt_hid_svc_uuid;
extern const ble_uuid16_t gatt_dis_svc_uuid;
extern const ble_uuid16_t gatt_bas_svc_uuid;

/**
 * @brief Get the BLE GAP event handler for HID device
 * @return Pointer to the GAP event handler function
 */
ble_gap_event_fn *ble_hid_get_gap_event_handler(void);

/**
 * @brief Initialize common BLE HID device functionality
 * @return ESP_OK if successful, otherwise an error code
 */
esp_err_t ble_hid_device_init(void);

/**
 * @brief Set advertisement data for the BLE HID device
 * @return 0 on success, non-zero error code on failure
 */
int ble_hid_device_set_adv_data(void);

/**
 * @brief Start BLE HID advertising if connections are available
 */
void ble_hid_device_start_advertising(void);

/**
 * @brief BLE GAP event handler for HID device
 * @param event The GAP event
 * @param arg Optional user argument
 * @return 0 on success, non-zero on error
 */
int ble_hid_gap_event(struct ble_gap_event *event, void *arg);

/**
 * @brief Check if focus can be switched (no buttons pressed, no keys held)
 * @return true if focus can be switched safely, false if locked
 */
bool ble_hid_can_switch_focus(void);

/**
 * @brief Check if the BLE HID device is connected
 * @return true if connected, false otherwise
 */
bool ble_hid_is_connected(void);
