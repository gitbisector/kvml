#include <string.h>
#include <inttypes.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "ble_hid_device.h"
#include "ble_hid_mouse.h"

// Function prototypes
static void _init_adv_normal(void);
char *ble_addr_to_str(const ble_addr_t *addr, char *dst);
static void generate_device_name_with_mac(char *name_buffer, size_t buffer_size);

static const char *TAG = "BLE_HID_DEVICE";

// Device state for tracking connections (single connection per ESP32)
ble_hid_dev_state_t ble_hid_dev_state = {
    .connected = false,
    .conn_handle = 0,
    .adv_restart_timer = NULL,
    .adv_retry_timer = NULL,
    .adv_retry_count = 0,
    .pairing_in_progress = false,
};

// Timer callback for delayed advertising restart
static void adv_restart_timer_cb(void* arg)
{
    ESP_LOGI(TAG, "Advertisement restart timer fired");

    // Check if we had Ubuntu disconnect reason 517 issue and need full reset
    if (ble_hid_dev_state.adv_retry_count > 2) {
        ESP_LOGW(TAG, "Multiple advertising failures detected - performing full BLE reset");

        // Reset advertisement data first
        ble_hid_device_set_adv_data();

        // Force controller reset by stopping/restarting the host
        ESP_LOGI(TAG, "Resetting BLE host and controller");
        ble_hs_sched_reset(0);

        // Reset our state tracking
        ble_hid_dev_state.adv_retry_count = 0;
        ble_hid_dev_state.pairing_in_progress = false;

        // We'll restart advertising after reset via the sync callback
        return;
    }

    // Standard advertising start
    ble_hid_device_start_advertising();
}

// BLE HID Service UUID (0x1812)
const ble_uuid16_t gatt_hid_svc_uuid = BLE_UUID16_INIT(BLE_SVC_HID_UUID16);

// Device Information Service UUID (0x180A)
const ble_uuid16_t gatt_dis_svc_uuid = BLE_UUID16_INIT(BLE_SVC_DIS_UUID16);

// Battery Service UUID (0x180F)
const ble_uuid16_t gatt_bas_svc_uuid = BLE_UUID16_INIT(BLE_SVC_BAS_UUID16);

/**
 * Debug function to dump advertisement data contents
 */
static void dump_adv_data(uint8_t *buf, uint8_t len) {
    ESP_LOGI(TAG, "Advertisement data (%d bytes):", len);
    for (int i = 0; i < len; i += 16) {
        char line[80];
        int linelen = 0;
        linelen += snprintf(line + linelen, sizeof(line) - linelen, "%04x: ", i);

        for (int j = 0; j < 16; j++) {
            if (i + j < len) {
                linelen += snprintf(line + linelen, sizeof(line) - linelen, "%02x ", buf[i + j]);
            } else {
                linelen += snprintf(line + linelen, sizeof(line) - linelen, "   ");
            }
        }

        linelen += snprintf(line + linelen, sizeof(line) - linelen, " | ");

        for (int j = 0; j < 16; j++) {
            if (i + j < len) {
                char c = buf[i + j];
                c = (c >= 32 && c <= 126) ? c : '.';
                linelen += snprintf(line + linelen, sizeof(line) - linelen, "%c", c);
            } else {
                break;
            }
        }

        ESP_LOGI(TAG, "%s", line);
    }
}

/**
 * Sets the advertisement data for BLE HID device
 * Implemented exactly like the working example
 */
int ble_hid_device_set_adv_data(void) {
    struct ble_hs_adv_fields fields = {0};
    const char *name;
    int rc;

    ESP_LOGI(TAG, "=== Setting advertisement data (KVML version) ===");

    /**
     *  Set the advertisement data included in our advertisements:
     *     o Flags (indicates advertisement type and other general info).
     *     o Advertising tx power.
     *     o Device name.
     *     o 16-bit service UUIDs (HID service).
     */

    /* Advertise flags:
     *     o Discoverability in forthcoming advertisement (general)
     *     o BLE-only (BR/EDR unsupported).
     */
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    ESP_LOGI(TAG, "ADV: flags=0x%02x (DISC_GEN | BREDR_UNSUP)", fields.flags);

    /* Indicate that the TX power level field should be included; have the
     * stack fill this value automatically.  This is done by assigning the
     * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
     */
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    ESP_LOGI(TAG, "ADV: tx_pwr_lvl=%d (AUTO)", fields.tx_pwr_lvl);

    /* Include advertising interval in advertisement */
    fields.adv_itvl_is_present = 1;
    fields.adv_itvl = 40;
    ESP_LOGI(TAG, "ADV: adv_itvl=%d", fields.adv_itvl);

    /* Use GAP service device name - same as working example */
    name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;
    ESP_LOGI(TAG, "ADV: name='%s' (len=%d, is_complete=%d)", name, fields.name_len, fields.name_is_complete);

    /* Set appearance for HID Keyboard+Mouse combo - Ubuntu should categorize as input device */
    fields.appearance = 0x03C1;  /* HID keyboard appearance - more generic for combo devices */
    fields.appearance_is_present = 1;
    ESP_LOGI(TAG, "ADV: appearance=0x%04x (HID keyboard+mouse combo), is_present=%d",
             fields.appearance, fields.appearance_is_present);
    /* Set the 16-bit service UUIDs - include HID, Battery, and Device Info services */
    static const ble_uuid16_t hid_service_uuid = BLE_UUID16_INIT(BLE_SVC_HID_UUID16);
    static const ble_uuid16_t battery_service_uuid = BLE_UUID16_INIT(BLE_SVC_BAS_UUID16);
    static const ble_uuid16_t device_info_service_uuid = BLE_UUID16_INIT(BLE_SVC_DIS_UUID16);
    fields.uuids16 = (ble_uuid16_t[]){ hid_service_uuid, battery_service_uuid, device_info_service_uuid };
    fields.num_uuids16 = 3;
    fields.uuids16_is_complete = 1;
    ESP_LOGI(TAG, "ADV: Services - HID=0x%04x, Battery=0x%04x, DevInfo=0x%04x, num=%d, is_complete=%d",
             BLE_SVC_HID_UUID16, BLE_SVC_BAS_UUID16, BLE_SVC_DIS_UUID16, fields.num_uuids16, fields.uuids16_is_complete);

    /* Test the fields in a buffer first */
    uint8_t buf[50];
    uint8_t buf_sz;
    rc = ble_hs_adv_set_fields(&fields, buf, &buf_sz, 50);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error setting advertisement data to buf; rc=%d", rc);
        return rc;
    }
    ESP_LOGI(TAG, "ADV: Calculated advertisement data size: %d bytes (max=%d)", buf_sz, BLE_HS_ADV_MAX_SZ);
    dump_adv_data(buf, buf_sz);
    if (buf_sz > BLE_HS_ADV_MAX_SZ) {
        ESP_LOGE(TAG, "Too long advertising data: name %s, appearance %x, uuid16 %x, advsize = %d",
            name, fields.appearance, BLE_SVC_HID_UUID16, buf_sz);
        return BLE_HS_EMSGSIZE;
    }

    /* Set the actual advertisement fields */
    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error setting advertisement fields: %d", rc);
        return rc;
    }
    ESP_LOGI(TAG, "Advertisement data set successfully");
    return 0;
}

// Forward declarations
static void adv_retry_timer_cb(void* arg);
static void ble_hid_on_sync(void);
static int ble_handle_security_event(struct ble_gap_event *event, void *arg);
void ble_hid_on_pairing_complete(struct ble_gap_event *event);

// External function declaration for BLE bonding store configuration
void ble_store_config_init(void);

// BLE address to string helper function
char *ble_addr_to_str(const ble_addr_t *addr, char *dst) {
    if (addr == NULL || dst == NULL) {
        return NULL;
    }
    sprintf(dst, "%02x:%02x:%02x:%02x:%02x:%02x",
            addr->val[5], addr->val[4], addr->val[3],
            addr->val[2], addr->val[1], addr->val[0]);
    return dst;
}

// Generate device name with last two digits of MAC address
static void generate_device_name_with_mac(char *name_buffer, size_t buffer_size) {
    uint8_t addr_val[6] = {0};
    int rc = ble_hs_id_copy_addr(BLE_OWN_ADDR_PUBLIC, addr_val, NULL);
    
    if (rc == 0) {
        // Use last two digits (bytes) of MAC address with shorter format
        snprintf(name_buffer, buffer_size, "KM%02X%02X", addr_val[1], addr_val[0]);
    } else {
        // Fallback to default name if MAC address is not available
        snprintf(name_buffer, buffer_size, "KM00");
    }
    
    ESP_LOGI(TAG, "Generated device name: %s", name_buffer);
}

// Advertisement normal initialization
static void _init_adv_normal(void) {
    ESP_LOGI(TAG, "Initializing normal advertising delay");
}

/**
 * Handle completed pairing
 */
void ble_hid_on_pairing_complete(struct ble_gap_event *event) {
    struct ble_gap_conn_desc desc;
    int rc;

    // Get connection descriptor to determine bonding state
    rc = ble_gap_conn_find(event->enc_change.conn_handle, &desc);
    if (rc == 0) {
        ESP_LOGI(TAG, "Pairing complete, encryption enabled, bonded=%d", desc.sec_state.bonded);

        // Verify that the connection is properly bonded
        if (desc.sec_state.bonded) {
            ESP_LOGI(TAG, "Bond has been stored for persistent reconnection");

            // Bonds are automatically persisted by NimBLE when properly initialized
            // The ble_store_config_init() call in initialization ensures this works
            ESP_LOGI(TAG, "Bond info is automatically persisted by NimBLE");
        } else {
            ESP_LOGW(TAG, "Connection is encrypted but not bonded - reconnection may require re-pairing");
        }
    } else {
        ESP_LOGW(TAG, "Could not find connection info: %d", rc);
    }

    ble_hid_dev_state.pairing_in_progress = false;
}

// Global state
static uint8_t own_addr_type;
static struct ble_gap_adv_params adv_params;
static int active_connections = 0;

/**
 * Start BLE advertising for the HID device
 */
void ble_hid_device_start_advertising(void) {
    int rc;

    ESP_LOGI(TAG, "=== Starting BLE advertising ===");
    ESP_LOGI(TAG, "Current state: active_connections=%d, retry_count=%d",
             active_connections, ble_hid_dev_state.adv_retry_count);

    if (active_connections > 0 || ble_gap_conn_active()) {
        ESP_LOGI(TAG, "Already connected to a device, not starting advertising");
        return;
    }

    // Ensure adv_params are properly initialized - this is important for Ubuntu compatibility
    memset(&adv_params, 0, sizeof(adv_params));

    /* Reset the device state when restarting advertisement */

    /* Set advertisement data with HID information - no need to clear first */
    rc = ble_hid_device_set_adv_data();
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set advertisement data: %d (%s)", rc,
                (rc == BLE_HS_EBUSY) ? "BLE_HS_EBUSY" :
                (rc == BLE_HS_EALREADY) ? "BLE_HS_EALREADY" :
                (rc == BLE_HS_EINVAL) ? "BLE_HS_EINVAL" :
                (rc == BLE_HS_EMSGSIZE) ? "BLE_HS_EMSGSIZE" :
                "UNKNOWN");
        goto retry_later;
    }

    /* Configure advertisement parameters exactly like working example */
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; /* Undirected connectable mode */
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; /* General discoverable mode */
    ESP_LOGI(TAG, "Adv params: conn_mode=%d (UND), disc_mode=%d (GEN)",
             adv_params.conn_mode, adv_params.disc_mode);

    /* Start advertising - use own_addr_type from the working example */
    uint8_t own_addr_type = BLE_OWN_ADDR_PUBLIC;
    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                         &adv_params, ble_hid_gap_event, NULL);
    if (rc == 0) {
        // BLE advertising started
        ble_hid_dev_state.adv_retry_count = 0; /* Reset retry count on success */
        ESP_LOGI(TAG, "BLE HID advertisement started");
        return;
    }

    /* Log error with detailed code information */
    ESP_LOGE(TAG, "Error starting advertisement: %d (%s)", rc,
            (rc == BLE_HS_EBUSY) ? "BLE_HS_EBUSY" :
            (rc == BLE_HS_EALREADY) ? "BLE_HS_EALREADY" :
            (rc == BLE_HS_EINVAL) ? "BLE_HS_EINVAL" :
            (rc == BLE_HS_EMSGSIZE) ? "BLE_HS_EMSGSIZE" :
            (rc == BLE_HS_EPREEMPTED) ? "BLE_HS_EPREEMPTED" :
            "UNKNOWN");

retry_later:
    /* On error, do a fast retry first */
    if (ble_hid_dev_state.adv_retry_count == 0) {
        /* On first failure, try again immediately with delay - per working example */
        ble_hid_dev_state.adv_retry_count = 1;
        ESP_LOGW(TAG, "Immediate retry after first failure");
        vTaskDelay(pdMS_TO_TICKS(20));

        /* Try starting advertisement again right away */
        rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                            &adv_params, ble_hid_gap_event, NULL);
        if (rc == 0) {
            ESP_LOGI(TAG, "BLE HID advertisement started on immediate retry");
            // Advertisement started successfully
            ble_hid_dev_state.adv_retry_count = 0;
            return;
        }
    }

    /* For subsequent retries, use exponential backoff */
    if (ble_hid_dev_state.adv_retry_count < 8) {
        /* Calculate delay with exponential backoff - same as working example */
        uint32_t delay_ms = 100 * (1 << ble_hid_dev_state.adv_retry_count);
        ESP_LOGW(TAG, "BLE stack busy, scheduling retry #%u in %"PRIu32"ms",
                ble_hid_dev_state.adv_retry_count, delay_ms);

        /* Create timer for retry if not already created */
        if (ble_hid_dev_state.adv_retry_timer == NULL) {
            esp_timer_create_args_t timer_args = {
                .callback = adv_retry_timer_cb,
                .name = "adv_retry_timer"
            };
            esp_timer_create(&timer_args, &ble_hid_dev_state.adv_retry_timer);
        } else {
            /* Make sure to stop any previous timer */
            esp_timer_stop(ble_hid_dev_state.adv_retry_timer);
        }

        /* Start the retry timer */
        esp_timer_start_once(ble_hid_dev_state.adv_retry_timer, delay_ms * 1000);

        /* Increment retry counter */
        ble_hid_dev_state.adv_retry_count++;
        return;
    } else {
        ESP_LOGE(TAG, "Max advertisement retry attempts reached, resetting BLE stack");
        /* After max retries, request NimBLE host reset to recover */
        ble_hid_dev_state.adv_retry_count = 0;
        ble_hs_sched_reset(BLE_HS_EAPP);
        return;
    }
}

// Retry timer callback
static void adv_retry_timer_cb(void* arg)
{
    ESP_LOGD(TAG, "Advertisement retry timer fired");
    ble_hid_device_start_advertising();
}

/**
 * NimBLE sync callback - called when BLE host syncs with controller
 * This is called on initial startup and after BLE resets
 */
static void ble_hid_on_sync(void) {
    int rc;
    ESP_LOGI(TAG, "BLE host synced with controller");

    // Reset adv retry count
    ble_hid_dev_state.adv_retry_count = 0;

    // Configure address type for advertising - important for Ubuntu compatibility
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error determining address type; rc=%d", rc);
        return;
    }

    // Get device address for logging and device name generation
    uint8_t addr_val[6] = {0};
    rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);
    if (rc == 0) {
        ESP_LOGI(TAG, "Device Address: %02x:%02x:%02x:%02x:%02x:%02x",
                addr_val[5], addr_val[4], addr_val[3], addr_val[2], addr_val[1], addr_val[0]);
        
        // Generate and set device name with MAC suffix
        char device_name[32];
        generate_device_name_with_mac(device_name, sizeof(device_name));
        ble_svc_gap_device_name_set(device_name);
        ESP_LOGI(TAG, "Device name set to: %s", device_name);
    }

    // Initialize essential BLE services (GAP and GATT) after sync
    ESP_LOGI(TAG, "Initializing GAP and GATT services...");
    ble_svc_gap_init();
    ble_svc_gatt_init();

    // Initialize BLE HID Mouse services now that the NimBLE stack is ready
    ESP_LOGI(TAG, "Initializing BLE HID Mouse services...");
    esp_err_t err = ble_hid_mouse_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init BLE HID mouse: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "BLE HID Mouse services initialized successfully");

    ESP_LOGI(TAG, "BLE host sync complete - starting advertising");

    // Start advertising after sync and service registration
    ble_hid_device_start_advertising();
}

// Fixed passkey for pairing - use a standard passkey value for Ubuntu
#define FIXED_PASSKEY 123456

/**
 * Handle security events for BLE
 */
static int ble_handle_security_event(struct ble_gap_event *event, void *arg) {
    int rc = 0;

    switch (event->type) {
        // Handle passkey action events for Ubuntu pairing
        case BLE_GAP_EVENT_PASSKEY_ACTION:
            ESP_LOGI(TAG, "PASSKEY_ACTION_EVENT started");
            struct ble_sm_io pkey = {0};

            if (event->passkey.params.action == BLE_SM_IOACT_DISP) {
                pkey.action = event->passkey.params.action;
                pkey.passkey = FIXED_PASSKEY;

                ESP_LOGI(TAG, "Enter passkey %lu on the peer side", (unsigned long)pkey.passkey);
                rc = ble_sm_inject_io(event->passkey.conn_handle, &pkey);
                ESP_LOGI(TAG, "ble_sm_inject_io result: %d", rc);
            } else if (event->passkey.params.action == BLE_SM_IOACT_INPUT ||
                      event->passkey.params.action == BLE_SM_IOACT_NUMCMP ||
                      event->passkey.params.action == BLE_SM_IOACT_OOB) {
                // For Just Works with MITM, always accept comparison
                if (event->passkey.params.action == BLE_SM_IOACT_NUMCMP) {
                    pkey.action = event->passkey.params.action;
                    pkey.numcmp_accept = 1;
                    ESP_LOGI(TAG, "Accepting numeric comparison for Just Works pairing");
                    rc = ble_sm_inject_io(event->passkey.conn_handle, &pkey);
                    ESP_LOGI(TAG, "Numeric comparison accept status: %d", rc);
                } else {
                    ESP_LOGW(TAG, "BLE_SM_IOACT_INPUT or BLE_SM_IOACT_OOB not fully supported");
                }
            } else {
                ESP_LOGW(TAG, "Unsupported passkey action: %d", event->passkey.params.action);
            }
            return 0;

        case BLE_GAP_EVENT_ENC_CHANGE:
            ESP_LOGI(TAG, "Encryption change event (status=%d)", event->enc_change.status);
            if (event->enc_change.status == 0) {
                // Encryption enabled - we've paired successfully
                ESP_LOGI(TAG, "Pairing/bonding successful");
                ble_hid_on_pairing_complete(event);
            } else {
                ESP_LOGE(TAG, "Pairing failed (status=%d)", event->enc_change.status);
            }
            break;

        case BLE_GAP_EVENT_REPEAT_PAIRING:
            /* We already have a bond with the peer, but it is attempting to
             * establish a new secure link. This app sacrifices security for
             * convenience: just throw away the old bond and accept the new link.
             */
            ESP_LOGI(TAG, "Repeat pairing event - deleting old bond");

            // Delete the old bond.
            struct ble_gap_conn_desc desc;
            int rc = ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
            assert(rc == 0);
            ble_store_util_delete_peer(&desc.peer_id_addr);
            ESP_LOGI(TAG, "Old bond deleted, retry pairing");

            /* Return BLE_GAP_REPEAT_PAIRING_RETRY to indicate that the host should
             * continue with the pairing operation.
             */
            return BLE_GAP_REPEAT_PAIRING_RETRY;
    }

    return 0;
}

/**
 * BLE GAP event handler for common HID device events
 */
int ble_hid_gap_event(struct ble_gap_event *event, void *arg) {
    ESP_LOGD(TAG, "BLE GAP event type: %d", event->type);

    // Debug print GAP event details based on type with enhanced information
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                // Get connection descriptor to access peer address
                struct ble_gap_conn_desc desc;
                char addr_str[18] = "unknown";
                int rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
                if (rc == 0) {
                    ble_addr_to_str(&desc.peer_ota_addr, addr_str);
                    ESP_LOGI(TAG, "GAP CONNECT: status=%d, handle=%d, peer=%s, type=%d",
                        event->connect.status, event->connect.conn_handle,
                        addr_str, desc.peer_ota_addr.type);

                    // Log connection parameters

                    ESP_LOGI(TAG, "Connection parameters: itvl=%d (%.2fms), latency=%d, supervision=%d",
                        desc.conn_itvl, desc.conn_itvl * 1.25,
                        desc.conn_latency, desc.supervision_timeout);
                }
            } else {
                ESP_LOGI(TAG, "GAP CONNECT FAILED: status=%d, handle=%d",
                    event->connect.status, event->connect.conn_handle);
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT: {
            char reason_name[32] = "UNKNOWN";

            /* Map common disconnect reason codes to names */
            switch (event->disconnect.reason) {
                case BLE_ERR_CONN_TERM_LOCAL:
                    strcpy(reason_name, "LOCAL_TERMINATION"); break;
                case BLE_ERR_CONN_TERM_MIC:
                    strcpy(reason_name, "MIC_FAILURE"); break;
                case BLE_ERR_CONN_ESTABLISHMENT:
                    strcpy(reason_name, "ESTABLISHMENT_FAILED"); break;
                case BLE_ERR_UNSUPP_REM_FEATURE:
                    strcpy(reason_name, "UNSUPPORTED_FEATURE"); break;
                case 517:
                    strcpy(reason_name, "UBUNTU_BLUEZ_UNPAIR(517)"); break;
            }

            ESP_LOGI(TAG, "GAP DISCONNECT: reason=0x%04x (%s), conn_handle=%d",
                     event->disconnect.reason, reason_name,
                     event->disconnect.conn.conn_handle);

            // Add BLE stack state for better disconnect debugging
            ESP_LOGW(TAG, "BLE stack state: conn_active=%d, adv_active=%d, HS ready=%d",
                ble_gap_conn_active(), ble_gap_adv_active(),
                ble_hs_is_enabled());
            break;
        }

        case BLE_GAP_EVENT_ENC_CHANGE:
            ESP_LOGI(TAG, "GAP ENC_CHANGE: status=%d, handle=%d",
                event->enc_change.status, event->enc_change.conn_handle);
            break;

        case BLE_GAP_EVENT_PASSKEY_ACTION:
            ESP_LOGI(TAG, "GAP PASSKEY_ACTION: action=%d (%s), conn_handle=%d",
                event->passkey.params.action,
                (event->passkey.params.action == BLE_SM_IOACT_DISP) ? "DISPLAY" :
                (event->passkey.params.action == BLE_SM_IOACT_INPUT) ? "INPUT" :
                (event->passkey.params.action == BLE_SM_IOACT_NUMCMP) ? "NUMCMP" : "UNKNOWN",
                event->passkey.conn_handle);
            break;

        // REPEAT_PAIRING is already handled in the security event handler above

        case BLE_GAP_EVENT_CONN_UPDATE:
            {
                struct ble_gap_conn_desc desc;
                int rc = ble_gap_conn_find(event->conn_update.conn_handle, &desc);
                if (rc == 0) {
                    ESP_LOGI(TAG, "GAP CONN_UPDATE: status=%d, conn_handle=%d, params: itvl=%.2fms, latency=%d, supervision=%dms",
                        event->conn_update.status, event->conn_update.conn_handle,
                        desc.conn_itvl * 1.25, desc.conn_latency,
                        desc.supervision_timeout * 10);
                } else {
                    ESP_LOGI(TAG, "GAP CONN_UPDATE: status=%d, conn_handle=%d (params unavailable)",
                        event->conn_update.status, event->conn_update.conn_handle);
                }
            }
            break;

        case BLE_GAP_EVENT_SUBSCRIBE:
            ESP_LOGI(TAG, "GAP SUBSCRIBE: conn=%d attr=%d reason=%d notify=%d=>%d indicate=%d=>%d",
                event->subscribe.conn_handle,
                event->subscribe.attr_handle,
                event->subscribe.reason,
                event->subscribe.prev_notify,
                event->subscribe.cur_notify,
                event->subscribe.prev_indicate,
                event->subscribe.cur_indicate);
            break;

        default:
            ESP_LOGD(TAG, "GAP event type %d (no detailed logging for this type)", event->type);
            break;
    }

    // Forward ALL security-related events to our security handler
    if (event->type == BLE_GAP_EVENT_ENC_CHANGE ||
        event->type == BLE_GAP_EVENT_REPEAT_PAIRING ||
        event->type == BLE_GAP_EVENT_PASSKEY_ACTION) {
        ESP_LOGI(TAG, "Forwarding security-related event %d to security handler", event->type);
        return ble_handle_security_event(event, arg);
    }

    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                // Connection established (single connection architecture)
                ble_hid_dev_state.connected = true;
                ble_hid_dev_state.conn_handle = event->connect.conn_handle;
                ESP_LOGI(TAG, "Connection established (handle=%d)", event->connect.conn_handle);

                // Immediately initiate security procedure as the peripheral device
                // This is critical for Ubuntu Bluez HID connections to start pairing
                ESP_LOGI(TAG, "Initiating security procedure as peripheral (handle=%d)", event->connect.conn_handle);
                ble_hid_dev_state.pairing_in_progress = true;
                int sec_rc = ble_gap_security_initiate(event->connect.conn_handle);
                if (sec_rc != 0) {
                    ESP_LOGW(TAG, "Failed to initiate security procedure: %d", sec_rc);
                } else {
                    ESP_LOGI(TAG, "Security procedure initiated successfully");
                }

                // Set connection parameters for better Ubuntu HID compatibility
                // Using more conservative parameters for Ubuntu
                struct ble_gap_upd_params params = {
                    .itvl_min = 24,    // 30ms (24 * 1.25ms)
                    .itvl_max = 40,    // 50ms (40 * 1.25ms)
                    .latency = 0,      // No slave latency for Ubuntu
                    .supervision_timeout = 100, // 1s (100 * 10ms)
                    .min_ce_len = 0,
                    .max_ce_len = 0,
                };

                ESP_LOGI(TAG, "Updating connection parameters for Ubuntu compatibility");
                int rc = ble_gap_update_params(event->connect.conn_handle, &params);
                if (rc != 0) {
                    ESP_LOGW(TAG, "Failed to update connection parameters: %d", rc);
                }

                // Stop advertising once connected
                ESP_LOGI(TAG, "Connection established - stopping advertising");
            } else {
                ESP_LOGE(TAG, "Connection failed with status: %d", event->connect.status);
                // Restart advertising after connection failure
                ble_hid_device_start_advertising();
            }
            return 0;

        case BLE_GAP_EVENT_DISCONNECT: {
            char reason_name[32] = "UNKNOWN";

            // Map common disconnect reason codes to names for better diagnostics
            switch (event->disconnect.reason) {
                case BLE_ERR_CONN_TERM_LOCAL:
                    strcpy(reason_name, "LOCAL_TERMINATION"); break;
                case BLE_ERR_CONN_TERM_MIC:
                    _init_adv_normal();   // Normal restart for MIC failures
                    break;
                case BLE_ERR_CONN_ESTABLISHMENT:
                    strcpy(reason_name, "ESTABLISHMENT_FAILED"); break;
                case BLE_ERR_UNSUPP_REM_FEATURE:
                    strcpy(reason_name, "UNSUPPORTED_FEATURE"); break;
                case 517:
                    strcpy(reason_name, "UBUNTU_BLUEZ_UNPAIR(517)"); break;
            }
            ESP_LOGI(TAG, "Disconnected: reason=0x%04x (%s), conn_handle=%d",
                     event->disconnect.reason, reason_name,
                     event->disconnect.conn.conn_handle);

            // Check BLE stack state for diagnostics
            ESP_LOGW(TAG, "BLE stack state after disconnect: conn_active=%d, adv_active=%d, HS enabled=%d",
                    ble_gap_conn_active(), ble_gap_adv_active(), ble_hs_is_enabled());

            // Reset connection data (single connection architecture)
            ble_hid_dev_state.connected = false;
            ble_hid_dev_state.conn_handle = BLE_HS_CONN_HANDLE_NONE;
            ESP_LOGI(TAG, "Connection terminated");

            // Clear pairing state
            ble_hid_dev_state.pairing_in_progress = false;

            // If disconnect reason is 517, Ubuntu host has unpairing issues
            // This requires special handling to ensure clean reconnection
            if (event->disconnect.reason == 517) {
                ESP_LOGW(TAG, "Ubuntu disconnect reason 517 detected - applying special handling");
                ESP_LOGW(TAG, "Applying targeted bond management for Ubuntu Bluez compatibility");

                // For Ubuntu Bluez disconnect reason 517, we need to handle it carefully
                // We'll only delete the specific peer bond rather than clearing all bonds
                // This helps maintain other device connections while fixing the problematic one

                ESP_LOGI(TAG, "Selectively deleting bond for the disconnected peer");
                if (ble_store_util_delete_peer(&event->disconnect.conn.peer_id_addr) == 0) {
                    ESP_LOGI(TAG, "Successfully deleted bond for the disconnected peer");

                    // Verify bond deletion success by checking if any bonds remain for this peer
                    struct ble_store_key_sec key_sec;
                    memset(&key_sec, 0, sizeof(key_sec));
                    key_sec.peer_addr = event->disconnect.conn.peer_id_addr;

                    struct ble_store_value_sec value_sec;
                    int rc = ble_store_read_peer_sec(&key_sec, &value_sec);
                    if (rc == 0) {
                        ESP_LOGW(TAG, "Bond still exists after deletion attempt - forcing complete store clear");
                        ble_store_clear();
                    } else {
                        ESP_LOGI(TAG, "Verified bond was successfully deleted");
                    }
                } else {
                    ESP_LOGW(TAG, "Failed to delete bond for specific peer - may already be deleted");
                }

                // Wait longer for Ubuntu - this is critical to allow stack reset
                ESP_LOGW(TAG, "Adding extended delay before advertising restart to allow BLE stack reset");
                vTaskDelay(pdMS_TO_TICKS(1500));  /* Give the BLE stack more time to reset */

                // Reset advertising and connection state completely
                ble_hid_dev_state.adv_retry_count = 0;
                ble_hid_dev_state.pairing_in_progress = false;
                ESP_LOGI(TAG, "Reset all state after Ubuntu disconnect");
            } else if (event->disconnect.reason == 0x0213 || event->disconnect.reason == BLE_ERR_REM_USER_CONN_TERM) {
                // For normal disconnections initiated by the host (Ubuntu)
                ESP_LOGI(TAG, "Normal disconnect from host - adding delay before advertising");

                // Add a delay to prevent immediate auto-reconnection
                ESP_LOGI(TAG, "Adding delay before advertising to prevent auto-reconnect");
                vTaskDelay(pdMS_TO_TICKS(3000)); // 3 second delay before advertising again

                // Reset advertising retry counter
                ble_hid_dev_state.adv_retry_count = 0;
            } else {
                // For other disconnect reasons
                // Reset some state
                ble_hid_dev_state.adv_retry_count = 0;
            }

            // Restart advertising after disconnect
            ble_hid_device_start_advertising();
            return 0;
        }

        case BLE_GAP_EVENT_ADV_COMPLETE:
            ESP_LOGI(TAG, "Advertising complete");
            // Restart advertising if we can support more connections
            ble_hid_device_start_advertising();
            return 0;

        // Handle encryption state change events - critical for Ubuntu pairing
        case BLE_GAP_EVENT_ENC_CHANGE:
            ESP_LOGI(TAG, "Encryption change event, status=%d", event->enc_change.status);
            if (event->enc_change.status == 0) {
                ESP_LOGI(TAG, "Security procedure successful - encrypted connection established");
                ble_hid_on_pairing_complete(event);
            } else {
                ESP_LOGE(TAG, "Security procedure failed: %d", event->enc_change.status);
                ble_hid_dev_state.pairing_in_progress = false;
            }
            return 0;

        // Handle security manager protocol events
        case BLE_GAP_EVENT_PASSKEY_ACTION:
            ESP_LOGI(TAG, "Passkey action event received");
            return 0;

        // Handle pair/bond events - note the spelling in ESP-IDF NimBLE
        case BLE_GAP_EVENT_IDENTITY_RESOLVED:
            ESP_LOGI(TAG, "Identity resolved event");
            return 0;

        // v3.x of ESP-IDF doesn't have more security events
        // We rely on ENC_CHANGE for most security status updates

        case BLE_GAP_EVENT_CONN_UPDATE:
            if (event->conn_update.status == 0) {
                ESP_LOGI(TAG, "Connection parameters updated");
            }
            return 0;

        case BLE_GAP_EVENT_SUBSCRIBE:
            ESP_LOGI(TAG, "Subscribe event; conn_handle=%d attr_handle=%04X reason=%d "
                      "prev_notify=%d cur_notify=%d prev_indicate=%d cur_indicate=%d",
                      event->subscribe.conn_handle,
                      event->subscribe.attr_handle,
                      event->subscribe.reason,
                      event->subscribe.prev_notify,
                      event->subscribe.cur_notify,
                      event->subscribe.prev_indicate,
                      event->subscribe.cur_indicate);

            // Forward subscribe events to appropriate HID services
            // This is important for Ubuntu to correctly receive HID reports
            return 0;

        case BLE_GAP_EVENT_MTU:
            ESP_LOGI(TAG, "MTU update event; conn_handle=%d cid=%d mtu=%d",
                   event->mtu.conn_handle,
                   event->mtu.channel_id,
                   event->mtu.value);
            return 0;

        // Handle any unknown events gracefully
        case 27:
        case 34:
        case 38:
            /* These are harmless unknown events that might occur in the working example */
            ESP_LOGI(TAG, "Received unknown (but harmless) GAP event: %d", event->type);
            return 0;

        default:
            ESP_LOGD(TAG, "Unknown GAP event: %d", event->type);
            return 0;
    }

    return 0;
}

/**
 * Get the BLE GAP event handler for the HID device
 */
ble_gap_event_fn *ble_hid_get_gap_event_handler(void)
{
    return ble_hid_gap_event;
}

/**
 * Initialize common BLE HID device functionality
 */
esp_err_t ble_hid_device_init(void) {
    ESP_LOGI(TAG, "Initializing BLE HID Device");

    // Security parameters specifically optimized for Ubuntu Bluez HID compatibility
    ESP_LOGI(TAG, "=== Setting security parameters for maximum Ubuntu Bluez compatibility ===");

    // Set security manager parameters for Ubuntu Bluez compatibility
    // Use NO_INPUT_NO_OUTPUT for just-works pairing which is more compatible
    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;
    ESP_LOGI(TAG, "SM: IO Capability = %d (NO_IO)", ble_hs_cfg.sm_io_cap);
    ble_hs_cfg.sm_oob_data_flag = 0;
    ble_hs_cfg.sm_bonding = 1;     // Enable bonding
    ble_hs_cfg.sm_mitm = 0;        // Disable MITM for just-works compatibility
    ble_hs_cfg.sm_sc = 0;          // Disable Secure Connections for Ubuntu compatibility

    // Use conservative key distribution values that work reliably with Ubuntu Bluez
    // Encryption key only (bit 0) ensures maximum compatibility
    ble_hs_cfg.sm_our_key_dist = 1;   // Encryption key only
    ble_hs_cfg.sm_their_key_dist = 1; // Encryption key only

    // Set our callback as the status callback to handle store events
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    ESP_LOGI(TAG, "Security configured with No-IO capability for just-works pairing with Ubuntu");
    ESP_LOGI(TAG, "Using legacy pairing with bonding enabled, MITM protection disabled");
    ESP_LOGI(TAG, "Conservative key distribution (encryption key only) for maximum compatibility");
    ESP_LOGI(TAG, "Key distribution settings: our_keys=0x%x, their_keys=0x%x",
             ble_hs_cfg.sm_our_key_dist, ble_hs_cfg.sm_their_key_dist);

    // Initialize bond storage configuration - critical for Ubuntu compatibility
    ESP_LOGI(TAG, "Initializing BLE bond storage configuration");
    ble_store_config_init();

    // Device name will be set in ble_hid_on_sync() when MAC address is available
    // Register appearance only 
    ble_svc_gap_device_appearance_set(0x03C2); // HID mouse device

    // Initialize connection state tracking
    memset(&ble_hid_dev_state, 0, sizeof(ble_hid_dev_state));
    ble_hid_dev_state.connected = false;
    ble_hid_dev_state.conn_handle = BLE_HS_CONN_HANDLE_NONE;

    // Create timers for advertising management
    // - Restart timer for delayed advertising after disconnect
    // - Retry timer for handling BLE_HS_EBUSY errors with exponential backoff
    esp_timer_create_args_t restart_timer_args = {
        .callback = adv_restart_timer_cb,
        .name = "adv_restart_timer"
    };
    esp_err_t err = esp_timer_create(&restart_timer_args, &ble_hid_dev_state.adv_restart_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create advertising restart timer: %d", err);
        return ESP_FAIL;
    }

    esp_timer_create_args_t retry_timer_args = {
        .callback = adv_retry_timer_cb,
        .name = "adv_retry_timer"
    };
    err = esp_timer_create(&retry_timer_args, &ble_hid_dev_state.adv_retry_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create advertising retry timer: %d", err);
        return ESP_FAIL;
    }

    // Register NimBLE sync callback to restart advertising after BLE resets
    ble_hs_cfg.sync_cb = ble_hid_on_sync;

    return ESP_OK;
}

/**
 * Check if the BLE HID device is connected (single connection architecture)
 */
bool ble_hid_is_connected(void) {
    return ble_hid_dev_state.connected;
}

/**
 * Check if focus can be switched safely
 */
bool ble_hid_can_switch_focus(void) {
    // Check if any mouse buttons are currently pressed
    if (ble_hid_mouse_get_button_state() != 0) {
        ESP_LOGD(TAG, "Focus locked - mouse buttons pressed: 0x%02x", ble_hid_mouse_get_button_state());
        return false;
    }
    
    // TODO: Add keyboard state check here when keyboard API is implemented
    // if (ble_hid_keyboard_has_keys_pressed()) {
    //     ESP_LOGD(TAG, "Focus locked - keyboard keys pressed");
    //     return false;
    // }
    
    return true;
}

