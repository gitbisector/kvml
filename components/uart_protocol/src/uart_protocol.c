#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "uart_protocol.h"
#include "uart_frame.h"
#include "ble_hid_mouse.h"
#include "ble_hid_keyboard.h"
#include "usb_host.h"  // For direct USB LED control

static const char *TAG = "UART_PROTOCOL";

// UART driver handle and status
static bool uart_initialized = false;
static QueueHandle_t uart_queue = NULL;

// Background listener task
static TaskHandle_t uart_listener_task_handle = NULL;
static bool uart_listener_running = false;

// Input routing state (only one board should be active at a time)
static bool input_routing_active = true;  // Start as active by default

// Track last forwarded keyboard report to avoid sending duplicates
static uint8_t last_forwarded_report[8] = {0};
static bool last_report_valid = false;

esp_err_t uart_protocol_init(void)
{
    ESP_LOGI(TAG, "Initializing UART protocol physical layer");
    
    if (uart_initialized) {
        ESP_LOGW(TAG, "UART protocol already initialized");
        return ESP_OK;
    }
    
    // Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_BITS,
        .parity = UART_PARITY,
        .stop_bits = UART_STOP_BITS,
        .flow_ctrl = UART_FLOW_CTRL,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_LOGI(TAG, "Configuring UART%d: %d baud, 8N1, pins TX=%d RX=%d", 
             UART_NUM, UART_BAUD_RATE, UART_TX_PIN, UART_RX_PIN);
    
    // Configure UART parameters
    esp_err_t ret = uart_param_config(UART_NUM, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART parameters: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Set UART pins
    ret = uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_RTS_PIN, UART_CTS_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Install UART driver with event queue
    ret = uart_driver_install(UART_NUM, UART_RX_BUF_SIZE, UART_TX_BUF_SIZE, 
                             UART_QUEUE_SIZE, &uart_queue, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Set UART threshold and timeout
    ret = uart_set_rx_timeout(UART_NUM, UART_FRAME_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set RX timeout: %s", esp_err_to_name(ret));
        // Continue anyway, this is not critical
    }
    
    // Clear any existing data in buffers
    uart_flush(UART_NUM);
    
    uart_initialized = true;
    ESP_LOGI(TAG, "UART protocol physical layer initialized successfully");
    
    return ESP_OK;
}

esp_err_t uart_protocol_deinit(void)
{
    ESP_LOGI(TAG, "Deinitializing UART protocol");
    
    if (!uart_initialized) {
        ESP_LOGW(TAG, "UART protocol not initialized");
        return ESP_OK;
    }
    
    // Stop listener task first
    uart_protocol_stop_listener();
    
    // Uninstall UART driver
    esp_err_t ret = uart_driver_delete(UART_NUM);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to delete UART driver: %s", esp_err_to_name(ret));
        return ret;
    }
    
    uart_queue = NULL;
    uart_initialized = false;
    
    ESP_LOGI(TAG, "UART protocol deinitialized successfully");
    return ESP_OK;
}

int uart_protocol_send(const uint8_t *data, size_t length)
{
    if (!uart_initialized) {
        ESP_LOGE(TAG, "UART protocol not initialized");
        return -1;
    }
    
    if (data == NULL || length == 0) {
        ESP_LOGE(TAG, "Invalid send parameters");
        return -1;
    }
    
    // Send data over UART
    int bytes_sent = uart_write_bytes(UART_NUM, data, length);
    if (bytes_sent < 0) {
        ESP_LOGE(TAG, "Failed to send data over UART");
        return -1;
    }
    
    // Wait for transmission to complete
    esp_err_t ret = uart_wait_tx_done(UART_NUM, pdMS_TO_TICKS(UART_TIMEOUT_MS));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "UART transmission timeout: %s", esp_err_to_name(ret));
        // Data may still have been sent, so don't return error
    }
    
    ESP_LOGD(TAG, "Sent %d bytes over UART", bytes_sent);
    return bytes_sent;
}

int uart_protocol_receive(uint8_t *data, size_t max_length, uint32_t timeout_ms)
{
    if (!uart_initialized) {
        ESP_LOGE(TAG, "UART protocol not initialized");
        return -1;
    }
    
    if (data == NULL || max_length == 0) {
        ESP_LOGE(TAG, "Invalid receive parameters");
        return -1;
    }
    
    // Read data from UART with timeout
    int bytes_received = uart_read_bytes(UART_NUM, data, max_length, 
                                        pdMS_TO_TICKS(timeout_ms));
    
    if (bytes_received < 0) {
        ESP_LOGE(TAG, "Failed to receive data from UART");
        return -1;
    }
    
    if (bytes_received > 0) {
        ESP_LOGD(TAG, "Received %d bytes from UART", bytes_received);
    }
    
    return bytes_received;
}

static void uart_protocol_send_auto_response(uint8_t original_command, const uint8_t* original_payload, uint16_t original_length)
{
    ESP_LOGI(TAG, "Sending auto-response to command 0x%02x", original_command);
    
    // Choose response command based on received command
    uint8_t response_cmd;
    const char* response_payload = NULL;
    size_t response_len = 0;
    
    switch (original_command) {
        case UART_CMD_TEST_LOOPBACK:
            response_cmd = UART_CMD_TEST_RESPONSE;
            response_payload = "TEST_ACK";
            response_len = strlen(response_payload);
            break;
            
        case UART_CMD_PING:
            response_cmd = UART_CMD_PONG;
            response_payload = "PONG";
            response_len = strlen(response_payload);
            break;
            
        default:
            ESP_LOGW(TAG, "No auto-response defined for command 0x%02x", original_command);
            return;
    }
    
    // Small delay to avoid collision with sender's receive attempt
    vTaskDelay(pdMS_TO_TICKS(UART_AUTO_RESPONSE_DELAY_MS));
    
    // Create response frame
    uart_frame_t response_frame;
    esp_err_t ret = uart_frame_create(response_cmd, (const uint8_t*)response_payload, response_len, &response_frame);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create response frame: %s", esp_err_to_name(ret));
        return;
    }
    
    // Serialize and send response
    uint8_t response_buffer[64];
    uint16_t bytes_written;
    ret = uart_frame_serialize(&response_frame, response_buffer, sizeof(response_buffer), &bytes_written);
    if (ret == ESP_OK) {
        int sent = uart_protocol_send(response_buffer, bytes_written);
        if (sent == bytes_written) {
            ESP_LOGI(TAG, "Auto-response sent: cmd=0x%02x, %d bytes", response_cmd, sent);
        } else {
            ESP_LOGW(TAG, "Auto-response send failed: sent %d bytes, expected %d", sent, bytes_written);
        }
    } else {
        ESP_LOGE(TAG, "Failed to serialize response frame: %s", esp_err_to_name(ret));
    }
    
    // Cleanup
    if (response_frame.payload) {
        free(response_frame.payload);
    }
}

static void uart_listener_task(void *param)
{
    ESP_LOGI(TAG, "UART listener task started");
    
    // Initialize frame handling
    esp_err_t ret = uart_frame_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize frame handling in listener: %s", esp_err_to_name(ret));
        uart_listener_running = false;
        vTaskDelete(NULL);
        return;
    }
    
    uint8_t rx_buffer[128];
    uart_event_t event;
    
    while (uart_listener_running) {
        // Wait for UART events (event-driven, no polling!)
        if (xQueueReceive(uart_queue, &event, pdMS_TO_TICKS(100))) {
            switch (event.type) {
                case UART_DATA:
                    // Data available - read it immediately without timeout
                    int received = uart_read_bytes(UART_NUM, rx_buffer, sizeof(rx_buffer), 0);
                    
                    if (received > 0) {
                        ESP_LOGD(TAG, "Listener received %d bytes from UART event, attempting frame parse", received);
                        
                        // Try to parse as a frame
                        uart_frame_t frame;
                        ret = uart_frame_parse(rx_buffer, received, &frame);
                        
                        if (ret == ESP_OK) {
                            ESP_LOGD(TAG, "Listener parsed valid frame: cmd=0x%02x, len=%d", frame.command, frame.length);
                            
                            // Check if this command needs an auto-response
                            if (frame.command == UART_CMD_TEST_LOOPBACK || frame.command == UART_CMD_PING) {
                                uart_protocol_send_auto_response(frame.command, frame.payload, frame.length);
                            }
                            
                            // Handle incoming USB HID reports from neighbor board
                            if (frame.command == UART_CMD_USB_MOUSE && frame.length == 4) {
                                // Received processed mouse data: [delta_x, delta_y, ble_buttons, scroll_y]
                                int8_t delta_x = (int8_t)frame.payload[0];
                                int8_t delta_y = (int8_t)frame.payload[1];
                                uint8_t ble_buttons = frame.payload[2];
                                int8_t scroll_y = (int8_t)frame.payload[3];
                                
                                ESP_LOGD(TAG, "Received mouse report from neighbor: delta=(%d,%d), buttons=0x%02x, scroll=%d",
                                         delta_x, delta_y, ble_buttons, scroll_y);
                                
                                // Only forward to BLE if this board is currently active
                                if (uart_protocol_is_input_active()) {
                                    ESP_LOGD(TAG, "This board is active - forwarding neighbor mouse report to BLE");
                                    esp_err_t ret = ble_hid_mouse_report_enhanced(delta_x, delta_y, ble_buttons, scroll_y);
                                    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
                                        ESP_LOGD(TAG, "Failed to forward neighbor mouse report to BLE: %s", esp_err_to_name(ret));
                                    }
                                } else {
                                    ESP_LOGD(TAG, "This board is inactive - ignoring neighbor mouse report");
                                }
                            } else if (frame.command == UART_CMD_USB_KEYBOARD && frame.length == 8) {
                                // Received raw keyboard HID report: [modifier, reserved, key0, key1, key2, key3, key4, key5]
                                uint8_t modifier = frame.payload[0];
                                uint8_t reserved = frame.payload[1];
                                uint8_t *keys = &frame.payload[2];  // Points to 6-byte key array
                                
                                ESP_LOGI(TAG, "Received keyboard report from neighbor: mod=0x%02x keys=%02x,%02x,%02x,%02x,%02x,%02x",
                                         modifier, keys[0], keys[1], keys[2], keys[3], keys[4], keys[5]);
                                
                                // Only forward to BLE if this board is currently active
                                if (uart_protocol_is_input_active()) {
                                    ESP_LOGI(TAG, "This board is active - forwarding neighbor keyboard report to BLE");
                                    esp_err_t ret = ble_hid_keyboard_report(modifier, reserved, keys);
                                    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
                                        ESP_LOGD(TAG, "Failed to forward neighbor keyboard report to BLE: %s", esp_err_to_name(ret));
                                    }
                                } else {
                                    ESP_LOGI(TAG, "This board is inactive - ignoring neighbor keyboard report");
                                }
                            } else if (frame.command == UART_CMD_BLE_LED_STATE && frame.length == 1) {
                                // Received BLE LED state from neighbor (neighbor is active host)
                                uint8_t led_state = frame.payload[0];
                                
                                ESP_LOGI(TAG, "Received LED state from neighbor (active host): 0x%02x", led_state);
                                ESP_LOGI(TAG, "  NumLock: %s, CapsLock: %s, ScrollLock: %s", 
                                         (led_state & 0x01) ? "ON" : "OFF",  // HID_LED_NUM_LOCK
                                         (led_state & 0x02) ? "ON" : "OFF",  // HID_LED_CAPS_LOCK  
                                         (led_state & 0x04) ? "ON" : "OFF"); // HID_LED_SCROLL_LOCK
                                
                                // Apply neighbor's LED state directly to our USB keyboard
                                esp_err_t ret = usb_host_send_keyboard_output_report(led_state);
                                if (ret != ESP_OK) {
                                    ESP_LOGW(TAG, "Failed to apply neighbor LED state to USB: %s", esp_err_to_name(ret));
                                } else {
                                    ESP_LOGI(TAG, "Applied neighbor LED state to USB keyboard: 0x%02x", led_state);
                                }
                            }
                            
                            // Handle host switching commands
                            if (frame.command == UART_CMD_HOST_SWITCH_REQUEST) {
                                ESP_LOGI(TAG, "Received host switch request from neighbor board");
                                // TODO: Implement host switching logic
                                // For now, just acknowledge the request
                                ESP_LOGI(TAG, "TODO: Implement host switching logic");
                            } else if (frame.command == UART_CMD_HOST_SWITCH_ACK) {
                                ESP_LOGI(TAG, "Received host switch acknowledgment from neighbor board");
                                // TODO: Complete host switching process
                                ESP_LOGI(TAG, "TODO: Complete host switching process");
                            }
                            
                            // Handle input routing commands
                            if (frame.command == UART_CMD_INPUT_ACTIVE) {
                                ESP_LOGI(TAG, "Received command to become ACTIVE input forwarder");
                                input_routing_active = true;
                            } else if (frame.command == UART_CMD_INPUT_INACTIVE) {
                                ESP_LOGI(TAG, "Received command to become INACTIVE (send idle reports)");
                                input_routing_active = false;
                            }
                            
                            // Cleanup frame
                            if (frame.payload) {
                                free(frame.payload);
                            }
                        } else {
                            ESP_LOGD(TAG, "Listener received data but failed to parse as frame: %s", esp_err_to_name(ret));
                        }
                    }
                    break;
                    
                case UART_FIFO_OVF:
                    ESP_LOGW(TAG, "UART FIFO overflow");
                    uart_flush_input(UART_NUM);
                    xQueueReset(uart_queue);
                    break;
                    
                case UART_BUFFER_FULL:
                    ESP_LOGW(TAG, "UART ring buffer full");
                    uart_flush_input(UART_NUM);
                    xQueueReset(uart_queue);
                    break;
                    
                case UART_BREAK:
                    ESP_LOGD(TAG, "UART break detected");
                    break;
                    
                case UART_PARITY_ERR:
                    ESP_LOGW(TAG, "UART parity error");
                    break;
                    
                case UART_FRAME_ERR:
                    ESP_LOGW(TAG, "UART frame error");
                    break;
                    
                default:
                    ESP_LOGD(TAG, "UART event type: %d", event.type);
                    break;
            }
        } else {
            // Timeout - no UART events, which is normal
            // The 100ms timeout provides natural yielding
        }
    }
    
    ESP_LOGI(TAG, "UART listener task stopping");
    uart_listener_task_handle = NULL;
    vTaskDelete(NULL);
}

esp_err_t uart_protocol_start_listener(void)
{
    if (!uart_initialized) {
        ESP_LOGE(TAG, "UART protocol not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (uart_listener_running) {
        ESP_LOGW(TAG, "UART listener already running");
        return ESP_OK;
    }
    
    uart_listener_running = true;
    
    BaseType_t result = xTaskCreate(
        uart_listener_task,
        "uart_listener",
        4096,  // Stack size
        NULL,  // Parameters
        5,     // Priority (medium)
        &uart_listener_task_handle
    );
    
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create UART listener task");
        uart_listener_running = false;
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "UART listener task started successfully");
    return ESP_OK;
}

esp_err_t uart_protocol_stop_listener(void)
{
    if (!uart_listener_running) {
        ESP_LOGW(TAG, "UART listener not running");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Stopping UART listener task");
    uart_listener_running = false;
    
    // Wait for task to finish
    if (uart_listener_task_handle != NULL) {
        // Give it some time to stop gracefully
        for (int i = 0; i < 50 && uart_listener_task_handle != NULL; i++) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        
        if (uart_listener_task_handle != NULL) {
            ESP_LOGW(TAG, "Force deleting UART listener task");
            vTaskDelete(uart_listener_task_handle);
            uart_listener_task_handle = NULL;
        }
    }
    
    ESP_LOGI(TAG, "UART listener task stopped");
    return ESP_OK;
}

esp_err_t uart_protocol_test_loopback(void)
{
    ESP_LOGI(TAG, "Starting UART frame loopback test with CRC");
    
    if (!uart_initialized) {
        ESP_LOGE(TAG, "UART protocol not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Initialize frame handling
    esp_err_t ret = uart_frame_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize frame handling: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Test message payload
    const char test_payload[] = "FRAME_TEST";
    const size_t payload_len = strlen(test_payload);
    const uint8_t test_command = UART_CMD_TEST_LOOPBACK; // Test command type
    
    // Clear any pending data
    uart_flush(UART_NUM);
    
    // Create test frame
    uart_frame_t tx_frame;
    ret = uart_frame_create(test_command, (const uint8_t*)test_payload, payload_len, &tx_frame);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create test frame: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Serialize frame for transmission
    uint8_t tx_buffer[64];
    uint16_t bytes_written;
    ret = uart_frame_serialize(&tx_frame, tx_buffer, sizeof(tx_buffer), &bytes_written);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to serialize frame: %s", esp_err_to_name(ret));
        if (tx_frame.payload) free(tx_frame.payload);
        return ret;
    }
    
    ESP_LOGI(TAG, "Sending frame: cmd=0x%02x, payload_len=%zu, total_frame_size=%d", 
             test_command, payload_len, bytes_written);
    
    // Send serialized frame
    int sent = uart_protocol_send(tx_buffer, bytes_written);
    if (sent != bytes_written) {
        ESP_LOGE(TAG, "Frame send failed: sent %d bytes, expected %d", sent, bytes_written);
        if (tx_frame.payload) free(tx_frame.payload);
        return ESP_FAIL;
    }
    
    // Wait for loopback or auto-response
    vTaskDelay(pdMS_TO_TICKS(20));
    
    // Receive frame
    uint8_t rx_buffer[64];
    int received = uart_protocol_receive(rx_buffer, sizeof(rx_buffer), 200);
    if (received <= 0) {
        ESP_LOGW(TAG, "Frame loopback: no data received (check GPIO%d-GPIO%d connection or neighbor board)", UART_TX_PIN, UART_RX_PIN);
        if (tx_frame.payload) free(tx_frame.payload);
        return ESP_ERR_NOT_FOUND;
    }
    
    ESP_LOGI(TAG, "Received %d bytes, parsing frame...", received);
    
    // Parse received frame
    uart_frame_t rx_frame;
    ret = uart_frame_parse(rx_buffer, received, &rx_frame);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Frame parse failed: %s", esp_err_to_name(ret));
        if (tx_frame.payload) free(tx_frame.payload);
        return ESP_FAIL;
    }
    
    // Check if this is a loopback (same command) or auto-response
    bool test_passed = true;
    bool is_auto_response = (rx_frame.command == UART_CMD_TEST_RESPONSE);
    
    if (is_auto_response) {
        ESP_LOGI(TAG, "Received auto-response: cmd=0x%02x (self-loopback or neighbor)", rx_frame.command);
        // For auto-response, just verify it's the expected response command
        if (rx_frame.command != UART_CMD_TEST_RESPONSE) {
            ESP_LOGE(TAG, "Unexpected response command: expected 0x%02x, got 0x%02x", 
                     UART_CMD_TEST_RESPONSE, rx_frame.command);
            test_passed = false;
        }
    } else {
        // Traditional loopback verification
        if (rx_frame.command != test_command) {
            ESP_LOGE(TAG, "Command mismatch: expected 0x%02x, got 0x%02x", test_command, rx_frame.command);
            test_passed = false;
        }
        if (rx_frame.length != payload_len) {
            ESP_LOGE(TAG, "Length mismatch: expected %zu, got %d", payload_len, rx_frame.length);
            test_passed = false;
        }
        if (rx_frame.payload && memcmp(rx_frame.payload, test_payload, payload_len) != 0) {
            ESP_LOGE(TAG, "Payload mismatch");
            test_passed = false;
        }
    }
    
    // Cleanup
    if (tx_frame.payload) free(tx_frame.payload);
    if (rx_frame.payload) free(rx_frame.payload);
    
    if (test_passed) {
        if (is_auto_response) {
            ESP_LOGI(TAG, "UART connectivity test PASSED (auto-response with CRC verification)");
        } else {
            ESP_LOGI(TAG, "UART frame loopback test PASSED (physical loopback with CRC verification)");
        }
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "UART frame test FAILED");
        return ESP_FAIL;
    }
}

esp_err_t uart_protocol_forward_mouse_report(const uint8_t *report_data, size_t report_length)
{
    if (!uart_initialized) {
        ESP_LOGD(TAG, "UART not initialized, skipping mouse report forward");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (report_data == NULL || report_length == 0) {
        ESP_LOGE(TAG, "Invalid mouse report parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGD(TAG, "Forwarding mouse report: %d bytes", report_length);
    
    // Create frame with USB mouse command
    uart_frame_t frame;
    esp_err_t ret = uart_frame_create(UART_CMD_USB_MOUSE, report_data, report_length, &frame);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create mouse report frame: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Serialize and send (fire-and-forget)
    uint8_t tx_buffer[128];  // Increased size for HID reports
    uint16_t bytes_written;
    ret = uart_frame_serialize(&frame, tx_buffer, sizeof(tx_buffer), &bytes_written);
    if (ret == ESP_OK) {
        int sent = uart_protocol_send(tx_buffer, bytes_written);
        if (sent == bytes_written) {
            ESP_LOGD(TAG, "Mouse report forwarded: %d bytes", sent);
        } else {
            ESP_LOGW(TAG, "Mouse report forward incomplete: sent %d bytes, expected %d", sent, bytes_written);
        }
    } else {
        ESP_LOGE(TAG, "Failed to serialize mouse report frame: %s", esp_err_to_name(ret));
    }
    
    // Cleanup
    if (frame.payload) {
        free(frame.payload);
    }
    
    return ret;
}

esp_err_t uart_protocol_forward_keyboard_report(const uint8_t *report_data, size_t report_length)
{
    if (!uart_initialized) {
        ESP_LOGW(TAG, "UART not initialized, skipping keyboard report forward");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (report_data == NULL || report_length == 0) {
        ESP_LOGE(TAG, "Invalid keyboard report parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Check if this report is the same as the last one we forwarded
    if (report_length == 8 && last_report_valid) {
        if (memcmp(report_data, last_forwarded_report, 8) == 0) {
            ESP_LOGD(TAG, "Skipping duplicate keyboard report");
            return ESP_OK;  // Don't forward duplicate reports
        }
    }
    
    // Log only non-idle keyboard reports to reduce spam
    if (report_length >= 8) {
        bool is_idle = (report_data[0] == 0x00);  // Check modifier
        for (int i = 2; i < 8 && is_idle; i++) {  // Check all key codes (skip reserved byte)
            if (report_data[i] != 0x00) {
                is_idle = false;
            }
        }
        
        if (!is_idle) {
            ESP_LOGI(TAG, "Forwarding keyboard report: mod=0x%02x keys=%02x,%02x,%02x,%02x,%02x,%02x", 
                     report_data[0], report_data[2], report_data[3], report_data[4], 
                     report_data[5], report_data[6], report_data[7]);
        } else {
            ESP_LOGD(TAG, "Forwarding idle keyboard report");
        }
    } else {
        ESP_LOGD(TAG, "Forwarding keyboard report: %d bytes", report_length);
    }
    
    // Create frame with USB keyboard command
    uart_frame_t frame;
    esp_err_t ret = uart_frame_create(UART_CMD_USB_KEYBOARD, report_data, report_length, &frame);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create keyboard report frame: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Serialize and send with better error reporting
    uint8_t tx_buffer[128];  // Increased size for HID reports
    uint16_t bytes_written;
    ret = uart_frame_serialize(&frame, tx_buffer, sizeof(tx_buffer), &bytes_written);
    if (ret == ESP_OK) {
        int sent = uart_protocol_send(tx_buffer, bytes_written);
        if (sent == bytes_written) {
            ESP_LOGD(TAG, "Keyboard report forwarded successfully: %d bytes", sent);
            
            // Update last forwarded report to avoid duplicates
            if (report_length == 8) {
                memcpy(last_forwarded_report, report_data, 8);
                last_report_valid = true;
            }
        } else {
            ESP_LOGE(TAG, "Keyboard report forward FAILED: sent %d bytes, expected %d", sent, bytes_written);
            ret = ESP_FAIL;
        }
    } else {
        ESP_LOGE(TAG, "Failed to serialize keyboard report frame: %s", esp_err_to_name(ret));
    }
    
    // Cleanup
    if (frame.payload) {
        free(frame.payload);
    }
    
    return ret;
}

esp_err_t uart_protocol_send_host_switch_request(void)
{
    if (!uart_initialized) {
        ESP_LOGD(TAG, "UART not initialized, skipping host switch request");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Sending host switch request to neighbor board");
    
    // Create frame with host switch request command (no payload needed)
    uart_frame_t frame;
    esp_err_t ret = uart_frame_create(UART_CMD_HOST_SWITCH_REQUEST, NULL, 0, &frame);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create host switch request frame: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Serialize and send (fire-and-forget)
    uint8_t tx_buffer[16];  // Small buffer for command frame
    uint16_t bytes_written;
    ret = uart_frame_serialize(&frame, tx_buffer, sizeof(tx_buffer), &bytes_written);
    if (ret == ESP_OK) {
        int sent = uart_protocol_send(tx_buffer, bytes_written);
        if (sent == bytes_written) {
            ESP_LOGI(TAG, "Host switch request sent: %d bytes", sent);
        } else {
            ESP_LOGW(TAG, "Host switch request send incomplete: sent %d bytes, expected %d", sent, bytes_written);
        }
    } else {
        ESP_LOGE(TAG, "Failed to serialize host switch request frame: %s", esp_err_to_name(ret));
    }
    
    // Cleanup
    if (frame.payload) {
        free(frame.payload);
    }
    
    return ret;
}

esp_err_t uart_protocol_send_input_routing_command(bool make_neighbor_active)
{
    if (!uart_initialized) {
        ESP_LOGD(TAG, "UART not initialized, skipping input routing command");
        return ESP_ERR_INVALID_STATE;
    }
    
    uint8_t command = make_neighbor_active ? UART_CMD_INPUT_ACTIVE : UART_CMD_INPUT_INACTIVE;
    ESP_LOGI(TAG, "Sending input routing command: make neighbor %s", 
             make_neighbor_active ? "ACTIVE" : "INACTIVE");
    
    // Create frame with input routing command (no payload needed)
    uart_frame_t frame;
    esp_err_t ret = uart_frame_create(command, NULL, 0, &frame);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create input routing command frame: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Serialize and send (fire-and-forget)
    uint8_t tx_buffer[16];  // Small buffer for command frame
    uint16_t bytes_written;
    ret = uart_frame_serialize(&frame, tx_buffer, sizeof(tx_buffer), &bytes_written);
    if (ret == ESP_OK) {
        int sent = uart_protocol_send(tx_buffer, bytes_written);
        if (sent == bytes_written) {
            ESP_LOGI(TAG, "Input routing command sent: %d bytes", sent);
            // Update our own state (opposite of what we told neighbor)
            input_routing_active = !make_neighbor_active;
            ESP_LOGI(TAG, "This board is now %s", input_routing_active ? "ACTIVE" : "INACTIVE");
        } else {
            ESP_LOGW(TAG, "Input routing command send incomplete: sent %d bytes, expected %d", sent, bytes_written);
        }
    } else {
        ESP_LOGE(TAG, "Failed to serialize input routing command frame: %s", esp_err_to_name(ret));
    }
    
    // Cleanup
    if (frame.payload) {
        free(frame.payload);
    }
    
    return ret;
}

bool uart_protocol_is_input_active(void)
{
    return input_routing_active;
}

void uart_protocol_set_input_active(bool active)
{
    ESP_LOGI(TAG, "Setting input routing state: %s", active ? "ACTIVE" : "INACTIVE");
    input_routing_active = active;
}

esp_err_t uart_protocol_forward_led_state(uint8_t led_state)
{
    if (!uart_initialized) {
        ESP_LOGD(TAG, "UART not initialized, skipping LED state forward");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGD(TAG, "Forwarding LED state: 0x%02x", led_state);
    
    // Create frame with BLE LED state command
    uart_frame_t frame;
    esp_err_t ret = uart_frame_create(UART_CMD_BLE_LED_STATE, &led_state, 1, &frame);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create LED state frame: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Serialize and send (fire-and-forget)
    uint8_t tx_buffer[16];  // Small buffer for single byte payload
    uint16_t bytes_written;
    ret = uart_frame_serialize(&frame, tx_buffer, sizeof(tx_buffer), &bytes_written);
    if (ret == ESP_OK) {
        int sent = uart_protocol_send(tx_buffer, bytes_written);
        if (sent == bytes_written) {
            ESP_LOGD(TAG, "LED state forwarded: %d bytes", sent);
        } else {
            ESP_LOGW(TAG, "LED state forward incomplete: sent %d bytes, expected %d", sent, bytes_written);
        }
    } else {
        ESP_LOGE(TAG, "Failed to serialize LED state frame: %s", esp_err_to_name(ret));
    }
    
    // Cleanup
    if (frame.payload) {
        free(frame.payload);
    }
    
    return ret;
}
