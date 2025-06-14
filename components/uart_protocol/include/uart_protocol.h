/**
 * UART Physical Wiring for Dual ESP32-S3 Communication
 * ====================================================
 * 
 * Board A (ESP32-S3 #1)         Board B (ESP32-S3 #2)
 * ---------------------         ---------------------
 * GPIO15 (TX) ----------*---*---------- GPIO40 (RX)
 * GPIO40 (RX) ----------*---*---------- GPIO15 (TX)
 * GND        ----------*---*---------- GND
 * 
 * NOTE: TX connects to RX on the other board (crossed connection)
 * 
 * For single-board loopback testing: Connect GPIO15 to GPIO40 on same board
 */

#pragma once

#include "esp_err.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// ===== UART Physical Layer Configuration =====

// UART hardware configuration
#define UART_NUM                UART_NUM_1              // Use UART1 for neighbor communication
#define UART_BAUD_RATE          921600                  // High-speed communication
#define UART_DATA_BITS          UART_DATA_8_BITS        // 8 data bits
#define UART_PARITY             UART_PARITY_DISABLE     // No parity
#define UART_STOP_BITS          UART_STOP_BITS_1        // 1 stop bit
#define UART_FLOW_CTRL          UART_HW_FLOWCTRL_DISABLE // No hardware flow control

// ESP32-S3 GPIO pin definitions for UART1
#define UART_TX_PIN             GPIO_NUM_15             // UART1 TX pin
#define UART_RX_PIN             GPIO_NUM_40             // UART1 RX pin
#define UART_RTS_PIN            UART_PIN_NO_CHANGE      // Not used
#define UART_CTS_PIN            UART_PIN_NO_CHANGE      // Not used

// UART buffer sizes
#define UART_TX_BUF_SIZE        1024                    // Transmit buffer size
#define UART_RX_BUF_SIZE        1024                    // Receive buffer size
#define UART_QUEUE_SIZE         20                      // Event queue size
#define UART_PATTERN_CHR        0                       // Pattern character (disabled)

// Protocol timing configuration
#define UART_TIMEOUT_MS         100                     // Read timeout in milliseconds
#define UART_FRAME_TIMEOUT_MS   50                      // Inter-frame timeout

// ===== UART Protocol Command Definitions =====

// Test and connectivity commands
#define UART_CMD_TEST_LOOPBACK  0xAB                    // Loopback test command
#define UART_CMD_TEST_RESPONSE  0xAC                    // Automatic response to test command
#define UART_CMD_PING           0xA0                    // Neighbor ping command
#define UART_CMD_PONG           0xA1                    // Neighbor pong response

// USB HID report forwarding commands (fire-and-forget, no acks)
#define UART_CMD_USB_MOUSE      0x10                    // USB mouse HID report
#define UART_CMD_USB_KEYBOARD   0x11                    // USB keyboard HID report
#define UART_CMD_BLE_LED_STATE  0x12                    // BLE keyboard LED state (output report)

// Host switching coordination commands
#define UART_CMD_HOST_SWITCH_REQUEST  0x20              // Request host switching
#define UART_CMD_HOST_SWITCH_ACK      0x21              // Acknowledge host switch

// Input routing coordination commands (both boards stay connected to BLE)
#define UART_CMD_INPUT_ACTIVE         0x30              // Become active input forwarder
#define UART_CMD_INPUT_INACTIVE       0x31              // Become inactive (send idle to BLE)

// Response timing
#define UART_AUTO_RESPONSE_DELAY_MS  5                  // Delay before sending auto-response

/**
 * @brief Initialize UART protocol with physical layer setup
 *
 * This function configures the UART hardware, installs the driver,
 * and sets up the communication parameters for neighbor coordination.
 *
 * @return ESP_OK if successful, otherwise an error code
 */
esp_err_t uart_protocol_init(void);

/**
 * @brief Deinitialize UART protocol and cleanup resources
 *
 * @return ESP_OK if successful, otherwise an error code
 */
esp_err_t uart_protocol_deinit(void);

/**
 * @brief Send raw data over UART
 *
 * @param data Pointer to data buffer
 * @param length Number of bytes to send
 * @return Number of bytes sent, or -1 on error
 */
int uart_protocol_send(const uint8_t *data, size_t length);

/**
 * @brief Receive raw data from UART
 *
 * @param data Pointer to receive buffer
 * @param max_length Maximum bytes to receive
 * @param timeout_ms Timeout in milliseconds
 * @return Number of bytes received, or -1 on error
 */
int uart_protocol_receive(uint8_t *data, size_t max_length, uint32_t timeout_ms);

/**
 * @brief Test UART communication with loopback
 *
 * Sends a test message and checks if it's received back.
 * Useful for testing when TX and RX pins are connected.
 *
 * @return ESP_OK if loopback test passes, otherwise an error code
 */
esp_err_t uart_protocol_test_loopback(void);

/**
 * @brief Start the UART protocol background listener task
 *
 * This starts a background task that listens for incoming frames
 * and automatically responds to test commands for connectivity testing.
 *
 * @return ESP_OK if successful, otherwise an error code
 */
esp_err_t uart_protocol_start_listener(void);

/**
 * @brief Stop the UART protocol background listener task
 *
 * @return ESP_OK if successful, otherwise an error code
 */
esp_err_t uart_protocol_stop_listener(void);

/**
 * @brief Forward USB mouse HID report to neighbor board (fire-and-forget)
 *
 * @param report_data Pointer to USB mouse HID report data
 * @param report_length Length of the HID report
 * @return ESP_OK if sent successfully, error code otherwise
 */
esp_err_t uart_protocol_forward_mouse_report(const uint8_t *report_data, size_t report_length);

/**
 * @brief Forward USB keyboard HID report to neighbor board (fire-and-forget)
 *
 * @param report_data Pointer to USB keyboard HID report data  
 * @param report_length Length of the HID report
 * @return ESP_OK if sent successfully, error code otherwise
 */
esp_err_t uart_protocol_forward_keyboard_report(const uint8_t *report_data, size_t report_length);

/**
 * @brief Forward BLE keyboard LED state to neighbor board (fire-and-forget)
 *
 * @param led_state LED state byte with bit flags
 * @return ESP_OK if sent successfully, error code otherwise
 */
esp_err_t uart_protocol_forward_led_state(uint8_t led_state);

/**
 * @brief Send host switch request to neighbor board
 *
 * @return ESP_OK if sent successfully, error code otherwise
 */
esp_err_t uart_protocol_send_host_switch_request(void);

/**
 * @brief Send input routing command to neighbor board
 *
 * @param make_neighbor_active If true, tell neighbor to become active (we become inactive)
 *                            If false, tell neighbor to become inactive (we become active)
 * @return ESP_OK if sent successfully, error code otherwise
 */
esp_err_t uart_protocol_send_input_routing_command(bool make_neighbor_active);

/**
 * @brief Get current input routing state
 *
 * @return true if this board is currently active (forwards USB input to BLE)
 *         false if this board is inactive (sends idle reports to BLE)
 */
bool uart_protocol_is_input_active(void);

/**
 * @brief Set input routing state locally
 *
 * @param active true to become active input forwarder, false to become inactive
 */
void uart_protocol_set_input_active(bool active);

#ifdef __cplusplus
}
#endif
