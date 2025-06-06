#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief UART frame structure
 */
typedef struct {
    uint8_t  start_byte;    // Frame start indicator (0xAA)
    uint8_t  command;       // Command identifier
    uint16_t length;        // Payload length
    uint8_t* payload;       // Payload data
    uint16_t crc;           // CRC16 for error detection
} uart_frame_t;

/**
 * @brief Initialize UART frame handling
 *
 * @return ESP_OK if successful, otherwise an error code
 */
esp_err_t uart_frame_init(void);

/**
 * @brief Create a new UART frame
 * 
 * @param command Command identifier
 * @param payload Pointer to payload data
 * @param length Length of payload data
 * @param frame Pointer to frame structure to fill
 * @return ESP_OK if successful, otherwise an error code
 */
esp_err_t uart_frame_create(uint8_t command, const uint8_t* payload, 
                            uint16_t length, uart_frame_t* frame);

/**
 * @brief Serialize a UART frame into a byte buffer
 * 
 * @param frame Frame to serialize
 * @param buffer Buffer to store serialized frame
 * @param buffer_size Size of the buffer
 * @param bytes_written Number of bytes written to buffer
 * @return ESP_OK if successful, otherwise an error code
 */
esp_err_t uart_frame_serialize(const uart_frame_t* frame, uint8_t* buffer, 
                              uint16_t buffer_size, uint16_t* bytes_written);

/**
 * @brief Parse a byte buffer into a UART frame
 * 
 * @param buffer Buffer containing serialized frame
 * @param buffer_size Size of the buffer
 * @param frame Pointer to frame structure to fill
 * @return ESP_OK if successful, otherwise an error code
 */
esp_err_t uart_frame_parse(const uint8_t* buffer, uint16_t buffer_size, 
                          uart_frame_t* frame);

#ifdef __cplusplus
}
#endif
