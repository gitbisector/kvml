#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "uart_frame.h"

static const char *TAG = "UART_FRAME";

// Start byte for UART frames
#define UART_FRAME_START_BYTE 0xAA

// Simple CRC16 calculation
static uint16_t calculate_crc16(const uint8_t* data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    
    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001; // CRC-16 polynomial
            } else {
                crc >>= 1;
            }
        }
    }
    
    return crc;
}

esp_err_t uart_frame_init(void) {
    ESP_LOGI(TAG, "Initializing UART frame handling");
    return ESP_OK;
}

esp_err_t uart_frame_create(uint8_t command, const uint8_t* payload, 
                          uint16_t length, uart_frame_t* frame) {
    if (frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    frame->start_byte = UART_FRAME_START_BYTE;
    frame->command = command;
    frame->length = length;
    
    if (length > 0 && payload != NULL) {
        frame->payload = (uint8_t*)malloc(length);
        if (frame->payload == NULL) {
            return ESP_ERR_NO_MEM;
        }
        memcpy(frame->payload, payload, length);
    } else {
        frame->payload = NULL;
    }
    
    // Calculate CRC16 over command, length and payload
    uint8_t crc_data[3 + length];
    crc_data[0] = frame->start_byte;
    crc_data[1] = frame->command;
    crc_data[2] = frame->length & 0xFF;
    crc_data[3] = (frame->length >> 8) & 0xFF;
    
    if (length > 0 && frame->payload != NULL) {
        memcpy(crc_data + 4, frame->payload, length);
    }
    
    frame->crc = calculate_crc16(crc_data, 4 + length);
    
    return ESP_OK;
}

esp_err_t uart_frame_serialize(const uart_frame_t* frame, uint8_t* buffer, 
                            uint16_t buffer_size, uint16_t* bytes_written) {
    if (frame == NULL || buffer == NULL || bytes_written == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    const uint16_t total_size = 6 + frame->length; // start + cmd + len(2) + payload + crc(2)
    
    if (buffer_size < total_size) {
        return ESP_ERR_INVALID_SIZE;
    }
    
    uint16_t idx = 0;
    buffer[idx++] = frame->start_byte;
    buffer[idx++] = frame->command;
    buffer[idx++] = frame->length & 0xFF;
    buffer[idx++] = (frame->length >> 8) & 0xFF;
    
    if (frame->length > 0 && frame->payload != NULL) {
        memcpy(buffer + idx, frame->payload, frame->length);
        idx += frame->length;
    }
    
    buffer[idx++] = frame->crc & 0xFF;
    buffer[idx++] = (frame->crc >> 8) & 0xFF;
    
    *bytes_written = idx;
    return ESP_OK;
}

esp_err_t uart_frame_parse(const uint8_t* buffer, uint16_t buffer_size, 
                        uart_frame_t* frame) {
    if (buffer == NULL || frame == NULL || buffer_size < 6) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (buffer[0] != UART_FRAME_START_BYTE) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    uint16_t idx = 0;
    frame->start_byte = buffer[idx++];
    frame->command = buffer[idx++];
    
    // Get length (little endian)
    frame->length = buffer[idx++];
    frame->length |= ((uint16_t)buffer[idx++]) << 8;
    
    // Validate buffer size
    if (buffer_size < 6 + frame->length) {
        return ESP_ERR_INVALID_SIZE;
    }
    
    // Copy payload
    if (frame->length > 0) {
        frame->payload = (uint8_t*)malloc(frame->length);
        if (frame->payload == NULL) {
            return ESP_ERR_NO_MEM;
        }
        memcpy(frame->payload, buffer + idx, frame->length);
        idx += frame->length;
    } else {
        frame->payload = NULL;
    }
    
    // Get CRC (little endian)
    frame->crc = buffer[idx++];
    frame->crc |= ((uint16_t)buffer[idx++]) << 8;
    
    // Verify CRC
    uint8_t crc_data[4 + frame->length];
    crc_data[0] = frame->start_byte;
    crc_data[1] = frame->command;
    crc_data[2] = frame->length & 0xFF;
    crc_data[3] = (frame->length >> 8) & 0xFF;
    
    if (frame->length > 0 && frame->payload != NULL) {
        memcpy(crc_data + 4, frame->payload, frame->length);
    }
    
    uint16_t calculated_crc = calculate_crc16(crc_data, 4 + frame->length);
    
    if (calculated_crc != frame->crc) {
        // Free allocated memory if CRC fails
        if (frame->payload != NULL) {
            free(frame->payload);
            frame->payload = NULL;
        }
        return ESP_ERR_INVALID_CRC;
    }
    
    return ESP_OK;
}
