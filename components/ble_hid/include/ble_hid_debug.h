#pragma once

/**
 * @file ble_hid_debug.h
 * @brief Debug configuration for BLE HID reporting
 * 
 * This file provides centralized debug configuration for BLE HID functionality.
 * 
 * USAGE:
 * To disable BLE reporting for debugging (e.g., testing different mouse):
 *   Set BLE_HID_DEBUG_DISABLE_REPORTING to 1
 * 
 * To enable normal BLE operation:
 *   Set BLE_HID_DEBUG_DISABLE_REPORTING to 0
 * 
 * When disabled, all mouse and keyboard reports will be logged but not sent to BLE.
 * This allows you to see the USB HID data flow without interfering with BLE.
 */

// Master debug flag - set to 1 to disable all BLE HID reporting for debugging
#ifndef BLE_HID_DEBUG_DISABLE_REPORTING
#define BLE_HID_DEBUG_DISABLE_REPORTING 0  // 1 = disabled (debug mode), 0 = enabled (normal operation)
#endif

// Additional debug flags can be added here as needed
#ifndef BLE_HID_DEBUG_VERBOSE_LOGGING
#define BLE_HID_DEBUG_VERBOSE_LOGGING 0    // 1 = verbose, 0 = normal
#endif