# BLE HID Component for the KVML project

idf_component_register(
    SRCS 
        "src/ble_hid.c"
        "src/ble_hid_device.c"
        "src/ble_hid_keyboard.c"
        "src/ble_hid_mouse.c"
        
    INCLUDE_DIRS 
        "include"
        
    REQUIRES 
        bt
        nvs_flash
        esp_hid
        esp_event
        esp_common
        esp_timer
        soc
        hal
        freertos
        common
        
    PRIV_REQUIRES
        usb_host
        uart_protocol
)

# Create required directories if they don't exist
execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
