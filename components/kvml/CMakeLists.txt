# KVML Core Component for the HID multiplexer project

idf_component_register(
    SRCS 
        "src/kvml_core.c"
        "src/host_switching.c"
        "src/device_discovery.c"
        
    INCLUDE_DIRS 
        "include"
        
    REQUIRES 
        nvs_flash
        esp_common
        esp_event
        freertos
        log
        ble_hid
        usb_host
        uart_protocol
        common
)

# Create required directories if they don't exist
execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
