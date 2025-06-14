# UART Protocol Component for the KVML project

idf_component_register(
    SRCS 
        "src/uart_protocol.c"
        "src/uart_frame.c"
        
    INCLUDE_DIRS 
        "include"
        
    PRIV_INCLUDE_DIRS
        "private_include"
        
    REQUIRES 
        driver
        esp_common
        esp_event
        freertos
        soc
        hal
        common
        ble_hid
        usb_host
)

# Create required directories if they don't exist
execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/private_include
)
