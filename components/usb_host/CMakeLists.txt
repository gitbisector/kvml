# USB Host Component for the KVML project

idf_component_register(
    SRCS
        "src/usb_host.c"
        "src/usb_hid_host.c"

    INCLUDE_DIRS
        "include"

    PRIV_INCLUDE_DIRS
        "private_include"

    REQUIRES
        usb
        usb_host
        esp_common
        esp_event
        freertos
        driver
        soc
        ble_hid
        uart_protocol
        kvml

    PRIV_REQUIRES
        usb_host_hid
        hal
        common
)

# Create required directories if they don't exist
execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/private_include
)
