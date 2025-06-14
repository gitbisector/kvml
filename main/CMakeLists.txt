# Main component for the KVML project
idf_component_register(
    SRCS
        "kvml.c"

    INCLUDE_DIRS
        "."

    REQUIRES
        json
        bt
        ble_hid
        usb
        nvs_flash
        esp_timer
        driver
        esp_rom
        esp_common
        common
        esp_system
        esp_hw_support
        freertos
        log
        heap
        soc
        hal
        newlib
        vfs
        pthread
        ble_hid
        usb_host
        uart_protocol
        kvml
)
# Enable aggressive USB host stack reset for better handling of devices connected at boot
set_property(TARGET ${COMPONENT_LIB} APPEND PROPERTY COMPILE_DEFINITIONS AGGRESSIVE_USB_RESET)
