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
        esp_http_server
        esp_https_ota
        esp_wifi
        esp_event
        esp_netif
        esp_timer
        driver
        esp_http_client
        mbedtls
        esp_rom
        esp_common
        common
        esp_system
        esp_hw_support
        bootloader_support
        app_update
        esp_partition
        spi_flash
        freertos
        log
        heap
        soc
        hal
        newlib
        vfs
        pthread
        lwip
        wpa_supplicant
        ble_hid
        usb_host
        uart_protocol
        web_ui
        kvml
)
# Enable aggressive USB host stack reset for better handling of devices connected at boot
set_property(TARGET ${COMPONENT_LIB} APPEND PROPERTY COMPILE_DEFINITIONS AGGRESSIVE_USB_RESET)
