# Common utilities Component for the KVML project

idf_component_register(
    SRCS 
        "src/common_utils.c"
        "src/config_manager.c"
        "src/error_handling.c"
        
    INCLUDE_DIRS 
        "include"
        
    REQUIRES 
        nvs_flash
        esp_common
        esp_event
        freertos
        log
        soc
        hal
)

# Create required directories if they don't exist
execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
