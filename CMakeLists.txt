# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Set target to ESP32-S3 before including project.cmake
set(IDF_TARGET "esp32s3")

# Set the project version
set(PROJECT_VER "0.1.0")

# Set default build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

# Set default log level
set(CONFIG_LOG_DEFAULT_LEVEL_INFO=y)

# Set partition table file
set(PARTITION_TABLE_CSV "partitions.csv" CACHE STRING "Partition table CSV file")

# Set C standard
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Define extra component directories
set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/components)

# Enable compile commands for IDE support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Include ESP-IDF project.cmake after setting variables
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Define project name after including project.cmake
project(kvml)
