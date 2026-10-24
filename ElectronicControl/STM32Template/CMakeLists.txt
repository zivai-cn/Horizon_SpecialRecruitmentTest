cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME STM32Template)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
    _1_BSP/usart/bsp_uart.c
    _1_BSP/spi/bsp_spi.c
    _2_Modules/Blue_tooth/blue_tooth.c
    _2_Modules/W25Q64/w25q64.c
    _3_App/app_bluetooth_test.c
    _3_App/app_w25q64_test.c
    _3_App/app_led_test.c)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    _1_BSP/usart
    _1_BSP/spi
    _2_Modules/Blue_tooth
    _2_Modules/W25Q64
    _3_App
)

# FDCAN BSP is only enabled when CubeMX has generated fdcan.h for the target.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc/fdcan.h")
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE _1_BSP/fdcan/bsp_fdcan.c)
    target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE _1_BSP/fdcan)
endif()
# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)