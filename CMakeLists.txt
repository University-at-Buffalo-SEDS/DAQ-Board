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
set(CMAKE_PROJECT_NAME DAQ-Board)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -T "${CMAKE_SOURCE_DIR}/STM32U585xx_FLASH.ld")

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    # STM32CubeMX generated application sources
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/can_bus.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/board_link_uart.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/telemetry_thread.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/telemetry.c 
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/telemetry_hooks.c 
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/sd_card.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/daq_adc1.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/daq_adc4.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/daq_board.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/daq_dac.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/daq_thread.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/mcp3564r.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/sd_writer_thread.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # The application vector table follows the LaunchCore image header.  Keep
    # early HAL interrupts on the application table before ThreadX relocates
    # VTOR in tx_initialize_low_level.
    VECT_TAB_OFFSET=0x00008200UL
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# SEDSNet v4
set(SEDSNET_DEVICE_IDENTIFIER "DAQ" CACHE STRING "" FORCE)
set(SEDSNET_MAX_STACK_PAYLOAD "128" CACHE STRING "" FORCE)
set(SEDSNET_MAX_QUEUE_BUDGET "12288" CACHE STRING "" FORCE)
set(SEDSNET_MAX_RECENT_RX_IDS "16" CACHE STRING "" FORCE)
set(SEDSNET_ENV_STARTING_QUEUE_SIZE "1024" CACHE STRING "" FORCE)
set(SEDSNET_ENV_QUEUE_GROW_STEP "1.25" CACHE STRING "" FORCE)
set(SEDSNET_ENV_PAYLOAD_COMPRESS_THRESHOLD "30" CACHE STRING "" FORCE)
set(SEDSNET_TARGET "thumbv8m.main-none-eabihf" CACHE STRING "" FORCE)
include(cmake/sedsnet_fetch.cmake)

# Option defaults to ON
option(ENABLE_TELEMETRY "Enable the telemetry subsystem" ON)
option(ENABLE_TELEMETRY_CAN_BUS "Use CAN as the telemetry transport" ON)
option(ENABLE_TELEMETRY_BOARD_LINK_UART "Use USART1 DMA as the telemetry transport" OFF)
set(TELEMETRY_TIMESYNC_MASTER_PRIO "-1" CACHE STRING
    "Timesync source priority. Values < 0 make this board a consumer; values >= 0 make it a source with that priority.")

if(ENABLE_TELEMETRY)
    if(ENABLE_TELEMETRY_CAN_BUS AND ENABLE_TELEMETRY_BOARD_LINK_UART)
        message(FATAL_ERROR "Telemetry transports are mutually exclusive: enable CAN or board-link UART, not both.")
    endif()
    if(NOT ENABLE_TELEMETRY_CAN_BUS AND NOT ENABLE_TELEMETRY_BOARD_LINK_UART)
        message(FATAL_ERROR "Telemetry is enabled but no transport is selected. Enable CAN or board-link UART.")
    endif()
endif()

# Print its state for debugging
message(STATUS "Telemetry enabled: ${ENABLE_TELEMETRY}")
message(STATUS "Telemetry CAN bus enabled: ${ENABLE_TELEMETRY_CAN_BUS}")
message(STATUS "Telemetry board-link UART enabled: ${ENABLE_TELEMETRY_BOARD_LINK_UART}")
message(STATUS "Telemetry timesync master priority: ${TELEMETRY_TIMESYNC_MASTER_PRIO}")

# Define or undefine the macro globally
if(ENABLE_TELEMETRY)
    add_compile_definitions(
        TELEMETRY_ENABLED
        TELEMETRY_TIMESYNC_MASTER_PRIO=${TELEMETRY_TIMESYNC_MASTER_PRIO}
    )
else()
    # Not defined → disabled
endif()

if(ENABLE_TELEMETRY_CAN_BUS)
    add_compile_definitions(TELEMETRY_CAN_BUS)
endif()

if(ENABLE_TELEMETRY_BOARD_LINK_UART)
    add_compile_definitions(TELEMETRY_BOARD_LINK_UART)
endif()

set(LAUNCHCORE_DEVICE_DEFINE STM32U585xx)
set(LAUNCHCORE_CMSIS_DEVICE_DIR "${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32U5xx/Include")
set(LAUNCHCORE_HAL_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/Drivers/STM32U5xx_HAL_Driver/Inc")
set(LAUNCHCORE_SYSTEM_SOURCE "${CMAKE_SOURCE_DIR}/Core/Src/system_stm32u5xx.c")
set(LAUNCHCORE_STARTUP_SOURCE "${CMAKE_SOURCE_DIR}/startup_stm32u585xx.s")
set(LAUNCHCORE_FLASH_SOURCES
    "${CMAKE_SOURCE_DIR}/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_flash.c"
    "${CMAKE_SOURCE_DIR}/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_flash_ex.c"
    "${CMAKE_SOURCE_DIR}/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_hash.c"
    "${CMAKE_SOURCE_DIR}/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_hash_ex.c")
set(LAUNCHCORE_HARDWARE_SHA256_SOURCE
    "${CMAKE_SOURCE_DIR}/Bootloader/sha256_hardware.c")
set(LAUNCHCORE_APP_SLOT_BASE 0x08008000)
set(LAUNCHCORE_APP_VECTOR_TABLE 0x08008200)
set(LAUNCHCORE_METADATA0_BASE 0x081F8000)
set(LAUNCHCORE_DELTA_SIZE 0x00020000)
include(cmake/launchcore_stm32.cmake)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    "${CMAKE_SOURCE_DIR}/Core/Src/sedsnet_hardware_crypto.c"
    "${CMAKE_SOURCE_DIR}/Bootloader/sha256_hardware.c"
    "${CMAKE_SOURCE_DIR}/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_hash.c"
    "${CMAKE_SOURCE_DIR}/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_hash_ex.c")
