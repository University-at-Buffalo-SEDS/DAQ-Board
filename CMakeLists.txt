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
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Telemetry Library
set(SEDSPRINTF_RS_DEVICE_IDENTIFIER "DAQ" CACHE STRING "" FORCE)
set(SEDSPRINTF_RS_MAX_STACK_PAYLOAD "128" CACHE STRING "" FORCE)
set(SEDSPRINTF_RS_ENV_MAX_QUEUE_SIZE "4096" CACHE STRING "" FORCE)
set(SEDSPRINTF_RS_ENV_STARTING_QUEUE_SIZE "512" CACHE STRING "" FORCE)
set(SEDSPRINTF_RS_ENV_QUEUE_GROW_STEP "1.25" CACHE STRING "" FORCE)
set(SEDSPRINTF_RS_ENV_MAX_RECENT_RX_IDS "32" CACHE STRING "" FORCE)
set(SEDSPRINTF_RS_ENV_STARTING_RECENT_RX_IDS "16" CACHE STRING "" FORCE)
set(SEDSPRINTF_RS_ENV_PAYLOAD_COMPRESS_THRESHOLD "30" CACHE STRING "" FORCE)
set(SEDSPRINTF_RS_TARGET "thumbv8m.main-none-eabihf" CACHE STRING "" FORCE) # Set target for embedded systems
# set(SEDSPRINTF_RS_TARGET "thumbv7em-none-eabihf" CACHE STRING "" FORCE) # Set target for embedded systems
add_subdirectory(${CMAKE_SOURCE_DIR}/sedsprintf_rs)
target_link_libraries(${CMAKE_PROJECT_NAME} sedsprintf_rs::sedsprintf_rs)

# Option defaults to ON
option(ENABLE_TELEMETRY "Enable the telemetry subsystem" ON)
set(TELEMETRY_TIMESYNC_MASTER_PRIO "-1" CACHE STRING
    "Timesync source priority. Values < 0 make this board a consumer; values >= 0 make it a source with that priority.")

# Print its state for debugging
message(STATUS "Telemetry enabled: ${ENABLE_TELEMETRY}")
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
