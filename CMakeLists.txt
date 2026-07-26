# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.0)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.13)

# Initialize Pico SDK
include(pico_sdk_import.cmake)

project(pico_wheel_control C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

# --------------------------------------------------------------------
# PATH DEFINITIONS
# --------------------------------------------------------------------
set(WIZNET_DIR "${CMAKE_CURRENT_LIST_DIR}/lib/ioLibrary_Driver")
set(PORT_DIR   "${CMAKE_CURRENT_LIST_DIR}/lib/port") 
set(ROBOCLAW_DIR "${CMAKE_CURRENT_LIST_DIR}/lib/Basicmicro")
set(PT_DIR     "${CMAKE_CURRENT_LIST_DIR}/lib/pt")

# --------------------------------------------------------------------
# TARGET CONFIGURATION
# --------------------------------------------------------------------
# Define a single target name for the master code integration
set(TARGET_NAME "pico_wheel_control")

message(STATUS "Configuring ${TARGET_NAME} for IP 10.42.0.34")

# 1. Create Executable
add_executable(${TARGET_NAME}
    drivingv2.c
    ${WIZNET_DIR}/Ethernet/wizchip_conf.c
    ${WIZNET_DIR}/Ethernet/socket.c
    ${WIZNET_DIR}/Ethernet/W5500/w5500.c
    ${PORT_DIR}/ioLibrary_Driver/src/w5x00_spi.c
    ${PORT_DIR}/timer/timer.c
    ${ROBOCLAW_DIR}/Basicmicro.cpp
    ${ROBOCLAW_DIR}/roboclaw_bridge.cpp
)

# 2. Inject Definitions (Hardcoded for RL config / .34 IP)
target_compile_definitions(${TARGET_NAME} PRIVATE
    LAST_IP_OCTET=34
    MAC_LAST_BYTE=0x34
    ENC_MIN=225
    ENC_MAX=1684
    INVERTED=1
)

# 3. Include Directories
target_include_directories(${TARGET_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${WIZNET_DIR}/Ethernet
    ${WIZNET_DIR}/Ethernet/W5500
    ${PORT_DIR}/ioLibrary_Driver/inc
    ${PORT_DIR}/timer
    ${ROBOCLAW_DIR}
    ${PORT_DIR}
    ${PT_DIR}
)

# 4. Link Libraries
target_link_libraries(${TARGET_NAME} PRIVATE
    pico_stdlib
    hardware_spi
    hardware_dma
    hardware_uart
    hardware_gpio
    hardware_timer
)

# 5. Generate Outputs
pico_enable_stdio_usb(${TARGET_NAME} 1)
pico_enable_stdio_uart(${TARGET_NAME} 0)
pico_add_extra_outputs(${TARGET_NAME})