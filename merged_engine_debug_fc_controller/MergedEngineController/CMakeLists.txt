# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()

set(PICO_BOARD pico CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(IntegratedMotorController_code C CXX ASM)

pico_sdk_init()

file(GLOB_RECURSE DRONECAN_GENERATED_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_LIST_DIR}/generated/*.c
)

add_executable(IntegratedMotorController_code
    src/main.c
    src/buzzer.c
    src/actuators.c
    src/engine_control.c
    src/pid.c
    src/sensors.c
    src/mcp2518fd.c
    src/dronecan_node.c
    src/custom_can_node.c
    src/status_indicator.c
    src/persistent_config.c
    external/libcanard/canard.c
    ${DRONECAN_GENERATED_SOURCES}
)

pico_set_program_name(IntegratedMotorController_code "IntegratedMotorController_code")
pico_set_program_version(IntegratedMotorController_code "0.1")

pico_enable_stdio_uart(IntegratedMotorController_code 0)
pico_enable_stdio_usb(IntegratedMotorController_code 1)

target_include_directories(IntegratedMotorController_code PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${CMAKE_CURRENT_LIST_DIR}/generated
    ${CMAKE_CURRENT_LIST_DIR}/generated/include
    ${CMAKE_CURRENT_LIST_DIR}/external/libcanard
)

target_link_libraries(IntegratedMotorController_code
    pico_stdlib
    hardware_spi
    hardware_pwm
    hardware_watchdog
    hardware_i2c
    hardware_adc
)

pico_add_extra_outputs(IntegratedMotorController_code)