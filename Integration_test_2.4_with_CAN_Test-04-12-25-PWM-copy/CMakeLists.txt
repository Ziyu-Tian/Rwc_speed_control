# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(pico-mcp2515 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
add_executable(pico-mcp2515
    # Library executables
    include/mcp2515/mcp2515.cpp
    
    # Main executalbes
    src/pico-mcp2515.cpp
)

# Add MCP2515 Lib
target_include_directories(pico-mcp2515 PUBLIC include/)

pico_set_program_name(pico-mcp2515 "pico-mcp2515")
pico_set_program_version(pico-mcp2515 "0.1")

pico_enable_stdio_uart(pico-mcp2515 0)
pico_enable_stdio_usb(pico-mcp2515 1)

# Add any user requested libraries
target_link_libraries(pico-mcp2515
    pico_stdlib
    hardware_spi
    pico_multicore
    hardware_adc
    hardware_pwm
)

# Encoder Lib
add_subdirectory(pico-quadrature-encoder)

# Link Encoder Lib
target_link_libraries(pico-mcp2515 quadrature_encoder_pico hardware_spi)

pico_add_extra_outputs(pico-mcp2515)

