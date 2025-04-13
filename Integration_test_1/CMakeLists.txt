cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-fexceptions)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)



# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(PIO_Test C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(PIO_Test PIO_Test.cpp)

pico_set_program_name(PIO_Test "PIO_Test")
pico_set_program_version(PIO_Test "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(PIO_Test 0) # Disable UART
pico_enable_stdio_usb(PIO_Test 1) # Enable USB-UART

# Add the standard library to the build
target_link_libraries(PIO_Test
        pico_stdlib
        hardware_adc
)

# Encoder Lib
add_subdirectory(pico-quadrature-encoder)

# Link Encoder Lib
target_link_libraries(PIO_Test quadrature_encoder_pico hardware_spi)

# Additional Output (.uf2 file) 
pico_add_extra_outputs(PIO_Test)
