cmake_minimum_required(VERSION 3.13)
set(ENV{PICO_SDK_PATH} "~/pico/pico-sdk/")
include(pico_sdk_import.cmake)
project(btsetup C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(btsetup
    btsetup.c
)

pico_enable_stdio_usb(btsetup 1)
pico_enable_stdio_uart(btsetup 0)

pico_add_extra_outputs(btsetup)
target_link_libraries(btsetup pico_stdlib hardware_gpio hardware_uart)