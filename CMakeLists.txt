cmake_minimum_required(VERSION 3.13)
set(ENV{PICO_SDK_PATH} "~/pico/pico-sdk/")
include(pico_sdk_import.cmake)
project(FlightControl_V1 C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(FlightControl_V1
    FlightControl_V1.c
)

pico_enable_stdio_usb(FlightControl_V1 1)
pico_enable_stdio_uart(FlightControl_V1 0)

pico_add_extra_outputs(FlightControl_V1)
target_link_libraries(FlightControl_V1 pico_stdlib hardware_gpio hardware_pwm hardware_timer hardware_i2c hardware_irq)