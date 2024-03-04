cmake_minimum_required(VERSION 3.13)
set(ENV{PICO_SDK_PATH} "~/pico/pico-sdk/")
include(pico_sdk_import.cmake)
project(Servo C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(Servo
    Servo.c
)

pico_enable_stdio_usb(Servo 1)
pico_enable_stdio_uart(Servo 0)

pico_add_extra_outputs(Servo)
target_link_libraries(Servo pico_stdlib hardware_gpio hardware_pwm)