cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(pico_c_nrf24l01_driver C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()
 
add_executable(main 
    main.c
    nrf24l01.c
)

pico_enable_stdio_uart(main 1)
pico_enable_stdio_usb(main 1)

pico_add_extra_outputs(main)

target_link_libraries(main
    pico_stdlib
    hardware_spi
)
