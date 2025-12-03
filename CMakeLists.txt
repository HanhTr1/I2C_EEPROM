# Set minimum required version of CMake
cmake_minimum_required(VERSION 3.12)

# Set board type
set(PICO_BOARD pico_w)
#set(PICO_BOARD pico)

# Include build functions from Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Set name of project (as PROJECT_NAME) and C/C   standards
project(I2C_EEPROM C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_NO_PICOTOOL 1)



# Creates a pico-sdk subdirectory in our project for the libraries
pico_sdk_init()

add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-maybe-uninitialized
)
# Common source files
set(COMMON_SRCS
        functions1.c
)
# Exercise 1 executable
add_executable(main_ex1
        main_ex1.c
        functions1.c
        ${COMMON_SRCS}
)
pico_add_extra_outputs(main_ex1)
target_link_libraries(main_ex1
        pico_stdlib
        hardware_i2c
        hardware_gpio
)
pico_enable_stdio_usb(main_ex1 0)
pico_enable_stdio_uart(main_ex1 1)

# Exercise 2 executable
add_executable(main_ex2
        main_ex2.c
        functions2.c
        ${COMMON_SRCS}

)

pico_add_extra_outputs(main_ex2)
target_link_libraries(main_ex2
        pico_stdlib
        hardware_i2c
        hardware_gpio
)
pico_enable_stdio_usb(main_ex2 0)
pico_enable_stdio_uart(main_ex2 1)

