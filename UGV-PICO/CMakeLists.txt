cmake_minimum_required(VERSION 3.12)

# Pull in PICO SDK (must be before project)
include(pico_sdk_import.cmake)

project(ugv_firm C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17) 

# Initialize the SDK
pico_sdk_init()

add_executable(ugv_firm
	main.cpp
	)

# Pull in our pico_stdlib which pulls in commonl
target_link_libraries(ugv_firm pico_stdlib pico_multicore hardware_uart hardware_irq pico_stdio_usb hardware_pwm)

# Enable/Disable usb/uart standard output (1 for enable, 0 for disable)
# Serial default baud: 115200
pico_enable_stdio_usb(ugv_firm 1)
pico_enable_stdio_uart(ugv_firm 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(ugv_firm)
