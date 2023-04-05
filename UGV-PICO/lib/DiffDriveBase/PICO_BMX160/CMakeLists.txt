
add_library(pico_bmx160 PICO_IMU.hpp PICO_DFRobot_BMX160.cpp PICO_DFRobot_BMX160.h)

target_link_libraries(pico_bmx160 hardware_i2c pico_stdlib)