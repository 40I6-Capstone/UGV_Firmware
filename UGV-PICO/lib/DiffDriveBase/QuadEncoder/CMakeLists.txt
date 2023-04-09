add_library(quad_enc QuadEncoder.hpp QuadEncoder.cpp)

target_link_libraries(quad_enc pico_stdlib pico_time hardware_gpio)