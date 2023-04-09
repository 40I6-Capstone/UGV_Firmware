add_library(pid_controller PIDController.hpp PIDController.cpp)
include_directories(../GeometryUtils)

target_link_libraries(pid_controller geometry_utils)