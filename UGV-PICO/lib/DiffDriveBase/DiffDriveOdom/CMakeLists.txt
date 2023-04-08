
add_library(diff_drive_odom DifferentialDriveOdometry.hpp DifferentialDriveOdometry.cpp)

find_package(Eigen3 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIR} ../GeometryUtils)

target_link_libraries(diff_drive_odom Eigen3::Eigen geometry_utils)