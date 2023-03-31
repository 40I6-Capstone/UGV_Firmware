# find_package(Eigen3 3.3 REQUIRED NO_MODULE)
# include_directories("/usr/include")
add_library(diff_drive_odom DifferentialDriveOdometry.hpp DifferentialDriveOdometry.cpp)

target_link_libraries(diff_drive_odom)