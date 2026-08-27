cmake_minimum_required(VERSION 3.15)
project(csm_utils)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install cmake extras
install(
  FILES
    cmake/auto_target_dependencies.cmake
    cmake/standard_compile_options.cmake
    cmake/robot_target.cmake
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  builtin_interfaces
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
)

ament_package(
  CONFIG_EXTRAS
    cmake/auto_target_dependencies.cmake
    cmake/standard_compile_options.cmake
    cmake/robot_target.cmake
)
