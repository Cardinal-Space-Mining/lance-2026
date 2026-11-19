cmake_minimum_required(VERSION 3.12)
project(mining_stats)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

# --- Dependencies -------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(phoenix_ros_driver REQUIRED)
find_package(mining_stats_interfaces REQUIRED)   # our own interface package

# --- Executable ---------------------------------------------------------------
add_executable(mining_stats_node src/mining_stats_node.cpp)
target_link_libraries(mining_stats_node
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
    ${phoenix_ros_driver_TARGETS}
    ${mining_stats_interfaces_TARGETS})

install(TARGETS mining_stats_node
    DESTINATION lib/${PROJECT_NAME})

ament_package()
