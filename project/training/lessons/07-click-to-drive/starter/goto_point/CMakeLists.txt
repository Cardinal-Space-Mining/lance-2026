cmake_minimum_required(VERSION 3.12)
project(goto_point)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

# --- Dependencies -------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(phoenix_ros_driver REQUIRED)

# --- Executable ---------------------------------------------------------------
add_executable(goto_point_node src/goto_point_node.cpp)
target_link_libraries(goto_point_node
    rclcpp::rclcpp
    tf2::tf2
    tf2_ros::tf2_ros
    tf2_geometry_msgs::tf2_geometry_msgs
    ${geometry_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    ${phoenix_ros_driver_TARGETS})

install(TARGETS goto_point_node
    DESTINATION lib/${PROJECT_NAME})

ament_package()
