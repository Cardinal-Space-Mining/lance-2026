cmake_minimum_required(VERSION 3.12)
project(net_adapter)

# --- Dependencies -------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(phoenix_ros_driver REQUIRED)
find_package(multiscan_driver REQUIRED)
find_package(csm_metrics REQUIRED)

find_package(zenohc REQUIRED)
find_package(zenohcxx REQUIRED)

# --- Presets ------------------------------------------------------------------
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
endif()

if(NOT MSVC)
    if(NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE Release)
    endif()
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Executables --------------------------------------------------------------
add_executable(robot_adapter_node
    src/robot_adapter.cpp)
target_link_libraries(robot_adapter_node
    csm_metrics::profiling
    zenohcxx::zenohc)
ament_target_dependencies(robot_adapter_node
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    tf2_ros
    tf2_geometry_msgs
    phoenix_ros_driver
    multiscan_driver
    csm_metrics)

add_executable(client_adapter_node
    src/client_adapter.cpp)
target_link_libraries(client_adapter_node
    csm_metrics::profiling
    zenohcxx::zenohc)
ament_target_dependencies(client_adapter_node
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    tf2_ros
    tf2_geometry_msgs
    phoenix_ros_driver
    multiscan_driver
    csm_metrics)

# --- Export -------------------------------------------------------------------
ament_export_dependencies(
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    tf2_ros
    tf2_geometry_msgs
    phoenix_ros_driver
    multiscan_driver
    csm_metrics)
ament_package()

install(TARGETS
    robot_adapter_node
    client_adapter_node
    DESTINATION lib/${PROJECT_NAME})
