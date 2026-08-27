cmake_minimum_required(VERSION 3.12)
project(net_adapter)

# --- Dependencies -------------------------------------------------------------
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(csm_utils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)

find_package(phoenix_ros_driver REQUIRED)
find_package(multiscan_driver REQUIRED)
# find_package(csm_metrics REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(zenohc REQUIRED)
find_package(zenohcxx REQUIRED)

find_package(zstd REQUIRED)

# --- Messages -----------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/Bytes.msg")

rosidl_get_typesupport_target(${PROJECT_NAME}_custom_types
    ${PROJECT_NAME} rosidl_typesupport_cpp)

# --- Common -------------------------------------------------------------------
add_library(adapters_core STATIC
    "src/core/zstd_ctx.cpp"
    "src/core/delay_queue.cpp"
    "src/core/latency_ping.cpp"
    "src/adapters/joy_adapter.cpp"
    "src/adapters/ms136_imu_adapter.cpp"
    "src/adapters/ms136_scan_adapter.cpp"
    "src/adapters/talon_adapter.cpp"
    "src/adapters/path_adapter.cpp")
target_link_libraries(adapters_core
    ${${PROJECT_NAME}_custom_types}
    Eigen3::Eigen
    zenohcxx::zenohc
    zstd::libzstd_shared)
auto_target_dependencies(adapters_core
    csm_utils
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf2_ros
    tf2_geometry_msgs
    rosgraph_msgs
    phoenix_ros_driver
    multiscan_driver )

# --- Executables --------------------------------------------------------------
add_executable(robot_endpoint
    "src/robot_endpoint.cpp")
target_link_libraries(robot_endpoint
    adapters_core)
    # csm_metrics::profiling
    # zenohcxx::zenohc)
# ament_target_dependencies(robot_endpoint
#     rclcpp)
    # csm_metrics)

add_executable(client_endpoint
    "src/client_endpoint.cpp")
target_link_libraries(client_endpoint
    adapters_core)
    # csm_metrics::profiling
    # zenohcxx::zenohc)
# ament_target_dependencies(client_endpoint
#     rclcpp)
    # csm_metrics)

# --- Export -------------------------------------------------------------------

ament_export_dependencies(
    csm_utils
    rclcpp
    rosidl_default_runtime
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf2_ros
    tf2_geometry_msgs
    rosgraph_msgs
    phoenix_ros_driver
    multiscan_driver)
    # csm_metrics)
ament_package()

install(TARGETS
    robot_endpoint
    client_endpoint
    DESTINATION lib/${PROJECT_NAME})
