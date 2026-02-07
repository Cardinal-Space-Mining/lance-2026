cmake_minimum_required(VERSION 3.12)
project(lance)

# --- Dependencies -------------------------------------------------------------
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(phoenix_ros_driver REQUIRED)
find_package(cardinal_perception REQUIRED)
find_package(csm_metrics REQUIRED)

find_package(Eigen3 REQUIRED)

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

# --- Target Config ------------------------------------------------------------
set(ROBOT_TARGET "ALL" CACHE STRING
    "Which robot(s) to build for: \"LANCE1\", \"LANCE2\", \"ALL\"")
set_property(CACHE ROBOT_TARGET PROPERTY STRINGS LANCE1 LANCE2 ALL)

set(BUILD_LANCE1 FALSE)
set(BUILD_LANCE2 FALSE)
if(ROBOT_TARGET STREQUAL "LANCE1" OR ROBOT_TARGET STREQUAL "ALL")
    set(BUILD_LANCE1 TRUE)
endif()
if(ROBOT_TARGET STREQUAL "LANCE2" OR ROBOT_TARGET STREQUAL "ALL")
    set(BUILD_LANCE2 TRUE)
endif()

message(STATUS
    "Robot target is \"${ROBOT_TARGET}\";\n"
    "\tLANCE-1 enabled: ${BUILD_LANCE1}\n"
    "\tLANCE-2 enabled: ${BUILD_LANCE2}")


# --- Custom Types -------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/SetRobotMode.srv")

rosidl_get_typesupport_target(lance_custom_types
    ${PROJECT_NAME} rosidl_typesupport_cpp)

# --- Watchdog -----------------------------------------------------------------
add_executable(robot_status
    "src/watchdog_node.cpp")
target_link_libraries(robot_status
    "${lance_custom_types}")
ament_target_dependencies(robot_status
    rclcpp
    std_msgs)

# --- Robot Controller ---------------------------------------------------------
set(robot_controller_srcs
    "src/control_node.cpp"
    "src/robot/robot_params.cpp"
    "src/robot/collection_state.cpp"
    "src/robot/robot_controller.cpp"
    "src/robot/controllers/auto_controller.cpp"
    "src/robot/controllers/mining_controller.cpp"
    "src/robot/controllers/teleop_controller.cpp"
    "src/robot/controllers/offload_controller.cpp"
    "src/robot/controllers/traversal_controller.cpp"
    "src/robot/controllers/auto_mining_controller.cpp"
    "src/robot/controllers/auto_offload_controller.cpp"
    "src/robot/controllers/localization_controller.cpp")
set(robot_controller_libs
    csm_metrics::profiling
    Eigen3::Eigen)
set(robot_controller_ament_deps
    rclcpp
    std_msgs
    std_srvs
    nav_msgs
    sensor_msgs
    geometry_msgs
    tf2_ros
    tf2_geometry_msgs
    phoenix_ros_driver
    cardinal_perception
    csm_metrics)

if(BUILD_LANCE1)
    add_executable(lance1_controller
        ${robot_controller_srcs})
    target_link_libraries(lance1_controller
        ${robot_controller_libs})
    target_compile_definitions(lance1_controller
        PRIVATE LANCE=1)
    ament_target_dependencies(lance1_controller
        ${robot_controller_ament_deps})
endif()
if(BUILD_LANCE2)
    add_executable(lance2_controller
        ${robot_controller_srcs})
    target_link_libraries(lance2_controller
        ${robot_controller_libs})
    target_compile_definitions(lance2_controller
        PRIVATE LANCE=2)
    ament_target_dependencies(lance2_controller
        ${robot_controller_ament_deps})
endif()

# --- Motor Sim ----------------------------------------------------------------
set(motor_sim_srcs
    "src/motor_sim.cpp")
set(motor_sim_ament_deps
    rclcpp
    nav_msgs
    std_msgs
    sensor_msgs
    geometry_msgs
    phoenix_ros_driver)

if(BUILD_LANCE1)
    add_executable(lance1_motor_sim
        ${motor_sim_srcs})
    target_compile_definitions(lance1_motor_sim
        PRIVATE LANCE=1)
    ament_target_dependencies(lance1_motor_sim
        ${motor_sim_ament_deps})
endif()
if(BUILD_LANCE2)
    add_executable(lance2_motor_sim
        ${motor_sim_srcs})
    target_compile_definitions(lance2_motor_sim
        PRIVATE LANCE=2)
    ament_target_dependencies(lance2_motor_sim
        ${motor_sim_ament_deps})
endif()

# --- Export -------------------------------------------------------------------
ament_export_dependencies(
    rosidl_default_runtime
    rclcpp
    std_msgs
    std_srvs
    nav_msgs
    sensor_msgs
    geometry_msgs
    tf2_ros
    tf2_geometry_msgs
    phoenix_ros_driver
    cardinal_perception
    csm_metrics)
ament_package()

if(BUILD_LANCE1)
    install(TARGETS
        lance1_controller
        lance1_motor_sim
        DESTINATION lib/${PROJECT_NAME} )
endif()
if(BUILD_LANCE2)
    install(TARGETS
        lance2_controller
        lance2_motor_sim
        DESTINATION lib/${PROJECT_NAME} )
endif()

install(TARGETS
    robot_status
    DESTINATION lib/${PROJECT_NAME} )
install(DIRECTORY
    config
    launch
    DESTINATION share/${PROJECT_NAME} )
