cmake_minimum_required(VERSION 3.12)
project(lance)

# --- Dependencies -------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(csm_metrics REQUIRED)
find_package(net_adapter REQUIRED)
find_package(phoenix_ros_driver REQUIRED)
find_package(cardinal_perception REQUIRED)

find_package(Eigen3 REQUIRED)

# if(BUILD_TESTING)
#     find_package(ament_cmake_gtest REQUIRED)
# endif()

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


include_directories("src")


# --- Common Lib ---------------------------------------------------------------
set(lance_core_srcs
    "src/robot/core/robot_params.cpp"
    "src/robot/core/stall_analyzer.cpp"
    "src/robot/core/collection_state.cpp"
    "src/robot/control/robot_controller.cpp"
    "src/robot/control/auto/auto_controller.cpp"
    "src/robot/control/auto/auto_mining_controller.cpp"
    "src/robot/control/auto/auto_offload_controller.cpp"
    "src/robot/control/shared/mining_controller.cpp"
    "src/robot/control/shared/offload_controller.cpp"
    "src/robot/control/shared/traversal_controller.cpp"
    "src/robot/control/shared/localization_controller.cpp"
    "src/robot/control/teleop/remote_commands.cpp"
    "src/robot/control/teleop/teleop_controller.cpp"
    "src/robot/sensing/tf_cache.cpp"
    "src/robot/sensing/path_plan.cpp"
    "src/robot/sensing/mining_eval.cpp"
    "src/robot/sensing/reflector_hint.cpp"
    "src/robot/planning/mining_planner.cpp"
    "src/robot/planning/offload_planner.cpp"
    "src/robot/telemetry/markers.cpp"
    "src/robot/telemetry/serializer.cpp"
    "src/robot/telemetry/deserializer.cpp")
set(lance_core_libs
    Eigen3::Eigen)
set(lance_core_ament_deps
    rclcpp
    std_msgs
    std_srvs
    nav_msgs
    sensor_msgs
    geometry_msgs
    visualization_msgs
    tf2_ros
    tf2_geometry_msgs
    net_adapter
    phoenix_ros_driver
    cardinal_perception)

if(BUILD_LANCE1)
    add_library(lance1_core
        STATIC
        ${lance_core_srcs})
    target_link_libraries(lance1_core
        ${lance_core_libs})
    target_compile_definitions(lance1_core
        PUBLIC LANCE=1)
    ament_target_dependencies(lance1_core
        ${lance_core_ament_deps})
endif()
if(BUILD_LANCE2)
    add_library(lance2_core
        STATIC
        ${lance_core_srcs})
    target_link_libraries(lance2_core
        ${lance_core_libs})
    target_compile_definitions(lance2_core
        PUBLIC LANCE=2)
    ament_target_dependencies(lance2_core
        ${lance_core_ament_deps})
endif()

# if(BUILD_TESTING)
#     ament_add_gtest(stall_analyzer_test
#         "test/stall_analyzer_test.cpp")
#     if(BUILD_LANCE2)
#         target_link_libraries(stall_analyzer_test lance2_core)
#     else()
#         target_link_libraries(stall_analyzer_test lance1_core)
#     endif()
#     ament_target_dependencies(stall_analyzer_test
#         phoenix_ros_driver)
# endif()

# --- Robot Controller ---------------------------------------------------------
set(robot_controller_srcs
    "src/control_node.cpp")
set(robot_controller_libs
    csm_metrics::profiling)     # eigen included from core lib
set(robot_controller_ament_deps
    csm_metrics)                # rest of deps included from core lib

if(BUILD_LANCE1)
    add_executable(lance1_controller
        ${robot_controller_srcs})
    target_link_libraries(lance1_controller
        ${robot_controller_libs}
        lance1_core)
    ament_target_dependencies(lance1_controller
        ${robot_controller_ament_deps})
endif()
if(BUILD_LANCE2)
    add_executable(lance2_controller
        ${robot_controller_srcs})
    target_link_libraries(lance2_controller
        ${robot_controller_libs}
        lance2_core)
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

# --- Mission Control ----------------------------------------------------------
set(mission_control_srcs
    "src/client_node.cpp"
    "src/mission_control/advanced_controls.cpp")
set(mission_control_ament_deps)     # rest of deps included from core lib

if(BUILD_LANCE1)
    add_executable(lance1_mission_control
        ${mission_control_srcs})
    target_link_libraries(lance1_mission_control
        lance1_core)
    ament_target_dependencies(lance1_mission_control
        ${mission_control_ament_deps})
endif()
if(BUILD_LANCE2)
    add_executable(lance2_mission_control
        ${mission_control_srcs})
    target_link_libraries(lance2_mission_control
        lance2_core)
    ament_target_dependencies(lance2_mission_control
        ${mission_control_ament_deps})
endif()

# --- Export -------------------------------------------------------------------
ament_export_dependencies(
    rclcpp
    std_msgs
    std_srvs
    nav_msgs
    sensor_msgs
    geometry_msgs
    visualization_msgs
    tf2_ros
    tf2_geometry_msgs
    csm_metrics
    net_adapter
    phoenix_ros_driver
    cardinal_perception)
ament_package()

if(BUILD_LANCE1)
    install(TARGETS
        lance1_controller
        lance1_motor_sim
        lance1_mission_control
        DESTINATION lib/${PROJECT_NAME} )
endif()
if(BUILD_LANCE2)
    install(TARGETS
        lance2_controller
        lance2_motor_sim
        lance2_mission_control
        DESTINATION lib/${PROJECT_NAME} )
endif()

install(PROGRAMS
    scripts/hopper_fullness.py
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
    config
    launch
    DESTINATION share/${PROJECT_NAME} )
