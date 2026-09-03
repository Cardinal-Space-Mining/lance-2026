cmake_minimum_required(VERSION 3.12)
project(lance)

# --- Dependencies -------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(csm_utils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(csm_metrics REQUIRED)
find_package(net_adapter REQUIRED)
find_package(phoenix_ros_driver REQUIRED)
find_package(cardinal_perception REQUIRED)

find_package(Eigen3 REQUIRED)

standard_compile_options()
robot_target()

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
    "src/robot/telemetry/markers.cpp"
    "src/robot/telemetry/serializer.cpp"
    "src/robot/telemetry/deserializer.cpp")
set(lance_core_libs
    Eigen3::Eigen)
set(lance_core_ament_deps
    csm_utils
    rclcpp
    std_msgs
    std_srvs
    nav_msgs
    sensor_msgs
    geometry_msgs
    visualization_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    net_adapter
    phoenix_ros_driver
    cardinal_perception)

# --- Robot Controller ---------------------------------------------------------
set(robot_controller_srcs
    "src/control_node.cpp")
set(robot_controller_libs
    csm_metrics::profiling)     # eigen included from core lib
set(robot_controller_ament_deps)    # rest of deps included from core lib

# --- Motor Sim --------------------------------------------------------------
set(motor_sim_srcs
    "src/motor_sim.cpp")
set(motor_sim_ament_deps
    csm_utils
    rclcpp
    nav_msgs
    std_msgs
    sensor_msgs
    geometry_msgs
    phoenix_ros_driver)

# --- Mission Control ----------------------------------------------------------
set(mission_control_srcs
    "src/client_node.cpp"
    "src/mission_control/advanced_controls.cpp")
set(mission_control_ament_deps)     # rest of deps included from core lib

# --- Per-Target Build ------------------------------------------------------
# Builds the core lib and all three executables for a single version number
# (1 or 2), reusing the source/dependency lists set above. Called once per
# enabled ROBOT_TARGET below instead of duplicating each block per target.
function(lance_build_target N)
    set(core_target "lance${N}_core")
    add_library(${core_target} STATIC
        ${lance_core_srcs})
    target_link_libraries(${core_target}
        ${lance_core_libs})
    target_compile_definitions(${core_target}
        PUBLIC LANCE=${N})
    auto_target_dependencies(${core_target}
        ${lance_core_ament_deps})

    set(controller_target "lance${N}_controller")
    add_executable(${controller_target}
        ${robot_controller_srcs})
    target_link_libraries(${controller_target}
        ${robot_controller_libs}
        ${core_target})
    auto_target_dependencies(${controller_target}
        ${robot_controller_ament_deps})

    set(motor_sim_target "lance${N}_motor_sim")
    add_executable(${motor_sim_target}
        ${motor_sim_srcs})
    target_compile_definitions(${motor_sim_target}
        PRIVATE LANCE=${N})
    auto_target_dependencies(${motor_sim_target}
        ${motor_sim_ament_deps})

    set(mission_control_target "lance${N}_mission_control")
    add_executable(${mission_control_target}
        ${mission_control_srcs})
    target_link_libraries(${mission_control_target}
        ${core_target})
    auto_target_dependencies(${mission_control_target}
        ${mission_control_ament_deps})

    install(TARGETS
        ${controller_target}
        ${motor_sim_target}
        ${mission_control_target}
        DESTINATION lib/${PROJECT_NAME})
endfunction()

if(BUILD_LANCE1)
    lance_build_target(1)
endif()
if(BUILD_LANCE2)
    lance_build_target(2)
endif()

# --- Export -------------------------------------------------------------------
ament_export_dependencies(
    csm_utils
    rclcpp
    std_msgs
    std_srvs
    nav_msgs
    sensor_msgs
    geometry_msgs
    visualization_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    csm_metrics
    net_adapter
    phoenix_ros_driver
    cardinal_perception)
ament_package()

install(PROGRAMS
    scripts/hopper_fullness.py
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
    config
    launch
    DESTINATION share/${PROJECT_NAME} )
