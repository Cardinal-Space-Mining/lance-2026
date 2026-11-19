cmake_minimum_required(VERSION 3.12)
project(motor_monitor)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

# --- Dependencies -------------------------------------------------------------
# Every package listed here must also be a <depend> in package.xml.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# TODO (Step 2): find_package() the message packages you use

# --- Executable ---------------------------------------------------------------
add_executable(motor_monitor_node src/motor_monitor_node.cpp)
target_link_libraries(motor_monitor_node
    rclcpp::rclcpp)
    # TODO (Step 2): link the message packages, e.g. ${std_msgs_TARGETS}

# Install the executable so `ros2 run motor_monitor motor_monitor_node` finds it
install(TARGETS motor_monitor_node
    DESTINATION lib/${PROJECT_NAME})

ament_package()
