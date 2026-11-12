cmake_minimum_required(VERSION 3.12)
project(hopper_sensor_sim)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

# --- Dependencies -------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(phoenix_ros_driver REQUIRED)

# --- Executable ---------------------------------------------------------------
add_executable(hopper_sensor_sim_node src/hopper_sensor_sim_node.cpp)
target_include_directories(hopper_sensor_sim_node PRIVATE include)
target_link_libraries(hopper_sensor_sim_node
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
    ${phoenix_ros_driver_TARGETS})

install(TARGETS hopper_sensor_sim_node
    DESTINATION lib/${PROJECT_NAME})

# --- Tests --------------------------------------------------------------------
if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(fullness_parser_test test/fullness_parser_test.cpp)
    target_include_directories(fullness_parser_test PRIVATE include)
endif()

ament_package()
