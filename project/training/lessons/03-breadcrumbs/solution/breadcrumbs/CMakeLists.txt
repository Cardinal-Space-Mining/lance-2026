cmake_minimum_required(VERSION 3.12)
project(breadcrumbs)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

# --- Dependencies -------------------------------------------------------------
# Every package listed here must also be a <depend> in package.xml.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# --- Executable ---------------------------------------------------------------
add_executable(breadcrumbs_node src/breadcrumbs_node.cpp)
target_link_libraries(breadcrumbs_node
    rclcpp::rclcpp
    tf2::tf2
    tf2_ros::tf2_ros
    ${geometry_msgs_TARGETS}
    ${visualization_msgs_TARGETS})

install(TARGETS breadcrumbs_node
    DESTINATION lib/${PROJECT_NAME})

ament_package()
