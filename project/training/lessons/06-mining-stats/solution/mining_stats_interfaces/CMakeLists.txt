cmake_minimum_required(VERSION 3.12)
project(mining_stats_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Generates C++ (and Python) code for every listed interface
rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/MiningStats.msg"
    "srv/ResetStats.srv"
    DEPENDENCIES std_msgs)   # MiningStats uses std_msgs/Header

ament_export_dependencies(rosidl_default_runtime)
ament_package()
