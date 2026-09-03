cmake_minimum_required(VERSION 3.8)
project(phoenix_ros_driver)

# --- Dependencies -------------------------------------------------------------
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(csm_utils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(phoenix6 REQUIRED)

standard_compile_options()
robot_target()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wno-psabi)
endif()

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

include_directories("src")


# --- Generate Message Types ---------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
    msg/TalonCtrl.msg
    msg/TalonInfo.msg
    msg/TalonFaults.msg
    DEPENDENCIES std_msgs)

rosidl_get_typesupport_target(phoenix_ros_driver_custom_types
    ${PROJECT_NAME} rosidl_typesupport_cpp)

# --- Lance 1 ---------------------------------------------------------
if(BUILD_LANCE1)
    # --- Phoenix 5 Driver ---------------------------------------------------------
    set(PHX5_EXAMPLE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/Phoenix5-Linux-Example)
    if(${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "aarch64-linux-gnu")  # aarch64
        set(PHX5_LIB_DIR ${PHX5_EXAMPLE_DIR}/lib/arm64)
    else()
        set(PHX5_LIB_DIR ${PHX5_EXAMPLE_DIR}/lib/x86-64)
    endif()

    add_executable(lance1_phx5_driver
        src/lance1/phx5_driver.cpp)
    target_include_directories(lance1_phx5_driver PUBLIC
        $<BUILD_INTERFACE:${PHX5_EXAMPLE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
    target_link_libraries(lance1_phx5_driver
        ${phoenix_ros_driver_custom_types}
        ${PHX5_LIB_DIR}/libCTRE_Phoenix.so
        ${PHX5_LIB_DIR}/libCTRE_PhoenixCCI.so
        ${PHX5_LIB_DIR}/libCTRE_PhoenixTools.so)
    auto_target_dependencies(lance1_phx5_driver
        csm_utils
        rclcpp
        std_msgs)
    target_compile_features(lance1_phx5_driver PUBLIC c_std_99 cxx_std_20)     # not necessary but silences a lot of warnings

    # --- Phoenix 6 Driver ---------------------------------------------------------
    add_executable(lance1_phx6_driver
        src/lance1/phx6_driver.cpp)
    target_link_libraries(lance1_phx6_driver
        phoenix6
        ${phoenix_ros_driver_custom_types})
    auto_target_dependencies(lance1_phx6_driver
        csm_utils
        rclcpp
        std_msgs)
    target_compile_features(lance1_phx6_driver PUBLIC c_std_99 cxx_std_20)     # not necessary but silences a lot of warnings
endif()

# --- Lance 2 ---------------------------------------------------------
if(BUILD_LANCE2)
    # --- Phoenix 6 Driver ---------------------------------------------------------
    add_executable(lance2_phx6_driver
        src/lance2/phx6_base.cpp
        src/lance2/phx6_driver.cpp)
    target_link_libraries(lance2_phx6_driver
        phoenix6
        ${phoenix_ros_driver_custom_types})
    auto_target_dependencies(lance2_phx6_driver
        csm_utils
        rclcpp
        std_msgs)
    target_compile_features(lance2_phx6_driver PUBLIC c_std_99 cxx_std_20)     # not necessary but silences a lot of warnings
endif()

# --- Export -------------------------------------------------------------------
ament_export_dependencies(
    csm_utils
    rclcpp
    nav_msgs
    std_msgs
    sensor_msgs
    geometry_msgs
    rosidl_default_runtime)
ament_package()

if(BUILD_LANCE1)
    install(TARGETS
        lance1_phx5_driver
        lance1_phx6_driver
        DESTINATION lib/${PROJECT_NAME} )
endif()   

if(BUILD_LANCE2)
    install(TARGETS
        lance2_phx6_driver
        DESTINATION lib/${PROJECT_NAME} )
endif()   

install(DIRECTORY
    config
    launch
    scripts
    DESTINATION share/${PROJECT_NAME} )

install(CODE
    "execute_process(COMMAND cp \
    ${PHX5_LIB_DIR}/libCTRE_Phoenix.so \
    ${PHX5_LIB_DIR}/libCTRE_PhoenixCCI.so \
    ${PHX5_LIB_DIR}/libCTRE_PhoenixTools.so \
    ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/ )" )
install(CODE
    "execute_process(COMMAND patchelf --set-rpath \
    ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME} \
    ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/libCTRE_Phoenix.so )" )
install(CODE
    "execute_process(COMMAND patchelf --set-rpath \
    ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME} \
    ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/lance1_phx5_driver )" )
