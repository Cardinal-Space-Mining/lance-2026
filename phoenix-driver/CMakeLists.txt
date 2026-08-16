cmake_minimum_required(VERSION 3.8)
project(phoenix_ros_driver)

# --- Dependencies -------------------------------------------------------------
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(phoenix6 REQUIRED)

# --- Presets ------------------------------------------------------------------
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
endif()

if(NOT MSVC)
    if(NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE Release)
    endif()
endif()

# --- Patch for Lyrical --------------------------------------------------------
#
# Add the interface targets or definitions, include directories and libraries
# of packages to a target.
#
# Each package name must have been find_package()-ed before.
# Additionally the exported variables must have a prefix with the same case
# and the suffixes must be either _INTERFACES or _DEFINITIONS, _INCLUDE_DIRS,
# _LIBRARIES, _LIBRARY_DIRS, and _LINK_FLAGS.
# If _INTERFACES is not empty it will be used exclusively, otherwise the other
# variables are being used.
# If _LIBRARY_DIRS is not empty, _LIBRARIES which are not absolute paths already
# will be searched in those directories and their absolute paths will be used instead.
#
# :param target: the target name
# :type target: string
# :param ARGN: a list of package names, which can optionally start
#   with a SYSTEM keyword, followed by an INTERFACE or PUBLIC keyword.
#   If it starts with a SYSTEM keyword, it will be used in
#   target_include_directories() calls.
#   If it starts (or follows) with an INTERFACE or PUBLIC keyword,
#   this keyword will be used in the target_*() calls.
# :type ARGN: list of strings
#
# @public
#
function(auto_target_dependencies target)
  if(NOT TARGET ${target})
    message(FATAL_ERROR "auto_target_dependencies() the first argument must be a valid target name")
  endif()
  if(${ARGC} GREATER 0)
    cmake_parse_arguments(ARG "INTERFACE;PUBLIC;SYSTEM" "" "" ${ARGN})
    set(ARGVIND 1)
    set(system_keyword "")
    set(optional_keyword "")
    set(required_keyword "PUBLIC")
    if(ARG_SYSTEM)
      if(NOT "${ARGV${ARGVIND}}" STREQUAL "SYSTEM")
        message(FATAL_ERROR "auto_target_dependencies() SYSTEM keyword is only allowed before the package names and other keywords")
      endif()
      set(system_keyword SYSTEM)
      math(EXPR ARGVIND "${ARGVIND} + 1")
    endif()
    if(ARG_INTERFACE)
      if(NOT "${ARGV${ARGVIND}}" STREQUAL "INTERFACE")
        message(FATAL_ERROR "auto_target_dependencies() INTERFACE keyword is only allowed before the package names")
      endif()
      set(optional_keyword INTERFACE)
      set(required_keyword INTERFACE)
    endif()
    if(ARG_PUBLIC)
      if(NOT "${ARGV${ARGVIND}}" STREQUAL "PUBLIC")
        message(FATAL_ERROR "auto_target_dependencies() PUBLIC keyword is only allowed before the package names")
      endif()
      set(optional_keyword PUBLIC)
    endif()
    set(definitions "")
    set(include_dirs "")
    set(interfaces "")
    set(libraries "")
    set(link_flags "")
    foreach(package_name ${ARG_UNPARSED_ARGUMENTS})
      if(NOT "${${package_name}_FOUND}")
        message(FATAL_ERROR "auto_target_dependencies() the passed package name '${package_name}' was not found before")
      endif()
      # if a package provides modern CMake interface targets use them
      # exclusively assuming the classic CMake variables only exist for
      # backward compatibility
      set(use_modern_cmake FALSE)
      if(NOT "${${package_name}_TARGETS}" STREQUAL "")
        foreach(_target ${${package_name}_TARGETS})
          # only use actual targets
          # in case a package uses this variable for other content
          if(TARGET "${_target}")
            list_append_unique(interfaces ${_target})
            set(use_modern_cmake TRUE)
          endif()
        endforeach()
      endif()
      if(NOT use_modern_cmake AND NOT "${${package_name}_INTERFACES}" STREQUAL "")
        foreach(_interface ${${package_name}_INTERFACES})
          # only use actual targets
          # in case a package uses this variable for other content
          if(TARGET "${_interface}")
            list_append_unique(interfaces ${_interface})
            set(use_modern_cmake TRUE)
          endif()
        endforeach()
        if(use_modern_cmake)
          message(DEPRECATION
            "Package ${package_name} is exporting the variable "
            "${package_name}_INTERFACES which is deprecated, it should export
            ${package_name}_TARGETS instead")
        endif()
      endif()
      if(NOT use_modern_cmake)
        # otherwise use the classic CMake variables
        list_append_unique(definitions ${${package_name}_DEFINITIONS})
        list_append_unique(include_dirs ${${package_name}_INCLUDE_DIRS})
        foreach(library ${${package_name}_LIBRARIES})
          if(NOT "${${package_name}_LIBRARY_DIRS}" STREQUAL "")
            if(NOT IS_ABSOLUTE ${library} OR NOT EXISTS ${library})
              unset(lib CACHE)
              find_library(lib NAMES ${library} PATHS ${${package_name}_LIBRARY_DIRS} NO_DEFAULT_PATH)
              if(lib)
                set(library ${lib})
              endif()
            endif()
          endif()
          list(APPEND libraries ${library})
        endforeach()
        list_append_unique(link_flags ${${package_name}_LINK_FLAGS})
      endif()
    endforeach()
    if(NOT ARG_INTERFACE)
      target_compile_definitions(${target}
        ${required_keyword} ${definitions})
      # the interface include dirs must be ordered
      set(interface_include_dirs)
      foreach(interface ${interfaces})
        get_target_property(_include_dirs ${interface} INTERFACE_INCLUDE_DIRECTORIES)
        if(_include_dirs)
          list_append_unique(interface_include_dirs ${_include_dirs})
        endif()
      endforeach()
      ament_include_directories_order(ordered_interface_include_dirs ${interface_include_dirs})
      # the interface include dirs are used privately to ensure proper order
      # and the interfaces cover the public case
      target_include_directories(${target} ${system_keyword}
        PRIVATE ${ordered_interface_include_dirs})
    endif()
    ament_include_directories_order(ordered_include_dirs ${include_dirs})
    target_link_libraries(${target}
      ${optional_keyword} ${interfaces})
    target_include_directories(${target} ${system_keyword}
      ${required_keyword} ${ordered_include_dirs})
    if(NOT ARG_INTERFACE)
      ament_libraries_deduplicate(unique_libraries ${libraries})
      target_link_libraries(${target}
        ${optional_keyword} ${unique_libraries})
      foreach(link_flag IN LISTS link_flags)
        set_property(TARGET ${target} APPEND_STRING PROPERTY LINK_FLAGS " ${link_flag} ")
      endforeach()
    endif()
  endif()
endfunction()

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


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wno-psabi)
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
        rclcpp
        std_msgs)
    target_compile_features(lance2_phx6_driver PUBLIC c_std_99 cxx_std_20)     # not necessary but silences a lot of warnings
endif()

# --- Export -------------------------------------------------------------------
ament_export_dependencies(
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
