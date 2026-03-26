cmake_minimum_required(VERSION 3.14)

if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  if(EXISTS "C:/vcpkg/scripts/buildsystems/vcpkg.cmake")
    set(CMAKE_TOOLCHAIN_FILE "C:/vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE FILEPATH "")
    message(STATUS "Auto-detected vcpkg at: C:/vcpkg")
  endif()
endif()

project(eskf_cpp_reduced VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include(GNUInstallDirs)

if(MSVC)
  add_compile_options(/W4)
  add_compile_definitions(_USE_MATH_DEFINES)
  if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(/O2)
  endif()
else()
  add_compile_options(-Wall -Wextra -Wpedantic)
  if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-O3 -march=native)
  endif()
endif()

find_package(Eigen3 QUIET NO_MODULE)
if(NOT Eigen3_FOUND)
  set(_eigen_hints
    "$ENV{EIGEN3_ROOT}"
    "$ENV{EIGEN_ROOT}"
    "C:/Program Files/MATLAB/R2023a/toolbox/shared/robotics/externalDependency/eigen/include/eigen3"
  )
  find_path(EIGEN3_INCLUDE_DIR
    NAMES Eigen/Core
    HINTS ${_eigen_hints}
    PATH_SUFFIXES eigen3 include/eigen3 include
  )
  if(NOT EIGEN3_INCLUDE_DIR)
    message(FATAL_ERROR
      "Eigen3 was not found. Set Eigen3_DIR or EIGEN3_ROOT, or install Eigen3.")
  endif()

  add_library(Eigen3::Eigen INTERFACE IMPORTED)
  set_target_properties(Eigen3::Eigen PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}"
  )
endif()

find_package(yaml-cpp QUIET CONFIG)
if(NOT yaml-cpp_FOUND)
  set(_yaml_hints
    "$ENV{YAML_CPP_ROOT}"
    "$ENV{yaml-cpp_DIR}"
    "C:/Program Files/MATLAB/R2023a/sys/ros2/win64/ros2/opt/yaml_cpp_vendor"
  )
  find_path(YAML_CPP_INCLUDE_DIR
    NAMES yaml-cpp/yaml.h
    HINTS ${_yaml_hints}
    PATH_SUFFIXES include
  )
  find_library(YAML_CPP_LIBRARY
    NAMES yaml-cpp libyaml-cpp yaml-cppd
    HINTS ${_yaml_hints}
    PATH_SUFFIXES lib
  )
  if(NOT YAML_CPP_INCLUDE_DIR OR NOT YAML_CPP_LIBRARY)
    message(FATAL_ERROR
      "yaml-cpp was not found. Set yaml-cpp_DIR or YAML_CPP_ROOT, or install yaml-cpp.")
  endif()

  add_library(yaml-cpp UNKNOWN IMPORTED)
  set_target_properties(yaml-cpp PROPERTIES
    IMPORTED_LOCATION "${YAML_CPP_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${YAML_CPP_INCLUDE_DIR}"
  )
endif()

find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(sensor_msgs QUIET)
find_package(geometry_msgs QUIET)
find_package(nav_msgs QUIET)
find_package(mavros_msgs QUIET)

set(ROS2_FOUND FALSE)
if(ament_cmake_FOUND AND rclcpp_FOUND AND sensor_msgs_FOUND AND geometry_msgs_FOUND AND nav_msgs_FOUND AND mavros_msgs_FOUND)
  set(ROS2_FOUND TRUE)
  message(STATUS "ROS2 found - building reduced ROS2 node")
else()
  message(STATUS "ROS2 not found - building reduced standalone library only")
endif()

set(ESKF_REDUCED_CORE_SOURCES
  src/eskf_jacobian_reduced.cpp
  src/eskf_core_reduced.cpp
  src/eskf_config.cpp
  src/eskf_logger.cpp
  src/utils/print.cpp
)

add_library(eskf_core_reduced STATIC ${ESKF_REDUCED_CORE_SOURCES})

target_include_directories(eskf_core_reduced PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(EIGEN3_INCLUDE_DIR)
  target_include_directories(eskf_core_reduced PUBLIC "${EIGEN3_INCLUDE_DIR}")
endif()

if(YAML_CPP_INCLUDE_DIR)
  target_include_directories(eskf_core_reduced PUBLIC "${YAML_CPP_INCLUDE_DIR}")
endif()

target_link_libraries(eskf_core_reduced PUBLIC
  Eigen3::Eigen
  yaml-cpp
)

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/config/eskf_reduced_params.yaml
  ${CMAKE_CURRENT_BINARY_DIR}/eskf_reduced_params.yaml
  COPYONLY
)

if(ROS2_FOUND)
  add_executable(eskf_node_reduced src/eskf_node_reduced.cpp)

  target_link_libraries(eskf_node_reduced eskf_core_reduced)
  ament_target_dependencies(eskf_node_reduced
    rclcpp
    sensor_msgs
    geometry_msgs
    nav_msgs
    mavros_msgs
  )

  ament_package()
endif()

install(TARGETS eskf_core_reduced
  EXPORT eskf_cpp_reduced_targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(ROS2_FOUND)
  install(TARGETS eskf_node_reduced
    DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
  )
else()
  install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
  )
endif()

install(DIRECTORY include/
  DESTINATION include
)

message(STATUS "===============================================")
message(STATUS "Reduced ESKF C++ Build Configuration")
message(STATUS "===============================================")
message(STATUS "  Project:         ${PROJECT_NAME}")
message(STATUS "  C++ Standard:    ${CMAKE_CXX_STANDARD}")
message(STATUS "  Build Type:      ${CMAKE_BUILD_TYPE}")
message(STATUS "  ROS2:            ${ROS2_FOUND}")
message(STATUS "===============================================")
