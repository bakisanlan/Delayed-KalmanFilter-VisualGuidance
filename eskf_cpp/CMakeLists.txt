cmake_minimum_required(VERSION 3.14)

# Auto-detect vcpkg if not already set
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  if(EXISTS "C:/vcpkg/scripts/buildsystems/vcpkg.cmake")
    set(CMAKE_TOOLCHAIN_FILE "C:/vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE FILEPATH "")
    message(STATUS "Auto-detected vcpkg at: C:/vcpkg")
  endif()
endif()

project(eskf_cpp VERSION 1.0.0)

# C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler warnings and optimization
if(MSVC)
  # MSVC-specific flags
  add_compile_options(/W4)
  add_compile_definitions(_USE_MATH_DEFINES)  # Enable M_PI
  if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(/O2)
  endif()
else()
  # GCC/Clang flags
  add_compile_options(-Wall -Wextra -Wpedantic)
  if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-O3 -march=native)
  endif()
endif()

# ============================================================================
# Find Dependencies
# ============================================================================

# Eigen3 (required)
find_package(Eigen3 REQUIRED NO_MODULE)

# yaml-cpp (required)
find_package(yaml-cpp REQUIRED)

# ROS2 packages (optional - for building the node)
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(sensor_msgs QUIET)
find_package(geometry_msgs QUIET)
find_package(nav_msgs QUIET)
find_package(std_msgs QUIET)
find_package(mavros_msgs QUIET)
find_package(rosidl_default_generators QUIET)

set(ROS2_FOUND FALSE)
if(ament_cmake_FOUND AND rclcpp_FOUND AND sensor_msgs_FOUND)
  set(ROS2_FOUND TRUE)
  message(STATUS "ROS2 found - building ROS2 node")
else()
  message(STATUS "ROS2 not found - building standalone library only")
endif()

# ============================================================================
# Core ESKF Library (No ROS2 dependencies)
# ============================================================================

set(ESKF_CORE_SOURCES
  src/eskf_jacobian.cpp
  src/eskf_core.cpp
  src/eskf_config.cpp
  src/utils/print.cpp
)

add_library(eskf_core STATIC ${ESKF_CORE_SOURCES})

target_include_directories(eskf_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(eskf_core PUBLIC
  Eigen3::Eigen
  yaml-cpp
)

# ============================================================================
# Standalone Test Harness (No ROS2)
# ============================================================================

add_executable(eskf_test_harness src/eskf_test_harness.cpp)

target_link_libraries(eskf_test_harness PRIVATE
  eskf_core
)

# Copy config file for testing
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/config/eskf_params.yaml
  ${CMAKE_CURRENT_BINARY_DIR}/eskf_params.yaml
  COPYONLY
)

# ============================================================================
# ROS2 Components (if available)
# ============================================================================

if(ROS2_FOUND)
  # Generate custom messages
  if(rosidl_default_generators_FOUND)
    rosidl_generate_interfaces(${PROJECT_NAME}
      "msg/ESKFState.msg"
      DEPENDENCIES std_msgs geometry_msgs
    )
  endif()
  
  # ROS2 Node
  add_executable(eskf_node src/eskf_node.cpp)
  
  # Link dependencies using target_link_libraries with keyword signature
  # Note: We must link eskf_core and ROS2 deps together to avoid signature conflicts
  target_link_libraries(eskf_node
    PUBLIC
      ${rclcpp_TARGETS}
      ${sensor_msgs_TARGETS}
      ${geometry_msgs_TARGETS}
      ${nav_msgs_TARGETS}
      ${std_msgs_TARGETS}
      ${mavros_msgs_TARGETS}
    PRIVATE
      eskf_core
  )
  
  # Add include directories from ROS2 packages
  target_include_directories(eskf_node PUBLIC
    ${rclcpp_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${mavros_msgs_INCLUDE_DIRS}
  )
  
  if(rosidl_default_generators_FOUND)
    rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
    target_link_libraries(eskf_node PUBLIC ${cpp_typesupport_target})
  endif()
  
  # Install
  install(TARGETS eskf_node eskf_test_harness
    DESTINATION lib/${PROJECT_NAME}
  )
  
  install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
  )
  
  install(DIRECTORY include/
    DESTINATION include
  )
  
  ament_package()
  
else()
  # Standalone installation (non-ROS2)
  install(TARGETS eskf_core eskf_test_harness
    EXPORT eskf_cpp_targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
  )
  
  install(DIRECTORY include/
    DESTINATION include
  )
  
  install(DIRECTORY config/
    DESTINATION share/eskf_cpp/config
  )
endif()

# ============================================================================
# Print Summary
# ============================================================================

message(STATUS "===============================================")
message(STATUS "ESKF C++ Build Configuration")
message(STATUS "===============================================")
message(STATUS "  C++ Standard:    ${CMAKE_CXX_STANDARD}")
message(STATUS "  Build Type:      ${CMAKE_BUILD_TYPE}")
message(STATUS "  Eigen3:          ${EIGEN3_INCLUDE_DIR}")
message(STATUS "  yaml-cpp:        FOUND")
message(STATUS "  ROS2:            ${ROS2_FOUND}")
message(STATUS "===============================================")
