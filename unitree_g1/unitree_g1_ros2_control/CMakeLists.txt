# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.16)
project(unitree_g1_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(realtime_tools REQUIRED)

# Unitree SDK2 version
set(UNITREE_SDK2_VERSION "2.0.2")

# Check if unitree_sdk2 target already exists
if(NOT TARGET unitree_sdk2)
  # Try to find installed SDK first
  find_package(unitree_sdk2 QUIET CONFIG)

  if(unitree_sdk2_FOUND)
    message(STATUS "Found unitree_sdk2: ${unitree_sdk2_DIR}")
  else()
    # Fetch and build the SDK
    message(STATUS "Fetching unitree_sdk2 v${UNITREE_SDK2_VERSION} from GitHub...")
    include(FetchContent)
    fetchcontent_declare(unitree_sdk2
      GIT_REPOSITORY https://github.com/unitreerobotics/unitree_sdk2
      GIT_TAG ${UNITREE_SDK2_VERSION}
      GIT_SHALLOW TRUE
    )
    fetchcontent_makeavailable(unitree_sdk2)
  endif()
endif()

# Prefer Unitree SDK DDS libs over the pixi environment to avoid runtime conflicts.
set(UNITREE_SDK_THIRDPARTY "${CMAKE_BINARY_DIR}/_deps/unitree_sdk2-src/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}")
if(EXISTS "${UNITREE_SDK_THIRDPARTY}")
  set(CMAKE_SHARED_LINKER_FLAGS "-Wl,-rpath,${UNITREE_SDK_THIRDPARTY} ${CMAKE_SHARED_LINKER_FLAGS}")
  message(STATUS "Prepended SDK DDS RPATH: ${UNITREE_SDK_THIRDPARTY}")
else()
  message(WARNING "SDK thirdparty DDS libraries not found at: ${UNITREE_SDK_THIRDPARTY}")
endif()

# Create library
add_library(${PROJECT_NAME} SHARED
  src/unitree_g1_system_interface.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} unitree_sdk2)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  diagnostic_msgs
  realtime_tools
)

# Export plugin
pluginlib_export_plugin_description_file(hardware_interface unitree_g1_system_interface_plugin.xml)

# Install
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  diagnostic_msgs
  realtime_tools
)

ament_package()
