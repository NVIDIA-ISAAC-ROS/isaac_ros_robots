# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.16)
project(unitree_hg_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(realtime_tools REQUIRED)

set(UNITREE_SDK2_REVISION "21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b")
if(NOT TARGET unitree_sdk2)
  # Unitree SDK2 bundles Cyclone DDS 0.10.2. Build that exact ABI from source
  # with null-safe validation of malformed TypeLookup replies from the G1.
  set(CYCLONEDDS_REVISION "9995905bce6c4cf9f740d6438bbf7fcfd1c83dfd")
  message(STATUS "Fetching Cyclone DDS ${CYCLONEDDS_REVISION} from GitHub")
  include(FetchContent)
  find_package(Threads REQUIRED)

  function(_unitree_hg_add_cyclonedds)
    # Keep private dependency options local to this function. Normal variables
    # take precedence over cache values without overwriting caller choices.
    set(BUILD_TESTING OFF)
    set(BUILD_IDLC OFF)
    set(BUILD_DDSPERF OFF)
    set(BUILD_EXAMPLES OFF)
    set(ENABLE_SECURITY OFF)
    fetchcontent_declare(cyclonedds
      GIT_REPOSITORY https://github.com/eclipse-cyclonedds/cyclonedds.git
      GIT_TAG ${CYCLONEDDS_REVISION}
      PATCH_COMMAND git apply --unidiff-zero --whitespace=nowarn
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/patches/cyclonedds-0.10.2-typeobject-null-safety.patch")
    fetchcontent_getproperties(cyclonedds)
    if(NOT cyclonedds_POPULATED)
      fetchcontent_populate(cyclonedds)
    endif()
    if(NOT TARGET ddsc)
      add_subdirectory(
        "${cyclonedds_SOURCE_DIR}" "${cyclonedds_BINARY_DIR}" EXCLUDE_FROM_ALL)
    endif()
  endfunction()
  _unitree_hg_add_cyclonedds()

  # Cyclone DDS is private to this package. Install its runtime library without
  # exporting the dependency's headers or CMake package metadata.
  install(TARGETS ddsc
    ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

  message(STATUS "Fetching unitree_sdk2 ${UNITREE_SDK2_REVISION} from GitHub")
  fetchcontent_declare(unitree_sdk2
    GIT_REPOSITORY https://github.com/unitreerobotics/unitree_sdk2.git
    GIT_TAG ${UNITREE_SDK2_REVISION})
  fetchcontent_getproperties(unitree_sdk2)
  if(NOT unitree_sdk2_POPULATED)
    fetchcontent_populate(unitree_sdk2)
  endif()

  set(UNITREE_SDK_LIB_DIR "${unitree_sdk2_SOURCE_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR}")
  set(UNITREE_SDK_THIRDPARTY
    "${unitree_sdk2_SOURCE_DIR}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}")

  add_library(unitree_ddscxx SHARED IMPORTED GLOBAL)
  set_target_properties(unitree_ddscxx PROPERTIES
    IMPORTED_LOCATION "${UNITREE_SDK_THIRDPARTY}/libddscxx.so"
    IMPORTED_NO_SONAME TRUE)
  target_link_libraries(unitree_ddscxx INTERFACE ddsc Threads::Threads)
  target_include_directories(unitree_ddscxx INTERFACE
    "${unitree_sdk2_SOURCE_DIR}/thirdparty/include"
    "${unitree_sdk2_SOURCE_DIR}/thirdparty/include/ddscxx")

  add_library(unitree_sdk2 STATIC IMPORTED GLOBAL)
  set_target_properties(unitree_sdk2 PROPERTIES
    IMPORTED_LOCATION "${UNITREE_SDK_LIB_DIR}/libunitree_sdk2.a")
  target_link_libraries(unitree_sdk2 INTERFACE ddsc unitree_ddscxx Threads::Threads)
  target_include_directories(unitree_sdk2 INTERFACE "${unitree_sdk2_SOURCE_DIR}/include")

  install(DIRECTORY "${unitree_sdk2_SOURCE_DIR}/include/" DESTINATION include)
  install(DIRECTORY "${unitree_sdk2_SOURCE_DIR}/thirdparty/include/ddscxx/"
    DESTINATION include/ddscxx)
  install(FILES
    "${UNITREE_SDK_THIRDPARTY}/libddscxx.so"
    "${UNITREE_SDK_THIRDPARTY}/libddscxx.so.0"
    DESTINATION lib)
endif()

add_library(${PROJECT_NAME} SHARED)
target_sources(${PROJECT_NAME} PRIVATE src/version.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} unitree_sdk2)
set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH "$ORIGIN")
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface pluginlib rclcpp rclcpp_lifecycle diagnostic_msgs realtime_tools)

target_sources(${PROJECT_NAME} PRIVATE src/unitree_hg_crc.cpp)
target_sources(${PROJECT_NAME} PRIVATE src/channel_factory_guard.cpp)
target_sources(${PROJECT_NAME} PRIVATE src/sdk_index_map.cpp)
target_sources(${PROJECT_NAME} PRIVATE src/joint_index_tables.cpp)
target_sources(${PROJECT_NAME} PRIVATE src/unitree_hg_system_interface.cpp)
target_sources(${PROJECT_NAME} PRIVATE src/unitree_dex3_system_interface.cpp)

pluginlib_export_plugin_description_file(hardware_interface unitree_hg_ros2_control_hardware.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_crc test/test_crc.cpp)
  target_link_libraries(test_crc ${PROJECT_NAME})
  ament_add_gtest(test_fresh_state_tracker test/test_fresh_state_tracker.cpp)
  target_link_libraries(test_fresh_state_tracker ${PROJECT_NAME})
  ament_add_gtest(test_channel_factory_guard test/test_channel_factory_guard.cpp)
  target_link_libraries(test_channel_factory_guard ${PROJECT_NAME})
  ament_add_gtest(test_sdk_index_map test/test_sdk_index_map.cpp)
  target_link_libraries(test_sdk_index_map ${PROJECT_NAME})
  ament_add_gtest(test_control_helpers test/test_control_helpers.cpp)
  target_link_libraries(test_control_helpers ${PROJECT_NAME})
  ament_add_pytest_test(test_install_metadata test/test_install_metadata.py)
  ament_add_pytest_test(test_runtime_source_contracts test/test_runtime_source_contracts.py)
endif()

ament_export_dependencies(
  hardware_interface pluginlib rclcpp rclcpp_lifecycle diagnostic_msgs realtime_tools)
ament_package()
