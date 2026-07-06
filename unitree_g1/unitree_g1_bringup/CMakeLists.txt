# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.16)
project(unitree_g1_bringup)

# Find dependencies.
find_package(ament_cmake REQUIRED)

# Install launch files, configurations, data, and scripts.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Testing.
if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  function(add_ros_isolated_launch_test path)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
  endfunction()

  function(add_conversion_test)
    set(options)
    set(one_value_args LEAPP_YAML PROTOMOTIONS_YAML)
    set(multi_value_args)
    cmake_parse_arguments(CONV "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})

    if(NOT CONV_LEAPP_YAML OR NOT CONV_PROTOMOTIONS_YAML)
      message(FATAL_ERROR "add_conversion_test requires LEAPP_YAML and PROTOMOTIONS_YAML.")
    endif()

    if(IS_ABSOLUTE "${CONV_LEAPP_YAML}")
      set(leapp_yaml "${CONV_LEAPP_YAML}")
    else()
      set(leapp_yaml "${CMAKE_CURRENT_SOURCE_DIR}/data/${CONV_LEAPP_YAML}")
    endif()

    if(IS_ABSOLUTE "${CONV_PROTOMOTIONS_YAML}")
      set(protomotions_yaml "${CONV_PROTOMOTIONS_YAML}")
    else()
      set(protomotions_yaml "${CMAKE_CURRENT_SOURCE_DIR}/data/${CONV_PROTOMOTIONS_YAML}")
    endif()

    get_filename_component(protomotions_stem "${protomotions_yaml}" NAME_WE)
    get_filename_component(leapp_stem "${leapp_yaml}" NAME_WE)
    set(test_name "test_convert_${protomotions_stem}_to_${leapp_stem}")
    string(MAKE_C_IDENTIFIER "${test_name}" test_name)

    add_test(
      NAME "${test_name}"
      COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/../../../isaac_ros_deploy/isaac_deploy/isaac_deploy_core/scripts/test_convert_yaml_protomotions_to_leapp.py"
        --protomotions-yaml "${protomotions_yaml}"
        --leapp-yaml "${leapp_yaml}"
    )
    set_tests_properties("${test_name}" PROPERTIES TIMEOUT 30)
  endfunction()

  set(PROTOMOTIONS_DATA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../isaac_ros_learned_policies/isaac_ros_protomotions_unitree_g1/data")
  add_conversion_test(
    LEAPP_YAML "${PROTOMOTIONS_DATA_DIR}/unitree_g1_wave_motion_policy_leapp.yaml"
    PROTOMOTIONS_YAML "${PROTOMOTIONS_DATA_DIR}/unitree_g1_wave_motion_policy_protomotions.yaml"
  )

  find_package(mujoco_ros2_control REQUIRED)

  # Agile velocity privileged policy ros2_control integration test.
  add_ros_isolated_launch_test(test/test_agile_velocity_ros2_control.launch.py
    TIMEOUT 60
  )

  # Agile velocity with IK controller integration test.
  add_ros_isolated_launch_test(test/test_agile_velocity_with_ik.launch.py
    TIMEOUT 120
  )

  # Protomotions tests require reference_motion_ros which is not always available.
  find_package(reference_motion_ros QUIET)
  if(reference_motion_ros_FOUND)
    add_ros_isolated_launch_test(test/test_protomotions_wave_motion_ros2_control.launch.py
      TIMEOUT 60
    )
    add_ros_isolated_launch_test(test/test_protomotions_wave_motion_inference_graph.launch.py
      TIMEOUT 120
    )
  endif()

  # Agile velocity privileged inference graph integration test.
  add_ros_isolated_launch_test(test/test_agile_velocity_inference_graph.launch.py
    TIMEOUT 120
  )
endif()

ament_package()
