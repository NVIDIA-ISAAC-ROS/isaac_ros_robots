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
  find_package(ament_cmake_ros REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  function(add_ros_isolated_launch_test path)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
  endfunction()

  find_package(mujoco_ros2_control REQUIRED)

  # Agile velocity privileged policy ros2_control integration test.
  add_ros_isolated_launch_test(test/test_agile_velocity_ros2_control.launch.py
    TIMEOUT 60
  )

  # Agile velocity with IK controller integration test.
  add_ros_isolated_launch_test(test/test_agile_velocity_with_ik.launch.py
    TIMEOUT 120
  )

  # Agile velocity privileged inference graph integration test.
  add_ros_isolated_launch_test(test/test_agile_velocity_inference_graph.launch.py
    TIMEOUT 120
  )

  ament_add_pytest_test(test_unitree_g1_controller_manager_launch
    test/test_unitree_g1_controller_manager_launch.py
  )
endif()

ament_package()
