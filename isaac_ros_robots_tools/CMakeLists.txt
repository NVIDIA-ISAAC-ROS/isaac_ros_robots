# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.16)
project(isaac_ros_robots_tools)

find_package(ament_cmake REQUIRED)

install(DIRECTORY scripts/ DESTINATION share/${PROJECT_NAME}/scripts)
install(PROGRAMS scripts/setup_network.py DESTINATION lib/${PROJECT_NAME})

ament_package()
