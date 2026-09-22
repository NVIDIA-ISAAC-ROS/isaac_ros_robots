// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace unitree_hg_ros2_control
{
namespace detail
{

constexpr std::size_t kNumSdkMotors = 35;

struct DiagnosticJointMetadata
{
  std::string name;
  int sdk_index{-1};
};

struct AtomicMotorTemperature
{
  std::atomic<int> surface{0};
  std::atomic<int> winding{0};
};

struct DiagnosticsContext
{
  DiagnosticsContext(
    std::vector<DiagnosticJointMetadata> joint_metadata,
    int warning_threshold,
    rclcpp::Logger diagnostics_logger)
  : joints(std::move(joint_metadata)),
    motor_temp_warn_threshold(warning_threshold),
    logger(std::move(diagnostics_logger))
  {
  }

  const std::vector<DiagnosticJointMetadata> joints;
  const int motor_temp_warn_threshold;
  std::array<AtomicMotorTemperature, kNumSdkMotors> temperatures{};
  rclcpp::Logger logger;
  rclcpp::Clock steady_clock{RCL_STEADY_TIME};
};

}  // namespace detail
}  // namespace unitree_hg_ros2_control
