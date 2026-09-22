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

#include <string>

namespace unitree_hg_ros2_control
{

// Names of the impedance-gain command interfaces exported by the Unitree hg
// hardware interfaces. Defined once here (a header both interface headers
// include) so the two plugins can share them without a duplicate definition.
constexpr char HW_IF_KP[] = "kp";
constexpr char HW_IF_KD[] = "kd";

struct JointData
{
  std::string name;

  double position_state = 0.0;
  double velocity_state = 0.0;
  double effort_state = 0.0;

  double position_command = 0.0;
  double velocity_command = 0.0;
  double effort_command = 0.0;
  double kp_command = 0.0;
  double kd_command = 0.0;

  int16_t surface_temperature = 0;
  int16_t winding_temperature = 0;

  bool is_position_control_enabled = false;
  bool is_velocity_control_enabled = false;
  bool is_effort_control_enabled = false;
  bool is_impedance_control_enabled = false;
  bool is_kp_claimed = false;
  bool is_kd_claimed = false;

  int sdk_index = -1;
};

struct IMUData
{
  std::string name;

  double orientation_w = 1.0;
  double orientation_x = 0.0;
  double orientation_y = 0.0;
  double orientation_z = 0.0;

  double angular_velocity_x = 0.0;
  double angular_velocity_y = 0.0;
  double angular_velocity_z = 0.0;

  double linear_acceleration_x = 0.0;
  double linear_acceleration_y = 0.0;
  double linear_acceleration_z = 0.0;
};

}  // namespace unitree_hg_ros2_control
