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

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "realtime_tools/realtime_buffer.hpp"
#include "unitree/idl/hg/MotorCmd_.hpp"
#include "unitree_hg_ros2_control/joint_data.hpp"
#include "unitree_hg_ros2_control/joint_index_tables.hpp"

namespace unitree_hg_ros2_control
{
namespace detail
{

struct JointStateSample
{
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
  int16_t surface_temperature{0};
  int16_t winding_temperature{0};
};

struct BodyStateSnapshot
{
  std::vector<JointStateSample> joints;
  IMUData imu;
  int hardware_mode{5};
};

inline bool copy_snapshot_without_allocation(
  const std::vector<JointStateSample> & source,
  std::vector<JointStateSample> & destination)
{
  if (source.size() != destination.size()) {
    return false;
  }
  std::copy(source.begin(), source.end(), destination.begin());
  return true;
}

inline bool copy_snapshot_without_allocation(
  const BodyStateSnapshot & source, BodyStateSnapshot & destination)
{
  if (!copy_snapshot_without_allocation(source.joints, destination.joints)) {
    return false;
  }
  destination.imu.orientation_w = source.imu.orientation_w;
  destination.imu.orientation_x = source.imu.orientation_x;
  destination.imu.orientation_y = source.imu.orientation_y;
  destination.imu.orientation_z = source.imu.orientation_z;
  destination.imu.angular_velocity_x = source.imu.angular_velocity_x;
  destination.imu.angular_velocity_y = source.imu.angular_velocity_y;
  destination.imu.angular_velocity_z = source.imu.angular_velocity_z;
  destination.imu.linear_acceleration_x = source.imu.linear_acceleration_x;
  destination.imu.linear_acceleration_y = source.imu.linear_acceleration_y;
  destination.imu.linear_acceleration_z = source.imu.linear_acceleration_z;
  destination.hardware_mode = source.hardware_mode;
  return true;
}

template<typename SnapshotT>
class StateHandoff
{
public:
  void publish(const SnapshotT & snapshot)
  {
    buffer_.writeFromNonRT(snapshot);
    has_snapshot_.store(true, std::memory_order_release);
  }

  bool copy_latest_without_allocation(SnapshotT & destination)
  {
    if (!has_snapshot_.load(std::memory_order_acquire)) {
      return false;
    }
    const auto * const snapshot = buffer_.readFromRT();
    return snapshot != nullptr && copy_snapshot_without_allocation(*snapshot, destination);
  }

  void reset()
  {
    has_snapshot_.store(false, std::memory_order_release);
    buffer_.reset();
  }

private:
  realtime_tools::RealtimeBuffer<SnapshotT> buffer_;
  std::atomic<bool> has_snapshot_{false};
};

template<typename ChildCleanupT, typename FactoryReleaseT>
void rollback_channel_initialization(
  ChildCleanupT && child_cleanup, FactoryReleaseT && factory_release)
{
  std::forward<ChildCleanupT>(child_cleanup)();
  std::forward<FactoryReleaseT>(factory_release)();
}

enum class CommandFlavor
{
  kBody,
  kDex3,
};

struct MotorCommand
{
  uint8_t mode{0};
  float q{0.0f};
  float dq{0.0f};
  float tau{0.0f};
  float kp{0.0f};
  float kd{0.0f};
};

constexpr uint8_t pack_dex3_mode(uint8_t motor_id, uint8_t status, bool timeout)
{
  return static_cast<uint8_t>(
    (motor_id & 0x0F) | ((status & 0x07) << 4) | (timeout ? 0x80 : 0x00));
}

inline void initialize_dex3_command_slots(
  std::vector<unitree_hg::msg::dds_::MotorCmd_> & motor_commands)
{
  for (size_t motor_id = 0; motor_id < motor_commands.size(); ++motor_id) {
    auto & motor = motor_commands[motor_id];
    motor.mode() = pack_dex3_mode(static_cast<uint8_t>(motor_id), 1, true);
    motor.q() = 0.0f;
    motor.dq() = 0.0f;
    motor.tau() = 0.0f;
    motor.kp() = 0.0f;
    motor.kd() = 0.0f;
    motor.reserve() = 0;
  }
}

inline bool hand_state_covers_configured_joints(
  size_t motor_state_size, const std::vector<JointData> & joints)
{
  return std::all_of(joints.begin(), joints.end(), [motor_state_size](const JointData & joint) {
             return joint.sdk_index >= 0 && static_cast<size_t>(joint.sdk_index) < motor_state_size;
    });
}

inline void set_command_interface_claim(
  JointData & joint, const std::string & interface_type, bool claimed)
{
  if (interface_type == HW_IF_KP) {
    joint.is_kp_claimed = claimed;
  } else if (interface_type == HW_IF_KD) {
    joint.is_kd_claimed = claimed;
  }
  joint.is_impedance_control_enabled = joint.is_kp_claimed && joint.is_kd_claimed;
}

inline bool command_is_finite(const MotorCommand & command)
{
  return std::isfinite(command.q) && std::isfinite(command.dq) &&
         std::isfinite(command.tau) && std::isfinite(command.kp) &&
         std::isfinite(command.kd);
}

inline bool compose_motor_command(
  const JointData & joint, CommandFlavor flavor, uint8_t motor_id, MotorCommand & command)
{
  const bool dex3 = flavor == CommandFlavor::kDex3;
  const auto active_mode = dex3 ? pack_dex3_mode(motor_id, 1, false) : 1;
  const auto inactive_mode = dex3 ? pack_dex3_mode(motor_id, 1, true) : 0;
  const float position_kp = dex3 ? 0.5f : 10.0f;
  const float position_kd = dex3 ? 0.1f : 1.0f;

  if (joint.is_impedance_control_enabled) {
    command = {active_mode, static_cast<float>(joint.position_command),
      static_cast<float>(joint.velocity_command), static_cast<float>(joint.effort_command),
      static_cast<float>(joint.kp_command), static_cast<float>(joint.kd_command)};
  } else if (joint.is_position_control_enabled && !joint.is_effort_control_enabled) {
    command = {active_mode, static_cast<float>(joint.position_command), 0.0f, 0.0f,
      position_kp, position_kd};
  } else if (joint.is_velocity_control_enabled && !joint.is_effort_control_enabled) {
    command = {active_mode, static_cast<float>(joint.position_state),
      static_cast<float>(joint.velocity_command), 0.0f, 0.0f, 1.0f};
  } else if (joint.is_effort_control_enabled) {
    command = {active_mode, static_cast<float>(joint.position_state),
      static_cast<float>(joint.velocity_command), static_cast<float>(joint.effort_command),
      0.0f, static_cast<float>(joint.kd_command)};
  } else {
    command = {inactive_mode, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  }
  return command_is_finite(command);
}

inline bool hand_joint_names_match_side(
  const std::string & hand_side, const std::vector<std::string> & joint_names)
{
  if (hand_side != "left" && hand_side != "right") {
    return false;
  }
  const std::string prefix = hand_side + "_hand_";
  std::set<std::string> expected;
  for (const auto & entry : dex3_hand_joint_index_table()) {
    if (entry.first.rfind(prefix, 0) == 0) {
      expected.insert(entry.first);
    }
  }
  const std::set<std::string> actual(joint_names.begin(), joint_names.end());
  return actual.size() == joint_names.size() && actual == expected;
}

inline bool joint_names_match_table(
  const std::vector<std::string> & joint_names, const JointIndexTable & table)
{
  std::set<std::string> expected;
  for (const auto & entry : table) {
    expected.insert(entry.first);
  }
  const std::set<std::string> actual(joint_names.begin(), joint_names.end());
  return actual.size() == joint_names.size() && actual == expected;
}

}  // namespace detail
}  // namespace unitree_hg_ros2_control
