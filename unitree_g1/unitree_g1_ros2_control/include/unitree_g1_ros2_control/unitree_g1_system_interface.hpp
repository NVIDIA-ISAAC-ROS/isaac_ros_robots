// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#include <memory>
#include <shared_mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/version.h"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "unitree/idl/hg/HandCmd_.hpp"
#include "unitree/idl/hg/HandState_.hpp"
#include "unitree/idl/hg/LowCmd_.hpp"
#include "unitree/idl/hg/LowState_.hpp"
#include "unitree/robot/channel/channel_factory.hpp"
#include "unitree/robot/channel/channel_publisher.hpp"
#include "unitree/robot/channel/channel_subscriber.hpp"

#define ROS_DISTRO_HUMBLE (HARDWARE_INTERFACE_VERSION_MAJOR < 3)

namespace unitree_g1_ros2_control
{

constexpr size_t kNumBodyMotors = 29;
constexpr size_t kNumLeftHandMotors = 7;
constexpr size_t kNumRightHandMotors = 7;
constexpr size_t kNumTotalMotors = kNumBodyMotors + kNumLeftHandMotors + kNumRightHandMotors;

constexpr char HW_IF_KP[] = "kp";
constexpr char HW_IF_KD[] = "kd";

// Joint names in SDK motor index order
static const std::array<std::string, kNumBodyMotors> kG1JointNames = {
  "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
  "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
  "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
  "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
  "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
  "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
  "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint",
  "left_wrist_yaw_joint", "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
  "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
  "right_wrist_pitch_joint", "right_wrist_yaw_joint"
};

// Dex3 hand joint names
static const std::array<std::string, kNumLeftHandMotors> kDex3LeftJointNames = {
  "left_hand_thumb_0_joint", "left_hand_thumb_1_joint", "left_hand_thumb_2_joint",
  "left_hand_middle_0_joint", "left_hand_middle_1_joint", "left_hand_index_0_joint",
  "left_hand_index_1_joint"
};

static const std::array<std::string, kNumRightHandMotors> kDex3RightJointNames = {
  "right_hand_thumb_0_joint", "right_hand_thumb_1_joint", "right_hand_thumb_2_joint",
  "right_hand_middle_0_joint", "right_hand_middle_1_joint", "right_hand_index_0_joint",
  "right_hand_index_1_joint"
};

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

/**
 * @brief ROS2 Control Hardware Interface for Unitree G1 humanoid robot
 *
 * This SystemInterface implementation communicates with the Unitree G1 robot
 * using the Unitree SDK via DDS. It exposes joint state/command interfaces
 * compatible with the existing MuJoCo simulation interface.
 */
class UnitreeG1SystemInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(UnitreeG1SystemInterface)

  UnitreeG1SystemInterface();
  ~UnitreeG1SystemInterface() override;

  hardware_interface::CallbackReturn
#if ROS_DISTRO_HUMBLE
  on_init(const hardware_interface::HardwareInfo & info) override;
#else
  on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;
#endif

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

#if ROS_DISTRO_HUMBLE
  const hardware_interface::HardwareInfo & get_hardware_info() const
  {
    return info_;
  }
#endif

private:
  bool initialize_sdk();
  void shutdown_sdk();
  void register_joints(const hardware_interface::HardwareInfo & info);
  void register_imu_sensor(const hardware_interface::HardwareInfo & info);
  void build_joint_sdk_mapping();
  void low_state_callback(const void * message);
  void hand_state_callback_l(const void * message);
  void hand_state_callback_r(const void * message);
  static uint32_t crc32_core(uint32_t * ptr, uint32_t len);
  rclcpp::Logger get_logger() const;

  // Configuration
  std::string network_interface_{"eth0"};
  std::atomic<int> hardware_mode_{5};
  bool with_hands_{true};
  size_t num_motors_{kNumBodyMotors};

  // Joint data
  std::vector<JointData> joint_data_;
  std::unordered_map<std::string, size_t> joint_name_to_index_;

  // IMU data
  IMUData imu_data_;
  bool has_imu_{false};

  // Thread safety for SDK callbacks
  mutable std::shared_mutex state_mutex_;
  mutable std::shared_mutex hand_left_mutex_;
  mutable std::shared_mutex hand_right_mutex_;

  // SDK state tracking
  std::atomic<bool> sdk_initialized_{false};
  std::atomic<bool> first_state_received_{false};

  // Unitree SDK communication
  unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> lowstate_subscriber_;
  unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_> lowcmd_publisher_;
  unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::HandState_>
  handstate_left_subscriber_;
  unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::HandState_>
  handstate_right_subscriber_;
  unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::HandCmd_> handcmd_left_publisher_;
  unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::HandCmd_> handcmd_right_publisher_;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_{nullptr};
  rclcpp::TimerBase::SharedPtr diagnostics_timer_{nullptr};
  int motor_temp_warn_threshold_{120};

  // Logger
  rclcpp::Logger logger_{rclcpp::get_logger("UnitreeG1SystemInterface")};
};

}  // namespace unitree_g1_ros2_control
