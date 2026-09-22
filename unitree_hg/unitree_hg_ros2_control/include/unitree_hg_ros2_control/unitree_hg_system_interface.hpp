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

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
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
#include "unitree/idl/hg/LowCmd_.hpp"
#include "unitree/idl/hg/LowState_.hpp"
#include "unitree/robot/channel/channel_publisher.hpp"
#include "unitree/robot/channel/channel_subscriber.hpp"

#include "unitree_hg_ros2_control/channel_factory_guard.hpp"
#include "unitree_hg_ros2_control/control_helpers.hpp"
#include "unitree_hg_ros2_control/diagnostics_context.hpp"
#include "unitree_hg_ros2_control/fresh_state_tracker.hpp"
#include "unitree_hg_ros2_control/joint_data.hpp"
#include "unitree_hg_ros2_control/sdk_index_map.hpp"
#include "unitree_hg_ros2_control/unitree_hg_crc.hpp"

#define ROS_DISTRO_HUMBLE (HARDWARE_INTERFACE_VERSION_MAJOR < 3)

namespace unitree_hg_ros2_control
{

/// ros2_control SystemInterface for Unitree unitree_hg humanoid bodies
/// (G1, H2, ...).
///
/// Communicates with the robot over DDS using the Unitree SDK2
/// `unitree_hg::msg::dds_::LowCmd_` / `LowState_` channels (`rt/lowcmd`,
/// `rt/lowstate`). The joint-to-SDK-motor-index mapping is model-driven: the
/// `robot_variant` hardware parameter ("g1" or "h2") selects a joint index
/// table, which is then matched against the URDF's joint names by
/// `map_joints_to_sdk_index()`.
class UnitreeHgSystemInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(UnitreeHgSystemInterface)

  UnitreeHgSystemInterface();
  ~UnitreeHgSystemInterface() override;

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
  hardware_interface::CallbackReturn on_error(
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
  using DiagnosticsContext = detail::DiagnosticsContext;

  bool initialize_sdk();
  void shutdown_sdk();
  void rollback_sdk_initialization();
  bool copy_inbound_state();
  // Releases this instance's ChannelFactory refcount exactly once, if held.
  void release_channel_factory();
  bool register_joints(const hardware_interface::HardwareInfo & info);
  void register_imu_sensor(const hardware_interface::HardwareInfo & info);

  void low_state_callback(const void * message);
  rclcpp::Logger get_logger() const;

  // Configuration.
  std::string network_interface_{"eth0"};
  std::string robot_variant_;
  std::atomic<int> hardware_mode_{5};

  // Joint data.
  std::vector<JointData> joint_data_;
  std::unordered_map<std::string, size_t> joint_name_to_index_;

  // IMU data.
  IMUData imu_data_;
  bool has_imu_{false};

  // SDK state tracking.
  std::atomic<bool> sdk_initialized_{false};
  // True while this instance holds a reference on the process-global
  // ChannelFactory (acquired in on_activate, released when leaving the active
  // state via on_deactivate/on_error or on destruction).
  bool channel_factory_held_{false};
  detail::FreshLowStateTracker fresh_low_state_;
  detail::StateHandoff<detail::BodyStateSnapshot> inbound_state_handoff_;
  detail::BodyStateSnapshot read_snapshot_;

  // Unitree SDK channels.
  unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> lowstate_subscriber_;
  unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_> lowcmd_publisher_;

  // Diagnostics (lifetime-owned on activate, reset on deactivate).
  std::shared_ptr<DiagnosticsContext> diagnostics_context_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_{nullptr};
  rclcpp::TimerBase::SharedPtr diagnostics_timer_{nullptr};
  int motor_temp_warn_threshold_{120};

  rclcpp::Logger logger_{rclcpp::get_logger("UnitreeHgSystemInterface")};

  // Persistent clock reused by all throttled log calls to avoid allocating a
  // fresh clock every control cycle.
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
};

}  // namespace unitree_hg_ros2_control
