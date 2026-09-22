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

#include "unitree_hg_ros2_control/unitree_dex3_system_interface.hpp"

#include <algorithm>
#include <chrono>
#include <string>
#include <thread>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include "unitree_hg_ros2_control/joint_index_tables.hpp"

namespace unitree_hg_ros2_control
{

// Each Dex3 hand exposes 7 motors.
constexpr int kNumHandMotors = 7;

UnitreeDex3SystemInterface::UnitreeDex3SystemInterface()
{
  hand_cmd_.motor_cmd().resize(kNumHandMotors);
}

UnitreeDex3SystemInterface::~UnitreeDex3SystemInterface()
{
  shutdown_sdk();
  release_channel_factory();
}

rclcpp::Logger UnitreeDex3SystemInterface::get_logger() const
{
  return logger_;
}

hardware_interface::CallbackReturn
#if ROS_DISTRO_HUMBLE
UnitreeDex3SystemInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  const auto & hw_info = info;
#else
UnitreeDex3SystemInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  const auto & hw_info = get_hardware_info();
#endif

  if (hw_info.hardware_parameters.count("network_interface")) {
    network_interface_ = hw_info.hardware_parameters.at("network_interface");
  }
  RCLCPP_INFO(get_logger(), "Using network interface: %s", network_interface_.c_str());

  if (hw_info.hardware_parameters.count("hand_side")) {
    hand_side_ = hw_info.hardware_parameters.at("hand_side");
  }
  if (hand_side_ != "left" && hand_side_ != "right") {
    RCLCPP_ERROR(
      get_logger(), "Invalid 'hand_side' hardware parameter '%s'; must be 'left' or 'right'",
      hand_side_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_logger(), "Hand side: %s", hand_side_.c_str());

  std::vector<std::string> joint_names;
  joint_names.reserve(hw_info.joints.size());
  for (const auto & joint : hw_info.joints) {
    joint_names.push_back(joint.name);
  }
  if (!detail::hand_joint_names_match_side(hand_side_, joint_names)) {
    RCLCPP_ERROR(
      get_logger(), "Configured Dex3 joint set does not exactly match hand_side '%s'",
      hand_side_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!register_joints(hw_info)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto map = map_joints_to_sdk_index(hw_info.joints, dex3_hand_joint_index_table());
  if (!map.ok) {
    RCLCPP_ERROR(get_logger(), "%s", map.error.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (size_t i = 0; i < joint_data_.size(); ++i) {
    joint_data_[i].sdk_index = map.sdk_index_by_joint[i];
  }
  read_snapshot_.resize(joint_data_.size());

  return hardware_interface::CallbackReturn::SUCCESS;
}

bool UnitreeDex3SystemInterface::register_joints(const hardware_interface::HardwareInfo & info)
{
  joint_data_.clear();
  joint_name_to_index_.clear();

  for (const auto & joint : info.joints) {
    JointData jd;
    jd.name = joint.name;

    for (const auto & state_iface : joint.state_interfaces) {
      if (state_iface.name == hardware_interface::HW_IF_POSITION) {
        auto it = state_iface.parameters.find("initial_value");
        if (it != state_iface.parameters.end()) {
          try {
            jd.position_state = std::stod(it->second);
          } catch (const std::exception & e) {
            RCLCPP_ERROR(
              get_logger(),
              "Invalid 'initial_value' for joint '%s' position interface '%s': %s",
              joint.name.c_str(), it->second.c_str(), e.what());
            return false;
          }
          jd.position_command = jd.position_state;
        }
      }
    }

    joint_name_to_index_[joint.name] = joint_data_.size();
    joint_data_.push_back(jd);
  }

  return true;
}

std::vector<hardware_interface::StateInterface>
UnitreeDex3SystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (auto & jd : joint_data_) {
    state_interfaces.emplace_back(jd.name, hardware_interface::HW_IF_POSITION, &jd.position_state);
    state_interfaces.emplace_back(jd.name, hardware_interface::HW_IF_VELOCITY, &jd.velocity_state);
    state_interfaces.emplace_back(jd.name, hardware_interface::HW_IF_EFFORT, &jd.effort_state);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
UnitreeDex3SystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (auto & jd : joint_data_) {
    command_interfaces.emplace_back(
      jd.name, hardware_interface::HW_IF_POSITION, &jd.position_command);
    command_interfaces.emplace_back(
      jd.name, hardware_interface::HW_IF_VELOCITY, &jd.velocity_command);
    command_interfaces.emplace_back(
      jd.name, hardware_interface::HW_IF_EFFORT, &jd.effort_command);
    command_interfaces.emplace_back(jd.name, HW_IF_KP, &jd.kp_command);
    command_interfaces.emplace_back(jd.name, HW_IF_KD, &jd.kd_command);
  }

  return command_interfaces;
}

bool UnitreeDex3SystemInterface::initialize_sdk()
{
  try {
    RCLCPP_INFO(
      get_logger(), "Initializing Unitree SDK on interface: %s",
      network_interface_.c_str());

    auto init = EnsureChannelFactoryInitialized(0, network_interface_);
    if (!init.ok) {
      RCLCPP_ERROR(get_logger(), "%s", init.error.c_str());
      return false;
    }
    channel_factory_held_ = true;

    // From this point on, the ChannelFactory refcount has been incremented.
    // ros2_control does not call on_deactivate() after a failed on_activate(),
    // so every failure path below must explicitly release it to avoid leaking
    // the refcount across repeated failed activations. Only the success path
    // (return true) keeps the refcount held; leaving the active state
    // (on_deactivate/on_error) or destruction releases it then.
    try {
      const std::string state_topic = "rt/dex3/" + hand_side_ + "/state";
      const std::string cmd_topic = "rt/dex3/" + hand_side_ + "/cmd";

      fresh_hand_state_.reset();
      handstate_subscriber_ =
        std::make_shared<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::HandState_>>(
        state_topic);
      handstate_subscriber_->InitChannel(
        std::bind(&UnitreeDex3SystemInterface::hand_state_callback, this, std::placeholders::_1),
        1);

      handcmd_publisher_ =
        std::make_shared<unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::HandCmd_>>(
        cmd_topic);
      handcmd_publisher_->InitChannel();

      RCLCPP_INFO(get_logger(), "Waiting for first state message from robot...");

      auto start_time = std::chrono::steady_clock::now();
      const auto timeout = std::chrono::seconds(10);
      while (!fresh_hand_state_.has_fresh_state()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (std::chrono::steady_clock::now() - start_time > timeout) {
          RCLCPP_ERROR(get_logger(), "Timeout waiting for hand state");
          rollback_sdk_initialization();
          return false;
        }
      }
    } catch (...) {
      rollback_sdk_initialization();
      throw;
    }

    RCLCPP_INFO(get_logger(), "Unitree SDK initialized successfully");
    sdk_initialized_ = true;
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize SDK: %s", e.what());
    return false;
  }
}

void UnitreeDex3SystemInterface::shutdown_sdk()
{
  if (sdk_initialized_.load()) {
    RCLCPP_INFO(get_logger(), "Shutting down Unitree SDK...");
  }
  handstate_subscriber_.reset();
  handcmd_publisher_.reset();
  fresh_hand_state_.reset();
  inbound_state_handoff_.reset();
  sdk_initialized_ = false;
}

void UnitreeDex3SystemInterface::rollback_sdk_initialization()
{
  detail::rollback_channel_initialization(
    [this]() {shutdown_sdk();}, [this]() {release_channel_factory();});
}

bool UnitreeDex3SystemInterface::copy_inbound_state()
{
  if (!inbound_state_handoff_.copy_latest_without_allocation(read_snapshot_) ||
    read_snapshot_.size() != joint_data_.size())
  {
    return false;
  }

  for (size_t index = 0; index < joint_data_.size(); ++index) {
    joint_data_[index].position_state = read_snapshot_[index].position;
    joint_data_[index].velocity_state = read_snapshot_[index].velocity;
    joint_data_[index].effort_state = read_snapshot_[index].effort;
  }
  return true;
}

void UnitreeDex3SystemInterface::release_channel_factory()
{
  if (channel_factory_held_) {
    ReleaseChannelFactory();
    channel_factory_held_ = false;
  }
}

void UnitreeDex3SystemInterface::hand_state_callback(const void * message)
{
  // HandState_ has no CRC field, so completeness is the acceptance boundary.
  const auto & hand_state = *static_cast<const unitree_hg::msg::dds_::HandState_ *>(message);
  if (!detail::hand_state_covers_configured_joints(hand_state.motor_state().size(), joint_data_)) {
    RCLCPP_WARN(get_logger(), "Ignoring incomplete Unitree HandState");
    return;
  }

  std::vector<detail::JointStateSample> snapshot(joint_data_.size());
  for (size_t index = 0; index < joint_data_.size(); ++index) {
    const auto & motor = hand_state.motor_state()[joint_data_[index].sdk_index];
    snapshot[index] = {motor.q(), motor.dq(), motor.tau_est(), 0, 0};
  }
  inbound_state_handoff_.publish(std::move(snapshot));
  fresh_hand_state_.mark_received();
}

hardware_interface::CallbackReturn UnitreeDex3SystemInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    get_logger(), "Activating Unitree Dex3 (%s) hardware interface...", hand_side_.c_str());

  if (!initialize_sdk()) {
    shutdown_sdk();
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!copy_inbound_state()) {
    RCLCPP_ERROR(get_logger(), "First complete hand state could not be copied during activation");
    rollback_sdk_initialization();
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize commands to the currently measured state so we never jump
  // the hand by sending stale (or zero) setpoints when a controller becomes
  // active.
  for (auto & jd : joint_data_) {
    jd.position_command = jd.position_state;
    jd.velocity_command = 0.0;
    jd.effort_command = 0.0;
  }

  RCLCPP_INFO(get_logger(), "Unitree Dex3 hardware interface activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn UnitreeDex3SystemInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    get_logger(), "Deactivating Unitree Dex3 (%s) hardware interface...", hand_side_.c_str());
  shutdown_sdk();
  release_channel_factory();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn UnitreeDex3SystemInterface::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // ros2_control drives the component through on_error() (instead of
  // on_deactivate()) when read()/write() report an error from the active
  // state, e.g. on loss of fresh hand state. Tear down the SDK and release
  // the ChannelFactory refcount here too, otherwise an errored component
  // leaks the process-global DDS participant for the lifetime of the process.
  RCLCPP_ERROR(
    get_logger(), "Unitree Dex3 (%s) hardware interface entered error state; cleaning up",
    hand_side_.c_str());
  shutdown_sdk();
  release_channel_factory();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type UnitreeDex3SystemInterface::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  // ros2_control serializes mode switches with its read/write update loop;
  // DDS callbacks only publish snapshots through the realtime handoff.

  auto parse_interface = [](const std::string & interface)
    -> std::pair<std::string, std::string> {
      auto pos = interface.rfind('/');
      if (pos == std::string::npos) {
        return {"", ""};
      }
      return {interface.substr(0, pos), interface.substr(pos + 1)};
    };


  for (const auto & interface : stop_interfaces) {
    auto [joint_name, interface_type] = parse_interface(interface);
    auto it = joint_name_to_index_.find(joint_name);
    if (it == joint_name_to_index_.end()) {
      continue;
    }
    auto & jd = joint_data_[it->second];

    if (interface_type == hardware_interface::HW_IF_POSITION) {
      jd.is_position_control_enabled = false;
    } else if (interface_type == hardware_interface::HW_IF_VELOCITY) {
      jd.is_velocity_control_enabled = false;
    } else if (interface_type == hardware_interface::HW_IF_EFFORT) {
      jd.is_effort_control_enabled = false;
    } else if (interface_type == HW_IF_KP || interface_type == HW_IF_KD) {
      detail::set_command_interface_claim(jd, interface_type, false);
    }
  }

  for (const auto & interface : start_interfaces) {
    auto [joint_name, interface_type] = parse_interface(interface);
    auto it = joint_name_to_index_.find(joint_name);
    if (it == joint_name_to_index_.end()) {
      continue;
    }
    auto & jd = joint_data_[it->second];

    if (interface_type == hardware_interface::HW_IF_POSITION) {
      jd.is_position_control_enabled = true;
      jd.position_command = jd.position_state;
    } else if (interface_type == hardware_interface::HW_IF_VELOCITY) {
      jd.is_velocity_control_enabled = true;
      jd.velocity_command = 0.0;
    } else if (interface_type == hardware_interface::HW_IF_EFFORT) {
      jd.is_effort_control_enabled = true;
      jd.effort_command = 0.0;
    } else if (interface_type == HW_IF_KP || interface_type == HW_IF_KD) {
      detail::set_command_interface_claim(jd, interface_type, true);
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type UnitreeDex3SystemInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!sdk_initialized_.load()) {
    return hardware_interface::return_type::ERROR;
  }

  if (!fresh_hand_state_.has_fresh_state()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), steady_clock_, 1000,
      "No fresh state received from hand; stopping hardware interface");
    return hardware_interface::return_type::ERROR;
  }
  if (!copy_inbound_state()) {
    RCLCPP_ERROR(get_logger(), "Fresh hand state was incomplete");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type UnitreeDex3SystemInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!sdk_initialized_.load()) {
    return hardware_interface::return_type::ERROR;
  }

  if (!fresh_hand_state_.has_fresh_state()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), steady_clock_, 1000,
      "No fresh state received from hand; not publishing commands");
    return hardware_interface::return_type::ERROR;
  }

  detail::initialize_dex3_command_slots(hand_cmd_.motor_cmd());
  for (const auto & joint : joint_data_) {
    if (joint.sdk_index < 0 || joint.sdk_index >= kNumHandMotors) {
      continue;
    }
    detail::MotorCommand command;
    if (!detail::compose_motor_command(
          joint, detail::CommandFlavor::kDex3, static_cast<uint8_t>(joint.sdk_index), command))
    {
      RCLCPP_ERROR(get_logger(), "Refusing to publish non-finite command for '%s'",
            joint.name.c_str());
      return hardware_interface::return_type::ERROR;
    }
    auto & motor = hand_cmd_.motor_cmd().at(joint.sdk_index);
    motor.mode() = command.mode;
    motor.q() = command.q;
    motor.dq() = command.dq;
    motor.tau() = command.tau;
    motor.kp() = command.kp;
    motor.kd() = command.kd;
  }

  // Note: unitree_hg::msg::dds_::HandCmd_ has no crc() field in the SDK
  // (unlike LowCmd_), so it is intentionally not set here. The combined G1
  // hand path never set a CRC on the outgoing HandCmd_ either.
  if (!handcmd_publisher_->Write(hand_cmd_)) {
    RCLCPP_ERROR(get_logger(), "Failed to write hand command to robot");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace unitree_hg_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  unitree_hg_ros2_control::UnitreeDex3SystemInterface,
  hardware_interface::SystemInterface)
