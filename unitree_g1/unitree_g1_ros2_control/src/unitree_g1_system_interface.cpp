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

#include "unitree_g1_ros2_control/unitree_g1_system_interface.hpp"

#include <algorithm>
#include <chrono>
#include <string>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

namespace unitree_g1_ros2_control
{

UnitreeG1SystemInterface::UnitreeG1SystemInterface() = default;

UnitreeG1SystemInterface::~UnitreeG1SystemInterface()
{
  shutdown_sdk();
}

rclcpp::Logger UnitreeG1SystemInterface::get_logger() const
{
  return logger_;
}

uint32_t UnitreeG1SystemInterface::crc32_core(uint32_t * ptr, uint32_t len)
{
  uint32_t xbit = 0;
  uint32_t data = 0;
  uint32_t CRC32 = 0xFFFFFFFF;
  const uint32_t dwPolynomial = 0x04c11db7;
  for (uint32_t i = 0; i < len; i++) {
    xbit = 1 << 31;
    data = ptr[i];
    for (uint32_t bits = 0; bits < 32; bits++) {
      if (CRC32 & 0x80000000) {
        CRC32 <<= 1;
        CRC32 ^= dwPolynomial;
      } else {
        CRC32 <<= 1;
      }
      if (data & xbit) {
        CRC32 ^= dwPolynomial;
      }
      xbit >>= 1;
    }
  }
  return CRC32;
}

hardware_interface::CallbackReturn
#if ROS_DISTRO_HUMBLE
UnitreeG1SystemInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  const auto & hw_info = info;
#else
UnitreeG1SystemInterface::on_init(
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

  if (hw_info.hardware_parameters.count("with_hands")) {
    const auto & val = hw_info.hardware_parameters.at("with_hands");
    with_hands_ = (val == "true" || val == "True" || val == "1");
  }
  RCLCPP_INFO(get_logger(), "With hands: %s", with_hands_ ? "true" : "false");

  if (hw_info.hardware_parameters.count("motor_temp_warn_threshold")) {
    motor_temp_warn_threshold_ =
      std::stoi(hw_info.hardware_parameters.at("motor_temp_warn_threshold"));
  }
  RCLCPP_INFO(
    get_logger(), "Motor temperature warn threshold: %d C", motor_temp_warn_threshold_);

  num_motors_ = with_hands_ ? kNumTotalMotors : kNumBodyMotors;

  register_joints(hw_info);
  register_imu_sensor(hw_info);
  build_joint_sdk_mapping();

  RCLCPP_INFO(
    get_logger(), "Unitree G1 SystemInterface initialized with %zu joints",
    joint_data_.size());

  return hardware_interface::CallbackReturn::SUCCESS;
}

void UnitreeG1SystemInterface::register_joints(const hardware_interface::HardwareInfo & info)
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
          jd.position_state = std::stod(it->second);
          jd.position_command = jd.position_state;
        }
      }
    }

    joint_name_to_index_[joint.name] = joint_data_.size();
    joint_data_.push_back(jd);
  }
}

void UnitreeG1SystemInterface::register_imu_sensor(const hardware_interface::HardwareInfo & info)
{
  has_imu_ = false;

  for (const auto & sensor : info.sensors) {
    // Check if this is an IMU sensor by looking for orientation interfaces
    bool has_orientation = false;
    for (const auto & iface : sensor.state_interfaces) {
      if (iface.name.find("orientation") != std::string::npos) {
        has_orientation = true;
        break;
      }
    }

    if (has_orientation) {
      imu_data_.name = sensor.name;
      has_imu_ = true;
      RCLCPP_INFO(get_logger(), "Registered IMU sensor: %s", sensor.name.c_str());
      break;  // Only support one IMU for now
    }
  }
}

void UnitreeG1SystemInterface::build_joint_sdk_mapping()
{
  auto find_index_in_array = [](const std::string & name, const auto & array,
    size_t offset) -> int {
      for (size_t j = 0; j < array.size(); ++j) {
        if (name == array[j]) {
          return static_cast<int>(offset + j);
        }
      }
      return -1;
    };

  for (auto & jd : joint_data_) {
    jd.sdk_index = find_index_in_array(jd.name, kG1JointNames, 0);

    if (jd.sdk_index < 0 && with_hands_) {
      jd.sdk_index = find_index_in_array(jd.name, kDex3LeftJointNames, kNumBodyMotors);
    }

    if (jd.sdk_index < 0 && with_hands_) {
      jd.sdk_index = find_index_in_array(
        jd.name, kDex3RightJointNames, kNumBodyMotors + kNumLeftHandMotors);
    }

    if (jd.sdk_index < 0) {
      RCLCPP_WARN(get_logger(), "Joint '%s' not found in SDK mapping", jd.name.c_str());
    } else {
      RCLCPP_DEBUG(
        get_logger(), "Joint '%s' mapped to SDK index %d", jd.name.c_str(),
        jd.sdk_index);
    }
  }
}

std::vector<hardware_interface::StateInterface> UnitreeG1SystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (auto & jd : joint_data_) {
    state_interfaces.emplace_back(jd.name, hardware_interface::HW_IF_POSITION, &jd.position_state);
    state_interfaces.emplace_back(jd.name, hardware_interface::HW_IF_VELOCITY, &jd.velocity_state);
    state_interfaces.emplace_back(jd.name, hardware_interface::HW_IF_EFFORT, &jd.effort_state);
  }

  if (has_imu_) {
    state_interfaces.emplace_back(imu_data_.name, "orientation.x", &imu_data_.orientation_x);
    state_interfaces.emplace_back(imu_data_.name, "orientation.y", &imu_data_.orientation_y);
    state_interfaces.emplace_back(imu_data_.name, "orientation.z", &imu_data_.orientation_z);
    state_interfaces.emplace_back(imu_data_.name, "orientation.w", &imu_data_.orientation_w);
    state_interfaces.emplace_back(
      imu_data_.name, "angular_velocity.x", &imu_data_.angular_velocity_x);
    state_interfaces.emplace_back(
      imu_data_.name, "angular_velocity.y", &imu_data_.angular_velocity_y);
    state_interfaces.emplace_back(
      imu_data_.name, "angular_velocity.z", &imu_data_.angular_velocity_z);
    state_interfaces.emplace_back(
      imu_data_.name, "linear_acceleration.x", &imu_data_.linear_acceleration_x);
    state_interfaces.emplace_back(
      imu_data_.name, "linear_acceleration.y", &imu_data_.linear_acceleration_y);
    state_interfaces.emplace_back(
      imu_data_.name, "linear_acceleration.z", &imu_data_.linear_acceleration_z);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> UnitreeG1SystemInterface::
export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (auto & jd : joint_data_) {
    command_interfaces.emplace_back(
      jd.name, hardware_interface::HW_IF_POSITION,
      &jd.position_command);
    command_interfaces.emplace_back(
      jd.name, hardware_interface::HW_IF_VELOCITY,
      &jd.velocity_command);
    command_interfaces.emplace_back(jd.name, hardware_interface::HW_IF_EFFORT, &jd.effort_command);
    command_interfaces.emplace_back(jd.name, HW_IF_KP, &jd.kp_command);
    command_interfaces.emplace_back(jd.name, HW_IF_KD, &jd.kd_command);
  }

  return command_interfaces;
}

bool UnitreeG1SystemInterface::initialize_sdk()
{
  try {
    RCLCPP_INFO(
      get_logger(), "Initializing Unitree SDK on interface: %s",
      network_interface_.c_str());

    // Initialize channel factory
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface_);

    // Release any existing control modes using motion switcher
    RCLCPP_INFO(get_logger(), "Releasing existing control modes...");
    unitree::robot::b2::MotionSwitcherClient msc;
    msc.SetTimeout(5.0f);
    msc.Init();

    std::string form, name;
    int attempts = 0;
    const int max_attempts = 5;

    while (attempts < max_attempts) {
      msc.CheckMode(form, name);
      if (name.empty()) {
        break;
      }
      RCLCPP_INFO(get_logger(), "Current mode: %s, releasing...", name.c_str());
      if (msc.ReleaseMode()) {
        RCLCPP_WARN(get_logger(), "Failed to release mode, retrying...");
      }
      std::this_thread::sleep_for(std::chrono::seconds(2));
      attempts++;
    }

    if (attempts >= max_attempts) {
      RCLCPP_ERROR(
        get_logger(), "Failed to release existing modes after %d attempts",
        max_attempts);
      return false;
    }

    // Create subscriber for low state
    lowstate_subscriber_ =
      std::make_shared<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>>(
      "rt/lowstate");
    lowstate_subscriber_->InitChannel(
      std::bind(&UnitreeG1SystemInterface::low_state_callback, this, std::placeholders::_1), 1);

    // Create publisher for low command
    lowcmd_publisher_ =
      std::make_shared<unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>>(
      "rt/lowcmd");
    lowcmd_publisher_->InitChannel();

    if (with_hands_) {
      handstate_left_subscriber_ =
        std::make_shared<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::HandState_>>(
        "rt/dex3/left/state");
      handstate_left_subscriber_->InitChannel(
        std::bind(&UnitreeG1SystemInterface::hand_state_callback_l, this, std::placeholders::_1),
        1);

      handstate_right_subscriber_ =
        std::make_shared<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::HandState_>>(
        "rt/dex3/right/state");
      handstate_right_subscriber_->InitChannel(
        std::bind(&UnitreeG1SystemInterface::hand_state_callback_r, this, std::placeholders::_1),
        1);

      handcmd_left_publisher_ =
        std::make_shared<unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::HandCmd_>>(
        "rt/dex3/left/cmd");
      handcmd_left_publisher_->InitChannel();

      handcmd_right_publisher_ =
        std::make_shared<unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::HandCmd_>>(
        "rt/dex3/right/cmd");
      handcmd_right_publisher_->InitChannel();

      RCLCPP_INFO(get_logger(), "Hand DDS channels initialized");
    }

    RCLCPP_INFO(get_logger(), "Waiting for first state message from robot...");

    // Wait for first state message
    auto start_time = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::seconds(10);

    while (!first_state_received_.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      if (std::chrono::steady_clock::now() - start_time > timeout) {
        RCLCPP_ERROR(get_logger(), "Timeout waiting for robot state");
        return false;
      }
    }

    RCLCPP_INFO(get_logger(), "Unitree SDK initialized successfully");
    sdk_initialized_ = true;
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize SDK: %s", e.what());
    return false;
  }
}

void UnitreeG1SystemInterface::shutdown_sdk()
{
  if (sdk_initialized_.load()) {
    RCLCPP_INFO(get_logger(), "Shutting down Unitree SDK...");
    lowstate_subscriber_.reset();
    lowcmd_publisher_.reset();
    handstate_left_subscriber_.reset();
    handstate_right_subscriber_.reset();
    handcmd_left_publisher_.reset();
    handcmd_right_publisher_.reset();
    sdk_initialized_ = false;
  }
}

void UnitreeG1SystemInterface::low_state_callback(const void * message)
{
  std::unique_lock<std::shared_mutex> lock(state_mutex_);

  const auto & low_state = *static_cast<const unitree_hg::msg::dds_::LowState_ *>(message);

  // Update joint states from SDK motor order
  for (auto & jd : joint_data_) {
    if (jd.sdk_index >= 0 && jd.sdk_index < static_cast<int>(kNumBodyMotors)) {
      jd.position_state = low_state.motor_state()[jd.sdk_index].q();
      jd.velocity_state = low_state.motor_state()[jd.sdk_index].dq();
      jd.effort_state = low_state.motor_state()[jd.sdk_index].tau_est();
      jd.surface_temperature = low_state.motor_state()[jd.sdk_index].temperature()[0];
      jd.winding_temperature = low_state.motor_state()[jd.sdk_index].temperature()[1];
    }
  }

  // Update IMU data
  if (has_imu_) {
    // Unitree SDK quaternion order is [w, x, y, z]
    imu_data_.orientation_w = low_state.imu_state().quaternion()[0];
    imu_data_.orientation_x = low_state.imu_state().quaternion()[1];
    imu_data_.orientation_y = low_state.imu_state().quaternion()[2];
    imu_data_.orientation_z = low_state.imu_state().quaternion()[3];

    imu_data_.angular_velocity_x = low_state.imu_state().gyroscope()[0];
    imu_data_.angular_velocity_y = low_state.imu_state().gyroscope()[1];
    imu_data_.angular_velocity_z = low_state.imu_state().gyroscope()[2];

    imu_data_.linear_acceleration_x = low_state.imu_state().accelerometer()[0];
    imu_data_.linear_acceleration_y = low_state.imu_state().accelerometer()[1];
    imu_data_.linear_acceleration_z = low_state.imu_state().accelerometer()[2];
  }

  // Overwrite hardware_mode
  hardware_mode_ = low_state.mode_machine();
  first_state_received_ = true;
}

void UnitreeG1SystemInterface::hand_state_callback_l(const void * message)
{
  const auto & hand_state = *static_cast<const unitree_hg::msg::dds_::HandState_ *>(message);

  std::unique_lock<std::shared_mutex> lock(hand_left_mutex_);
  for (auto & jd : joint_data_) {
    if (jd.sdk_index >= static_cast<int>(kNumBodyMotors) &&
      jd.sdk_index < static_cast<int>(kNumBodyMotors + kNumLeftHandMotors))
    {
      int hand_idx = jd.sdk_index - static_cast<int>(kNumBodyMotors);
      jd.position_state = hand_state.motor_state()[hand_idx].q();
      jd.velocity_state = hand_state.motor_state()[hand_idx].dq();
      jd.effort_state = hand_state.motor_state()[hand_idx].tau_est();
    }
  }
}

void UnitreeG1SystemInterface::hand_state_callback_r(const void * message)
{
  const auto & hand_state = *static_cast<const unitree_hg::msg::dds_::HandState_ *>(message);

  std::unique_lock<std::shared_mutex> lock(hand_right_mutex_);
  for (auto & jd : joint_data_) {
    if (jd.sdk_index >= static_cast<int>(kNumBodyMotors + kNumLeftHandMotors) &&
      jd.sdk_index < static_cast<int>(kNumTotalMotors))
    {
      int hand_idx = jd.sdk_index - static_cast<int>(kNumBodyMotors + kNumLeftHandMotors);
      jd.position_state = hand_state.motor_state()[hand_idx].q();
      jd.velocity_state = hand_state.motor_state()[hand_idx].dq();
      jd.effort_state = hand_state.motor_state()[hand_idx].tau_est();
    }
  }
}

hardware_interface::CallbackReturn UnitreeG1SystemInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Activating Unitree G1 hardware interface...");

  if (!initialize_sdk()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize commands to current state
  {
    std::shared_lock<std::shared_mutex> lock(state_mutex_);
    for (auto & jd : joint_data_) {
      jd.position_command = jd.position_state;
      jd.velocity_command = 0.0;
      jd.effort_command = 0.0;
    }
  }

  // Create diagnostics publisher and 2 Hz wall timer (outside the RT control loop)
  diagnostics_pub_ = get_node()->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", rclcpp::SystemDefaultsQoS());
  diagnostics_timer_ = get_node()->create_wall_timer(
    std::chrono::milliseconds(500),
    [this]() {
      auto msg = diagnostic_msgs::msg::DiagnosticArray{};
      msg.header.stamp = get_node()->now();
      {
        std::shared_lock<std::shared_mutex> lock(state_mutex_);
        for (const auto & jd : joint_data_) {
          if (jd.sdk_index < 0 || jd.sdk_index >= static_cast<int>(kNumBodyMotors)) {
            continue;
          }
          diagnostic_msgs::msg::DiagnosticStatus status;
          status.name = jd.name;
          status.hardware_id = "unitree_g1";
          if (static_cast<int>(jd.winding_temperature) >= motor_temp_warn_threshold_) {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            RCLCPP_WARN(
              get_logger(), "Motor '%s' winding temperature %d C exceeds threshold %d C",
              jd.name.c_str(), static_cast<int>(jd.winding_temperature),
              motor_temp_warn_threshold_);
          } else {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
          }
          status.message = "Motor temperature: surface=" + std::to_string(jd.surface_temperature) +
          "C winding=" + std::to_string(jd.winding_temperature) + "C";
          diagnostic_msgs::msg::KeyValue kv_surface;
          kv_surface.key = "surface_temperature_C";
          kv_surface.value = std::to_string(jd.surface_temperature);
          diagnostic_msgs::msg::KeyValue kv_winding;
          kv_winding.key = "winding_temperature_C";
          kv_winding.value = std::to_string(jd.winding_temperature);
          status.values = {kv_surface, kv_winding};
          msg.status.push_back(status);
        }
      }
      diagnostics_pub_->publish(msg);
    });

  RCLCPP_INFO(get_logger(), "Unitree G1 hardware interface activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn UnitreeG1SystemInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating Unitree G1 hardware interface...");
  shutdown_sdk();
  diagnostics_timer_.reset();
  diagnostics_pub_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type UnitreeG1SystemInterface::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  auto parse_interface = [](const std::string & interface)
    -> std::pair<std::string, std::string> {
      auto pos = interface.rfind('/');
      if (pos == std::string::npos) {
        return {"", ""};
      }
      return {interface.substr(0, pos), interface.substr(pos + 1)};
    };

  auto contains_interface = [](const std::vector<std::string> & interfaces,
    const std::string & joint_name, const char * type) {
      return std::find(
        interfaces.begin(), interfaces.end(),
        joint_name + "/" + type) != interfaces.end();
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
      if (contains_interface(stop_interfaces, joint_name, HW_IF_KP) &&
        contains_interface(stop_interfaces, joint_name, HW_IF_KD))
      {
        jd.is_impedance_control_enabled = false;
      }
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
      if (contains_interface(start_interfaces, joint_name, HW_IF_KP) &&
        contains_interface(start_interfaces, joint_name, HW_IF_KD))
      {
        jd.is_impedance_control_enabled = true;
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type UnitreeG1SystemInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!sdk_initialized_.load()) {
    return hardware_interface::return_type::ERROR;
  }

  if (!first_state_received_.load()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *rclcpp::Clock::make_shared(), 1000,
      "No state received from robot yet");
    return hardware_interface::return_type::OK;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type UnitreeG1SystemInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!sdk_initialized_.load() || !first_state_received_.load()) {
    return hardware_interface::return_type::OK;
  }

  unitree_hg::msg::dds_::LowCmd_ low_cmd;
  low_cmd.mode_pr() = 0;
  low_cmd.mode_machine() = hardware_mode_;

  unitree_hg::msg::dds_::HandCmd_ hand_cmd_left;
  unitree_hg::msg::dds_::HandCmd_ hand_cmd_right;
  if (with_hands_) {
    hand_cmd_left.motor_cmd().resize(kNumLeftHandMotors);
    hand_cmd_right.motor_cmd().resize(kNumRightHandMotors);
  }

  constexpr int kMotorsEnabled = 1;
  constexpr int kMotorsDisabled = 0;

  auto fill_motor_cmd = [&](auto & motor_cmd, const JointData & jd) {
      if (jd.is_impedance_control_enabled) {
        motor_cmd.mode() = kMotorsEnabled;
        motor_cmd.q() = static_cast<float>(jd.position_command);
        motor_cmd.dq() = static_cast<float>(jd.velocity_command);
        motor_cmd.tau() = static_cast<float>(jd.effort_command);
        motor_cmd.kp() = static_cast<float>(jd.kp_command);
        motor_cmd.kd() = static_cast<float>(jd.kd_command);
      } else if (jd.is_position_control_enabled && !jd.is_effort_control_enabled) {
        motor_cmd.mode() = kMotorsEnabled;
        motor_cmd.q() = static_cast<float>(jd.position_command);
        motor_cmd.dq() = 0.0f;
        motor_cmd.tau() = 0.0f;
        motor_cmd.kp() = 10.0f;
        motor_cmd.kd() = 1.0f;
      } else if (jd.is_effort_control_enabled) {
        motor_cmd.mode() = kMotorsEnabled;
        motor_cmd.q() = static_cast<float>(jd.position_state);
        motor_cmd.dq() = static_cast<float>(jd.velocity_command);
        motor_cmd.tau() = static_cast<float>(jd.effort_command);
        motor_cmd.kp() = 0.0f;
        motor_cmd.kd() = static_cast<float>(jd.kd_command);
      } else {
        motor_cmd.mode() = kMotorsDisabled;
        motor_cmd.q() = 0.0f;
        motor_cmd.dq() = 0.0f;
        motor_cmd.tau() = 0.0f;
        motor_cmd.kp() = 0.0f;
        motor_cmd.kd() = 0.0f;
      }
    };

  {
    std::shared_lock<std::shared_mutex> lock(state_mutex_);

    for (const auto & jd : joint_data_) {
      if (jd.sdk_index < 0) {
        continue;
      }

      const int left_hand_end = static_cast<int>(kNumBodyMotors + kNumLeftHandMotors);
      if (jd.sdk_index < static_cast<int>(kNumBodyMotors)) {
        fill_motor_cmd(low_cmd.motor_cmd().at(jd.sdk_index), jd);
      } else if (with_hands_ && jd.sdk_index < left_hand_end) {
        int hand_idx = jd.sdk_index - static_cast<int>(kNumBodyMotors);
        fill_motor_cmd(hand_cmd_left.motor_cmd().at(hand_idx), jd);
      } else if (with_hands_ && jd.sdk_index < static_cast<int>(kNumTotalMotors)) {
        int hand_idx = jd.sdk_index - left_hand_end;
        fill_motor_cmd(hand_cmd_right.motor_cmd().at(hand_idx), jd);
      }
    }
  }

  low_cmd.crc() = crc32_core(reinterpret_cast<uint32_t *>(&low_cmd), (sizeof(low_cmd) >> 2) - 1);
  if (!lowcmd_publisher_->Write(low_cmd)) {
    RCLCPP_ERROR(get_logger(), "Failed to write low command to robot");
    return hardware_interface::return_type::ERROR;
  }

  if (with_hands_) {
    if (!handcmd_left_publisher_->Write(hand_cmd_left)) {
      RCLCPP_ERROR(get_logger(), "Failed to write left hand command to robot");
      return hardware_interface::return_type::ERROR;
    }
    if (!handcmd_right_publisher_->Write(hand_cmd_right)) {
      RCLCPP_ERROR(get_logger(), "Failed to write right hand command to robot");
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace unitree_g1_ros2_control

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  unitree_g1_ros2_control::UnitreeG1SystemInterface,
  hardware_interface::SystemInterface)
