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

#include "unitree_hg_ros2_control/unitree_hg_system_interface.hpp"

#include <algorithm>
#include <chrono>
#include <string>
#include <thread>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "unitree_hg_ros2_control/joint_index_tables.hpp"

namespace unitree_hg_ros2_control
{

// SDK motor_state/motor_cmd arrays on unitree_hg::msg::dds_::LowState_ /
// LowCmd_ are fixed-size arrays of 35 entries.
constexpr int kMaxSdkMotorIndex = 35;

UnitreeHgSystemInterface::UnitreeHgSystemInterface() = default;

UnitreeHgSystemInterface::~UnitreeHgSystemInterface()
{
  shutdown_sdk();
  release_channel_factory();
}

rclcpp::Logger UnitreeHgSystemInterface::get_logger() const
{
  return logger_;
}

hardware_interface::CallbackReturn
#if ROS_DISTRO_HUMBLE
UnitreeHgSystemInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  const auto & hw_info = info;
#else
UnitreeHgSystemInterface::on_init(
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

  if (hw_info.hardware_parameters.count("motor_temp_warn_threshold")) {
    const auto & value = hw_info.hardware_parameters.at("motor_temp_warn_threshold");
    try {
      motor_temp_warn_threshold_ = std::stoi(value);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        get_logger(), "Invalid 'motor_temp_warn_threshold' hardware parameter '%s': %s",
        value.c_str(), e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  RCLCPP_INFO(
    get_logger(), "Motor temperature warn threshold: %d C", motor_temp_warn_threshold_);

  if (hw_info.hardware_parameters.count("robot_variant")) {
    robot_variant_ = hw_info.hardware_parameters.at("robot_variant");
  }
  if (robot_variant_ != "g1" && robot_variant_ != "h2") {
    RCLCPP_ERROR(
      get_logger(),
      "Invalid or missing 'robot_variant' hardware parameter '%s'; must be 'g1' or 'h2'",
      robot_variant_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_logger(), "Robot variant: %s", robot_variant_.c_str());

  if (!register_joints(hw_info)) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  register_imu_sensor(hw_info);

  bool table_ok = false;
  const auto & table = get_body_joint_index_table(robot_variant_, table_ok);
  if (!table_ok) {
    RCLCPP_ERROR(
      get_logger(), "Unknown 'robot_variant' hardware parameter '%s'", robot_variant_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::vector<std::string> joint_names;
  joint_names.reserve(hw_info.joints.size());
  for (const auto & joint : hw_info.joints) {
    joint_names.push_back(joint.name);
  }
  if (!detail::joint_names_match_table(joint_names, table)) {
    RCLCPP_ERROR(
      get_logger(), "Configured body joint set does not exactly match robot variant '%s'",
      robot_variant_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto map = map_joints_to_sdk_index(hw_info.joints, table);
  if (!map.ok) {
    RCLCPP_ERROR(get_logger(), "%s", map.error.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (size_t i = 0; i < joint_data_.size(); ++i) {
    joint_data_[i].sdk_index = map.sdk_index_by_joint[i];
  }
  read_snapshot_.joints.resize(joint_data_.size());

  std::vector<detail::DiagnosticJointMetadata> diagnostics_joints;
  diagnostics_joints.reserve(joint_data_.size());
  for (const auto & joint : joint_data_) {
    diagnostics_joints.push_back({joint.name, joint.sdk_index});
  }
  diagnostics_context_ = std::make_shared<DiagnosticsContext>(
    std::move(diagnostics_joints), motor_temp_warn_threshold_, get_logger());

  return hardware_interface::CallbackReturn::SUCCESS;
}

bool UnitreeHgSystemInterface::register_joints(const hardware_interface::HardwareInfo & info)
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

void UnitreeHgSystemInterface::register_imu_sensor(const hardware_interface::HardwareInfo & info)
{
  has_imu_ = false;

  for (const auto & sensor : info.sensors) {
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
      break;
    }
  }
}

std::vector<hardware_interface::StateInterface>
UnitreeHgSystemInterface::export_state_interfaces()
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

std::vector<hardware_interface::CommandInterface>
UnitreeHgSystemInterface::export_command_interfaces()
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

bool UnitreeHgSystemInterface::initialize_sdk()
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
      // Release any existing control modes using motion switcher before we
      // start writing LowCmd, otherwise the robot will reject commands.
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

      // The loop can exit on the final attempt right after a successful
      // ReleaseMode() without re-checking, so confirm the mode is really
      // cleared before treating a still-active mode as a failure.
      msc.CheckMode(form, name);
      if (!name.empty()) {
        RCLCPP_ERROR(
          get_logger(), "Failed to release existing modes after %d attempts",
          max_attempts);
        rollback_sdk_initialization();
        return false;
      }
      RCLCPP_INFO(get_logger(), "Existing control modes released");

      fresh_low_state_.reset();
      lowstate_subscriber_ =
        std::make_shared<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>>(
        "rt/lowstate");
      lowstate_subscriber_->InitChannel(
        std::bind(&UnitreeHgSystemInterface::low_state_callback, this, std::placeholders::_1), 1);

      lowcmd_publisher_ =
        std::make_shared<unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>>(
        "rt/lowcmd");
      lowcmd_publisher_->InitChannel();

      RCLCPP_INFO(get_logger(), "Waiting for first state message from robot...");

      auto start_time = std::chrono::steady_clock::now();
      const auto timeout = std::chrono::seconds(10);
      while (!fresh_low_state_.has_fresh_state()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (std::chrono::steady_clock::now() - start_time > timeout) {
          RCLCPP_ERROR(get_logger(), "Timeout waiting for robot state");
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

void UnitreeHgSystemInterface::shutdown_sdk()
{
  if (sdk_initialized_.load()) {
    RCLCPP_INFO(get_logger(), "Shutting down Unitree SDK...");
  }
  lowstate_subscriber_.reset();
  lowcmd_publisher_.reset();
  fresh_low_state_.reset();
  inbound_state_handoff_.reset();
  hardware_mode_ = 5;
  sdk_initialized_ = false;
}

void UnitreeHgSystemInterface::rollback_sdk_initialization()
{
  detail::rollback_channel_initialization(
    [this]() {shutdown_sdk();}, [this]() {release_channel_factory();});
}

bool UnitreeHgSystemInterface::copy_inbound_state()
{
  if (!inbound_state_handoff_.copy_latest_without_allocation(read_snapshot_) ||
    read_snapshot_.joints.size() != joint_data_.size())
  {
    return false;
  }

  for (size_t index = 0; index < joint_data_.size(); ++index) {
    const auto & sample = read_snapshot_.joints[index];
    auto & joint = joint_data_[index];
    joint.position_state = sample.position;
    joint.velocity_state = sample.velocity;
    joint.effort_state = sample.effort;
    joint.surface_temperature = sample.surface_temperature;
    joint.winding_temperature = sample.winding_temperature;
  }
  if (has_imu_) {
    const auto & imu = read_snapshot_.imu;
    imu_data_.orientation_w = imu.orientation_w;
    imu_data_.orientation_x = imu.orientation_x;
    imu_data_.orientation_y = imu.orientation_y;
    imu_data_.orientation_z = imu.orientation_z;
    imu_data_.angular_velocity_x = imu.angular_velocity_x;
    imu_data_.angular_velocity_y = imu.angular_velocity_y;
    imu_data_.angular_velocity_z = imu.angular_velocity_z;
    imu_data_.linear_acceleration_x = imu.linear_acceleration_x;
    imu_data_.linear_acceleration_y = imu.linear_acceleration_y;
    imu_data_.linear_acceleration_z = imu.linear_acceleration_z;
  }
  hardware_mode_ = read_snapshot_.hardware_mode;
  return true;
}

void UnitreeHgSystemInterface::release_channel_factory()
{
  if (channel_factory_held_) {
    ReleaseChannelFactory();
    channel_factory_held_ = false;
  }
}

void UnitreeHgSystemInterface::low_state_callback(const void * message)
{
  const auto & low_state = *static_cast<const unitree_hg::msg::dds_::LowState_ *>(message);
  if (!detail::LowStateCrc::is_valid(low_state)) {
    RCLCPP_WARN(get_logger(), "Ignoring Unitree LowState with invalid CRC");
    return;
  }

  detail::BodyStateSnapshot snapshot;
  snapshot.joints.resize(joint_data_.size());
  for (size_t index = 0; index < joint_data_.size(); ++index) {
    const auto sdk_index = joint_data_[index].sdk_index;
    if (sdk_index >= 0 && sdk_index < kMaxSdkMotorIndex) {
      const auto & motor = low_state.motor_state()[sdk_index];
      snapshot.joints[index] = {
        motor.q(), motor.dq(), motor.tau_est(), motor.temperature()[0], motor.temperature()[1]};
      diagnostics_context_->temperatures[sdk_index].surface.store(
        motor.temperature()[0], std::memory_order_relaxed);
      diagnostics_context_->temperatures[sdk_index].winding.store(
        motor.temperature()[1], std::memory_order_relaxed);
    }
  }

  if (has_imu_) {
    snapshot.imu.name = imu_data_.name;
    snapshot.imu.orientation_w = low_state.imu_state().quaternion()[0];
    snapshot.imu.orientation_x = low_state.imu_state().quaternion()[1];
    snapshot.imu.orientation_y = low_state.imu_state().quaternion()[2];
    snapshot.imu.orientation_z = low_state.imu_state().quaternion()[3];
    snapshot.imu.angular_velocity_x = low_state.imu_state().gyroscope()[0];
    snapshot.imu.angular_velocity_y = low_state.imu_state().gyroscope()[1];
    snapshot.imu.angular_velocity_z = low_state.imu_state().gyroscope()[2];
    snapshot.imu.linear_acceleration_x = low_state.imu_state().accelerometer()[0];
    snapshot.imu.linear_acceleration_y = low_state.imu_state().accelerometer()[1];
    snapshot.imu.linear_acceleration_z = low_state.imu_state().accelerometer()[2];
  }
  snapshot.hardware_mode = low_state.mode_machine();
  inbound_state_handoff_.publish(std::move(snapshot));
  fresh_low_state_.mark_received();
}

hardware_interface::CallbackReturn UnitreeHgSystemInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Activating Unitree Hg hardware interface...");

  if (!initialize_sdk()) {
    shutdown_sdk();
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!copy_inbound_state()) {
    RCLCPP_ERROR(get_logger(), "First complete robot state could not be copied during activation");
    rollback_sdk_initialization();
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize commands to the currently measured state so we never jump
  // the robot by sending stale (or zero) setpoints when a controller
  // becomes active.
  for (auto & jd : joint_data_) {
    jd.position_command = jd.position_state;
    jd.velocity_command = 0.0;
    jd.effort_command = 0.0;
  }

  diagnostics_pub_ = get_node()->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", rclcpp::SystemDefaultsQoS());
  const auto diagnostics_context = diagnostics_context_;
  const auto diagnostics_pub = diagnostics_pub_;
  const auto diagnostics_node = get_node();
  diagnostics_timer_ = diagnostics_node->create_wall_timer(
    std::chrono::milliseconds(500),
    [diagnostics_context, diagnostics_pub, diagnostics_node]() {
      auto msg = diagnostic_msgs::msg::DiagnosticArray{};
      msg.header.stamp = diagnostics_node->now();
      for (const auto & joint : diagnostics_context->joints) {
        if (joint.sdk_index < 0 || joint.sdk_index >= kMaxSdkMotorIndex) {
          continue;
        }
        const auto & temperature = diagnostics_context->temperatures[joint.sdk_index];
        const int surface_temperature = temperature.surface.load(std::memory_order_relaxed);
        const int winding_temperature = temperature.winding.load(std::memory_order_relaxed);
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = joint.name;
        status.hardware_id = "unitree_hg";
        if (winding_temperature >= diagnostics_context->motor_temp_warn_threshold) {
          status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
          RCLCPP_WARN_THROTTLE(
            diagnostics_context->logger, diagnostics_context->steady_clock, 10000,
            "Motor '%s' winding temperature %d C exceeds threshold %d C",
            joint.name.c_str(), winding_temperature,
            diagnostics_context->motor_temp_warn_threshold);
        } else {
          status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        }
        status.message =
        "Motor temperature: surface=" + std::to_string(surface_temperature) +
        "C winding=" + std::to_string(winding_temperature) + "C";
        diagnostic_msgs::msg::KeyValue kv_surface;
        kv_surface.key = "surface_temperature_C";
        kv_surface.value = std::to_string(surface_temperature);
        diagnostic_msgs::msg::KeyValue kv_winding;
        kv_winding.key = "winding_temperature_C";
        kv_winding.value = std::to_string(winding_temperature);
        status.values = {kv_surface, kv_winding};
        msg.status.push_back(status);
      }
      diagnostics_pub->publish(msg);
    });

  RCLCPP_INFO(get_logger(), "Unitree Hg hardware interface activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn UnitreeHgSystemInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating Unitree Hg hardware interface...");
  shutdown_sdk();
  release_channel_factory();
  diagnostics_timer_.reset();
  diagnostics_pub_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn UnitreeHgSystemInterface::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // ros2_control drives the component through on_error() (instead of
  // on_deactivate()) when read()/write() report an error from the active
  // state, e.g. on loss of fresh robot state. Tear down the SDK and release
  // the ChannelFactory refcount here too, otherwise an errored component
  // leaks the process-global DDS participant for the lifetime of the process.
  RCLCPP_ERROR(get_logger(), "Unitree Hg hardware interface entered error state; cleaning up");
  shutdown_sdk();
  release_channel_factory();
  diagnostics_timer_.reset();
  diagnostics_pub_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type UnitreeHgSystemInterface::perform_command_mode_switch(
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

hardware_interface::return_type UnitreeHgSystemInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!sdk_initialized_.load()) {
    return hardware_interface::return_type::ERROR;
  }

  if (!fresh_low_state_.has_fresh_state()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), steady_clock_, 1000,
      "No fresh state received from robot; stopping hardware interface");
    return hardware_interface::return_type::ERROR;
  }
  if (!copy_inbound_state()) {
    RCLCPP_ERROR(get_logger(), "Fresh robot state was incomplete");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type UnitreeHgSystemInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!sdk_initialized_.load()) {
    return hardware_interface::return_type::ERROR;
  }

  if (!fresh_low_state_.has_fresh_state()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), steady_clock_, 1000,
      "No fresh state received from robot; not publishing commands");
    return hardware_interface::return_type::ERROR;
  }

  unitree_hg::msg::dds_::LowCmd_ low_cmd;
  low_cmd.mode_pr() = 0;
  low_cmd.mode_machine() = hardware_mode_;

  for (const auto & joint : joint_data_) {
    if (joint.sdk_index < 0 || joint.sdk_index >= kMaxSdkMotorIndex) {
      continue;
    }
    detail::MotorCommand command;
    if (!detail::compose_motor_command(
          joint, detail::CommandFlavor::kBody, static_cast<uint8_t>(joint.sdk_index), command))
    {
      RCLCPP_ERROR(get_logger(), "Refusing to publish non-finite command for '%s'",
            joint.name.c_str());
      return hardware_interface::return_type::ERROR;
    }
    auto & motor = low_cmd.motor_cmd().at(joint.sdk_index);
    motor.mode() = command.mode;
    motor.q() = command.q;
    motor.dq() = command.dq;
    motor.tau() = command.tau;
    motor.kp() = command.kp;
    motor.kd() = command.kd;
  }

  low_cmd.crc() = detail::calculate_crc32(
    reinterpret_cast<const uint32_t *>(&low_cmd), (sizeof(low_cmd) >> 2) - 1);
  if (!lowcmd_publisher_->Write(low_cmd)) {
    RCLCPP_ERROR(get_logger(), "Failed to write low command to robot");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace unitree_hg_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  unitree_hg_ros2_control::UnitreeHgSystemInterface,
  hardware_interface::SystemInterface)
