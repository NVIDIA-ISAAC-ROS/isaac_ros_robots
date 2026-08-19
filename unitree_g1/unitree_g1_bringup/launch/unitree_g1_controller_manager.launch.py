#!/usr/bin/env python3

# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Unified launch file for Unitree G1 robot - supports both MuJoCo and real hardware."""

from pathlib import Path
import tempfile
from typing import Any

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from controller_manager_msgs.srv import SwitchController
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from rcl_interfaces.msg import Parameter as RclParameter
from rcl_interfaces.msg import ParameterValue as RclParameterValue
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.context import Context as RclpyContext
from rclpy.duration import Duration as RclpyDuration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node as RclpyNode
import yaml


SAFETY_CONTROLLER_TYPE = "isaac_ros_deploy_ros2_control/SafetyController"


def _load_controller_groups() -> dict[str, Any]:
    """Load controller group configurations from controller_groups.yaml."""
    bringup_share = Path(get_package_share_directory("unitree_g1_bringup"))
    return yaml.safe_load((bringup_share / "config/controller_groups.yaml").read_text())


def _load_controller_manager_config() -> dict[str, Any]:
    """Load controller manager configuration from controller_manager.yaml."""
    bringup_share = Path(get_package_share_directory("unitree_g1_bringup"))
    return yaml.safe_load((bringup_share / "config/controller_manager.yaml").read_text())


def _controller_names_by_type(config: dict[str, Any], controller_type: str) -> set[str]:
    """Return controller names declared with the requested plugin type."""
    controller_params = config["controller_manager"]["ros__parameters"]
    return {
        name for name, params in controller_params.items()
        if isinstance(params, dict) and params.get("type") == controller_type
    }


def _resolve_inference_controller_config_path(
    group_config: dict[str, Any],
    inference_controller_config_override: str,
) -> str:
    """Resolve the LEAPP YAML path used by inference_controller."""
    if inference_controller_config_override:
        return str(Path(inference_controller_config_override).expanduser().resolve())

    data_package = group_config.get("data_package", "unitree_g1_bringup")
    if group_config.get("agile_config"):
        agile_pkg = group_config.get("agile_data_package", data_package)
        agile_pkg_share = Path(get_package_share_directory(agile_pkg))
        return str(agile_pkg_share / "data" / group_config["agile_config"])

    data_pkg_share = Path(get_package_share_directory(data_package))
    return str(data_pkg_share / "data" / group_config["config"])


CONTROLLER_GROUPS = _load_controller_groups()
CONTROLLER_MANAGER_CONFIG = _load_controller_manager_config()
SAFETY_CONTROLLER_NAMES = _controller_names_by_type(
    CONTROLLER_MANAGER_CONFIG, SAFETY_CONTROLLER_TYPE)


def _upper_body_joints() -> list[str]:
    """Return safety_controller_upper_body's joints from controller_manager.yaml."""
    return CONTROLLER_MANAGER_CONFIG["safety_controller_upper_body"]["ros__parameters"]["joints"]


def _unique_preserve_order(values: list[str]) -> list[str]:
    """Return values without duplicates, preserving first occurrence order."""
    return list(dict.fromkeys(values))


def _launch_bool(context: LaunchContext, argument_name: str, default_value: bool) -> bool:
    """Parse a boolean launch argument with a caller-provided default."""
    value = context.launch_configurations.get(argument_name, "")
    if not value:
        return default_value

    normalized = value.lower()
    if normalized in ("true", "1", "yes", "on"):
        return True
    if normalized in ("false", "0", "no", "off"):
        return False
    raise ValueError(
        f"Launch argument '{argument_name}' must be true or false, got '{value}'."
    )


def _auto_start_safety_blend_ratio(context: LaunchContext) -> bool:
    """Return whether launch should ramp active safety controllers after spawn."""
    return _launch_bool(
        context,
        "auto_start_safety_blend_ratio",
        default_value=False,
    )


def _startup_safety_blend_controllers(active_controllers: list[str]) -> list[str]:
    """Return lower-body safety controllers whose blend ratio should be auto-started."""
    # Split-blend groups keep safety_controller_upper_body independent so arms
    # can be ramped separately after startup.
    return [
        controller
        for controller in ("safety_controller", "safety_controller_lower_body")
        if controller in active_controllers
    ]


def _set_controller_double_parameter(
    node: RclpyNode,
    executor: SingleThreadedExecutor,
    controller_name: str,
    parameter_name: str,
    value: float,
    timeout_s: float = 10.0,
) -> None:
    """Set a double parameter on a controller through its parameter service."""
    service_name = f"/{controller_name}/set_parameters"
    client = node.create_client(SetParameters, service_name)
    if not client.wait_for_service(timeout_sec=timeout_s):
        raise RuntimeError(f"Parameter service '{service_name}' is not available.")

    parameter = RclParameter()
    parameter.name = parameter_name
    parameter.value = RclParameterValue(
        type=rclpy.Parameter.Type.DOUBLE.value,
        double_value=value,
    )

    request = SetParameters.Request()
    request.parameters = [parameter]
    future = client.call_async(request)
    executor.spin_until_future_complete(future, timeout_sec=timeout_s)

    response = future.result()
    if response is None:
        raise RuntimeError(
            f"Setting '{controller_name}.{parameter_name}' timed out."
        )
    for result in response.results:
        if not result.successful:
            raise RuntimeError(
                f"Setting '{controller_name}.{parameter_name}' failed: {result.reason}"
            )

    node.get_logger().info(
        f"Set {controller_name}.{parameter_name} to {value:.3f}"
    )


def _switch_controllers(
    node: RclpyNode,
    executor: SingleThreadedExecutor,
    controllers_to_activate: list[str],
    controllers_to_deactivate: list[str] | None = None,
    timeout_s: float = 60.0,
) -> None:
    """Activate and deactivate controllers through controller_manager."""
    service_name = "/controller_manager/switch_controller"
    client = node.create_client(SwitchController, service_name)
    if not client.wait_for_service(timeout_sec=timeout_s):
        raise RuntimeError(f"Service '{service_name}' is not available.")

    request = SwitchController.Request()
    request.activate_controllers = controllers_to_activate
    request.deactivate_controllers = controllers_to_deactivate or []
    request.strictness = SwitchController.Request.STRICT
    request.activate_asap = False
    request.timeout = RclpyDuration(seconds=timeout_s).to_msg()

    node.get_logger().info(
        f"Activating controllers after inactive load: {controllers_to_activate}"
    )
    future = client.call_async(request)
    executor.spin_until_future_complete(future, timeout_sec=timeout_s)
    response = future.result()
    if response is None:
        raise RuntimeError("Controller switch timed out.")
    if not response.ok:
        raise RuntimeError(
            "Controller switch failed while activating "
            f"{controllers_to_activate}."
        )


def _activate_startup_controllers(
    _context: LaunchContext,
    active_controllers: list[str],
    startup_safety_blend_controllers: list[str],
) -> list[Any]:
    """Activate pre-loaded startup controllers and optionally ramp safety blend."""
    rclpy_context = RclpyContext()
    rclpy.init(context=rclpy_context)
    node = rclpy.create_node(
        "unitree_g1_startup_controller_activation",
        context=rclpy_context,
    )
    executor = SingleThreadedExecutor(context=rclpy_context)
    executor.add_node(node)
    try:
        _switch_controllers(node, executor, active_controllers)
        for controller_name in startup_safety_blend_controllers:
            _set_controller_double_parameter(
                node, executor, controller_name, "blend_ratio", 1.0
            )
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=rclpy_context)
    return []


def generate_launch_description() -> LaunchDescription:
    """Generate unified launch description supporting both MuJoCo and real hardware."""
    try:
        reference_motion_ros_share = Path(get_package_share_directory("reference_motion_ros"))
        default_motion_file = str(reference_motion_ros_share / "test_data/wave_left.motion")
    except PackageNotFoundError:
        default_motion_file = ""

    declared_arguments = [
        # Hardware selection
        DeclareLaunchArgument(
            "hardware_type",
            default_value="mujoco",
            description="Hardware type: 'mujoco' for MuJoCo, 'real' for physical G1 robot,"
            " 'isaacsim' for Isaac Sim.",
            choices=["mujoco", "real", "isaacsim"],
        ),
        # Controller group selection
        DeclareLaunchArgument(
            "initial_controller_group",
            default_value="agile_velocity",
            description="Controller group from controller_groups.yaml. Options: "
            + ", ".join(CONTROLLER_GROUPS.keys()),
        ),
        DeclareLaunchArgument(
            "initial_controller",
            default_value="",
            description=(
                "Comma-separated list of controllers to spawn in order."
                + " Overrides initial_controller_group when set."
            ),
        ),
        # Visualization
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Start RViz for visualization.",
        ),
        DeclareLaunchArgument(
            "use_foxglove",
            default_value="false",
            description="Start Foxglove Studio bridge for visualization.",
        ),
        # Reference motion
        DeclareLaunchArgument(
            "use_reference_motion",
            default_value="false",
            description="Start reference motion node for motion tracking.",
        ),
        DeclareLaunchArgument(
            "motion_file_path",
            default_value=default_motion_file,
            description="Path to the .motion file for reference motion tracking.",
        ),
        # MuJoCo-specific arguments
        DeclareLaunchArgument(
            "enable_viewer",
            default_value="true",
            description="[MuJoCo only] Enable MuJoCo viewer GUI.",
        ),
        DeclareLaunchArgument(
            "mujoco_model_path",
            default_value="",
            description="[MuJoCo only] Absolute path to the MuJoCo scene XML. "
            "Defaults to unitree_g1_description/mjcf/scene_29dof_with_hand.xml.",
        ),
        # Real hardware-specific arguments
        DeclareLaunchArgument(
            "network_interface",
            default_value="eno1",
            description="[Real hardware only] Network interface for G1 communication.",
        ),
        DeclareLaunchArgument(
            "with_hands",
            default_value="true",
            description="[Real hardware only] Enable hand control.",
        ),
        DeclareLaunchArgument(
            "publish_static_world_tf",
            default_value="true",
            description="[Real hardware only] Publish a static identity world->pelvis TF. "
            "Set false when another node (e.g. teleop's pose_reset_node) owns that edge.",
        ),
        DeclareLaunchArgument(
            "ik_reference_pose_topic",
            default_value="",
            description="Internal: topic to remap /ik_controller/reference_pose to.",
        ),
        DeclareLaunchArgument(
            "cmd_vel_topic",
            default_value="",
            description="Internal: topic to remap /cmd_vel to. When set, configures"
            " inference_controller to subscribe to geometry_msgs/msg/TwistStamped.",
        ),
        DeclareLaunchArgument(
            "inference_controller_config_path",
            default_value="",
            description=(
                "Absolute path to a LEAPP policy YAML for inference_controller. "
                "Empty string uses controller_groups.yaml for initial_controller_group."
            ),
        ),
        DeclareLaunchArgument(
            "auto_start_safety_blend_ratio",
            default_value="",
            description=(
                "Set active safety-controller blend_ratio to 1.0 after controllers "
                "are spawned. Empty defaults to false for every hardware type; set "
                "blend_ratio to 1.0 after publishing the first /cmd_vel instead."
            ),
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )


def launch_setup(context: LaunchContext) -> list[Any]:
    """Create all nodes, resolving launch arguments at launch time."""
    description_pkg_share = Path(get_package_share_directory("unitree_g1_description"))
    bringup_pkg_share = Path(get_package_share_directory("unitree_g1_bringup"))

    # Resolve hardware type
    hardware_type = context.launch_configurations.get("hardware_type", "mujoco")

    # Resolve controller group configuration
    group = context.launch_configurations.get("initial_controller_group", "agile_velocity")
    group_config = CONTROLLER_GROUPS[group]

    # Get hardware-specific configuration
    if hardware_type == "mujoco":
        hw_config = _get_mujoco_config(context, description_pkg_share, bringup_pkg_share)
    elif hardware_type == "real":
        hw_config = _get_real_hardware_config(context, description_pkg_share)
    elif hardware_type == "isaacsim":
        hw_config = _get_isaacsim_config(context, description_pkg_share, bringup_pkg_share)
    else:
        raise ValueError(
            f"Invalid hardware_type: {hardware_type}. "
            "Must be 'mujoco', 'real', or 'isaacsim'."
        )

    # Build robot description
    robot_description_content = Command(hw_config["xacro_command"])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # Common paths
    controller_config_path = str(bringup_pkg_share / "config/controller_manager.yaml")

    # Resolve inference controller config: explicit override first, otherwise
    # preserve the group's agile_config/config default behavior.
    inference_controller_config_override = context.launch_configurations.get(
        "inference_controller_config_path", "").strip()
    inference_controller_config_path = _resolve_inference_controller_config_path(
        group_config, inference_controller_config_override)

    # Groups with command_prefix_lower_body / _upper_body get split-blend
    # control; groups with a single command_prefix use it for both.
    single_prefix = group_config.get("command_prefix")
    has_lower = "command_prefix_lower_body" in group_config
    has_upper = "command_prefix_upper_body" in group_config
    if has_lower != has_upper:
        raise RuntimeError(
            "split-blend requires both command_prefix_lower_body and "
            "command_prefix_upper_body (group config sets only one)")
    lower_body_prefix = group_config.get("command_prefix_lower_body", single_prefix)
    upper_body_prefix = group_config.get("command_prefix_upper_body", single_prefix)

    # Build inference controller parameters from group config.
    inference_ros_params = {
        "config_path": inference_controller_config_path,
        "decimation": 4,  # TODO(lgulich): do not hardcode decimation
    }
    for key in ("command_suffix", "source_to_topic"):
        if group_config.get(key):
            inference_ros_params[key] = group_config[key]
    if lower_body_prefix:
        inference_ros_params["command_prefix"] = lower_body_prefix

    cmd_vel_topic = context.launch_configurations.get("cmd_vel_topic", "")
    if cmd_vel_topic:
        # Override source_to_topic to point directly at the external topic, bypassing the
        # /cmd_vel remapping which does not propagate into controller nodes.
        inference_ros_params["source_to_topic"] = {"command/body/velocity": cmd_vel_topic}
        inference_ros_params["source_message_type"] = {
            "command/body/velocity": "geometry_msgs/msg/TwistStamped"
        }

    # Build runtime parameter overrides (written to a temp YAML loaded after the base config).
    # This includes inference_controller params and safety_controller startup defaults.
    # Safety controllers start at blend_ratio=0.0; MuJoCo defaults to a launch hook that sets
    # the target to 1.0 after all controllers are loaded so the controller ramps in.
    initial_controller = context.launch_configurations.get("initial_controller", "")
    startup_controllers = [
        c.strip() for c in initial_controller.split(",") if c.strip()
    ] or list(group_config.get("controllers", []))
    emergency_deactivate_controllers: list[str] = []
    if any(c in startup_controllers for c in SAFETY_CONTROLLER_NAMES):
        emergency_deactivate_controllers = _unique_preserve_order([
            "joint_command_broadcaster",
            *startup_controllers,
        ])

    def safety_ros_params(blend_ratio: float) -> dict[str, Any]:
        params: dict[str, Any] = {"blend_ratio": blend_ratio}
        if group_config.get("blend_strategy"):
            params["blend_strategy"] = group_config["blend_strategy"]
        if emergency_deactivate_controllers:
            # Compute this once from the startup controller set. ROS Jazzy's FORCE_AUTO does
            # not auto-deactivate conflicting controllers, so safety_controller explicitly
            # deactivates the startup chain before activating freeze_controller.
            # TODO(lgulich): Remove this functionality after moving to ROS Kilted or newer.
            params["out_of_domain_detection"] = {
                "deactivate_controllers": list(emergency_deactivate_controllers),
            }
        return params

    safety_blend_ratio = 0.0

    # Split-blend: default upper_body to 0 so legs come up first, then operator
    # ramps arms in via `ros2 param set /safety_controller_upper_body blend_ratio 1`.
    upper_body_blend_ratio = (
        0.0 if group_config.get("command_prefix_upper_body") else safety_blend_ratio
    )

    # Safe to set all three: unloaded safety controllers are silently ignored.
    runtime_params = {
        "inference_controller": {
            "ros__parameters": inference_ros_params,
        },
        "safety_controller": {
            "ros__parameters": safety_ros_params(safety_blend_ratio),
        },
        "safety_controller_lower_body": {
            "ros__parameters": safety_ros_params(safety_blend_ratio),
        },
        "safety_controller_upper_body": {
            "ros__parameters": safety_ros_params(upper_body_blend_ratio),
        },
    }

    if upper_body_prefix and (
        "upper_body_forward_joint_command_controller"
        in group_config.get("controllers", [])
    ):
        runtime_params["upper_body_forward_joint_command_controller"] = {
            "ros__parameters": {"command_prefix": upper_body_prefix},
        }

    # Split-blend redirects joint_command_broadcaster at upper_body. Lower-body
    # joints are not published on /applied_joint_commands (follow-up: multi-prefix).
    if group_config.get("command_prefix_upper_body") and upper_body_prefix:
        runtime_params["joint_command_broadcaster"] = {
            "ros__parameters": {
                "command_prefix": upper_body_prefix,
                "joints": _upper_body_joints(),
            },
        }

    # Forward command_prefix/command_suffix and robot-specific file paths to
    # ik_controller when it's in the group.
    if "ik_controller" in group_config.get("controllers", []):
        description_share = get_package_share_directory("unitree_g1_description")
        g1_ctrl_share = get_package_share_directory("unitree_g1_ros2_control")
        ik_ros_params: dict[str, Any] = {
            "urdf_path": str(
                Path(description_share) / "urdf/g1_29dof_with_hand_rev_1_0_fixed.urdf"
            ),
            "xrdf_path": str(Path(g1_ctrl_share) / "config/g1_arms_only.xrdf"),
            "rmpflow_config_path": str(
                Path(g1_ctrl_share) / "config/g1_bimanual_rmpflow.yaml"
            ),
        }
        if group_config.get("command_suffix"):
            ik_ros_params["command_suffix"] = group_config["command_suffix"]
        if upper_body_prefix:
            ik_ros_params["command_prefix"] = upper_body_prefix
        runtime_params["ik_controller"] = {"ros__parameters": ik_ros_params}

        safety_gravity_params = {
            "gravity_compensation_urdf_path": str(
                Path(description_share) / "urdf/g1_29dof_with_hand_rev_1_0_fixed.urdf"
            ),
        }
        runtime_params.setdefault("safety_controller", {}).setdefault(
            "ros__parameters", {}
        ).update(safety_gravity_params)
    runtime_params_file = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False)
    yaml.dump(runtime_params, runtime_params_file)
    runtime_params_file.close()

    # Build parameter list for controller manager.
    # runtime_params_file is loaded AFTER the base config so its values take precedence.
    controller_params = [
        robot_description,
        {
            "use_sim_time": hardware_type in ("mujoco", "isaacsim"),
            "mujoco.lockstep": hardware_type == "mujoco",
        },
        controller_config_path,
        runtime_params_file.name,
    ]

    # Add MuJoCo-specific PID parameters
    if hw_config.get("mujoco_pid_config"):
        controller_params.insert(3, hw_config["mujoco_pid_config"])

    # Build optional remappings used when an external topic source replaces the defaults.
    remappings = []
    ik_reference_pose_topic = context.launch_configurations.get("ik_reference_pose_topic", "")
    if ik_reference_pose_topic:
        remappings.append(("/ik_controller/reference_pose", ik_reference_pose_topic))

    # Controller manager node (package varies by hardware type).
    # When CUDA MPS is active, limit this process to 20% of GPU threads so the
    # ros2_control policy gets predictable low-latency inference while any
    # companion inference graph uses the remaining 80%. The env var is ignored
    # when MPS is not running.
    controller_manager_node = Node(
        package=hw_config["controller_manager_package"],
        executable="ros2_control_node",
        parameters=controller_params,
        remappings=remappings,
        output="both",
        emulate_tty=True,
        additional_env={"CUDA_MPS_ACTIVE_THREAD_PERCENTAGE": "20"},
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Static identity world->pelvis TF for real hardware, unless another node
    # (teleop's pose_reset_node) owns that edge.
    static_tf_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="pelvis_to_world_tf",
        arguments=["0", "0", "0", "0", "0", "0", "world", "pelvis"],
        output="screen",
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration("hardware_type"), "' == 'real' and '",
                LaunchConfiguration("publish_static_world_tf"), "' == 'true'",
            ])
        ),
    )

    # Reference motion node
    reference_motion_node = Node(
        package="reference_motion_ros",
        executable="reference_motion_node_exe",
        name="reference_motion_node",
        parameters=[
            {
                "motion_file_path": LaunchConfiguration("motion_file_path"),
                "loop": True,
                "publish_rate": 50.0,
                "world_frame_id": "world",
                "robot_root_frame_id": "pelvis",
                "tf_prefix": "reference_motion",
            }
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_reference_motion")),
        on_exit=Shutdown(),
    )

    # RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("unitree_g1_bringup"), "config", "unitree_g1.rviz"]
            ),
        ],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    # Foxglove bridge
    foxglove_bridge_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="log",
        arguments=["--ros-args", "--log-level", "foxglove_bridge:=warn"],
        condition=IfCondition(LaunchConfiguration("use_foxglove")),
    )

    return [
        controller_manager_node,
        robot_state_publisher_node,
        static_tf_publisher,
        reference_motion_node,
        OpaqueFunction(function=spawn_controllers_sequentially),
        rviz_node,
        foxglove_bridge_node,
    ]


def _get_mujoco_config(
    context: LaunchContext,
    description_pkg_share: Path,
    bringup_pkg_share: Path,
) -> dict[str, Any]:
    """Get configuration for MuJoCo."""
    mujoco_model_path = context.launch_configurations.get("mujoco_model_path", "")
    if not mujoco_model_path:
        mujoco_model_path = str(description_pkg_share / "mjcf/scene_29dof_with_hand.xml")
    urdf_xacro_path = str(description_pkg_share / "urdf/g1_with_ros2_control_full.urdf.xacro")
    mujoco_pid_config_path = str(bringup_pkg_share / "config/mujoco_pid.yaml")

    enable_viewer = context.launch_configurations.get("enable_viewer", "true")

    return {
        "controller_manager_package": "mujoco_ros2_control",
        "mujoco_pid_config": mujoco_pid_config_path,
        "xacro_command": [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ", urdf_xacro_path,
            " ", "mujoco_model_path:=", mujoco_model_path,
            " ", "enable_viewer:=", enable_viewer,
        ],
    }


def _get_real_hardware_config(
    context: LaunchContext,
    description_pkg_share: Path,
) -> dict[str, Any]:
    """Get configuration for real G1 hardware."""
    urdf_xacro_path = str(description_pkg_share / "urdf/g1_real_hardware.urdf.xacro")

    network_interface = context.launch_configurations.get("network_interface", "eno1")
    mode_machine = "5"
    with_hands = context.launch_configurations.get("with_hands", "true")

    return {
        "controller_manager_package": "controller_manager",
        "xacro_command": [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ", urdf_xacro_path,
            " ", "network_interface:=", network_interface,
            " ", "mode_machine:=", mode_machine,
            " ", "with_hands:=", with_hands,
        ],
    }


def _get_isaacsim_config(
    context: LaunchContext,
    description_pkg_share: Path,
    bringup_pkg_share: Path,
) -> dict[str, Any]:
    """Get configuration for Isaac Sim topic-based interface."""
    del context, bringup_pkg_share  # unused; kept for parity with the mujoco helper
    urdf_xacro_path = str(description_pkg_share / "urdf/g1_isaacsim.urdf.xacro")

    # Per-joint actuator dynamics (PD gains, effort envelope) live in the
    # Isaac Sim USD as NewtonActuator prims, not in ros2_control. The Isaac
    # Sim topics are fixed here; remap them at the ROS level if needed.
    return {
        "controller_manager_package": "controller_manager",
        "xacro_command": [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ", urdf_xacro_path,
            " ", "joint_states_topic:=/isaac_sim_joint_states",
            " ", "joint_commands_topic:=/isaac_sim_joint_commands",
            " ", "imu_topic:=/isaac_sim_imu",
        ],
    }


def spawn_controllers_sequentially(context: LaunchContext) -> list[Any]:
    """
    Spawn controllers sequentially with OnProcessExit chaining.

    Spawn order: all startup controllers loaded inactive, then runtime group activated.
    This keeps safety_controller from running until freeze_controller already exists
    and all group controllers are loaded/configured.
    Controllers requested via initial_controller override the inactive list.
    When initial_controller is empty, falls back to the controller group's default list.
    """
    initial_controller = context.launch_configurations.get("initial_controller", "")
    group_controllers = [c.strip() for c in initial_controller.split(",") if c.strip()]

    # Fall back to controller group's default controllers when none are specified
    if not group_controllers:
        group = context.launch_configurations.get("initial_controller_group", "agile_velocity")
        group_controllers = CONTROLLER_GROUPS[group].get("controllers", [])

    # Always activate base broadcasters first, then group-specific controllers.
    active_controllers = [
        "joint_state_broadcaster", "imu_sensor_broadcaster",
        "joint_command_broadcaster",
    ] + group_controllers
    startup_safety_blend_controllers = (
        _startup_safety_blend_controllers(active_controllers)
        if _auto_start_safety_blend_ratio(context)
        else []
    )

    # Load freeze/disable controllers as inactive (unless explicitly requested).
    inactive_controllers = [
        c for c in ["freeze_controller", "disable_controller"]
        if c not in active_controllers
    ]
    load_controllers = _unique_preserve_order(active_controllers + inactive_controllers)

    # Load/configure all controllers first, then activate the runtime group
    # explicitly. This avoids remote-exec startup races where safety_controller
    # runs before the emergency controller has been loaded.
    if not load_controllers:
        return []

    # Using fewer spawner processes avoids DDS discovery issues that
    # occur when many short-lived DDS participants are created sequentially.
    load_inactive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_inactive",
        arguments=[
            *load_controllers,
            "--inactive",
            "--controller-manager-timeout", "60",
            "--service-call-timeout", "60",
        ],
        output="screen",
    )

    return [
        load_inactive_spawner,
        RegisterEventHandler(
            OnProcessExit(
                target_action=load_inactive_spawner,
                on_exit=[
                    OpaqueFunction(
                        function=_activate_startup_controllers,
                        args=[active_controllers, startup_safety_blend_controllers],
                    )
                ],
            )
        ),
    ]
