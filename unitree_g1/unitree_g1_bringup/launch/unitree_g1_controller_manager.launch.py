#!/usr/bin/env python3

# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Unified launch file for Unitree G1 robot - supports both MuJoCo and real hardware."""

from pathlib import Path
import tempfile
from typing import Any

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, Shutdown
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
            description="Hardware type: 'mujoco' for MuJoCo, 'real' for physical G1 robot.",
            choices=["mujoco", "real"],
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
            "inference_config_path",
            default_value="",
            description=(
                "Absolute path to a LEAPP policy YAML for inference_controller. "
                "Empty string uses controller_groups.yaml for initial_controller_group."
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
    else:
        raise ValueError(f"Invalid hardware_type: {hardware_type}. Must be 'mujoco' or 'real'.")

    # Build robot description
    robot_description_content = Command(hw_config["xacro_command"])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # Common paths
    controller_config_path = str(bringup_pkg_share / "config/controller_manager.yaml")
    data_package = group_config.get("data_package", "unitree_g1_bringup")
    data_pkg_share = Path(get_package_share_directory(data_package))

    # Resolve inference controller config: use agile_config if present (for
    # groups that run AGILE balancing alongside an inference graph), otherwise
    # use the group's main config.
    inference_config_override = context.launch_configurations.get(
        "inference_config_path", "").strip()
    if group_config.get("agile_config"):
        agile_pkg = group_config.get("agile_data_package", data_package)
        agile_pkg_share = Path(get_package_share_directory(agile_pkg))
        inference_config_path = str(agile_pkg_share / "data" / group_config["agile_config"])
    elif inference_config_override:
        inference_config_path = str(Path(inference_config_override).expanduser().resolve())
    else:
        inference_config_path = str(data_pkg_share / "data" / group_config["config"])

    # Split-blend groups override with command_prefix_lower_body / _upper_body;
    # legacy groups fall back to a single command_prefix for both.
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
        "config_path": inference_config_path,
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
    # This includes inference_controller params and hardware-specific safety_controller defaults.
    # MuJoCo: blend_ratio=1.0 (policy fully enabled).
    # Real hardware: blend_ratio=0.0 (policy disabled).
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

    safety_blend_ratio = hw_config["blend_ratio_default"]

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
        {"use_sim_time": hardware_type == "mujoco"},
        controller_config_path,
        runtime_params_file.name,
    ]

    # Add MuJoCo-specific PID parameters
    if hw_config.get("mujoco_pid_config"):
        controller_params.insert(3, hw_config["mujoco_pid_config"])

    # Build optional remappings — used when an external topic source replaces the defaults.
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

    # Static TF publisher for real hardware (pelvis to world transform)
    static_tf_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="pelvis_to_world_tf",
        arguments=["0", "0", "0", "0", "0", "0", "world", "pelvis"],
        output="screen",
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("hardware_type"), "' == 'real'"])
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
        "blend_ratio_default": 1.0,  # MuJoCo: safety off by default
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
        "blend_ratio_default": 0.0,  # Real hardware: safety on by default
        "xacro_command": [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ", urdf_xacro_path,
            " ", "network_interface:=", network_interface,
            " ", "mode_machine:=", mode_machine,
            " ", "with_hands:=", with_hands,
        ],
    }


def spawn_controllers_sequentially(context: LaunchContext) -> list[Any]:
    """
    Spawn controllers sequentially with OnProcessExit chaining.

    Spawn order: base broadcasters + group controllers (active) → inactive controllers.
    When safety_controller is active, joint_command_broadcaster is split out and
    spawned between those steps so safety_controller can export its reference interfaces first.
    Controllers requested via initial_controller override the inactive list.
    When initial_controller is empty, falls back to the controller group's default list.
    """
    initial_controller = context.launch_configurations.get("initial_controller", "")
    group_controllers = [c.strip() for c in initial_controller.split(",") if c.strip()]

    # Fall back to controller group's default controllers when none are specified
    if not group_controllers:
        group = context.launch_configurations.get("initial_controller_group", "agile_velocity")
        group_controllers = CONTROLLER_GROUPS[group].get("controllers", [])

    # Always spawn base broadcasters first in the active batch, then group-specific controllers.
    active_controllers = [
        "joint_state_broadcaster", "imu_sensor_broadcaster",
        "joint_command_broadcaster",
    ] + group_controllers

    # Spawn after whichever safety_controller variant is active — broadcaster
    # reads its reference interfaces so ordering matters.
    spawn_jcb_separately = any(
        c in active_controllers for c in SAFETY_CONTROLLER_NAMES
    )

    # Load freeze/disable controllers as inactive (unless explicitly requested).
    inactive_controllers = [
        c for c in ["freeze_controller", "disable_controller"]
        if c not in active_controllers
    ]

    # Remove joint_command_broadcaster from the main batch when it needs
    # to be spawned separately (after safety_controller).
    batch_controllers = [
        c for c in active_controllers
        if not (spawn_jcb_separately and c == "joint_command_broadcaster")
    ]

    # Spawn active and inactive controllers as batched spawner processes.
    # Using fewer spawner processes avoids DDS discovery issues that
    # occur when many short-lived DDS participants are created sequentially.
    spawners = []
    if batch_controllers:
        spawners.append(Node(
            package="controller_manager",
            executable="spawner",
            name="spawner_active",
            arguments=[
                *batch_controllers,
                "--controller-manager-timeout", "60",
                "--service-call-timeout", "60",
            ],
            output="screen",
        ))
    # Chain joint_command_broadcaster after the active batch so that safety_controller
    # reference interfaces are registered before broadcaster activation.
    if spawn_jcb_separately:
        spawners.append(Node(
            package="controller_manager",
            executable="spawner",
            name="spawner_joint_command_broadcaster",
            arguments=[
                "joint_command_broadcaster",
                "--controller-manager-timeout", "60",
                "--service-call-timeout", "60",
            ],
            output="screen",
        ))
    if inactive_controllers:
        spawners.append(Node(
            package="controller_manager",
            executable="spawner",
            name="spawner_inactive",
            arguments=[
                *inactive_controllers,
                "--inactive",
                "--controller-manager-timeout", "60",
                "--service-call-timeout", "60",
            ],
            output="screen",
        ))

    if not spawners:
        return []

    # Chain spawners via OnProcessExit: active → optional joint_command_broadcaster → inactive.
    actions = [spawners[0]]
    for i in range(1, len(spawners)):
        actions.append(RegisterEventHandler(
            OnProcessExit(target_action=spawners[i - 1], on_exit=[spawners[i]])
        ))

    return actions
