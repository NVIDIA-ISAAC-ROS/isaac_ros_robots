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


def _load_controller_groups() -> dict[str, Any]:
    """Load controller group configurations from controller_groups.yaml."""
    bringup_share = Path(get_package_share_directory("unitree_g1_bringup"))
    return yaml.safe_load((bringup_share / "config/controller_groups.yaml").read_text())


CONTROLLER_GROUPS = _load_controller_groups()


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
    inference_config_path = str(data_pkg_share / "data" / group_config["config"])

    # Build inference controller parameters from group config.
    inference_ros_params = {
        "config_path": inference_config_path,
        "decimation": 4,  # TODO(lgulich): do not hardcode decimation
    }
    for key in ("command_prefix", "command_suffix", "source_to_topic"):
        if group_config.get(key):
            inference_ros_params[key] = group_config[key]

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
    safety_ros_params: dict[str, Any] = {"blend_ratio": hw_config["blend_ratio_default"]}
    if group_config.get("blend_strategy"):
        safety_ros_params["blend_strategy"] = group_config["blend_strategy"]

    runtime_params = {
        "inference_controller": {
            "ros__parameters": inference_ros_params,
        },
        "safety_controller": {
            "ros__parameters": safety_ros_params,
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
        for key in ("command_prefix", "command_suffix"):
            if group_config.get(key):
                ik_ros_params[key] = group_config[key]
        runtime_params["ik_controller"] = {"ros__parameters": ik_ros_params}
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

    # Controller manager node (package varies by hardware type)
    controller_manager_node = Node(
        package=hw_config["controller_manager_package"],
        executable="ros2_control_node",
        parameters=controller_params,
        remappings=remappings,
        output="both",
        emulate_tty=True,
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

    Spawn order: base broadcasters → user controllers (active) → inactive controllers.
    Controllers requested via initial_controller override the inactive list.
    When initial_controller is empty, falls back to the controller group's default list.
    """
    initial_controller = context.launch_configurations.get("initial_controller", "")
    active_controllers = [c.strip() for c in initial_controller.split(",") if c.strip()]

    # Fall back to controller group's default controllers when none are specified
    if not active_controllers:
        group = context.launch_configurations.get("initial_controller_group", "agile_velocity")
        active_controllers = CONTROLLER_GROUPS[group].get("controllers", [])

    # Always spawn broadcasters first
    active_controllers = ["joint_state_broadcaster", "imu_sensor_broadcaster"] + active_controllers

    # Load freeze/disable controllers as inactive (unless explicitly requested).
    inactive_controllers = [
        c for c in ["freeze_controller", "disable_controller"]
        if c not in active_controllers
    ]

    # Spawn active and inactive controllers as two batched spawner processes.
    # Using fewer spawner processes avoids DDS discovery issues that
    # occur when many short-lived DDS participants are created sequentially.
    spawners = []
    if active_controllers:
        spawners.append(Node(
            package="controller_manager",
            executable="spawner",
            name="spawner_active",
            arguments=[
                *active_controllers,
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

    # Chain spawners: active first, then inactive
    actions = [spawners[0]]
    for i in range(1, len(spawners)):
        actions.append(RegisterEventHandler(
            OnProcessExit(target_action=spawners[i - 1], on_exit=[spawners[i]])
        ))

    return actions
