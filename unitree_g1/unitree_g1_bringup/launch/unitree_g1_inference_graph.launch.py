#!/usr/bin/env python3

# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Launch file for Unitree G1 with node-based inference pipeline."""

from pathlib import Path
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _load_controller_groups() -> dict[str, Any]:
    """Load controller group configurations from controller_groups.yaml."""
    bringup_share = Path(get_package_share_directory("unitree_g1_bringup"))
    return yaml.safe_load((bringup_share / "config/controller_groups.yaml").read_text())


CONTROLLER_GROUPS = _load_controller_groups()

# Standard source-to-topic mappings for the inference graph.
# These map hardware state kinds to the ROS topics published by ros2_control broadcasters.
INPUT_KIND_TO_TOPIC = {
    "state/joint/position": "/joint_states",
    "state/joint/velocity": "/joint_states",
    "state/body/rotation": "/imu_sensor_broadcaster/imu",
    "state/body/angular_velocity": "/imu_sensor_broadcaster/imu",
    "state/camera/image": "/camera/image_raw",
}

# Standard output kind-to-topic mappings for the inference graph.
# Maps output kinds to the ROS topics that the ImpedanceController subscribes to.
OUTPUT_KIND_TO_TOPIC = {
    "target/joint/position": "joint_commands",
    "kp": "joint_commands",
    "kd": "joint_commands",
    # Legacy kind names (used by protomotions policies).
    # TODO(lgulich): Figure out how we can get rid of these.
    "joint_pos_targets": "joint_commands",
    "actions": "joint_commands",
    "stiffness_targets": "joint_commands",
    "damping_targets": "joint_commands",
}


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for G1 with node-based inference pipeline."""
    declared_arguments = [
        # Hardware selection
        DeclareLaunchArgument(
            'hardware_type',
            default_value='mujoco',
            description="Hardware type: 'mujoco' for MuJoCo, 'real' for physical G1 robot.",
            choices=['mujoco', 'real'],
        ),
        # Common arguments
        DeclareLaunchArgument(
            "initial_controller_group",
            default_value="agile_velocity",
            description="Controller group from controller_groups.yaml. Options: "
            + ", ".join(CONTROLLER_GROUPS.keys()),
        ),
        DeclareLaunchArgument(
            'enable_viewer',
            default_value='true',
            description='[MuJoCo only] Set to true to enable the MuJoCo viewer GUI.',
        ),
        DeclareLaunchArgument(
            'use_foxglove',
            default_value='true',
            description='Start Foxglove Studio bridge for visualization.',
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='50.0',
            description='Rate at which InputBuilderNode publishes (Hz).',
        ),
        DeclareLaunchArgument(
            'use_reference_motion',
            default_value='false',
            description='Start reference motion node for motion tracking.',
        ),
        DeclareLaunchArgument(
            'visualize_commands',
            default_value='true',
            description='Publish commanded joint positions as a ghost robot for visualization.',
        ),
        # Real hardware-specific arguments
        DeclareLaunchArgument(
            'network_interface',
            default_value='eno1',
            description='[Real hardware only] Network interface for G1 communication.',
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )


def launch_setup(context: LaunchContext) -> list[Any]:
    """Create all nodes, resolving launch arguments at launch time."""
    description_pkg_share = Path(get_package_share_directory('unitree_g1_description'))

    # Resolve controller group configuration.
    group = context.launch_configurations.get("initial_controller_group", "agile_velocity")
    group_config = CONTROLLER_GROUPS[group]

    data_package = group_config.get("data_package", "unitree_g1_bringup")
    data_pkg_share = Path(get_package_share_directory(data_package))
    config_path = str(data_pkg_share / 'data' / group_config['config'])

    # Build source_to_topic: standard hardware mappings + policy-specific mappings.
    source_to_topic = dict(INPUT_KIND_TO_TOPIC)
    if group_config.get("source_to_topic"):
        source_to_topic.update(group_config["source_to_topic"])
    source_to_topic_str = ",".join(f"{k}:{v}" for k, v in source_to_topic.items())

    # Build output_to_topic by parsing YAML outputs and mapping name→topic via kind.
    config_yaml = yaml.safe_load(Path(config_path).read_text())
    output_to_topic = {}
    for model_config in config_yaml.get("models", {}).values():
        for output in model_config.get("outputs", []):
            kind = output.get("kind", "")
            if kind in OUTPUT_KIND_TO_TOPIC:
                output_to_topic[output["name"]] = OUTPUT_KIND_TO_TOPIC[kind]
    output_to_topic_str = ",".join(f"{k}:{v}" for k, v in output_to_topic.items())

    # Robot description for command visualization (ghost robot).
    urdf_xacro_path = str(description_pkg_share / 'urdf' / 'g1_with_ros2_control_full.urdf.xacro')
    mujoco_model_path = str(description_pkg_share / 'mjcf' / 'scene_29dof_with_hand.xml')
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ', urdf_xacro_path,
        ' ', 'mujoco_model_path:=', mujoco_model_path,
        ' ', 'enable_viewer:=false',
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    controller_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('unitree_g1_bringup'),
                'launch',
                'unitree_g1_controller_manager.launch.py'
            ])
        ),
        launch_arguments={
            'hardware_type': LaunchConfiguration('hardware_type'),
            'enable_viewer': LaunchConfiguration('enable_viewer'),
            'use_foxglove': LaunchConfiguration('use_foxglove'),
            'use_reference_motion': LaunchConfiguration('use_reference_motion'),
            'network_interface': LaunchConfiguration('network_interface'),
            'initial_controller': 'safety_controller,forward_joint_command_controller',
        }.items(),
    )

    inference_pipeline = GroupAction([
        # Remap output topics from namespace-relative to global.
        SetRemap(src='joint_commands', dst='/joint_commands'),
        SetRemap(src='body_commands', dst='/body_commands'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('isaac_ros_deploy_bringup'),
                    'launch',
                    'inference_graph.launch.py'
                ])
            ),
            launch_arguments={
                'config_path': config_path,
                'publish_rate': LaunchConfiguration('publish_rate'),
                'source_to_topic': source_to_topic_str,
                'output_to_topic': output_to_topic_str,
            }.items(),
        ),
    ])

    # Command visualization: converts JointCommand to JointState for robot_state_publisher.
    joint_command_to_joint_state_node = Node(
        package='isaac_ros_deploy_converters',
        executable='joint_command_to_joint_state_node',
        name='joint_command_to_joint_state',
        remappings=[
            ('~/input', '/joint_commands'),
            ('~/output', '/commanded_joint_states'),
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('visualize_commands')),
    )

    # Robot state publisher for commanded pose (ghost robot with TF prefix).
    commanded_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='commanded_robot_state_publisher',
        parameters=[
            robot_description,
            {'frame_prefix': 'commanded/'},
        ],
        remappings=[
            ('joint_states', '/commanded_joint_states'),
            ('robot_description', '/commanded_robot_description'),
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('visualize_commands')),
    )

    return [
        controller_manager_launch,
        inference_pipeline,
        joint_command_to_joint_state_node,
        commanded_robot_state_publisher,
    ]
