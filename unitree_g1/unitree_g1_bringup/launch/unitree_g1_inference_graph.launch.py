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

"""Launch file for Unitree G1 with node-based inference pipeline."""

from pathlib import Path
from typing import Any

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
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node, SetRemap
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml


def _load_controller_groups() -> dict[str, Any]:
    """Load controller group configurations from controller_groups.yaml."""
    bringup_share = Path(get_package_share_directory("unitree_g1_bringup"))
    return yaml.safe_load((bringup_share / "config/controller_groups.yaml").read_text())


CONTROLLER_GROUPS = _load_controller_groups()


def _resolve_inference_graph_config_path(
    group_config: dict[str, Any],
    inference_graph_config_override: str,
) -> str:
    """Resolve the LEAPP YAML path used by the node-based inference graph."""
    if inference_graph_config_override:
        return str(Path(inference_graph_config_override).expanduser().resolve())

    data_package = group_config.get("data_package", "unitree_g1_bringup")
    data_pkg_share = Path(get_package_share_directory(data_package))
    return str(data_pkg_share / "data" / group_config["config"])


# Standard source-to-topic mappings for the inference graph.
# These map hardware state kinds to the ROS topics published by ros2_control broadcasters.
INPUT_KIND_TO_TOPIC = {
    "state/joint/position": "/joint_states",
    "state/joint/velocity": "/joint_states",
    "state/body/rotation": "/imu_sensor_broadcaster/imu",
    "state/body/angular_velocity": "/imu_sensor_broadcaster/imu",
}


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for G1 with node-based inference pipeline."""
    declared_arguments = [
        # Hardware selection
        DeclareLaunchArgument(
            'hardware_type',
            default_value='mujoco',
            description="Hardware type: 'mujoco' for MuJoCo, 'isaacsim' for Isaac Sim, "
            "'real' for physical G1 robot.",
            choices=['mujoco', 'real', 'isaacsim'],
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
            'mujoco_model_path',
            default_value='',
            description='[MuJoCo only] Absolute path to the MuJoCo scene XML. '
            'Defaults to unitree_g1_description/mjcf/scene_29dof_with_hand.xml.',
        ),
        DeclareLaunchArgument(
            'use_foxglove',
            default_value='true',
            description='Start Foxglove Studio bridge for visualization.',
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='5.0',
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
        DeclareLaunchArgument(
            'inference_controller_config_path',
            default_value='',
            description=(
                'Absolute path to the LEAPP policy YAML for the ros2_control '
                'inference_controller. Empty string lets the controller manager '
                'use the selected controller group default.'
            ),
        ),
        DeclareLaunchArgument(
            'inference_graph_config_path',
            default_value='',
            description=(
                'Absolute path to the LEAPP policy YAML for the node-based inference '
                'graph. Empty string uses the selected controller group config.'
            ),
        ),
        DeclareLaunchArgument(
            'triton_cpu_models',
            default_value='',
            description='Comma-separated LEAPP model names to run on Triton CPU.',
        ),
        DeclareLaunchArgument(
            'joint_commands_trajectory_output_topic',
            default_value='/joint_commands_trajectory',
            description='Global topic that the inference graph publishes joint command '
                        'trajectories to.',
        ),
        DeclareLaunchArgument(
            'cmd_vel_output_topic',
            default_value='/cmd_vel',
            description='Global topic that the inference graph publishes velocity commands to.',
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

    inference_graph_config_override = context.launch_configurations.get(
        "inference_graph_config_path", "").strip()
    config_path = _resolve_inference_graph_config_path(
        group_config, inference_graph_config_override)

    # Build source_to_topic: standard hardware mappings + policy-specific mappings.
    source_to_topic = dict(INPUT_KIND_TO_TOPIC)
    if group_config.get("source_to_topic"):
        source_to_topic.update(group_config["source_to_topic"])
    source_to_topic_str = ",".join(f"{k}:{v}" for k, v in source_to_topic.items())

    # Robot description for command visualization (ghost robot).
    urdf_xacro_path = str(description_pkg_share / 'urdf' / 'g1_with_ros2_control_full.urdf.xacro')
    mujoco_model_path = context.launch_configurations.get('mujoco_model_path', '')
    if not mujoco_model_path:
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
            'initial_controller_group': LaunchConfiguration('initial_controller_group'),
            'hardware_type': LaunchConfiguration('hardware_type'),
            'enable_viewer': LaunchConfiguration('enable_viewer'),
            'mujoco_model_path': LaunchConfiguration('mujoco_model_path'),
            'use_foxglove': LaunchConfiguration('use_foxglove'),
            'use_reference_motion': LaunchConfiguration('use_reference_motion'),
            'network_interface': LaunchConfiguration('network_interface'),
            'inference_controller_config_path': LaunchConfiguration(
                'inference_controller_config_path'),
            'initial_controller': ','.join(
                group_config.get(
                    'controllers',
                    ['safety_controller', 'forward_joint_command_controller'],
                )
            ),
            'inference_config_path': '',
        }.items(),
    )

    joint_commands_trajectory_output_topic = context.perform_substitution(
        LaunchConfiguration('joint_commands_trajectory_output_topic'))
    cmd_vel_output_topic = context.perform_substitution(
        LaunchConfiguration('cmd_vel_output_topic'))

    inference_pipeline = GroupAction([
        # Remap output topics from namespace-relative to global.
        SetRemap(src='joint_commands', dst='/joint_commands'),
        SetRemap(src='joint_commands_trajectory', dst=joint_commands_trajectory_output_topic),
        SetRemap(src='body_commands', dst='/body_commands'),
        SetRemap(src='cmd_vel', dst=cmd_vel_output_topic),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('isaac_ros_deploy_bringup'),
                    'launch',
                    'inference_graph.launch.py'
                ])
            ),
            launch_arguments={
                'config_path': TextSubstitution(text=config_path),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'source_to_topic': source_to_topic_str,
                'triton_cpu_models': LaunchConfiguration('triton_cpu_models'),
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
