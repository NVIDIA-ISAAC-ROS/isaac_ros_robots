# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

"""Launch file for the Unitree G1 locomotion bridge node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'velocity_duration',
            default_value='0.5',
            description='Duration (s) attached to each velocity command',
        ),
        DeclareLaunchArgument(
            'standup_fsm_sequence',
            default_value='[1, 4, 500]',
            description='FSM IDs for standup (damp=1, stand=4, start=500)',
        ),
        DeclareLaunchArgument(
            'standup_delays',
            default_value='[3.0, 6.0, 3.0]',
            description='Delays (s) after each FSM step in the standup sequence',
        ),
        DeclareLaunchArgument(
            'balance_mode',
            default_value='0',
            description='Balance mode after standup (0=stand, 1=continuous gait)',
        ),
        DeclareLaunchArgument(
            'auto_standup',
            default_value='false',
            description='Automatically execute standup sequence on node start',
        ),
        DeclareLaunchArgument(
            'qos_depth',
            default_value='10',
            description='QoS depth for publishers and subscribers',
        ),

        Node(
            package='unitree_g1_bridge',
            executable='unitree_bridge_node',
            name='unitree_bridge_node',
            output='screen',
            parameters=[{
                'velocity_duration': LaunchConfiguration('velocity_duration'),
                'standup_fsm_sequence': LaunchConfiguration(
                    'standup_fsm_sequence',
                ),
                'standup_delays': LaunchConfiguration('standup_delays'),
                'balance_mode': LaunchConfiguration('balance_mode'),
                'auto_standup': LaunchConfiguration('auto_standup'),
                'qos_depth': LaunchConfiguration('qos_depth'),
            }],
            remappings=[
                ('cmd_vel', '/cmd_vel'),
            ],
        ),
    ])
