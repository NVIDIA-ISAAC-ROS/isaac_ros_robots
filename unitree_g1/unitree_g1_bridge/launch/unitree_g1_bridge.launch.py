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
            default_value='[1, 4, 200]',
            description=(
                'FSM IDs for standup '
                '(damp=1, lock_stand=4, start=200).'
            ),
        ),
        DeclareLaunchArgument(
            'standup_delays',
            default_value='[3.0, 8.0, 3.0]',
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
            'auto_standup_delay',
            default_value='2.0',
            description='Delay (s) before auto_standup fires (DDS settle).',
        ),
        DeclareLaunchArgument(
            'qos_depth',
            default_value='10',
            description='QoS depth for publishers and subscribers',
        ),
        DeclareLaunchArgument(
            'require_acks',
            default_value='false',
            description=(
                'If true, standup verifies each step by waiting up to '
                'step_ack_timeout for a firmware ack and aborts on '
                'rejection/timeout. If false, fire-and-forget. '
                'standup_delays applies in both modes.'
            ),
        ),
        DeclareLaunchArgument(
            'step_ack_timeout',
            default_value='30.0',
            description='Ack wait (s) per standup step when require_acks=true.',
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
                'auto_standup_delay': LaunchConfiguration(
                    'auto_standup_delay',
                ),
                'qos_depth': LaunchConfiguration('qos_depth'),
                'require_acks': LaunchConfiguration('require_acks'),
                'step_ack_timeout': LaunchConfiguration('step_ack_timeout'),
            }],
            remappings=[
                ('cmd_vel', '/cmd_vel'),
            ],
        ),
    ])
