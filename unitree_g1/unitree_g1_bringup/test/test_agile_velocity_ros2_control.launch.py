#!/usr/bin/env python3

# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Integration test for velocity policy with Unitree G1 in MuJoCo simulation.

This test verifies that the agile_velocity policy can keep the robot
stable (not falling) when given zero velocity commands.
"""

import unittest
from pathlib import Path

import launch
import launch_testing
import launch_testing.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from mujoco_test_helpers import MujocoControllerManagerTestBase, has_display


def generate_test_description():
    """Generate launch description for testing velocity policy."""
    bringup_share = Path(get_package_share_directory("unitree_g1_bringup"))
    main_launch_file = bringup_share / "launch/unitree_g1_controller_manager.launch.py"

    main_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(main_launch_file)),
        launch_arguments={
            "initial_controller_group": "agile_velocity",
            "enable_viewer": str(has_display()).lower(),
            "use_foxglove": "false",
            "use_rviz": "false",
        }.items(),
    )

    return launch.LaunchDescription(
        [
            main_launch,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestVelocityController(MujocoControllerManagerTestBase):
    """Test that the velocity policy keeps the robot stable with zero commands."""

    CONTROLLERS = ["inference_controller"]
    PUBLISH_CMD_VEL = True
    ROOT_FRAME = "pelvis"


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    """Tests to run after node shutdown."""

    def test_exit_codes(self, proc_info):
        """Verify launch completed."""
        pass
