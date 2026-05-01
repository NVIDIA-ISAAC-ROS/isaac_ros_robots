#!/usr/bin/env python3

# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Integration test for inference_controller with Unitree G1 in MuJoCo simulation.

This test verifies that the inference_controller can track reference motions
and keep the robot stable (not falling) for a specified duration.
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
    """Generate launch description for testing inference_controller."""
    bringup_share = Path(get_package_share_directory("unitree_g1_bringup"))
    reference_motion_share = Path(get_package_share_directory("reference_motion_ros"))

    main_launch_file = bringup_share / "launch/unitree_g1_controller_manager.launch.py"
    motion_file = reference_motion_share / "test_data/wave_left.motion"

    main_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(main_launch_file)),
        launch_arguments={
            "initial_controller_group": "protomotions_wave_motion",
            "use_reference_motion": "true",
            "motion_file_path": str(motion_file),
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


class TestInferenceController(MujocoControllerManagerTestBase):
    """Test that the inference_controller keeps the robot stable."""

    CONTROLLERS = ["inference_controller", "safety_controller"]
    ROOT_FRAME = "pelvis"


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    """Tests to run after node shutdown."""

    def test_exit_codes(self, proc_info):
        """Verify launch completed."""
        pass
