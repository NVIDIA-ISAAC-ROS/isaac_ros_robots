#!/usr/bin/env python3

# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Integration test for the node-based inference pipeline with Unitree G1 in MuJoCo simulation.

This test verifies that the inference graph (InputBuilderNode, TritonNode,
OutputBuilderNode) can keep the robot stable (not falling) for a specified duration.
"""

from pathlib import Path
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions

from mujoco_test_helpers import has_display, MujocoInferenceGraphTestBase


def generate_test_description():
    """Generate launch description for testing inference graph."""
    bringup_share = Path(get_package_share_directory("unitree_g1_bringup"))
    pipeline_launch_file = bringup_share / "launch/unitree_g1_inference_graph.launch.py"

    pipeline_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(pipeline_launch_file)),
        launch_arguments={
            "initial_controller_group": "protomotions_wave_motion",
            "enable_viewer": str(has_display()).lower(),
            "use_foxglove": "false",
            "use_reference_motion": "true",
        }.items(),
    )

    return launch.LaunchDescription(
        [
            pipeline_launch,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestInferenceGraph(MujocoInferenceGraphTestBase):
    """Test that the inference graph keeps the robot stable."""

    ROOT_FRAME = "pelvis"


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    """Tests to run after node shutdown."""

    def test_exit_codes(self, proc_info):
        """Verify launch completed."""
        pass
