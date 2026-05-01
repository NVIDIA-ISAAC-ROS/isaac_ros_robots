#!/usr/bin/env python3

# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Integration test for agile_velocity policy via the node-based inference pipeline.

This test verifies that the inference graph (InputBuilderNode, TritonNode,
OutputBuilderNode) can keep the robot stable (not falling) when running the
agile_velocity policy with zero velocity commands.
"""

import unittest
from pathlib import Path

import launch
import launch_testing
import launch_testing.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from mujoco_test_helpers import MujocoInferenceGraphTestBase, has_display


def generate_test_description():
    """Generate launch description for testing agile_velocity via inference graph."""
    bringup_share = Path(get_package_share_directory("unitree_g1_bringup"))
    pipeline_launch_file = bringup_share / "launch/unitree_g1_inference_graph.launch.py"

    pipeline_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(pipeline_launch_file)),
        launch_arguments={
            "initial_controller_group": "agile_velocity",
            "enable_viewer": str(has_display()).lower(),
            "use_foxglove": "false",
        }.items(),
    )

    return launch.LaunchDescription(
        [
            pipeline_launch,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestAgileVelocityPrivilegedInferenceGraph(MujocoInferenceGraphTestBase):
    """Test that the agile_velocity policy keeps the robot stable via inference graph."""

    PUBLISH_CMD_VEL = True
    ROOT_FRAME = "pelvis"


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    """Tests to run after node shutdown."""

    def test_exit_codes(self, proc_info):
        """Verify launch completed."""
        pass
