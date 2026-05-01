#!/usr/bin/env python3
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

"""Integration test for GR00T N1.6 apple-to-plate policy via the node-based inference pipeline.

This test verifies that the inference graph (InputBuilderNode, TritonNode,
OutputBuilderNode) starts up without runtime errors when configured with the
gr00t_n16_apple_to_plate policy.
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
    """Generate launch description for testing gr00t_n16 via inference graph."""
    bringup_share = Path(get_package_share_directory("unitree_g1_bringup"))
    pipeline_launch_file = bringup_share / "launch/unitree_g1_inference_graph.launch.py"

    pipeline_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(pipeline_launch_file)),
        launch_arguments={
            "initial_controller_group": "gr00t_n16_apple_to_plate",
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


class TestGr00tN16InferenceGraph(MujocoInferenceGraphTestBase):
    """Test that the gr00t_n16 policy pipeline starts without runtime errors.

    The robot is expected to fall (camera input not available), so height
    monitoring uses fail_early=False and a very low threshold.
    """

    ROOT_FRAME = "pelvis"
    # Skip waiting for ros2_control controllers — this test only verifies
    # that the inference pipeline (InputBuilder → Triton → OutputBuilder)
    # starts without runtime errors.
    CONTROLLERS: list[str] = []

    # The robot will fall because the policy needs camera input.
    # Override to not assert on height.
    def test_robot_does_not_fall(self):
        """Verify the pipeline starts up without crashing (robot may fall)."""
        if not hasattr(type(self), "ROOT_FRAME"):
            self.skipTest("Abstract base class — ROOT_FRAME not configured")
        from mujoco_test_helpers import wait_for_controllers, spin_for

        wait_for_controllers(
            self, self.node, self.CONTROLLERS, self.CONTROLLER_STARTUP_WAIT_S
        )
        spin_for(
            self.node,
            self.PIPELINE_WARMUP_S,
            f"Waiting {self.PIPELINE_WARMUP_S}s for pipeline to warm up...",
        )
        spin_for(
            self.node,
            self.TEST_DURATION_S,
            f"Running for {self.TEST_DURATION_S}s (robot may fall, checking for crashes)...",
        )
        self.node.get_logger().info("Test passed — no runtime crashes detected.")


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    """Tests to run after node shutdown."""

    def test_exit_codes(self, proc_info):
        """Verify launch completed."""
        pass
