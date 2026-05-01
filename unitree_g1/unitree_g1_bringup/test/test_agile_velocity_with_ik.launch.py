#!/usr/bin/env python3

# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Integration test for agile_velocity_with_ik policy with Unitree G1 in MuJoCo simulation.

This test verifies that:
  - The robot stays above the height threshold (does not fall).
  - Valid TF transforms are published for both end-effector frames
    (left_hand_palm_link and right_hand_palm_link).
"""

import math
from pathlib import Path
import unittest

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseArray
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
from mujoco_test_helpers import (
    has_display,
    MujocoControllerManagerTestBase,
    spin_for,
)
from rclpy.duration import Duration
from scipy.spatial.transform import Rotation
import tf2_ros

LEFT_EE_FRAME = "left_hand_palm_link"
RIGHT_EE_FRAME = "right_hand_palm_link"
REFERENCE_FRAME = "pelvis"

# Feasible arm targets in the pelvis frame.
# Right arm: forward 0.3m, 0.25m to the right, 0.05m below pelvis, 40deg yaw.
# Left arm:  forward 0.35m, 0.2m to the left,  0.1m below pelvis,  30deg pitch.
_RIGHT_POSE = Pose()
_RIGHT_POSE.position.x = 0.35
_RIGHT_POSE.position.y = -0.2
_RIGHT_POSE.position.z = 0.1
_RIGHT_POSE.orientation.x = 0.0
_RIGHT_POSE.orientation.y = 0.0
_RIGHT_POSE.orientation.z = 0.342
_RIGHT_POSE.orientation.w = 0.940

_LEFT_POSE = Pose()
_LEFT_POSE.position.x = 0.35
_LEFT_POSE.position.y = 0.2
_LEFT_POSE.position.z = 0.1
_LEFT_POSE.orientation.x = 0.0
_LEFT_POSE.orientation.y = 0.259
_LEFT_POSE.orientation.z = 0.0
_LEFT_POSE.orientation.w = 0.966

MAX_POSITION_ERROR_M = 0.1
MAX_ORIENTATION_ERROR_DEG = 15.0


def _assert_ee_pose_within_tolerance(test_case, tf, ref_pose, frame_name):
    """Assert that a TF transform is within position and orientation tolerances of ref_pose."""
    t = tf.transform.translation
    pos_error = math.sqrt(
        (t.x - ref_pose.position.x) ** 2
        + (t.y - ref_pose.position.y) ** 2
        + (t.z - ref_pose.position.z) ** 2
    )
    test_case.assertLessEqual(
        pos_error,
        MAX_POSITION_ERROR_M,
        f"{frame_name} position error {pos_error:.3f}m exceeds {MAX_POSITION_ERROR_M}m",
    )

    actual_quat = tf.transform.rotation
    actual_rotation = Rotation.from_quat(
        [actual_quat.x, actual_quat.y, actual_quat.z, actual_quat.w])
    ref_quat = ref_pose.orientation
    ref_rotation = Rotation.from_quat([ref_quat.x, ref_quat.y, ref_quat.z, ref_quat.w])
    angle_error_deg = math.degrees((actual_rotation * ref_rotation.inv()).magnitude())
    test_case.assertLessEqual(
        angle_error_deg,
        MAX_ORIENTATION_ERROR_DEG,
        f"{frame_name} orientation error {angle_error_deg:.1f}° "
        f"exceeds {MAX_ORIENTATION_ERROR_DEG}°",
    )


def generate_test_description():
    """Generate launch description for testing agile_velocity_with_ik."""
    bringup_share = Path(get_package_share_directory("unitree_g1_bringup"))
    main_launch_file = bringup_share / "launch/unitree_g1_controller_manager.launch.py"

    main_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(main_launch_file)),
        launch_arguments={
            "initial_controller_group": "agile_velocity_with_ik",
            "enable_viewer": str(has_display()).lower(),
            "use_foxglove": "false",
            "use_rviz": "false",
        }.items(),
    )

    # so the IK controller can look them up without requiring the teleop stack to be running.
    # cmd_T_ee = identity means the IK tracks reference poses with no frame correction.
    left_openxr_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0",
                   "left_hand_palm_link", "left_hand_palm_link_openxr"],
    )
    right_openxr_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0",
                   "right_hand_palm_link", "right_hand_palm_link_openxr"],
    )

    return launch.LaunchDescription(
        [
            main_launch,
            left_openxr_tf,
            right_openxr_tf,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestAgileVelocityWithIK(MujocoControllerManagerTestBase):
    """Test that agile_velocity_with_ik keeps the robot stable and publishes EE poses."""

    CONTROLLERS = [
        "inference_controller", "safety_controller", "ik_controller", "finger_controller"
    ]
    PUBLISH_CMD_VEL = True
    ROOT_FRAME = "pelvis"

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls._ref_pub = cls.node.create_publisher(
            PoseArray, "/ik_controller/reference_pose", 10
        )

        def _publish_reference():
            msg = PoseArray()
            msg.header.stamp = cls.node.get_clock().now().to_msg()
            msg.header.frame_id = REFERENCE_FRAME
            msg.poses.append(_RIGHT_POSE)  # poses[0] = right EE
            msg.poses.append(_LEFT_POSE)   # poses[1] = left EE
            cls._ref_pub.publish(msg)

        cls._ref_timer = cls.node.create_timer(1.0 / 50.0, _publish_reference)

    @classmethod
    def tearDownClass(cls):
        cls._ref_timer.cancel()
        super().tearDownClass()

    def test_robot_does_not_fall(self):
        """Verify robot stays above height threshold and both EE TF frames are reachable."""
        super().test_robot_does_not_fall()

        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer, self.node)  # noqa: F841
        spin_for(self.node, 1.0, "Collecting TF data for EE pose checks...")

        left_ee_tf = None
        right_ee_tf = None
        try:
            left_ee_tf = tf_buffer.lookup_transform(
                REFERENCE_FRAME, LEFT_EE_FRAME, tf2_ros.Time(), timeout=Duration(seconds=1.0)
            )
        except tf2_ros.TransformException:
            pass
        try:
            right_ee_tf = tf_buffer.lookup_transform(
                REFERENCE_FRAME, RIGHT_EE_FRAME, tf2_ros.Time(), timeout=Duration(seconds=1.0)
            )
        except tf2_ros.TransformException:
            pass

        self.assertIsNotNone(
            left_ee_tf, f"No TF transform received for left EE frame '{LEFT_EE_FRAME}'"
        )
        self.assertIsNotNone(
            right_ee_tf, f"No TF transform received for right EE frame '{RIGHT_EE_FRAME}'"
        )

        _assert_ee_pose_within_tolerance(self, left_ee_tf, _LEFT_POSE, LEFT_EE_FRAME)
        _assert_ee_pose_within_tolerance(self, right_ee_tf, _RIGHT_POSE, RIGHT_EE_FRAME)

        self.node.get_logger().info("Test passed - robot stable, both EE poses within tolerance.")


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    """Tests to run after node shutdown."""

    def test_exit_codes(self, proc_info):
        """Verify launch completed."""
        pass
