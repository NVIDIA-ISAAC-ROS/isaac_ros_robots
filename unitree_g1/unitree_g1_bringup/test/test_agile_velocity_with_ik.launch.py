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

"""Integration test for agile_velocity_with_ik policy with Unitree G1 in MuJoCo simulation.

This test verifies that:
  - The IK controller group launches and its controllers become active.
  - Valid TF transforms are published for both end-effector frames
    (left_hand_palm_link and right_hand_palm_link).
  - The end-effectors track their commanded target poses within tolerance.
"""

import math
from pathlib import Path
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseArray
from geometry_msgs.msg import Twist
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
from mujoco_test_helpers import (
    disable_gantry,
    has_display,
    monitor_height,
    spin_for,
    wait_for_controllers,
    wait_for_is_active,
)
from rcl_interfaces.msg import Parameter as RclParameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue as RclParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
import rclpy
from rclpy.duration import Duration
import rclpy.parameter
from rclpy.time import Time
from scipy.spatial.transform import Rotation, Slerp
import tf2_ros

LEFT_EE_FRAME = "left_hand_palm_link"
RIGHT_EE_FRAME = "right_hand_palm_link"
REFERENCE_FRAME = "pelvis"
MAX_POSITION_ERROR_M = 0.1
MAX_ORIENTATION_ERROR_DEG = 15.0
IK_TRACKING_DURATION_S = 8.0
IK_TARGET_RAMP_DURATION_S = 4.0
BLEND_RATIO_RAMP_WAIT_S = 1.0
GANTRY_HOLD_STABILIZATION_S = 2.0
GANTRY_RELEASE_SETTLE_S = 0.5
IK_COMMAND_WAIT_AFTER_GANTRY_RELEASE_S = 0.5
SAFETY_CONTROLLER = "safety_controller"

# Feasible arm targets in the pelvis frame.
# Right arm: forward 0.35m, 0.2m to the right, 0.1m above pelvis, 40deg yaw.
# Left arm:  forward 0.35m, 0.2m to the left,  0.1m above pelvis, 30deg pitch.
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


def _assert_transform_is_finite(test_case, tf, frame_name):
    """Assert that a TF transform contains finite values."""
    t = tf.transform.translation
    q = tf.transform.rotation
    values = [t.x, t.y, t.z, q.x, q.y, q.z, q.w]
    test_case.assertTrue(
        all(math.isfinite(value) for value in values),
        f"Non-finite TF transform received for '{frame_name}'",
    )


def _assert_ee_pose_within_tolerance(test_case, tf, ref_pose, frame_name):
    """Assert that a TF transform is within position and orientation tolerances of ref_pose."""
    t = tf.transform.translation
    ref_position = ref_pose.position
    pos_error = math.sqrt(
        (t.x - ref_position.x) ** 2
        + (t.y - ref_position.y) ** 2
        + (t.z - ref_position.z) ** 2
    )
    test_case.assertLessEqual(
        pos_error,
        MAX_POSITION_ERROR_M,
        f"{frame_name} position error {pos_error:.3f}m exceeds {MAX_POSITION_ERROR_M}m "
        f"(actual=({t.x:.3f}, {t.y:.3f}, {t.z:.3f}), "
        f"target=({ref_position.x:.3f}, {ref_position.y:.3f}, {ref_position.z:.3f}))",
    )

    actual_quat = tf.transform.rotation
    actual_rotation = Rotation.from_quat(
        [actual_quat.x, actual_quat.y, actual_quat.z, actual_quat.w]
    )
    ref_quat = ref_pose.orientation
    ref_rotation = Rotation.from_quat([ref_quat.x, ref_quat.y, ref_quat.z, ref_quat.w])
    angle_error_deg = math.degrees((actual_rotation * ref_rotation.inv()).magnitude())
    test_case.assertLessEqual(
        angle_error_deg,
        MAX_ORIENTATION_ERROR_DEG,
        f"{frame_name} orientation error {angle_error_deg:.1f}deg "
        f"exceeds {MAX_ORIENTATION_ERROR_DEG}deg "
        f"(actual=({actual_quat.x:.3f}, {actual_quat.y:.3f}, "
        f"{actual_quat.z:.3f}, {actual_quat.w:.3f}), "
        f"target=({ref_quat.x:.3f}, {ref_quat.y:.3f}, {ref_quat.z:.3f}, "
        f"{ref_quat.w:.3f}))",
    )


def _pose_from_transform(tf):
    """Convert a TransformStamped into a Pose in the transform parent frame."""
    pose = Pose()
    pose.position.x = tf.transform.translation.x
    pose.position.y = tf.transform.translation.y
    pose.position.z = tf.transform.translation.z
    pose.orientation.x = tf.transform.rotation.x
    pose.orientation.y = tf.transform.rotation.y
    pose.orientation.z = tf.transform.rotation.z
    pose.orientation.w = tf.transform.rotation.w
    return pose


def _wait_for_controller_double_parameter(
    test_case,
    node,
    controller_name,
    parameter_name,
    expected_value,
    timeout_s=10.0,
    tolerance=1e-6,
):
    """Wait until a controller double parameter reaches an expected value."""
    client = node.create_client(GetParameters, f"/{controller_name}/get_parameters")
    test_case.assertTrue(
        client.wait_for_service(timeout_sec=5.0),
        f"Parameter service for '{controller_name}' not available",
    )

    request = GetParameters.Request()
    request.names = [parameter_name]
    start = time.time()
    last_value = None

    while time.time() - start < timeout_s:
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
        result = future.result()
        if result is not None and result.values:
            value = result.values[0]
            if value.type == ParameterType.PARAMETER_DOUBLE:
                last_value = value.double_value
                if abs(last_value - expected_value) <= tolerance:
                    node.destroy_client(client)
                    return
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_client(client)
    test_case.fail(
        f"'{controller_name}.{parameter_name}' did not reach {expected_value} "
        f"within {timeout_s}s; last value was {last_value}"
    )


def _set_controller_double_parameter(
    test_case,
    node,
    controller_name,
    parameter_name,
    value,
    timeout_s=10.0,
):
    """Set a controller double parameter and fail the test if the request fails."""
    client = node.create_client(SetParameters, f"/{controller_name}/set_parameters")
    test_case.assertTrue(
        client.wait_for_service(timeout_sec=5.0),
        f"Parameter service for '{controller_name}' not available",
    )

    parameter = RclParameter()
    parameter.name = parameter_name
    parameter.value = RclParameterValue(
        type=rclpy.Parameter.Type.DOUBLE.value,
        double_value=value,
    )
    request = SetParameters.Request()
    request.parameters = [parameter]

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s)
    response = future.result()
    node.destroy_client(client)

    test_case.assertIsNotNone(
        response, f"Setting '{controller_name}.{parameter_name}' timed out"
    )
    for result in response.results:
        test_case.assertTrue(
            result.successful,
            f"Setting '{controller_name}.{parameter_name}' failed: {result.reason}",
        )


def _interpolate_pose(start_pose, target_pose, alpha):
    """Linearly interpolate position and slerp orientation."""
    clamped_alpha = max(0.0, min(1.0, alpha))
    pose = Pose()
    pose.position.x = (
        start_pose.position.x
        + clamped_alpha * (target_pose.position.x - start_pose.position.x)
    )
    pose.position.y = (
        start_pose.position.y
        + clamped_alpha * (target_pose.position.y - start_pose.position.y)
    )
    pose.position.z = (
        start_pose.position.z
        + clamped_alpha * (target_pose.position.z - start_pose.position.z)
    )

    rotations = Rotation.from_quat(
        [
            [
                start_pose.orientation.x,
                start_pose.orientation.y,
                start_pose.orientation.z,
                start_pose.orientation.w,
            ],
            [
                target_pose.orientation.x,
                target_pose.orientation.y,
                target_pose.orientation.z,
                target_pose.orientation.w,
            ],
        ]
    )
    interpolated = Slerp([0.0, 1.0], rotations)([clamped_alpha])[0]
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (
        interpolated.as_quat()
    )
    return pose


def generate_test_description():
    """Generate launch description for testing agile_velocity_with_ik."""
    bringup_share = Path(get_package_share_directory("unitree_g1_bringup"))
    main_launch_file = bringup_share / "launch/unitree_g1_controller_manager.launch.py"

    main_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(main_launch_file)),
        launch_arguments={
            "initial_controller_group": "agile_velocity_with_ik",
            "auto_start_safety_blend_ratio": "false",
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


class TestAgileVelocityWithIK(unittest.TestCase):
    """Test that agile_velocity_with_ik launches, activates, and tracks EE targets."""

    CONTROLLERS = [
        "inference_controller", "safety_controller", "ik_controller",
        "fingers_forward_joint_command_controller",
    ]
    # AGILE defers its core until first /cmd_vel arrives; wait on the latched
    # is_active signal so the policy is actually producing commands.
    READY_CONTROLLERS = ["inference_controller"]
    CONTROLLER_STARTUP_WAIT_S = 60.0
    CMD_VEL_PUBLISH_PERIOD_S = 0.05

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node(
            "test_agile_velocity_with_ik_node",
            parameter_overrides=[
                rclpy.parameter.Parameter("use_sim_time", value=True),
            ],
        )
        cls.cmd_vel_pub = cls.node.create_publisher(Twist, "/cmd_vel", 10)
        cls.publish_timer = cls.node.create_timer(
            cls.CMD_VEL_PUBLISH_PERIOD_S,
            lambda: cls.cmd_vel_pub.publish(Twist()),
        )
        cls._ref_pub = cls.node.create_publisher(
            PoseArray, "/ik_controller/reference_pose", 10
        )
        cls._ref_timer = None
        cls._ref_start_time = None
        cls._left_start_pose = None
        cls._right_start_pose = None

    @classmethod
    def tearDownClass(cls):
        if cls._ref_timer is not None:
            cls._ref_timer.cancel()
        cls.publish_timer.cancel()
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _publish_reference_pose(cls):
        if (
            cls._ref_start_time is None
            or cls._left_start_pose is None
            or cls._right_start_pose is None
        ):
            return
        elapsed = (cls.node.get_clock().now() - cls._ref_start_time).nanoseconds / 1e9
        alpha = elapsed / IK_TARGET_RAMP_DURATION_S

        msg = PoseArray()
        msg.header.stamp = cls.node.get_clock().now().to_msg()
        msg.header.frame_id = REFERENCE_FRAME
        msg.poses.append(_interpolate_pose(cls._left_start_pose, _LEFT_POSE, alpha))
        msg.poses.append(_interpolate_pose(cls._right_start_pose, _RIGHT_POSE, alpha))
        cls._ref_pub.publish(msg)

    @classmethod
    def _start_reference_publishing(cls, left_start_pose, right_start_pose):
        cls._left_start_pose = left_start_pose
        cls._right_start_pose = right_start_pose
        cls._ref_start_time = cls.node.get_clock().now()
        if cls._ref_timer is None:
            cls._publish_reference_pose()
            cls._ref_timer = cls.node.create_timer(
                1.0 / 50.0, cls._publish_reference_pose
            )

    def _lookup_ee_transform(self, tf_buffer, frame_name):
        try:
            return tf_buffer.lookup_transform(
                REFERENCE_FRAME, frame_name, Time(), timeout=Duration(seconds=1.0)
            )
        except tf2_ros.TransformException:
            return None

    def test_ik_tracks_target_poses(self):
        """Verify IK controllers track commanded EE targets after the test run."""
        wait_for_controllers(
            self, self.node, self.CONTROLLERS, self.CONTROLLER_STARTUP_WAIT_S
        )
        wait_for_is_active(
            self, self.node, self.READY_CONTROLLERS, self.CONTROLLER_STARTUP_WAIT_S
        )

        self.node.get_logger().info("Ramping safety blend before releasing gantry...")
        _set_controller_double_parameter(
            self, self.node, SAFETY_CONTROLLER, "blend_ratio", 1.0
        )
        _wait_for_controller_double_parameter(
            self, self.node, SAFETY_CONTROLLER, "blend_ratio", 1.0
        )
        spin_for(
            self.node,
            BLEND_RATIO_RAMP_WAIT_S,
            f"Waiting {BLEND_RATIO_RAMP_WAIT_S}s for safety blend ramp...",
        )
        spin_for(
            self.node,
            GANTRY_HOLD_STABILIZATION_S,
            f"Waiting {GANTRY_HOLD_STABILIZATION_S}s before releasing gantry...",
        )
        self.node.get_logger().info("Releasing gantry after safety blend ramp...")
        self.assertTrue(disable_gantry(self.node))
        spin_for(
            self.node,
            GANTRY_RELEASE_SETTLE_S,
            f"Waiting {GANTRY_RELEASE_SETTLE_S}s after gantry release...",
        )
        spin_for(
            self.node,
            IK_COMMAND_WAIT_AFTER_GANTRY_RELEASE_S,
            f"Waiting {IK_COMMAND_WAIT_AFTER_GANTRY_RELEASE_S}s before commanding IK target...",
        )

        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer, self.node)  # noqa: F841
        spin_for(self.node, 1.0, "Collecting initial TF data for IK ramp...")
        left_start_tf = self._lookup_ee_transform(tf_buffer, LEFT_EE_FRAME)
        right_start_tf = self._lookup_ee_transform(tf_buffer, RIGHT_EE_FRAME)
        self.assertIsNotNone(
            left_start_tf, f"No initial TF transform for left EE frame '{LEFT_EE_FRAME}'"
        )
        self.assertIsNotNone(
            right_start_tf, f"No initial TF transform for right EE frame '{RIGHT_EE_FRAME}'"
        )

        self._start_reference_publishing(
            _pose_from_transform(left_start_tf),
            _pose_from_transform(right_start_tf),
        )
        # Track IK targets while asserting the robot stays standing: the IK
        # reference timer keeps publishing during the spin, so this monitors the
        # post-release window and fails immediately on a collapse (rather than
        # letting a fall pass silently behind the EE-pose check below).
        monitor_height(
            self,
            self.node,
            IK_TRACKING_DURATION_S,
            root_frame=REFERENCE_FRAME,
            fail_early=True,
        )

        spin_for(self.node, 1.0, "Collecting TF data for EE pose checks...")

        left_ee_tf = self._lookup_ee_transform(tf_buffer, LEFT_EE_FRAME)
        right_ee_tf = self._lookup_ee_transform(tf_buffer, RIGHT_EE_FRAME)
        self.assertIsNotNone(
            left_ee_tf, f"No TF transform received for left EE frame '{LEFT_EE_FRAME}'"
        )
        self.assertIsNotNone(
            right_ee_tf, f"No TF transform received for right EE frame '{RIGHT_EE_FRAME}'"
        )
        _assert_transform_is_finite(self, left_ee_tf, LEFT_EE_FRAME)
        _assert_transform_is_finite(self, right_ee_tf, RIGHT_EE_FRAME)
        _assert_ee_pose_within_tolerance(self, left_ee_tf, _LEFT_POSE, LEFT_EE_FRAME)
        _assert_ee_pose_within_tolerance(self, right_ee_tf, _RIGHT_POSE, RIGHT_EE_FRAME)

        self.node.get_logger().info("Test passed - both EE poses within tolerance.")


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    """Tests to run after node shutdown."""

    def test_exit_codes(self, proc_info):
        """Verify launch completed."""
        pass
