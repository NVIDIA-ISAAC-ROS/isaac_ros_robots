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

import threading
import traceback

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from unitree_api.msg import Request

from . import g1_api


class UnitreeG1BridgeNode(Node):
    """Bridges ``/cmd_vel`` to unitree_api sport requests for G1."""

    def __init__(self):
        super().__init__('unitree_bridge_node')

        # Parameters
        self.declare_parameter('velocity_duration', 0.5)
        self.declare_parameter('standup_fsm_sequence', [
            g1_api.FSM_DAMP,
            g1_api.FSM_STAND_UP,
            g1_api.FSM_START,
        ])
        self.declare_parameter('standup_delays', [3.0, 6.0, 3.0])
        self.declare_parameter('balance_mode', g1_api.BALANCE_STAND)
        self.declare_parameter('auto_standup', False)
        self.declare_parameter('qos_depth', 10)

        self._velocity_duration = (
            self.get_parameter('velocity_duration').value
        )
        self._standup_fsm_seq: list[int] = list(
            self.get_parameter('standup_fsm_sequence').value
        )
        self._standup_delays: list[float] = list(
            self.get_parameter('standup_delays').value
        )
        self._balance_mode: int = (
            self.get_parameter('balance_mode').value
        )
        auto_standup: bool = self.get_parameter('auto_standup').value
        qos_depth: int = self.get_parameter('qos_depth').value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=qos_depth,
        )

        # Publisher
        self._req_pub = self.create_publisher(
            Request, '/api/sport/request', qos,
        )

        # Subscriber
        self._cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self._cmd_vel_cb, qos,
        )

        # Services
        self._standup_srv = self.create_service(
            Trigger, '~/standup', self._standup_cb,
        )
        self._damp_srv = self.create_service(
            Trigger, '~/damp', self._damp_cb,
        )
        self._zero_torque_srv = self.create_service(
            Trigger, '~/zero_torque', self._zero_torque_cb,
        )

        # Guard against concurrent standup sequences
        self._standup_lock = threading.Lock()
        self._standup_in_progress = False

        self.get_logger().info('unitree_bridge_node started')

        if auto_standup:
            self.get_logger().info('auto_standup enabled, executing sequence')
            with self._standup_lock:
                self._standup_in_progress = True
            threading.Thread(
                target=self._run_standup_sequence, daemon=True,
            ).start()

    # /cmd_vel callback

    def _cmd_vel_cb(self, msg: Twist) -> None:
        """Convert a Twist into a G1 SetVelocity request and publish."""
        req = g1_api.make_velocity_request(
            vx=msg.linear.x,
            vy=msg.linear.y,
            omega=msg.angular.z,
            duration=self._velocity_duration,
        )
        self._req_pub.publish(req)

    # Standup service

    def _standup_cb(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Handle ~/standup service calls."""
        with self._standup_lock:
            if self._standup_in_progress:
                response.success = False
                response.message = 'Standup sequence already in progress'
                return response
            self._standup_in_progress = True

        threading.Thread(
            target=self._run_standup_sequence, daemon=True,
        ).start()

        response.success = True
        response.message = 'Standup sequence started'
        return response

    def _run_standup_sequence(self) -> None:
        """Execute damp -> stand -> start -> balance-walk."""
        try:
            for i, fsm_id in enumerate(self._standup_fsm_seq):
                step_num = i + 1
                total = len(self._standup_fsm_seq)
                self.get_logger().info(
                    f'Standup step {step_num}/{total}: SetFsmId({fsm_id})'
                )
                self._req_pub.publish(g1_api.make_fsm_request(fsm_id))

                if i < len(self._standup_delays):
                    delay = self._standup_delays[i]
                    self.get_logger().debug(
                        f'Waiting {delay:.1f} s before next step'
                    )
                    self._interruptible_sleep(delay)

            self.get_logger().info(
                f'Setting balance mode to {self._balance_mode}'
            )
            self._req_pub.publish(
                g1_api.make_balance_mode_request(self._balance_mode),
            )
            self.get_logger().info('Standup sequence complete')
        except Exception:
            self.get_logger().error(
                f'Error during standup sequence:\n{traceback.format_exc()}'
            )
        finally:
            with self._standup_lock:
                self._standup_in_progress = False

    # Damp service

    def _damp_cb(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Immediately send damp command (FSM 1)."""
        self._req_pub.publish(g1_api.make_fsm_request(g1_api.FSM_DAMP))
        response.success = True
        response.message = 'Damp command sent'
        self.get_logger().info('Damp command sent via ~/damp service')
        return response

    # Zero-torque service

    def _zero_torque_cb(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Immediately send zero-torque command (FSM 0)."""
        self._req_pub.publish(g1_api.make_fsm_request(g1_api.FSM_ZERO_TORQUE))
        response.success = True
        response.message = 'Zero-torque command sent'
        self.get_logger().info('Zero-torque command sent via ~/zero_torque service')
        return response

    # Helpers

    def _interruptible_sleep(self, seconds: float) -> None:
        """Sleep in small increments so the node can still shut down."""
        remaining = seconds
        while remaining > 0 and rclpy.ok():
            step = min(remaining, 0.25)
            self.get_clock().sleep_for(
                rclpy.duration.Duration(seconds=step),
            )
            remaining -= step


def main(args=None):
    """Entry point for the unitree_bridge_node."""
    rclpy.init(args=args)
    node = UnitreeG1BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
