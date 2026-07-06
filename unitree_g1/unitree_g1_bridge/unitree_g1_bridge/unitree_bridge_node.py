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
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from unitree_api.msg import Request, Response

from . import g1_api


class UnitreeG1BridgeNode(Node):
    """Bridges ``/cmd_vel`` to unitree_api sport requests for G1."""

    def __init__(self):
        super().__init__('unitree_bridge_node')

        # Parameters
        self.declare_parameter('velocity_duration', 0.5)
        self.declare_parameter('standup_fsm_sequence', [
            g1_api.FSM_DAMP,
            g1_api.FSM_LOCK_STAND,
            g1_api.FSM_START,
        ])
        self.declare_parameter('standup_delays', [3.0, 3.0, 3.0])
        self.declare_parameter('balance_mode', g1_api.BALANCE_STAND)
        self.declare_parameter('auto_standup', False)
        self.declare_parameter('auto_standup_delay', 2.0)
        self.declare_parameter('qos_depth', 10)
        self.declare_parameter('require_acks', False)
        self.declare_parameter('step_ack_timeout', 30.0)

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
        auto_standup_delay: float = float(
            self.get_parameter('auto_standup_delay').value
        )
        qos_depth: int = self.get_parameter('qos_depth').value
        self._require_acks: bool = bool(
            self.get_parameter('require_acks').value
        )
        self._step_ack_timeout: float = float(
            self.get_parameter('step_ack_timeout').value
        )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=qos_depth,
        )

        # Publisher
        self._req_pub = self.create_publisher(
            Request, '/api/sport/request', qos,
        )

        # Response correlation. Reentrant group so the callback can fire
        # while a service callback is blocked waiting on the reply.
        self._pending: dict[int, dict] = {}
        self._pending_lock = threading.Lock()
        self._resp_cb_group = ReentrantCallbackGroup()
        self._resp_sub = self.create_subscription(
            Response, '/api/sport/response', self._response_cb, qos,
            callback_group=self._resp_cb_group,
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

        self._auto_standup_timer = None
        if auto_standup:
            self.get_logger().info(
                f'auto_standup enabled, will start in '
                f'{auto_standup_delay:.1f} s'
            )
            # Defer kick until executor is spinning and DDS discovery
            # has converged, else the first request can be dropped.
            self._auto_standup_timer = self.create_timer(
                auto_standup_delay, self._auto_standup_kick,
            )

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

    def _auto_standup_kick(self) -> None:
        """Fire once when auto_standup is enabled, after the startup delay."""
        if self._auto_standup_timer is not None:
            self._auto_standup_timer.cancel()
            self._auto_standup_timer = None

        sub_count = self._req_pub.get_subscription_count()
        if sub_count == 0:
            self.get_logger().warning(
                'auto_standup: no subscribers on /api/sport/request yet — '
                'firmware bridge may not be running. Attempting anyway.'
            )
        else:
            self.get_logger().info(
                f'auto_standup: {sub_count} subscriber(s) on '
                f'/api/sport/request, starting sequence'
            )

        with self._standup_lock:
            if self._standup_in_progress:
                return
            self._standup_in_progress = True

        threading.Thread(
            target=self._run_standup_sequence, daemon=True,
        ).start()

    def _run_standup_sequence(self) -> None:
        """Execute damp -> stand -> start -> balance-walk."""
        try:
            for i, fsm_id in enumerate(self._standup_fsm_seq):
                label = f'SetFsmId({fsm_id})'
                step_num = i + 1
                total = len(self._standup_fsm_seq)
                self.get_logger().info(
                    f'Standup step {step_num}/{total}: {label} ...'
                )
                if not self._send_and_verify(
                    lambda fid=fsm_id: g1_api.make_fsm_request(fid),
                    label=f'step {step_num}/{total} {label}',
                ):
                    self.get_logger().error(
                        f'Aborting standup at step {step_num}/{total}'
                    )
                    return

                if i < len(self._standup_delays):
                    delay = self._standup_delays[i]
                    self.get_logger().debug(
                        f'Waiting {delay:.1f} s before next step'
                    )
                    self._interruptible_sleep(delay)

            bal_label = f'SetBalanceMode({self._balance_mode})'
            self.get_logger().info(f'{bal_label} ...')
            if not self._send_and_verify(
                lambda: g1_api.make_balance_mode_request(self._balance_mode),
                label=bal_label,
            ):
                self.get_logger().error(
                    'Aborting standup at balance-mode step'
                )
                return

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
        """Send damp command (FSM 1) and report whether the robot accepted it."""
        ok, msg = self._send_fsm_with_result(g1_api.FSM_DAMP, 'damp')
        response.success = ok
        response.message = msg
        return response

    # Zero-torque service

    def _zero_torque_cb(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Send zero-torque command (FSM 0) and report acceptance."""
        ok, msg = self._send_fsm_with_result(
            g1_api.FSM_ZERO_TORQUE, 'zero-torque',
        )
        response.success = ok
        response.message = msg
        return response

    # Helpers

    def _response_cb(self, msg: Response) -> None:
        """Match an incoming Response to a pending request and unblock it."""
        rid = msg.header.identity.id
        with self._pending_lock:
            slot = self._pending.get(rid)
            if slot is None:
                return
            slot['code'] = msg.header.status.code
            slot['data'] = msg.data
            slot['event'].set()

    def _publish_and_wait(
        self, req: Request, timeout: float | None = None,
    ) -> dict | None:
        """
        Publish ``req`` and block until the matching response arrives.

        Returns the response slot ({'code', 'data'}), or ``None`` on timeout.
        ``timeout`` defaults to ``self._step_ack_timeout`` when ``None``.
        """
        if timeout is None:
            timeout = self._step_ack_timeout
        rid = req.header.identity.id
        slot = {'event': threading.Event(), 'code': None, 'data': None}
        with self._pending_lock:
            self._pending[rid] = slot
        self._req_pub.publish(req)
        got = slot['event'].wait(timeout)
        with self._pending_lock:
            self._pending.pop(rid, None)
        return slot if got else None

    def _send_and_verify(self, make_req, label: str) -> bool:
        """Send a standup step; wait for ack if require_acks, else fire-and-forget."""
        if not self._require_acks:
            self._req_pub.publish(make_req())
            self.get_logger().info(f'{label}: sent')
            return True

        result = self._publish_and_wait(make_req(), self._step_ack_timeout)
        if result is None:
            self.get_logger().error(
                f'{label}: no response within '
                f'{self._step_ack_timeout:.1f} s'
            )
            return False
        if result['code'] != 0:
            self.get_logger().error(
                f'{label}: rejected '
                f'(code={result["code"]}, data={result["data"]!r})'
            )
            return False
        self.get_logger().info(f'{label}: accepted')
        return True

    def _send_fsm_with_result(
        self, fsm_id: int, name: str,
    ) -> tuple[bool, str]:
        """Send a one-shot SetFsmId and return ``(success, message)``."""
        label = f'{name} (SetFsmId({fsm_id}))'
        if not self._require_acks:
            self._req_pub.publish(g1_api.make_fsm_request(fsm_id))
            sent_msg = f'{label}: sent'
            self.get_logger().info(sent_msg)
            return True, sent_msg

        result = self._publish_and_wait(
            g1_api.make_fsm_request(fsm_id), self._step_ack_timeout,
        )
        if result is None:
            err = (
                f'{label}: no response within '
                f'{self._step_ack_timeout:.1f} s'
            )
            self.get_logger().error(err)
            return False, err
        if result['code'] != 0:
            err = (
                f'{label}: rejected '
                f'(code={result["code"]}, data={result["data"]!r})'
            )
            self.get_logger().error(err)
            return False, err
        ok_msg = f'{label}: accepted'
        self.get_logger().info(ok_msg)
        return True, ok_msg

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
    # MultiThreadedExecutor lets the reentrant response subscription fire
    # while a service callback (~/damp, ~/zero_torque) is blocked waiting
    # on the matching /api/sport/response reply.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
