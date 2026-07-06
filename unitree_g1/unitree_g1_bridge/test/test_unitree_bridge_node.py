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

"""Unit tests for UnitreeG1BridgeNode with mocked publishers."""

import json
import unittest
from unittest.mock import MagicMock, patch

from geometry_msgs.msg import Twist
import rclpy
from std_srvs.srv import Trigger
from unitree_g1_bridge import g1_api
from unitree_g1_bridge.unitree_bridge_node import UnitreeG1BridgeNode


class TestUnitreeG1BridgeNode(unittest.TestCase):
    """Node-level tests that mock the publisher to avoid hardware."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = UnitreeG1BridgeNode()
        self.node._req_pub = MagicMock()
        # Exercise the ack-verification paths in tests.
        self.node._require_acks = True

        # Bypass /api/sport/response wait: publish via the mock and pretend
        # the firmware acknowledged immediately with code=0.
        def fake_publish_and_wait(req, timeout=None):
            self.node._req_pub.publish(req)
            return {'code': 0, 'data': ''}

        self.node._publish_and_wait = fake_publish_and_wait

    def tearDown(self):
        self.node.destroy_node()

    # -- cmd_vel tests --------------------------------------------------------

    def test_cmd_vel_publishes_velocity_request(self):
        """Verify that a Twist callback publishes a SetVelocity request."""
        twist = Twist()
        twist.linear.x = 0.3
        twist.linear.y = -0.1
        twist.angular.z = 0.5

        self.node._cmd_vel_cb(twist)

        self.node._req_pub.publish.assert_called_once()
        req = self.node._req_pub.publish.call_args[0][0]
        assert req.header.identity.api_id == g1_api.API_ID_SET_VELOCITY
        params = json.loads(req.parameter)
        assert params['velocity'] == [0.3, -0.1, 0.5]

    def test_cmd_vel_uses_configured_duration(self):
        """Verify the node's velocity_duration parameter is forwarded."""
        self.node._velocity_duration = 2.5

        twist = Twist()
        self.node._cmd_vel_cb(twist)

        req = self.node._req_pub.publish.call_args[0][0]
        params = json.loads(req.parameter)
        assert params['duration'] == 2.5

    def test_cmd_vel_zero_velocity(self):
        """A zero Twist should still publish a valid velocity request."""
        twist = Twist()
        self.node._cmd_vel_cb(twist)

        self.node._req_pub.publish.assert_called_once()
        req = self.node._req_pub.publish.call_args[0][0]
        params = json.loads(req.parameter)
        assert params['velocity'] == [0.0, 0.0, 0.0]

    # -- damp service tests ---------------------------------------------------

    def test_damp_service_publishes_fsm_damp(self):
        """~/damp should publish a SetFsmId(1) request."""
        request = Trigger.Request()
        response = Trigger.Response()

        self.node._damp_cb(request, response)

        self.node._req_pub.publish.assert_called_once()
        req = self.node._req_pub.publish.call_args[0][0]
        assert req.header.identity.api_id == g1_api.API_ID_SET_FSM_ID
        params = json.loads(req.parameter)
        assert params['data'] == g1_api.FSM_DAMP

    def test_damp_service_response(self):
        """~/damp should return success=True when the firmware acks."""
        request = Trigger.Request()
        response = Trigger.Response()

        result = self.node._damp_cb(request, response)

        assert result.success is True
        assert 'damp' in result.message.lower()

    def test_damp_service_reports_failure_on_rejection(self):
        """~/damp should return success=False if the firmware rejects."""
        self.node._publish_and_wait = (
            lambda req, timeout=None: {'code': 5, 'data': 'busy'}
        )
        request = Trigger.Request()
        response = Trigger.Response()

        result = self.node._damp_cb(request, response)

        assert result.success is False
        assert 'rejected' in result.message
        assert 'code=5' in result.message

    def test_damp_service_reports_failure_on_timeout(self):
        """~/damp should return success=False if no response arrives."""
        self.node._publish_and_wait = lambda req, timeout=None: None
        request = Trigger.Request()
        response = Trigger.Response()

        result = self.node._damp_cb(request, response)

        assert result.success is False
        assert 'no response' in result.message

    # -- standup service tests ------------------------------------------------

    def test_standup_rejects_concurrent(self):
        """If a standup is already running, the service should reject."""
        self.node._standup_in_progress = True
        request = Trigger.Request()
        response = Trigger.Response()

        result = self.node._standup_cb(request, response)

        assert result.success is False
        assert 'already in progress' in result.message

    def test_standup_accepts_when_idle(self):
        """When no standup is running, the service should accept."""
        self.node._standup_in_progress = False
        request = Trigger.Request()
        response = Trigger.Response()

        with patch.object(self.node, '_run_standup_sequence'):
            result = self.node._standup_cb(request, response)

        assert result.success is True

    @patch.object(UnitreeG1BridgeNode, '_interruptible_sleep')
    def test_standup_sequence_publishes_correct_messages(self, mock_sleep):
        """Run the standup sequence synchronously and check published msgs."""
        mock_sleep.return_value = None

        self.node._run_standup_sequence()

        calls = self.node._req_pub.publish.call_args_list
        # Default sequence: damp(1), lock_stand(4), start(200), then
        # SetBalanceMode(0)
        assert len(calls) == 4

        fsm_damp = calls[0][0][0]
        assert fsm_damp.header.identity.api_id == g1_api.API_ID_SET_FSM_ID
        assert json.loads(fsm_damp.parameter)['data'] == g1_api.FSM_DAMP

        fsm_lock = calls[1][0][0]
        assert fsm_lock.header.identity.api_id == g1_api.API_ID_SET_FSM_ID
        assert json.loads(fsm_lock.parameter)['data'] == g1_api.FSM_LOCK_STAND

        fsm_start = calls[2][0][0]
        assert fsm_start.header.identity.api_id == g1_api.API_ID_SET_FSM_ID
        assert json.loads(fsm_start.parameter)['data'] == g1_api.FSM_START

        balance = calls[3][0][0]
        assert balance.header.identity.api_id == g1_api.API_ID_SET_BALANCE_MODE
        assert json.loads(balance.parameter)['data'] == g1_api.BALANCE_STAND

    @patch.object(UnitreeG1BridgeNode, '_interruptible_sleep')
    def test_standup_aborts_on_rejection(self, mock_sleep):
        """If a step is rejected, no later steps should be published."""
        mock_sleep.return_value = None

        # Accept the first two steps, then reject the third.
        replies = [
            {'code': 0, 'data': ''},
            {'code': 0, 'data': ''},
            {'code': 7, 'data': 'denied'},
        ]

        def fake(req, timeout=None):
            self.node._req_pub.publish(req)
            return replies.pop(0)

        self.node._publish_and_wait = fake

        self.node._run_standup_sequence()

        # Only the first three FSM requests were published; balance + later
        # steps must not have been sent.
        calls = self.node._req_pub.publish.call_args_list
        assert len(calls) == 3
        assert self.node._standup_in_progress is False

    @patch.object(UnitreeG1BridgeNode, '_interruptible_sleep')
    def test_standup_aborts_on_timeout(self, mock_sleep):
        """If a step receives no response, the sequence should abort."""
        mock_sleep.return_value = None

        def fake(req, timeout=None):
            self.node._req_pub.publish(req)
            return None  # simulate timeout

        self.node._publish_and_wait = fake

        # Single attempt, then abort — no later steps should be sent.
        self.node._run_standup_sequence()

        calls = self.node._req_pub.publish.call_args_list
        assert len(calls) == 1
        assert self.node._standup_in_progress is False

    # -- response routing -----------------------------------------------------

    def test_response_callback_wakes_matching_request(self):
        """Inbound Response should populate the pending slot keyed by id."""
        import threading

        from unitree_api.msg import Response

        slot = {'event': threading.Event(), 'code': None, 'data': None}
        self.node._pending[42] = slot

        msg = Response()
        msg.header.identity.id = 42
        msg.header.status.code = 0
        msg.data = 'ok'

        self.node._response_cb(msg)

        assert slot['event'].is_set()
        assert slot['code'] == 0
        assert slot['data'] == 'ok'

    def test_response_callback_ignores_unknown_id(self):
        """Responses with no matching pending request should be no-ops."""
        from unitree_api.msg import Response

        msg = Response()
        msg.header.identity.id = 999
        msg.header.status.code = 0

        # No pending entry registered — should simply not raise.
        self.node._response_cb(msg)


if __name__ == '__main__':
    unittest.main()
