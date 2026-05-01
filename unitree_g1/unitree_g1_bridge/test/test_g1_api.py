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

"""Unit tests for g1_api request builders and constants."""

import json

from unitree_g1_bridge import g1_api


class TestConstants:
    """Verify well-known constant values match the Unitree SDK."""

    def test_fsm_damp(self):
        assert g1_api.FSM_DAMP == 1

    def test_fsm_squat(self):
        assert g1_api.FSM_SQUAT == 2

    def test_fsm_sit(self):
        assert g1_api.FSM_SIT == 3

    def test_fsm_stand_up(self):
        assert g1_api.FSM_STAND_UP == 4

    def test_fsm_start(self):
        assert g1_api.FSM_START == 500

    def test_fsm_zero_torque(self):
        assert g1_api.FSM_ZERO_TORQUE == 0

    def test_balance_stand(self):
        assert g1_api.BALANCE_STAND == 0

    def test_continuous_gait(self):
        assert g1_api.CONTINUOUS_GAIT == 1

    def test_api_id_set_velocity(self):
        assert g1_api.API_ID_SET_VELOCITY == 7105

    def test_api_id_set_fsm_id(self):
        assert g1_api.API_ID_SET_FSM_ID == 7101

    def test_api_id_set_balance_mode(self):
        assert g1_api.API_ID_SET_BALANCE_MODE == 7102


class TestMakeVelocityRequest:
    """Tests for make_velocity_request."""

    def test_api_id(self):
        req = g1_api.make_velocity_request(0.0, 0.0, 0.0)
        assert req.header.identity.api_id == g1_api.API_ID_SET_VELOCITY

    def test_velocity_values(self):
        req = g1_api.make_velocity_request(0.5, -0.2, 0.1)
        params = json.loads(req.parameter)
        assert params['velocity'] == [0.5, -0.2, 0.1]

    def test_default_duration(self):
        req = g1_api.make_velocity_request(0.0, 0.0, 0.0)
        params = json.loads(req.parameter)
        assert params['duration'] == 0.5

    def test_custom_duration(self):
        req = g1_api.make_velocity_request(0.0, 0.0, 0.0, duration=2.0)
        params = json.loads(req.parameter)
        assert params['duration'] == 2.0

    def test_header_fields(self):
        req = g1_api.make_velocity_request(0.0, 0.0, 0.0)
        assert req.header.lease.id == 0
        assert req.header.policy.priority == 0
        assert req.header.policy.noreply is False


class TestMakeFsmRequest:
    """Tests for make_fsm_request."""

    def test_api_id(self):
        req = g1_api.make_fsm_request(g1_api.FSM_DAMP)
        assert req.header.identity.api_id == g1_api.API_ID_SET_FSM_ID

    def test_damp_value(self):
        req = g1_api.make_fsm_request(g1_api.FSM_DAMP)
        params = json.loads(req.parameter)
        assert params['data'] == 1

    def test_stand_up_value(self):
        req = g1_api.make_fsm_request(g1_api.FSM_STAND_UP)
        params = json.loads(req.parameter)
        assert params['data'] == 4

    def test_start_value(self):
        req = g1_api.make_fsm_request(g1_api.FSM_START)
        params = json.loads(req.parameter)
        assert params['data'] == 500


class TestMakeBalanceModeRequest:
    """Tests for make_balance_mode_request."""

    def test_api_id(self):
        req = g1_api.make_balance_mode_request(g1_api.CONTINUOUS_GAIT)
        assert req.header.identity.api_id == g1_api.API_ID_SET_BALANCE_MODE

    def test_continuous_gait(self):
        req = g1_api.make_balance_mode_request(g1_api.CONTINUOUS_GAIT)
        params = json.loads(req.parameter)
        assert params['data'] == 1

    def test_balance_stand(self):
        req = g1_api.make_balance_mode_request(g1_api.BALANCE_STAND)
        params = json.loads(req.parameter)
        assert params['data'] == 0


class TestHeaderUniqueIds:
    """Verify that successive requests get distinct monotonic IDs."""

    def test_ids_are_unique(self):
        req_a = g1_api.make_fsm_request(1)
        req_b = g1_api.make_fsm_request(1)
        assert req_a.header.identity.id != req_b.header.identity.id

    def test_ids_are_monotonic(self):
        req_a = g1_api.make_velocity_request(0.0, 0.0, 0.0)
        req_b = g1_api.make_velocity_request(0.0, 0.0, 0.0)
        assert req_b.header.identity.id > req_a.header.identity.id
