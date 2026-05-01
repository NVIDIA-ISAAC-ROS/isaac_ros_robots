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

import json
import time

from unitree_api.msg import (
    Request,
    RequestHeader,
    RequestIdentity,
    RequestLease,
    RequestPolicy,
)

# G1 Locomotion API IDs
API_ID_SET_FSM_ID = 7101
API_ID_SET_BALANCE_MODE = 7102
API_ID_SET_VELOCITY = 7105

# Well-known FSM IDs
FSM_ZERO_TORQUE = 0
FSM_DAMP = 1
FSM_SQUAT = 2
FSM_SIT = 3
FSM_STAND_UP = 4
FSM_START = 500

BALANCE_STAND = 0
CONTINUOUS_GAIT = 1


def _make_header(api_id: int) -> RequestHeader:
    """Build a ``RequestHeader`` with a monotonic unique ID."""
    identity = RequestIdentity()
    identity.id = time.monotonic_ns()
    identity.api_id = api_id

    lease = RequestLease()
    lease.id = 0

    policy = RequestPolicy()
    policy.priority = 0
    policy.noreply = False

    header = RequestHeader()
    header.identity = identity
    header.lease = lease
    header.policy = policy
    return header


# ---------------------------------------------------------------------------
# Public builder helpers
# ---------------------------------------------------------------------------

def make_velocity_request(
    vx: float, vy: float, omega: float, duration: float = 0.5,
) -> Request:
    """Create a ``SetVelocity`` request (API 7105)."""
    req = Request()
    req.header = _make_header(API_ID_SET_VELOCITY)
    req.parameter = json.dumps({
        'velocity': [vx, vy, omega],
        'duration': duration,
    })
    return req


def make_fsm_request(fsm_id: int) -> Request:
    """Create a ``SetFsmId`` request (API 7101)."""
    req = Request()
    req.header = _make_header(API_ID_SET_FSM_ID)
    req.parameter = json.dumps({'data': fsm_id})
    return req


def make_balance_mode_request(mode: int) -> Request:
    """Create a ``SetBalanceMode`` request (API 7102)."""
    req = Request()
    req.header = _make_header(API_ID_SET_BALANCE_MODE)
    req.parameter = json.dumps({'data': mode})
    return req
