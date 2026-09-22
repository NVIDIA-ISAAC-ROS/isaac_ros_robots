// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <string>
#include <utility>
#include <vector>

namespace unitree_hg_ros2_control
{

/// Ordered list of (URDF joint name, SDK motor index) pairs for a single robot
/// variant or hand.
using JointIndexTable = std::vector<std::pair<std::string, int>>;

/// G1 body joint -> SDK motor index table, built from the SDK's
/// `unitree::robot::g1::JointIndex` enum (29 joints).
const JointIndexTable & g1_body_joint_index_table();

/// H2 body joint -> SDK motor index table (31 joints). H2 has no includable
/// SDK enum, so this table is a literal transcription; see joint_index_tables.cpp.
const JointIndexTable & h2_body_joint_index_table();

/// Dex3 hand joint -> per-hand SDK motor index table (0-6). Shared by both
/// hands: contains both `left_hand_*` and `right_hand_*` names, each mapping
/// to the same 0-6 range as its counterpart on the other hand.
const JointIndexTable & dex3_hand_joint_index_table();

/// Selects the body joint index table for the given robot variant
/// ("g1" or "h2"). On an unknown variant, sets `ok` to false and returns the
/// g1 table (caller must check `ok`).
const JointIndexTable & get_body_joint_index_table(const std::string & variant, bool & ok);

}  // namespace unitree_hg_ros2_control
