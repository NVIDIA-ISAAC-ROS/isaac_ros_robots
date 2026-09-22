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
#include "unitree_hg_ros2_control/joint_index_tables.hpp"

#include "unitree/dds_wrapper/robots/g1/defines.h"

namespace unitree_hg_ros2_control
{

const JointIndexTable & g1_body_joint_index_table()
{
  namespace g1 = unitree::robot::g1;
  static const JointIndexTable table{
    {"left_hip_pitch_joint", g1::LeftHipPitch},
    {"left_hip_roll_joint", g1::LeftHipRoll},
    {"left_hip_yaw_joint", g1::LeftHipYaw},
    {"left_knee_joint", g1::LeftKnee},
    {"left_ankle_pitch_joint", g1::LeftAnklePitch},
    {"left_ankle_roll_joint", g1::LeftAnkleRoll},
    {"right_hip_pitch_joint", g1::RightHipPitch},
    {"right_hip_roll_joint", g1::RightHipRoll},
    {"right_hip_yaw_joint", g1::RightHipYaw},
    {"right_knee_joint", g1::RightKnee},
    {"right_ankle_pitch_joint", g1::RightAnklePitch},
    {"right_ankle_roll_joint", g1::RightAnkleRoll},
    {"waist_yaw_joint", g1::WaistYaw},
    {"waist_roll_joint", g1::WaistRoll},
    {"waist_pitch_joint", g1::WaistPitch},
    {"left_shoulder_pitch_joint", g1::LeftShoulderPitch},
    {"left_shoulder_roll_joint", g1::LeftShoulderRoll},
    {"left_shoulder_yaw_joint", g1::LeftShoulderYaw},
    {"left_elbow_joint", g1::LeftElbow},
    {"left_wrist_roll_joint", g1::LeftWristRoll},
    {"left_wrist_pitch_joint", g1::LeftWristPitch},
    {"left_wrist_yaw_joint", g1::LeftWristYaw},
    {"right_shoulder_pitch_joint", g1::RightShoulderPitch},
    {"right_shoulder_roll_joint", g1::RightShoulderRoll},
    {"right_shoulder_yaw_joint", g1::RightShoulderYaw},
    {"right_elbow_joint", g1::RightElbow},
    {"right_wrist_roll_joint", g1::RightWristRoll},
    {"right_wrist_pitch_joint", g1::RightWristPitch},
    {"right_wrist_yaw_joint", g1::RightWristYaw},
  };
  return table;
}

// H2 has no includable SDK enum for its joint ordering. This table is a
// literal transcription of the `H2JointIndex` enum from the official SDK2
// example `example/h2/low_level/h2_ankle_swing_example.cpp`
// (left leg [0..5] -> right leg [6..11] -> waist [12..14] ->
// left arm [15..21] -> right arm [22..28]), plus the two head joints
// [29..30] which are not part of that example.
// Note H2's ankle ordering (roll=4, pitch=5) is the OPPOSITE of G1's
// (pitch=4, roll=5).
const JointIndexTable & h2_body_joint_index_table()
{
  static const JointIndexTable table{
    {"left_hip_pitch_joint", 0},
    {"left_hip_roll_joint", 1},
    {"left_hip_yaw_joint", 2},
    {"left_knee_joint", 3},
    {"left_ankle_roll_joint", 4},
    {"left_ankle_pitch_joint", 5},
    {"right_hip_pitch_joint", 6},
    {"right_hip_roll_joint", 7},
    {"right_hip_yaw_joint", 8},
    {"right_knee_joint", 9},
    {"right_ankle_roll_joint", 10},
    {"right_ankle_pitch_joint", 11},
    {"waist_yaw_joint", 12},
    {"waist_roll_joint", 13},
    {"waist_pitch_joint", 14},
    {"left_shoulder_pitch_joint", 15},
    {"left_shoulder_roll_joint", 16},
    {"left_shoulder_yaw_joint", 17},
    {"left_elbow_joint", 18},
    {"left_wrist_roll_joint", 19},
    {"left_wrist_pitch_joint", 20},
    {"left_wrist_yaw_joint", 21},
    {"right_shoulder_pitch_joint", 22},
    {"right_shoulder_roll_joint", 23},
    {"right_shoulder_yaw_joint", 24},
    {"right_elbow_joint", 25},
    {"right_wrist_roll_joint", 26},
    {"right_wrist_pitch_joint", 27},
    {"right_wrist_yaw_joint", 28},
    {"head_pitch_joint", 29},
    {"head_yaw_joint", 30},
  };
  return table;
}

const JointIndexTable & dex3_hand_joint_index_table()
{
  static const JointIndexTable table{
    {"left_hand_thumb_0_joint", 0},
    {"left_hand_thumb_1_joint", 1},
    {"left_hand_thumb_2_joint", 2},
    {"left_hand_middle_0_joint", 3},
    {"left_hand_middle_1_joint", 4},
    {"left_hand_index_0_joint", 5},
    {"left_hand_index_1_joint", 6},
    {"right_hand_thumb_0_joint", 0},
    {"right_hand_thumb_1_joint", 1},
    {"right_hand_thumb_2_joint", 2},
    {"right_hand_middle_0_joint", 3},
    {"right_hand_middle_1_joint", 4},
    {"right_hand_index_0_joint", 5},
    {"right_hand_index_1_joint", 6},
  };
  return table;
}

const JointIndexTable & get_body_joint_index_table(const std::string & variant, bool & ok)
{
  if (variant == "g1") {
    ok = true;
    return g1_body_joint_index_table();
  }
  if (variant == "h2") {
    ok = true;
    return h2_body_joint_index_table();
  }
  ok = false;
  return g1_body_joint_index_table();
}

}  // namespace unitree_hg_ros2_control
