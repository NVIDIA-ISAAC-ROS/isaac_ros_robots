// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <vector>
#include "unitree_hg_ros2_control/joint_index_tables.hpp"
#include "unitree_hg_ros2_control/sdk_index_map.hpp"

namespace unitree_hg_ros2_control
{
namespace
{
hardware_interface::ComponentInfo make_joint(const std::string & name)
{
  hardware_interface::ComponentInfo c;
  c.name = name;
  return c;
}

std::vector<hardware_interface::ComponentInfo> make_joints(
  const std::vector<std::string> & names)
{
  std::vector<hardware_interface::ComponentInfo> joints;
  joints.reserve(names.size());
  for (const auto & name : names) {
    joints.push_back(make_joint(name));
  }
  return joints;
}

TEST(MapJointsToSdkIndex, ValidMap)
{
  JointIndexTable table{{"a", 0}, {"b", 2}, {"c", 1}};
  std::vector<hardware_interface::ComponentInfo> joints{
    make_joint("a"), make_joint("b"), make_joint("c")};
  auto r = map_joints_to_sdk_index(joints, table);
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(r.sdk_index_by_joint, (std::vector<int>{0, 2, 1}));
}

TEST(MapJointsToSdkIndex, UnknownJointNameFails)
{
  JointIndexTable table{{"a", 0}, {"b", 1}};
  std::vector<hardware_interface::ComponentInfo> joints{make_joint("a"), make_joint("head_yaw")};
  auto r = map_joints_to_sdk_index(joints, table);
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.error.find("head_yaw"), std::string::npos);
}

TEST(MapJointsToSdkIndex, SubsetOfTableIsFine)
{
  JointIndexTable table{{"a", 0}, {"b", 1}, {"c", 2}};
  std::vector<hardware_interface::ComponentInfo> joints{make_joint("b")};
  auto r = map_joints_to_sdk_index(joints, table);
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(r.sdk_index_by_joint, (std::vector<int>{1}));
}

// The 29 G1 body joint names, in SDK motor index order. Kept in sync (by
// hand) with g1_body_joint_index_table() in joint_index_tables.cpp; used here
// to cross-check the name<->enum-symbol pairing in g1_body_joint_index_table()
// against the SDK's JointIndex enum order.
const std::vector<std::string> kG1JointNames{
  "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
  "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
  "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
  "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
  "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
  "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
  "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint",
  "left_wrist_yaw_joint", "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
  "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
  "right_wrist_pitch_joint", "right_wrist_yaw_joint"
};

// The 31 H2 body joint names, in SDK motor index order. Kept in sync (by
// hand) with h2_body_joint_index_table() in joint_index_tables.cpp.
const std::vector<std::string> kH2JointNames{
  "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
  "left_knee_joint", "left_ankle_roll_joint", "left_ankle_pitch_joint",
  "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
  "right_knee_joint", "right_ankle_roll_joint", "right_ankle_pitch_joint",
  "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
  "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
  "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint",
  "left_wrist_yaw_joint", "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
  "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
  "right_wrist_pitch_joint", "right_wrist_yaw_joint",
  "head_pitch_joint", "head_yaw_joint"
};

TEST(BodyJointIndexTable, G1TableMatchesSdkEnumOrder)
{
  const auto & table = g1_body_joint_index_table();
  EXPECT_EQ(table.size(), 29u);
  auto r = map_joints_to_sdk_index(make_joints(kG1JointNames), table);
  ASSERT_TRUE(r.ok) << r.error;
  std::vector<int> expected(29);
  for (int i = 0; i < 29; ++i) {
    expected[i] = i;
  }
  EXPECT_EQ(r.sdk_index_by_joint, expected);
}

TEST(BodyJointIndexTable, H2TableMatchesLiteralOrder)
{
  const auto & table = h2_body_joint_index_table();
  EXPECT_EQ(table.size(), 31u);
  auto r = map_joints_to_sdk_index(make_joints(kH2JointNames), table);
  ASSERT_TRUE(r.ok) << r.error;
  std::vector<int> expected(31);
  for (int i = 0; i < 31; ++i) {
    expected[i] = i;
  }
  EXPECT_EQ(r.sdk_index_by_joint, expected);
}

TEST(BodyJointIndexTable, H2HeadYawIs30)
{
  auto r = map_joints_to_sdk_index(
    make_joints({"head_yaw_joint"}), h2_body_joint_index_table());
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(r.sdk_index_by_joint, (std::vector<int>{30}));
}

TEST(BodyJointIndexTable, AnkleOrderingDivergesBetweenG1AndH2)
{
  auto g1 = map_joints_to_sdk_index(
    make_joints({"left_ankle_roll_joint"}), g1_body_joint_index_table());
  auto h2 = map_joints_to_sdk_index(
    make_joints({"left_ankle_roll_joint"}), h2_body_joint_index_table());
  ASSERT_TRUE(g1.ok) << g1.error;
  ASSERT_TRUE(h2.ok) << h2.error;
  EXPECT_EQ(g1.sdk_index_by_joint, (std::vector<int>{5}));
  EXPECT_EQ(h2.sdk_index_by_joint, (std::vector<int>{4}));
}

TEST(Dex3JointIndexTable, LeftAndRightHandIndices)
{
  const auto & table = dex3_hand_joint_index_table();
  EXPECT_EQ(table.size(), 14u);
  auto r = map_joints_to_sdk_index(
    make_joints({"left_hand_thumb_0_joint", "left_hand_index_1_joint", "right_hand_thumb_0_joint"}),
    table);
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(r.sdk_index_by_joint, (std::vector<int>{0, 6, 0}));
}

TEST(GetBodyJointIndexTable, SelectsG1)
{
  bool ok = false;
  const std::string variant = "g1";
  const auto & table = get_body_joint_index_table(variant, ok);
  EXPECT_TRUE(ok);
  EXPECT_EQ(table.size(), 29u);
}

TEST(GetBodyJointIndexTable, SelectsH2)
{
  bool ok = false;
  const std::string variant = "h2";
  const auto & table = get_body_joint_index_table(variant, ok);
  EXPECT_TRUE(ok);
  EXPECT_EQ(table.size(), 31u);
}

TEST(GetBodyJointIndexTable, UnknownVariantFails)
{
  bool ok = true;
  get_body_joint_index_table("unknown_variant", ok);
  EXPECT_FALSE(ok);
}
}  // namespace
}  // namespace unitree_hg_ros2_control
