// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "unitree_hg_ros2_control/unitree_hg_crc.hpp"

namespace unitree_hg_ros2_control
{
namespace
{
TEST(UnitreeHgCrc, LowStateRoundTrip)
{
  unitree_hg::msg::dds_::LowState_ low_state;
  low_state.mode_machine() = 5;
  low_state.crc() = detail::LowStateCrc::compute(low_state);
  EXPECT_TRUE(detail::LowStateCrc::is_valid(low_state));
  low_state.crc() ^= 1U;
  EXPECT_FALSE(detail::LowStateCrc::is_valid(low_state));
}

// HandStateRoundTrip is intentionally omitted: unitree_hg::msg::dds_::HandState_ has no
// `crc` field in the pinned unitree_sdk2, so detail::HandStateCrc as specified in the
// Task 2 brief cannot compile. See unitree_hg_crc.hpp and task-2-report.md.
}  // namespace
}  // namespace unitree_hg_ros2_control
