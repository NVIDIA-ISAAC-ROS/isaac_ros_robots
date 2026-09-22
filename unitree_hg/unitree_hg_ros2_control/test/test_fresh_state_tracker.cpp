// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "unitree_hg_ros2_control/fresh_state_tracker.hpp"

namespace unitree_hg_ros2_control
{
namespace
{

TEST(FreshLowStateTrackerTest, RequiresNewStateAfterReset)
{
  detail::FreshLowStateTracker tracker;

  EXPECT_FALSE(tracker.has_fresh_state());

  tracker.mark_received();
  EXPECT_TRUE(tracker.has_fresh_state());

  tracker.reset();
  EXPECT_FALSE(tracker.has_fresh_state());
}

TEST(FreshLowStateTrackerTest, ExpiresStateAfterDdsStops)
{
  detail::FreshLowStateTracker tracker;

  tracker.mark_received();
  EXPECT_TRUE(tracker.has_fresh_state());

  std::this_thread::sleep_for(std::chrono::milliseconds(600));
  EXPECT_FALSE(tracker.has_fresh_state());
}

}  // namespace
}  // namespace unitree_hg_ros2_control
