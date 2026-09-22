// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <string>
#include <vector>
#include "unitree_hg_ros2_control/channel_factory_guard.hpp"

namespace unitree_hg_ros2_control
{
namespace
{
struct Recorder
{
  int inits = 0;
  int releases = 0;
  int32_t last_domain = -1;
  std::string last_iface;
};

class GuardTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rec_ = Recorder{};
    detail::set_channel_factory_hooks_for_testing(
      [this](int32_t d, const std::string & i) {
        rec_.inits++; rec_.last_domain = d; rec_.last_iface = i;
      },
      [this]() {rec_.releases++;});
    detail::reset_guard_state_for_testing();
  }
  void TearDown() override {detail::clear_channel_factory_hooks_for_testing();}
  Recorder rec_;
};

TEST_F(GuardTest, FirstInitCallsRealInitOnce)
{
  auto r = EnsureChannelFactoryInitialized(0, "eth0");
  EXPECT_TRUE(r.ok);
  EXPECT_EQ(rec_.inits, 1);
  EXPECT_EQ(rec_.last_iface, "eth0");
}

TEST_F(GuardTest, MatchingReinitDoesNotReinitButRefcounts)
{
  EnsureChannelFactoryInitialized(0, "eth0");
  auto r = EnsureChannelFactoryInitialized(0, "eth0");
  EXPECT_TRUE(r.ok);
  EXPECT_EQ(rec_.inits, 1);          // still one real init
  ReleaseChannelFactory();           // refcount 2->1, no release yet
  EXPECT_EQ(rec_.releases, 0);
  ReleaseChannelFactory();           // refcount 1->0, releases
  EXPECT_EQ(rec_.releases, 1);
}

TEST_F(GuardTest, MismatchedInterfaceErrors)
{
  EnsureChannelFactoryInitialized(0, "eth0");
  auto r = EnsureChannelFactoryInitialized(0, "eth1");
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.error.find("eth1"), std::string::npos);
  EXPECT_EQ(rec_.inits, 1);
}
}  // namespace
}  // namespace unitree_hg_ros2_control
