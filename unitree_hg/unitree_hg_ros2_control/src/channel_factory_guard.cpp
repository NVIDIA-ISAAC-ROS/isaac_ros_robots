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
#include "unitree_hg_ros2_control/channel_factory_guard.hpp"

#include <mutex>
#include "unitree/robot/channel/channel_factory.hpp"

namespace unitree_hg_ros2_control
{
namespace
{
struct GuardState
{
  std::mutex mutex;
  bool initialized = false;
  int32_t domain = 0;
  std::string iface;
  int refcount = 0;
  detail::InitHook init_hook;        // when set, used instead of the real SDK call (tests)
  detail::ReleaseHook release_hook;
};

GuardState & guard_state()
{
  static GuardState state;
  return state;
}

void do_real_init(int32_t domain, const std::string & iface)
{
  if (guard_state().init_hook) {guard_state().init_hook(domain, iface); return;}
  unitree::robot::ChannelFactory::Instance()->Init(domain, iface);
}

void do_real_release()
{
  if (guard_state().release_hook) {guard_state().release_hook(); return;}
  unitree::robot::ChannelFactory::Instance()->Release();
}
}  // namespace

ChannelFactoryInitResult EnsureChannelFactoryInitialized(int32_t domain, const std::string & iface)
{
  std::lock_guard<std::mutex> lock(guard_state().mutex);
  if (!guard_state().initialized) {
    do_real_init(domain, iface);
    guard_state().initialized = true;
    guard_state().domain = domain;
    guard_state().iface = iface;
    guard_state().refcount = 1;
    return {true, ""};
  }
  if (guard_state().domain != domain || guard_state().iface != iface) {
    return {false,
      "ChannelFactory already initialized on interface '" + guard_state().iface + "' (domain " +
      std::to_string(guard_state().domain) + "); cannot re-init on interface '" + iface +
      "' (domain " + std::to_string(domain) + ") in the same process."};
  }
  ++guard_state().refcount;
  return {true, ""};
}

void ReleaseChannelFactory()
{
  std::lock_guard<std::mutex> lock(guard_state().mutex);
  if (guard_state().refcount == 0) {return;}
  if (--guard_state().refcount == 0) {
    do_real_release();
    guard_state().initialized = false;
    guard_state().iface.clear();
    guard_state().domain = 0;
  }
}

namespace detail
{
void set_channel_factory_hooks_for_testing(InitHook init_hook, ReleaseHook release_hook)
{
  std::lock_guard<std::mutex> lock(guard_state().mutex);
  guard_state().init_hook = std::move(init_hook);
  guard_state().release_hook = std::move(release_hook);
}
void clear_channel_factory_hooks_for_testing()
{
  std::lock_guard<std::mutex> lock(guard_state().mutex);
  guard_state().init_hook = nullptr;
  guard_state().release_hook = nullptr;
}
void reset_guard_state_for_testing()
{
  std::lock_guard<std::mutex> lock(guard_state().mutex);
  guard_state().initialized = false; guard_state().domain = 0; guard_state().iface.clear();
  guard_state().refcount = 0;
}
}  // namespace detail
}  // namespace unitree_hg_ros2_control
