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
#include <cstdint>
#include <functional>
#include <string>

namespace unitree_hg_ros2_control
{
struct ChannelFactoryInitResult
{
  bool ok;
  std::string error;
};

/// Idempotent, refcounted, process-global ChannelFactory init.
/// First caller performs the real Init; matching callers refcount; a caller
/// requesting a different (domain, iface) gets ok=false with an explanatory error.
ChannelFactoryInitResult EnsureChannelFactoryInitialized(int32_t domain, const std::string & iface);

/// Decrement the refcount; the last release performs the real ChannelFactory::Release().
void ReleaseChannelFactory();

namespace detail
{
using InitHook = std::function<void(int32_t, const std::string &)>;
using ReleaseHook = std::function<void()>;
void set_channel_factory_hooks_for_testing(InitHook init_hook, ReleaseHook release_hook);
void clear_channel_factory_hooks_for_testing();
void reset_guard_state_for_testing();
}  // namespace detail
}  // namespace unitree_hg_ros2_control
