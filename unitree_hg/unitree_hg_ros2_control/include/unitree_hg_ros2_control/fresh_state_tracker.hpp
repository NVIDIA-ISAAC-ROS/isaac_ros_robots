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

#include <atomic>
#include <chrono>
#include <cstdint>

namespace unitree_hg_ros2_control
{
namespace detail
{

class FreshLowStateTracker
{
public:
  static constexpr auto kMaximumStateAge = std::chrono::milliseconds(500);

  void reset()
  {
    received_.store(false, std::memory_order_release);
    last_received_nanoseconds_.store(0, std::memory_order_relaxed);
  }

  void mark_received()
  {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    const auto now_nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
    last_received_nanoseconds_.store(now_nanoseconds, std::memory_order_relaxed);
    received_.store(true, std::memory_order_release);
  }

  bool has_fresh_state() const
  {
    if (!received_.load(std::memory_order_acquire)) {
      return false;
    }

    const auto received_nanoseconds = last_received_nanoseconds_.load(std::memory_order_relaxed);
    if (received_nanoseconds == 0) {
      return false;
    }

    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    const auto age = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count() -
      received_nanoseconds;
    return age >= std::chrono::nanoseconds::zero().count() &&
           age <= std::chrono::duration_cast<std::chrono::nanoseconds>(kMaximumStateAge).count();
  }

private:
  std::atomic<bool> received_{false};
  std::atomic<int64_t> last_received_nanoseconds_{0};
};

}  // namespace detail
}  // namespace unitree_hg_ros2_control
