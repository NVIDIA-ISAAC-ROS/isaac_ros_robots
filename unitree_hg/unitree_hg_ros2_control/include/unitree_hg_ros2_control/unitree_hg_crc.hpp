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
#include "unitree/idl/hg/LowState_.hpp"

// NOTE: There is intentionally no HandStateCrc here. The unitree_hg
// HandState_/HandCmd_ messages carry no `crc` field on the wire (only
// LowState_/LowCmd_ do), so hand state is not CRC-validated -- matching the
// behavior of the superseded unitree_g1 hand path.

namespace unitree_hg_ros2_control
{
namespace detail
{
uint32_t calculate_crc32(const uint32_t * data_words, uint32_t word_count);

class LowStateCrc
{
public:
  static uint32_t compute(const unitree_hg::msg::dds_::LowState_ & s)
  {
    return calculate_crc32(reinterpret_cast<const uint32_t *>(&s), (sizeof(s) >> 2) - 1);
  }
  static bool is_valid(const unitree_hg::msg::dds_::LowState_ & s) {return s.crc() == compute(s);}
};
}  // namespace detail
}  // namespace unitree_hg_ros2_control
