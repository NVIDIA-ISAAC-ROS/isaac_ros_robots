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
#include "unitree_hg_ros2_control/sdk_index_map.hpp"

namespace unitree_hg_ros2_control
{
SdkIndexMapResult map_joints_to_sdk_index(
  const std::vector<hardware_interface::ComponentInfo> & joints, const JointIndexTable & table)
{
  SdkIndexMapResult result{true, "", {}};
  result.sdk_index_by_joint.reserve(joints.size());
  for (const auto & joint : joints) {
    bool found = false;
    for (const auto & entry : table) {
      if (entry.first == joint.name) {
        result.sdk_index_by_joint.push_back(entry.second);
        found = true;
        break;
      }
    }
    if (!found) {
      return {false, "Joint '" + joint.name + "' not found in joint index table.", {}};
    }
  }
  return result;
}
}  // namespace unitree_hg_ros2_control
