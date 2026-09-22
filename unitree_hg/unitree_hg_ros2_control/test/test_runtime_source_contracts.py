# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import os
from pathlib import Path
import re
import unittest


def package_root():
    test_srcdir = os.environ.get("TEST_SRCDIR")
    test_workspace = os.environ.get("TEST_WORKSPACE")
    if test_srcdir and test_workspace:
        return (Path(test_srcdir) / test_workspace / "ros_ws/src/isaac_ros_robots/"
                "unitree_hg/unitree_hg_ros2_control")
    return Path(__file__).resolve().parents[1]


class RuntimeSourceContractsTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.package = package_root()

    def test_diagnostics_timer_has_no_hardware_owner_dependency(self):
        header = (self.package / "include/unitree_hg_ros2_control/"
                  "unitree_hg_system_interface.hpp").read_text()
        body = (self.package / "src/unitree_hg_system_interface.cpp").read_text()
        callback = body.split("diagnostics_timer_ =", 1)[1].split(
            "Unitree Hg hardware interface activated", 1)[0]
        self.assertIn("std::shared_ptr<DiagnosticsContext> diagnostics_context_", header)
        self.assertIn("const auto diagnostics_context = diagnostics_context_;", body)
        self.assertIn("const auto diagnostics_pub = diagnostics_pub_;", body)
        self.assertIn("const auto diagnostics_node = get_node();", body)
        self.assertIn(
            "[diagnostics_context, diagnostics_pub, diagnostics_node]()", callback)
        self.assertIn("diagnostics_pub->publish(msg);", body)
        for owner_dependency in (
                "this", "get_node()", "joint_data_", "state_mutex_",
                "motor_temp_warn_threshold_", "steady_clock_"):
            self.assertNotIn(owner_dependency, callback)

    def test_ros2_control_read_handoff_has_no_blocking_mutex(self):
        helper = (self.package / "include/unitree_hg_ros2_control/"
                  "control_helpers.hpp").read_text()
        handoff = helper.split("class StateHandoff", 1)[1].split(
            "template<typename ChildCleanupT", 1)[0]
        self.assertIn("realtime_tools::RealtimeBuffer<SnapshotT>", handoff)
        self.assertIn("readFromRT()", handoff)
        self.assertNotIn("std::mutex", handoff)
        self.assertNotIn("lock_guard", handoff)
        for source_name in ("unitree_hg_system_interface.cpp",
                            "unitree_dex3_system_interface.cpp"):
            source = (self.package / "src" / source_name).read_text()
            copy = source.split("copy_inbound_state()", 1)[1].split(
                "release_channel_factory()", 1)[0]
            self.assertNotIn("state_mutex_", copy)
            self.assertNotIn("lock(", copy)

    def test_dex3_command_storage_is_owned_outside_write(self):
        header = (self.package / "include/unitree_hg_ros2_control/"
                  "unitree_dex3_system_interface.hpp").read_text()
        dex3 = (self.package / "src/unitree_dex3_system_interface.cpp").read_text()
        constructor = dex3.split(
            "UnitreeDex3SystemInterface::~UnitreeDex3SystemInterface", 1)[0]
        write = dex3.split("UnitreeDex3SystemInterface::write", 1)[1]
        self.assertIn("unitree_hg::msg::dds_::HandCmd_ hand_cmd_;", header)
        self.assertIn("hand_cmd_.motor_cmd().resize(kNumHandMotors);", constructor)
        self.assertIsNone(re.search(r"HandCmd_\s+hand_cmd\s*;", write))
        self.assertIn("handcmd_publisher_->Write(hand_cmd_)", write)


if __name__ == "__main__":
    unittest.main()
