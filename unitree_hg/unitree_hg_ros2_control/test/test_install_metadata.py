# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import os
from pathlib import Path
import subprocess
import tempfile
import textwrap
import unittest
import xml.etree.ElementTree as ET


def package_root():
    test_srcdir = os.environ.get("TEST_SRCDIR")
    test_workspace = os.environ.get("TEST_WORKSPACE")
    if test_srcdir and test_workspace:
        return (Path(test_srcdir) / test_workspace / "ros_ws/src/isaac_ros_robots/"
                "unitree_hg/unitree_hg_ros2_control")
    return Path(__file__).resolve().parents[1]


class InstallMetadataTest(unittest.TestCase):

    def test_cyclonedds_is_built_as_a_private_dependency(self):
        package = package_root()
        cmake = (package / "CMakeLists.txt").read_text()
        normalized_cmake = " ".join(cmake.lower().split())

        self.assertNotIn("fetchcontent_makeavailable(cyclonedds)", normalized_cmake)
        self.assertIn("fetchcontent_populate(cyclonedds)", normalized_cmake)
        self.assertRegex(
            normalized_cmake,
            r"add_subdirectory\([^)]*cyclonedds_source_dir[^)]*exclude_from_all\)",
        )
        self.assertIn("install(targets ddsc", normalized_cmake)
        for option in (
                "BUILD_IDLC", "BUILD_DDSPERF", "BUILD_EXAMPLES", "ENABLE_SECURITY"):
            self.assertIn(f"set({option.lower()} off)", normalized_cmake)
            self.assertNotRegex(
                normalized_cmake,
                rf"set\({option.lower()} off cache\b",
            )

        build = (package / "BUILD.bazel").read_text()
        patch = "cmake/patches/cyclonedds-0.10.2-typeobject-null-safety.patch"
        self.assertGreaterEqual(build.count(f'"{patch}"'), 3)

    def test_ament_registers_python_contract_tests(self):
        package = package_root()
        cmake = (package / "CMakeLists.txt").read_text()
        self.assertIn("find_package(ament_cmake_pytest REQUIRED)", cmake)
        self.assertIn(
            "ament_add_pytest_test(test_install_metadata "
            "test/test_install_metadata.py)", cmake)
        self.assertIn(
            "ament_add_pytest_test(test_runtime_source_contracts "
            "test/test_runtime_source_contracts.py)", cmake)

        package_xml = ET.parse(package / "package.xml").getroot()
        test_dependencies = {
            dependency.text for dependency in package_xml.findall("test_depend")
        }
        self.assertIn("ament_cmake_pytest", test_dependencies)
        self.assertIn("python3-pytest", test_dependencies)

    def test_configure_generates_discoverable_plugin_without_link_export(self):
        package = package_root()
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary = Path(temporary_directory)
            prefix = temporary / "prefix"
            self._write_mock_packages(prefix)
            source = temporary / "source"
            source.mkdir()
            (source / "CMakeLists.txt").write_text(textwrap.dedent(f"""
                cmake_minimum_required(VERSION 3.16)
                project(metadata_fixture)
                add_library(unitree_sdk2 INTERFACE)
                set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
                add_subdirectory("{package}" unitree_hg_ros2_control)
            """))
            build = temporary / "build"
            configure = subprocess.run(
                ["cmake", "-S", str(source), "-B", str(build),
                 f"-DCMAKE_PREFIX_PATH={prefix}"],
                check=False, capture_output=True, text=True)
            self.assertEqual(
                configure.returncode, 0, configure.stdout + configure.stderr)

            install_script = (build / "unitree_hg_ros2_control" /
                              "cmake_install.cmake").read_text()
            self.assertIn("libunitree_hg_ros2_control.so", install_script)
            self.assertIn("unitree_hg_ros2_control_hardware.xml", install_script)
            self.assertIn(
                "hardware_interface__pluginlib__plugin", install_script)
            self.assertIn("unitree_hg_ros2_controlConfig.cmake", install_script)
            self.assertNotIn("unitree_hg_ros2_controlTargets.cmake", install_script)

            plugin = ET.parse(
                package / "unitree_hg_ros2_control_hardware.xml").getroot()
            self.assertEqual(plugin.attrib["path"], "unitree_hg_ros2_control")
            plugin_types = {entry.attrib["type"] for entry in plugin.findall("class")}
            self.assertEqual(plugin_types, {
                "unitree_hg_ros2_control::UnitreeHgSystemInterface",
                "unitree_hg_ros2_control::UnitreeDex3SystemInterface",
            })

    @staticmethod
    def _write_mock_packages(prefix):
        packages = ["ament_cmake", "hardware_interface", "pluginlib", "rclcpp",
                    "rclcpp_lifecycle", "diagnostic_msgs", "realtime_tools"]
        for package in packages:
            config_dir = prefix / "share" / package / "cmake"
            config_dir.mkdir(parents=True)
            body = ""
            if package == "ament_cmake":
                body = textwrap.dedent("""
                    function(ament_target_dependencies target)
                    endfunction()
                    function(ament_export_dependencies)
                    endfunction()
                    function(ament_export_targets export_name)
                      set_property(GLOBAL PROPERTY MOCK_EXPORT_NAME "${export_name}")
                    endfunction()
                    function(ament_package)
                      set(config "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake")
                      file(WRITE "${config}" "# generated package config\\n")
                      install(FILES "${config}" DESTINATION "share/${PROJECT_NAME}/cmake")
                      get_property(export_name GLOBAL PROPERTY MOCK_EXPORT_NAME)
                      if(export_name)
                        install(EXPORT "${export_name}"
                          FILE "${PROJECT_NAME}Targets.cmake"
                          DESTINATION "share/${PROJECT_NAME}/cmake")
                      endif()
                    endfunction()
                """)
            elif package == "pluginlib":
                body = textwrap.dedent("""
                    function(pluginlib_export_plugin_description_file base_package plugin_xml)
                      set(resource "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}__pluginlib")
                      file(WRITE "${resource}" "share/${PROJECT_NAME}/${plugin_xml}\\n")
                      set(resource_index
                        "share/ament_index/resource_index/${base_package}__pluginlib__plugin")
                      install(FILES "${resource}"
                        DESTINATION "${resource_index}"
                        RENAME "${PROJECT_NAME}")
                      install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/${plugin_xml}"
                        DESTINATION "share/${PROJECT_NAME}")
                    endfunction()
                """)
            (config_dir / f"{package}Config.cmake").write_text(body)


if __name__ == "__main__":
    unittest.main()
