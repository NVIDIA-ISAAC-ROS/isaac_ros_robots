#!/usr/bin/env python3

# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Unit tests for Unitree G1 launch-file LEAPP config path logic."""

import ast
from pathlib import Path
from typing import Any


BRINGUP_ROOT = Path(__file__).parents[1]


def _load_launch_function(launch_file_name, function_name):
    launch_file = BRINGUP_ROOT / "launch" / launch_file_name
    tree = ast.parse(launch_file.read_text())
    function_def = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == function_name
    )
    module = ast.Module(body=[function_def], type_ignores=[])
    ast.fix_missing_locations(module)
    namespace = {
        "Any": Any,
        "Path": Path,
        "get_package_share_directory": lambda package_name: f"/opt/ros/{package_name}",
    }
    exec(compile(module, str(launch_file), "exec"), namespace)
    return namespace[function_name]


def _contains_string(node, value):
    return any(
        isinstance(child, ast.Constant) and child.value == value
        for child in ast.walk(node)
    )


def _dict_keys(node):
    if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
        if node.func.attr == "items":
            node = node.func.value
    if not isinstance(node, ast.Dict):
        return set()
    return {
        key.value
        for key in node.keys
        if isinstance(key, ast.Constant) and isinstance(key.value, str)
    }


def _dict_string_value(node, key_name):
    if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
        if node.func.attr == "items":
            node = node.func.value
    if not isinstance(node, ast.Dict):
        return None
    for key, value in zip(node.keys, node.values):
        if isinstance(key, ast.Constant) and key.value == key_name:
            return value.value if isinstance(value, ast.Constant) else None
    return None


def _declared_launch_arguments(tree):
    names = set()
    for call in [node for node in ast.walk(tree) if isinstance(node, ast.Call)]:
        if getattr(call.func, "id", "") != "DeclareLaunchArgument":
            continue
        if not call.args:
            continue
        name = call.args[0]
        if isinstance(name, ast.Constant) and isinstance(name.value, str):
            names.add(name.value)
    return names


def _launch_argument_default(tree, argument_name):
    for call in [node for node in ast.walk(tree) if isinstance(node, ast.Call)]:
        if getattr(call.func, "id", "") != "DeclareLaunchArgument":
            continue
        if not call.args:
            continue
        name = call.args[0]
        if not isinstance(name, ast.Constant) or name.value != argument_name:
            continue
        return next(
            (
                keyword.value.value
                for keyword in call.keywords
                if keyword.arg == "default_value"
                and isinstance(keyword.value, ast.Constant)
            ),
            None,
        )
    return None


def test_inference_graph_forwards_inference_controller_config_path_to_controller_manager():
    launch_file = BRINGUP_ROOT / "launch" / "unitree_g1_inference_graph.launch.py"
    tree = ast.parse(launch_file.read_text())

    for call in [node for node in ast.walk(tree) if isinstance(node, ast.Call)]:
        if not getattr(call.func, "id", "") == "IncludeLaunchDescription":
            continue
        if not _contains_string(call, "unitree_g1_controller_manager.launch.py"):
            continue
        launch_arguments = next(
            keyword.value
            for keyword in call.keywords
            if keyword.arg == "launch_arguments"
        )

        assert "inference_controller_config_path" in _dict_keys(launch_arguments)
        return

    raise AssertionError("unitree_g1_controller_manager.launch.py include not found")


def test_inference_graph_declares_and_forwards_triton_cpu_models():
    launch_file = BRINGUP_ROOT / "launch" / "unitree_g1_inference_graph.launch.py"
    tree = ast.parse(launch_file.read_text())

    assert "triton_cpu_models" in _declared_launch_arguments(tree)

    for call in [node for node in ast.walk(tree) if isinstance(node, ast.Call)]:
        if not getattr(call.func, "id", "") == "IncludeLaunchDescription":
            continue
        if not _contains_string(call, "inference_graph.launch.py"):
            continue
        launch_arguments = next(
            keyword.value
            for keyword in call.keywords
            if keyword.arg == "launch_arguments"
        )

        assert "triton_cpu_models" in _dict_keys(launch_arguments)
        return

    raise AssertionError("inference_graph.launch.py include not found")


def test_controller_manager_exposes_configurable_spawner_timeout():
    launch_file = BRINGUP_ROOT / "launch" / "unitree_g1_controller_manager.launch.py"
    tree = ast.parse(launch_file.read_text())

    assert "controller_spawner_timeout" in _declared_launch_arguments(tree)
    assert _launch_argument_default(tree, "controller_spawner_timeout") == "60"


def test_spawner_parameter_files_exclude_hardware_pid_yaml():
    spawner_parameter_files = _load_launch_function(
        "unitree_g1_controller_manager.launch.py",
        "_spawner_parameter_files",
    )
    controller_params = [
        {"robot_description": "<robot/>"},
        {"use_sim_time": True},
        "/tmp/controller_manager.yaml",
        "/tmp/mujoco_pid.yaml",
        "/tmp/runtime_params.yaml",
    ]

    assert spawner_parameter_files(
        controller_params,
        hardware_parameter_files=("/tmp/mujoco_pid.yaml",),
    ) == [
        "/tmp/controller_manager.yaml",
        "/tmp/runtime_params.yaml",
    ]


def test_spawner_parameter_files_keep_controller_yaml_when_pid_absent():
    spawner_parameter_files = _load_launch_function(
        "unitree_g1_controller_manager.launch.py",
        "_spawner_parameter_files",
    )
    controller_params = [
        {"robot_description": "<robot/>"},
        {"use_sim_time": False},
        "/tmp/controller_manager.yaml",
        "/tmp/runtime_params.yaml",
    ]

    assert spawner_parameter_files(controller_params) == [
        "/tmp/controller_manager.yaml",
        "/tmp/runtime_params.yaml",
    ]


def test_spawner_loads_controller_parameter_files_via_param_file():
    launch_file = BRINGUP_ROOT / "launch" / "unitree_g1_controller_manager.launch.py"
    tree = ast.parse(launch_file.read_text())
    spawn = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "spawn_controllers_sequentially"
    )

    assert _contains_string(spawn, "--param-file")
    assert _contains_string(spawn, "_controller_parameter_files")


def test_controller_loading_places_inference_last():
    order_controllers = _load_launch_function(
        "unitree_g1_controller_manager.launch.py",
        "_inference_controller_last",
    )

    assert order_controllers(
        [
            "safety_controller_lower_body",
            "inference_controller",
            "upper_body_forward_joint_command_controller",
            "freeze_controller",
        ]
    ) == [
        "safety_controller_lower_body",
        "upper_body_forward_joint_command_controller",
        "freeze_controller",
        "inference_controller",
    ]


def test_topic_input_sources_match_source_to_topic_keys():
    topic_input_sources = _load_launch_function(
        "unitree_g1_controller_manager.launch.py",
        "_topic_input_sources",
    )

    assert topic_input_sources(
        {
            "source_to_topic": {
                "command/body/velocity": "/cmd_vel",
                "state/camera/image": "/camera/image",
            }
        }
    ) == ["command/body/velocity", "state/camera/image"]


def test_gr00t_observation_replay_allows_slow_controller_configuration():
    launch_file = (
        BRINGUP_ROOT.parents[2]
        / "isaac_ros_physical_ai"
        / "unitree_g1_gr00t_tests"
        / "test"
        / "test_dataset_observation_replay_pipeline.launch.py"
    )
    tree = ast.parse(launch_file.read_text())

    for call in [node for node in ast.walk(tree) if isinstance(node, ast.Call)]:
        if not getattr(call.func, "id", "") == "IncludeLaunchDescription":
            continue
        if not _contains_string(call, "unitree_g1_controller_manager.launch.py"):
            continue
        launch_arguments = next(
            keyword.value
            for keyword in call.keywords
            if keyword.arg == "launch_arguments"
        )
        assert _dict_string_value(launch_arguments, "controller_spawner_timeout") == "300"
        return

    raise AssertionError("unitree_g1_controller_manager.launch.py include not found")


def test_inference_controller_config_path_override_takes_precedence_over_agile_config(
    tmp_path
):
    resolve_path = _load_launch_function(
        "unitree_g1_controller_manager.launch.py",
        "_resolve_inference_controller_config_path",
    )
    override_path = tmp_path / "custom_policy.yaml"
    group_config = {
        "data_package": "policy_pkg",
        "config": "policy.yaml",
        "agile_data_package": "agile_pkg",
        "agile_config": "agile.yaml",
    }

    assert resolve_path(
        group_config, str(override_path)
    ) == str(override_path.resolve())


def test_inference_controller_config_path_uses_agile_config_default_when_override_empty():
    resolve_path = _load_launch_function(
        "unitree_g1_controller_manager.launch.py",
        "_resolve_inference_controller_config_path",
    )
    group_config = {
        "data_package": "policy_pkg",
        "config": "policy.yaml",
        "agile_data_package": "agile_pkg",
        "agile_config": "agile.yaml",
    }

    assert resolve_path(
        group_config, ""
    ) == "/opt/ros/agile_pkg/data/agile.yaml"


def test_inference_graph_config_path_override_takes_precedence_over_group_config(tmp_path):
    resolve_path = _load_launch_function(
        "unitree_g1_inference_graph.launch.py",
        "_resolve_inference_graph_config_path",
    )
    override_path = tmp_path / "graph_policy.yaml"
    group_config = {
        "data_package": "policy_pkg",
        "config": "policy.yaml",
    }

    assert resolve_path(
        group_config, str(override_path)
    ) == str(override_path.resolve())


def test_inference_graph_config_path_uses_group_config_when_override_empty():
    resolve_path = _load_launch_function(
        "unitree_g1_inference_graph.launch.py",
        "_resolve_inference_graph_config_path",
    )
    group_config = {
        "data_package": "policy_pkg",
        "config": "policy.yaml",
        "agile_data_package": "agile_pkg",
        "agile_config": "agile.yaml",
    }

    assert resolve_path(
        group_config, ""
    ) == "/opt/ros/policy_pkg/data/policy.yaml"
