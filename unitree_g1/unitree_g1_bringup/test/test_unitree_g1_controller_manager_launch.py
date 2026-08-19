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
