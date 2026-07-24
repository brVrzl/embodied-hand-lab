from __future__ import annotations

import ast
import importlib
import pathlib
import sys

import numpy as np
import pytest

MODULE_DIR = pathlib.Path(__file__).parent
sys.path.insert(0, str(MODULE_DIR))

shadow_client = importlib.import_module("shadow_client")


def test_shadow_sources_have_no_robot_or_command_imports() -> None:
    forbidden_import_roots = {
        "jaka_driver_adapter",
        "rh56_driver",
        "robot_bringup",
        "teleoperation",
        "teleop_tools",
    }
    for path in (MODULE_DIR / "camera_probe.py", MODULE_DIR / "shadow_client.py"):
        tree = ast.parse(path.read_text(encoding="utf-8"))
        imported: set[str] = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imported.update(alias.name.split(".")[0] for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imported.add(node.module.split(".")[0])
        assert imported.isdisjoint(forbidden_import_roots)


def test_six_joint_state_is_rejected(tmp_path: pathlib.Path) -> None:
    path = tmp_path / "state.json"
    path.write_text(
        '{"schema":"openpi.pi05_droid_state.v1",'
        '"joint_position":[0,0,0,0,0,0],"gripper_position":[0]}',
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="seven Franka joint positions"):
        shadow_client.load_droid_state(path, synthetic=False)


def test_action_shape_is_strict() -> None:
    accepted = shadow_client.validate_actions(np.zeros((15, 8), dtype=np.float32))
    assert accepted.shape == (15, 8)
    with pytest.raises(ValueError, match="Expected action chunk"):
        shadow_client.validate_actions(np.zeros((10, 8), dtype=np.float32))
