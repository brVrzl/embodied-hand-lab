from __future__ import annotations

import ast
import importlib
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PRODUCTION = ROOT / "src" / "teleoperation"


def test_production_tree_has_no_legacy_hebi_rh56_or_quest_imports() -> None:
    forbidden = (
        "teleop_tools",
        "hebi",
        "rh56_driver",
        "robot_bringup",
        "motion_input",
        "quest",
    )
    violations: list[str] = []
    for path in PRODUCTION.rglob("*.py"):
        tree = ast.parse(path.read_text(), filename=str(path))
        for node in ast.walk(tree):
            names: list[str] = []
            if isinstance(node, ast.Import):
                names = [alias.name for alias in node.names]
            elif isinstance(node, ast.ImportFrom) and node.module:
                names = [node.module]
            for name in names:
                if any(name == item or name.startswith(item + ".") for item in forbidden):
                    violations.append(f"{path.relative_to(ROOT)}:{node.lineno}:{name}")
    assert violations == []


def test_production_supervision_imports_when_legacy_modules_are_unavailable(monkeypatch) -> None:
    for name in (
        "hebi",
        "teleop_tools",
        "teleop_tools.relative_pose_lag_follow",
        "rh56_driver",
    ):
        monkeypatch.setitem(sys.modules, name, None)
    importlib.import_module("teleoperation.supervision")
