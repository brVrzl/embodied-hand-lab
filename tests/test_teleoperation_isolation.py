from __future__ import annotations

import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_teleoperation_package_imports_without_device_or_legacy_stacks() -> None:
    script = f"""
import importlib
import importlib.abc
import pkgutil
import sys

sys.path.insert(0, {str(ROOT / "src")!r})
forbidden = {{"hebi", "motion_input", "rh56_driver", "robot_bringup", "teleop_tools"}}

class BlockForbidden(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split(".", 1)[0] in forbidden:
            raise ModuleNotFoundError(fullname)
        return None

sys.meta_path.insert(0, BlockForbidden())
import teleoperation
for module in pkgutil.walk_packages(teleoperation.__path__, prefix="teleoperation."):
    importlib.import_module(module.name)
"""
    completed = subprocess.run(
        [sys.executable, "-I", "-c", script],
        cwd=ROOT,
        text=True,
        capture_output=True,
    )
    assert completed.returncode == 0, completed.stderr or completed.stdout
