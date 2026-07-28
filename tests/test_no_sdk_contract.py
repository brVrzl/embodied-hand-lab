from __future__ import annotations

import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_offline_adapters_do_not_import_device_sdks() -> None:
    script = f"""
import builtins
import sys

sys.path.insert(0, {str(ROOT / "src")!r})
real_import = builtins.__import__

def guarded_import(name, globals=None, locals=None, fromlist=(), level=0):
    if name.split(".", 1)[0] in {{"hebi", "jkrc"}}:
        raise AssertionError(f"device SDK imported: {{name}}")
    return real_import(name, globals, locals, fromlist, level)

builtins.__import__ = guarded_import
import teleoperation
from jaka_driver_adapter.adapter import JakaDriverAdapter
from rh56_driver.node import RH56Driver

arm = JakaDriverAdapter.from_yaml("configs/robot/jaka_mini2.yaml")
hand = RH56Driver.from_yaml("configs/hand/rh56.yaml")
assert arm.connect()
assert hand.connect()
"""
    completed = subprocess.run(
        [sys.executable, "-I", "-c", script],
        cwd=ROOT,
        text=True,
        capture_output=True,
    )
    assert completed.returncode == 0, completed.stderr or completed.stdout
