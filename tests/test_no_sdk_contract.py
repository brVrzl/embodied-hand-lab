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
from teleoperation.jaka.fake_backend import FakeJakaBackend
from rh56_driver.serial_backend import RH56SerialBackend

arm = FakeJakaBackend()
hand = RH56SerialBackend({{"serial": {{"port": "/dev/serial/by-id/not-opened"}}}})
with arm:
    assert arm.read_state().powered
assert hand.ser is None
"""
    completed = subprocess.run(
        [sys.executable, "-I", "-c", script],
        cwd=ROOT,
        text=True,
        capture_output=True,
    )
    assert completed.returncode == 0, completed.stderr or completed.stdout
