from __future__ import annotations

import ast
import os
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_clean_runtime_imports_without_legacy_hebi_or_hand_modules() -> None:
    script = """
import json, sys
import teleoperation
import teleoperation.runtime.arm_only
bad = [name for name in sys.modules if name == 'hebi' or name.startswith('hebi.') or
       name == 'teleop_tools' or name.startswith('teleop_tools.') or 'rh56' in name.lower()]
print(json.dumps(bad))
raise SystemExit(bool(bad))
"""
    environment = dict(os.environ, PYTHONPATH=str(ROOT / "src"))
    completed = subprocess.run([sys.executable, "-I", "-c", script], env=environment,
                               cwd=ROOT, text=True, capture_output=True)
    # -I ignores PYTHONPATH; explicitly inject the clean source tree without site packages.
    if completed.returncode != 0 and "No module named 'teleoperation'" in completed.stderr:
        script = f"import sys; sys.path.insert(0, {str(ROOT / 'src')!r});\n" + script
        completed = subprocess.run([sys.executable, "-I", "-c", script], cwd=ROOT,
                                   text=True, capture_output=True)
    assert completed.returncode == 0, completed.stderr or completed.stdout
    assert completed.stdout.strip() == "[]"


def test_arm_only_composition_source_has_no_forbidden_dependency() -> None:
    forbidden = ("teleop_tools", "hebi", "rh56")
    for path in (ROOT / "src" / "teleoperation").rglob("*.py"):
        tree = ast.parse(path.read_text())
        imports = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imports.extend(alias.name.lower() for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imports.append(node.module.lower())
        assert not any(token in name for name in imports for token in forbidden), path
    native_includes = [line.lower() for line in (ROOT / "native/jaka_servo_worker/main.cpp").read_text().splitlines()
                       if line.lstrip().startswith("#include")]
    assert not any(token in line for line in native_includes for token in forbidden)
