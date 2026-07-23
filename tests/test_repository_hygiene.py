from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
EMBEDDED_BASIC_AUTH = re.compile(r"https?://[^/\\s:@]+:[^@\\s/]+@")


def test_runtime_sources_do_not_embed_url_credentials() -> None:
    offenders: list[str] = []
    for directory in ("src", "tools", "scripts", "configs"):
        for path in (ROOT / directory).rglob("*"):
            if path.is_file() and path.suffix in {".py", ".sh", ".yaml", ".yml", ".json"}:
                if EMBEDDED_BASIC_AUTH.search(path.read_text(encoding="utf-8", errors="replace")):
                    offenders.append(str(path.relative_to(ROOT)))
    assert offenders == []


@pytest.mark.parametrize(
    "tool",
    [
        "tools/check_iphone_camera_stream.py",
        "tools/iphone_mediapipe_hand_teleop.py",
    ],
)
def test_camera_tools_require_an_explicit_source(tool: str) -> None:
    result = subprocess.run(
        [sys.executable, tool],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 2
    assert "required" in result.stderr
