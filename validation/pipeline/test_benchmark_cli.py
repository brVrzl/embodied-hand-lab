from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[2]
CLI = ROOT / "tools/validation/benchmark_episode_pipeline.py"


def test_paced_benchmark_smoke_reports_bounded_quality_fields() -> None:
    completed = subprocess.run(
        [
            sys.executable,
            str(CLI),
            "--samples",
            "10",
            "--paced-seconds",
            "0.1",
        ],
        cwd=ROOT,
        check=True,
        text=True,
        capture_output=True,
    )
    report = json.loads(completed.stdout)
    paced = {
        key: value for key, value in report.items() if key.startswith("wall_clock_")
    }
    assert set(paced) == {
        "wall_clock_writer_0ms",
        "wall_clock_writer_50ms",
        "wall_clock_writer_100ms",
        "wall_clock_writer_150ms",
    }
    for result in paced.values():
        assert result["expected_camera_frames_per_role"] == result["samples"]
        assert 0.0 <= result["validity_ratio"] <= 1.0
        assert "process_rss_kb" in result
        assert result["bounded_shutdown"] is True
