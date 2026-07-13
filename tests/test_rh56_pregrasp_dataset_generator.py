from __future__ import annotations

import json
import subprocess
import sys


def test_generate_pregrasp_dataset_smoke(tmp_path) -> None:
    out_dir = tmp_path / "dataset"

    result = subprocess.run(
        [
            sys.executable,
            "tools/generate_rh56_pregrasp_dataset.py",
            "--objects",
            "foam_cube",
            "--offsets-per-primitive",
            "1",
            "--point-count",
            "32",
            "--duration",
            "0.25",
            "--out-dir",
            str(out_dir),
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    stdout = json.loads(result.stdout)
    manifest = json.loads((out_dir / "manifest.json").read_text(encoding="utf-8"))
    samples = [
        json.loads(line)
        for line in (out_dir / "samples.jsonl").read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    assert stdout["num_samples"] == 5
    assert manifest["schema_version"] == "rh56_pregrasp_mujoco_v0.1"
    assert len(samples) == 5
    assert "hardware_constraints" in samples[0]["candidate"]
    assert "failure_mode" in samples[0]["label"]
