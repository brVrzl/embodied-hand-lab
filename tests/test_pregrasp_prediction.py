from __future__ import annotations

import json
import subprocess
import sys

import numpy as np

from pregrasp import (
    GeometryAwarePregraspPredictor,
    estimate_tactile_correction,
    geometry_from_point_cloud,
    load_primitive_config,
)


def _box_points(extents: tuple[float, float, float], n: int = 64) -> np.ndarray:
    rng = np.random.default_rng(3)
    return rng.uniform(-0.5, 0.5, size=(n, 3)) * np.asarray(extents, dtype=np.float64)


def test_predictor_prefers_power_envelope_for_ball_sized_round_object() -> None:
    points = _box_points((0.055, 0.054, 0.052))
    geometry = geometry_from_point_cloud(points, shape_hint="round")
    candidate = GeometryAwarePregraspPredictor().predict(geometry, task_mode="pick", top_k=1)[0]

    assert candidate.primitive.name == "power_envelope"
    assert candidate.score > 0.75
    assert len(candidate.hand_command) == 6


def test_predictor_prefers_lateral_clamp_for_flat_pick() -> None:
    points = _box_points((0.045, 0.035, 0.006))
    geometry = geometry_from_point_cloud(points, shape_hint="flat")
    candidate = GeometryAwarePregraspPredictor().predict(geometry, task_mode="pick", top_k=1)[0]

    assert candidate.primitive.name == "lateral_clamp"
    assert "thin_object_bias" in candidate.reasons


def test_tactile_correction_closes_missing_expected_contacts() -> None:
    correction = estimate_tactile_correction(
        {"inspire6": {"contact_binary": [True, False, False, False, False, False], "forces": [20, 0, 0, 0, 0, 0]}},
        target_contacts=["index", "middle", "thumb_close"],
    )

    assert correction.status == "close_or_reseat"
    assert correction.hand_delta[1] > 0.0
    assert correction.hand_delta[4] > 0.0


def test_load_yaml_config_matches_default_shape() -> None:
    primitives = load_primitive_config("configs/pregrasp/rh56_pregrasp.yaml")

    assert [item.name for item in primitives][:2] == ["power_envelope", "tripod_support"]
    assert all(len(item.hand_command) == 6 for item in primitives)


def test_cli_predicts_from_geometry_json(tmp_path) -> None:
    geometry_path = tmp_path / "geometry.json"
    geometry_path.write_text(
        json.dumps({"extents_xyz_m": [0.055, 0.054, 0.052], "shape_hint": "round"}),
        encoding="utf-8",
    )
    result = subprocess.run(
        [
            sys.executable,
            "tools/predict_rh56_pregrasp.py",
            "--geometry-json",
            str(geometry_path),
            "--top-k",
            "1",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(result.stdout)
    assert payload["candidates"][0]["primitive"]["name"] == "power_envelope"
