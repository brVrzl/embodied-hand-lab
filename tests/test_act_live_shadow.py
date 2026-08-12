from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
TOOL = ROOT / "tools" / "act_live_shadow.py"


def _load_tool():
    spec = importlib.util.spec_from_file_location("act_live_shadow", TOOL)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_live_rgb_preprocessing_matches_recorded_bgr_conversion_path() -> None:
    tool = _load_tool()
    rows = np.arange(480, dtype=np.uint16)[:, None]
    cols = np.arange(640, dtype=np.uint16)[None, :]
    rgb = np.stack(
        [
            np.broadcast_to((rows % 256).astype(np.uint8), (480, 640)),
            np.broadcast_to((cols % 256).astype(np.uint8), (480, 640)),
            ((rows + cols) % 256).astype(np.uint8),
        ],
        axis=2,
    )
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    recorded_rgb = cv2.cvtColor(
        cv2.resize(bgr, (320, 240), interpolation=cv2.INTER_AREA),
        cv2.COLOR_BGR2RGB,
    )
    expected = np.ascontiguousarray(recorded_rgb.transpose(2, 0, 1), dtype=np.float32) / 255.0

    actual = tool.preprocess_live_rgb(rgb)

    assert actual.shape == (3, 240, 320)
    assert actual.dtype == np.float32
    np.testing.assert_array_equal(actual, expected)


def test_shadow_adapter_has_no_robot_or_hand_command_calls() -> None:
    source = TOOL.read_text(encoding="utf-8")
    forbidden = (
        ".write_register(",
        ".set_canonical_angles(",
        ".set_angles(",
        ".set_speeds(",
        ".set_forces(",
        "servo_j(",
        "joint_move(",
        "linear_move(",
        "enable_robot(",
    )
    assert not any(token in source for token in forbidden)


def test_training_envelope_accepts_lerobot_action_stats(tmp_path: Path) -> None:
    tool = _load_tool()
    path = tmp_path / "stats.json"
    path.write_text(
        json.dumps({"action": {"min": [0.0] * 12, "max": [1.0] * 12}}),
        encoding="utf-8",
    )
    minimum, maximum = tool._load_training_envelope(path)
    assert minimum.shape == (12,)
    assert maximum.shape == (12,)
    assert np.all(minimum == 0.0)
    assert np.all(maximum == 1.0)
