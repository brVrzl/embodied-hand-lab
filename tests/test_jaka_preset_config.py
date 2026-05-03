from __future__ import annotations

from embodiment_core.config import load_yaml


def test_jaka_upright_preset_exists() -> None:
    config = load_yaml("configs/robot/jaka_mini2.yaml")
    presets = config.get("joint_presets", {})
    assert "upright" in presets
    assert presets["upright"] == [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
