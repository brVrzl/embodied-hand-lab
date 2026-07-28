from __future__ import annotations

from embodiment_core.config import load_yaml


def test_jaka_upright_preset_remains_the_neutral_six_joint_pose() -> None:
    config = load_yaml("configs/robot/jaka_mini2.yaml")
    assert config["joint_presets"]["upright"] == [0.0] * 6
