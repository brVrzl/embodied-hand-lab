from __future__ import annotations

from pathlib import Path

from embodiment_core.config import load_yaml


def test_all_yaml_configs_load() -> None:
    config_root = Path("configs")
    for yaml_path in config_root.rglob("*.yaml"):
        data = load_yaml(yaml_path)
        assert isinstance(data, dict), yaml_path


def test_required_robot_config_fields() -> None:
    jaka = load_yaml("configs/robot/jaka_mini2.yaml")
    rh56 = load_yaml("configs/hand/rh56.yaml")
    camera = load_yaml("configs/camera/default_rgbd.yaml")
    assert jaka["mode"] in {"mock", "real"}
    assert "joint_names" in jaka
    assert rh56["mode"] in {"mock", "real"}
    assert "serial" in rh56
    assert camera["device_type"] == "rgbd"
    assert "topics" in camera
