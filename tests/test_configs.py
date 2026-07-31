from __future__ import annotations

from pathlib import Path

from embodiment_core.config import load_yaml


def test_all_yaml_configs_load() -> None:
    config_root = Path("configs")
    for yaml_path in config_root.rglob("*.yaml"):
        data = load_yaml(yaml_path)
        assert isinstance(data, dict), yaml_path


def test_required_robot_config_fields() -> None:
    jaka = load_yaml("configs/sim/quest_hts_jaka_mini2_live_demo.yaml")
    rh56 = load_yaml("configs/hand/rh56_pc_direct_teleop.yaml")
    camera = load_yaml("configs/camera/default_rgbd.yaml")
    assert jaka["schema_version"] == "quest_hts_jaka_mini2_live_demo.v1"
    assert len(jaka["simulation"]["initial_arm_joints_rad"]) == 6
    assert rh56["backend_type"] == "serial_protocol"
    assert "serial" in rh56
    assert camera["device_type"] == "rgbd"
    assert "topics" in camera
