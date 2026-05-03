from __future__ import annotations

import json
import importlib.util
from pathlib import Path
from typing import Any

from data_recorder.episode_recorder import EpisodeRecorder
from embodiment_core.config import load_yaml
from jaka_driver_adapter.adapter import JakaDriverAdapter
from lerobot_bridge.exporter import export_to_lerobot_stub
from quadruped_adapter.adapter import QuadrupedAdapter
from rh56_driver.node import RH56Driver
from vision_interface.mock_camera import MockRGBDCamera


def _load_schema_validator() -> Any:
    path = Path("tools/validate_episode_schema.py").resolve()
    spec = importlib.util.spec_from_file_location("validate_episode_schema", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.validate


def test_episode_recorder_roundtrip(tmp_path: Path) -> None:
    recorder = EpisodeRecorder(load_yaml("configs/logging/default.yaml"), data_root=tmp_path / "episodes")
    arm = JakaDriverAdapter.from_yaml("configs/robot/jaka_mini2.yaml")
    hand = RH56Driver.from_yaml("configs/hand/rh56.yaml")
    dog = QuadrupedAdapter.from_yaml("configs/quadruped/default.yaml")
    camera = MockRGBDCamera()

    arm.connect()
    hand.connect()
    dog.connect()

    episode_dir = recorder.start_episode(
        task_name="pick_and_place",
        instruction="pick the cube and place it into the tray",
        operator="tester",
    )
    recorder.record_step(
        observation={
            "rgb": camera.get_rgb(),
            "depth": camera.get_depth(),
            "camera_timestamp": camera.get_timestamp(),
            "arm_joint_states": arm.get_joint_state().to_dict(),
            "arm_ee_pose": {
                "position": [0.35, 0.0, 0.2],
                "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                "frame_id": "jaka_base",
            },
            "hand_states": hand.read_state().to_dict(),
            "dog_states": dog.get_robot_state().to_dict(),
        },
        action={"type": "teleop", "target": "arm_hand"},
        operator_notes="step 1",
    )
    recorder.mark_success(True, operator_notes="completed")
    recorder.stop_episode()

    metadata = json.loads((episode_dir / "metadata.json").read_text(encoding="utf-8"))
    assert metadata["success"] is True
    assert metadata["failure_mode"] == "none"
    assert metadata["schema_version"] == "jaka_rh56_palm_handcode_v0.1"
    assert metadata["canonical_hand_order"] == ["index", "middle", "ring", "pinky", "thumb_close", "thumb_lateral"]
    assert (episode_dir / "steps.jsonl").exists()

    structured_dir = recorder.export_dataset(tmp_path / "exports" / "structured")
    assert (structured_dir / "samples.jsonl").exists()
    validation_summary = _load_schema_validator()(structured_dir)
    assert validation_summary["schema_version"] == "jaka_rh56_palm_handcode_v0.1"
    manifest = json.loads((structured_dir / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["episodes"][0]["task_name"] == "pick_and_place"
    assert manifest["episodes"][0]["failure_mode"] == "none"
    sample = json.loads((structured_dir / "samples.jsonl").read_text(encoding="utf-8").splitlines()[0])
    assert sample["episode_index"] == 0
    assert sample["frame_index"] == 0
    assert sample["index"] == 0
    assert sample["task_index"] == 0
    assert sample["is_first"] is True
    assert sample["is_last"] is True
    assert sample["is_terminal"] is True
    assert sample["reward"] == 1.0
    assert sample["discount"] == 0.0
    assert sample["episode_failure_mode"] == "none"
    assert sample["metadata"]["action_delta_base"] == "command"
    assert sample["metadata"]["schema_version"] == "jaka_rh56_palm_handcode_v0.1"
    assert sample["metadata"]["format_family"] == "episode_step_jsonl"
    assert sample["metadata"]["hand_delta_state_raw_available"] is True
    assert sample["observation"]["state"]["hand_cmd_last"] == [0.0] * 6
    assert sample["action"]["hand_delta_cmd"] == [0.0] * 6
    assert sample["action"]["hand_delta_state_raw"] == [0.0] * 6
    assert sample["action"]["hand_delta_state"] == [0.0] * 6

    lerobot_dir = export_to_lerobot_stub(tmp_path / "episodes", tmp_path / "exports" / "lerobot")
    assert (lerobot_dir / "meta" / "info.json").exists()
    assert (lerobot_dir / "train" / "samples.jsonl").exists()
    lerobot_sample = json.loads((lerobot_dir / "train" / "samples.jsonl").read_text(encoding="utf-8").splitlines()[0])
    assert lerobot_sample["episode_index"] == 0
    assert lerobot_sample["episode_id"] == metadata["episode_id"]
    assert lerobot_sample["task_index"] == 0
    assert lerobot_sample["is_last"] is True
