from __future__ import annotations

import argparse
from pathlib import Path

from embodiment_core.config import load_yaml
from jaka_driver_adapter.adapter import JakaDriverAdapter
from rh56_driver.node import RH56Driver
from vision_interface.mock_camera import MockRGBDCamera

from .episode_recorder import EpisodeRecorder


def main() -> None:
    parser = argparse.ArgumentParser(description="Start a minimal recording session.")
    parser.add_argument("--logging-config", default="configs/logging/default.yaml")
    parser.add_argument("--task", required=True)
    parser.add_argument("--instruction", required=True)
    parser.add_argument("--operator", default="operator")
    parser.add_argument("--output-dir", default=None)
    args = parser.parse_args()

    recorder = EpisodeRecorder(load_yaml(Path(args.logging_config)), data_root=args.output_dir)
    camera = MockRGBDCamera()
    arm = JakaDriverAdapter.from_yaml("configs/robot/jaka_mini2.yaml")
    hand = RH56Driver.from_yaml("configs/hand/rh56.yaml")

    arm.connect()
    hand.connect()

    recorder.start_episode(task_name=args.task, instruction=args.instruction, operator=args.operator)
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
        },
        action={"type": "noop"},
    )
    recorder.mark_success(True, operator_notes="minimal recording path")
    episode_dir = recorder.stop_episode()
    export_dir = recorder.export_dataset(Path(episode_dir).parent.parent / "exports" / "structured")
    print(f"Episode saved to: {episode_dir}")
    print(f"Structured export written to: {export_dir}")


if __name__ == "__main__":
    main()
