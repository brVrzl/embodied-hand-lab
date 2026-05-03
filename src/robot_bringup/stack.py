from __future__ import annotations

from pathlib import Path

from embodiment_core.config import load_yaml
from jaka_driver_adapter.adapter import JakaDriverAdapter
from quadruped_adapter.adapter import QuadrupedAdapter
from rh56_driver.node import RH56Driver
from vision_interface.mock_camera import MockRGBDCamera


class EmbodiedLabStack:
    def __init__(
        self,
        arm_config: str | Path = "configs/robot/jaka_mini2.yaml",
        hand_config: str | Path = "configs/hand/rh56.yaml",
        quadruped_config: str | Path = "configs/quadruped/default.yaml",
        camera_config: str | Path = "configs/camera/default_rgbd.yaml",
        include_arm_hand: bool = True,
        include_quadruped: bool = True,
        include_camera: bool = True,
    ) -> None:
        self.include_arm_hand = include_arm_hand
        self.include_quadruped = include_quadruped
        self.include_camera = include_camera
        self.arm = JakaDriverAdapter.from_yaml(arm_config) if include_arm_hand else None
        self.hand = RH56Driver.from_yaml(hand_config) if include_arm_hand else None
        self.quadruped = (
            QuadrupedAdapter.from_yaml(quadruped_config) if include_quadruped else None
        )
        camera_cfg = load_yaml(camera_config) if include_camera else {}
        self.camera = (
            MockRGBDCamera(
                width=camera_cfg.get("width", 64),
                height=camera_cfg.get("height", 48),
                frame_id=camera_cfg.get("frames", {}).get("rgb_optical", "camera_color_optical_frame"),
            )
            if include_camera
            else None
        )

    def connect_all(self) -> dict[str, bool]:
        results = {}
        if self.arm:
            results["arm"] = self.arm.connect()
        if self.hand:
            results["hand"] = self.hand.connect()
        if self.quadruped:
            results["quadruped"] = self.quadruped.connect()
        if self.camera:
            results["camera"] = True
        return results

    def snapshot(self) -> dict:
        return {
            "arm_joint_state": self.arm.get_joint_state().to_dict() if self.arm else None,
            "hand_state": self.hand.read_state().to_dict() if self.hand else None,
            "quadruped_state": self.quadruped.get_robot_state().to_dict() if self.quadruped else None,
            "camera_intrinsics": self.camera.get_intrinsics().to_dict() if self.camera else None,
        }
