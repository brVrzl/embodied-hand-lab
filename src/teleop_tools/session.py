from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

from embodiment_core.logger import get_logger
from embodiment_core.types import Pose
from jaka_driver_adapter.adapter import JakaDriverAdapter
from quadruped_adapter.adapter import QuadrupedAdapter
from rh56_driver.node import RH56Driver


class TeleopSession:
    def __init__(
        self,
        arm: JakaDriverAdapter | None = None,
        hand: RH56Driver | None = None,
        dog: QuadrupedAdapter | None = None,
        log_path: str | Path = "data/teleop_actions.jsonl",
    ) -> None:
        self.arm = arm
        self.hand = hand
        self.dog = dog
        self.log_path = Path(log_path).resolve()
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.logger = get_logger("TeleopSession")

    def nudge_arm_ee(self, dx: float, dy: float, dz: float) -> bool:
        if self.arm is None:
            raise RuntimeError("Arm adapter is not configured.")
        pose = Pose(
            position=[0.35 + dx, dy, 0.2 + dz],
            orientation_xyzw=[0.0, 0.0, 0.0, 1.0],
            frame_id="jaka_base",
        )
        ok = self.arm.move_pose(pose, blocking=True)
        self._log("arm", "nudge_ee", {"dx": dx, "dy": dy, "dz": dz})
        return ok

    def hand_command(self, command: str, preset: str = "") -> bool:
        if self.hand is None:
            raise RuntimeError("Hand driver is not configured.")
        if command == "open":
            ok = self.hand.open()
        elif command == "close":
            ok = self.hand.close()
        elif command == "pinch":
            ok = self.hand.pinch()
        elif command == "preset":
            ok = self.hand.preset_grasp(preset)
        else:
            raise ValueError(f"Unsupported hand command: {command}")
        self._log("hand", command, {"preset": preset})
        return ok

    def dog_teleop(self, linear_x: float, linear_y: float, angular_z: float) -> bool:
        if self.dog is None:
            raise RuntimeError("Quadruped adapter is not configured.")
        payload = {"linear_x": linear_x, "linear_y": linear_y, "angular_z": angular_z}
        ok = self.dog.teleop(payload)
        self._log("dog", "cmd_vel", payload)
        return ok

    def _log(self, source: str, action_type: str, payload: dict[str, Any]) -> None:
        event = {
            "timestamp": time.time(),
            "source": source,
            "action_type": action_type,
            "payload": payload,
        }
        with self.log_path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(event) + "\n")
        self.logger.info("Teleop event: %s", event)

