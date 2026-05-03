from __future__ import annotations

from pathlib import Path

from embodiment_core.config import load_yaml
from embodiment_core.logger import get_logger
from embodiment_core.types import JointState, Pose

from .interfaces import JakaBackend
from .jaka_sdk_backend import JakaSDKBackend
from .mock_jaka_backend import MockJakaBackend


class JakaDriverAdapter:
    def __init__(self, config: dict, backend: JakaBackend | None = None) -> None:
        self.config = config
        self.mode = config.get("mode", "mock")
        self.backend = backend or self._build_backend()
        self.logger = get_logger("JakaDriverAdapter")

    @classmethod
    def from_yaml(cls, path: str | Path) -> "JakaDriverAdapter":
        return cls(load_yaml(path))

    def _build_backend(self) -> JakaBackend:
        backend_type = self.config.get("backend_type", self.mode)
        if self.mode == "mock" or backend_type == "mock":
            return MockJakaBackend(
                joint_names=self.config.get("joint_names", [f"joint_{i+1}" for i in range(6)])
            )
        if self.mode == "real" and backend_type == "jaka_sdk":
            return JakaSDKBackend(self.config)
        raise NotImplementedError(f"Unsupported JAKA backend_type={backend_type!r}")

    def connect(self) -> bool:
        return self.backend.connect()

    def get_joint_state(self) -> JointState:
        return self.backend.get_joint_state()

    def move_joints(self, joints: list[float], blocking: bool = True) -> bool:
        return self.backend.move_joints(joints, blocking=blocking)

    def move_pose(self, pose: Pose, blocking: bool = True) -> bool:
        return self.backend.move_pose(pose, blocking=blocking)

    def stop(self) -> None:
        self.backend.stop()

    def set_speed_scale(self, scale: float) -> None:
        self.backend.set_speed_scale(scale)
