from __future__ import annotations

from dataclasses import replace

from embodiment_core.logger import get_logger
from embodiment_core.types import JointState, Pose

from .interfaces import JakaBackend


class MockJakaBackend(JakaBackend):
    def __init__(self, joint_names: list[str] | None = None) -> None:
        names = joint_names or [f"joint_{index + 1}" for index in range(6)]
        self._joint_state = JointState(names=names, positions=[0.0] * len(names))
        self._last_pose = Pose(
            position=[0.35, 0.0, 0.2],
            orientation_xyzw=[0.0, 0.0, 0.0, 1.0],
            frame_id="jaka_base",
        )
        self._connected = False
        self._speed_scale = 0.2
        self._logger = get_logger("MockJakaBackend")

    def connect(self) -> bool:
        self._connected = True
        self._logger.info("Connected to mock JAKA backend.")
        return True

    def get_joint_state(self) -> JointState:
        if not self._connected:
            raise RuntimeError("JAKA backend is not connected.")
        return replace(self._joint_state)

    def move_joints(self, joints: list[float], blocking: bool = True) -> bool:
        if len(joints) != len(self._joint_state.names):
            raise ValueError("Joint vector length mismatch.")
        self._joint_state.positions = list(joints)
        self._logger.info("Mock move_joints called: %s blocking=%s", joints, blocking)
        return True

    def move_pose(self, pose: Pose, blocking: bool = True) -> bool:
        self._last_pose = pose
        self._logger.info("Mock move_pose called: %s blocking=%s", pose, blocking)
        return True

    def stop(self) -> None:
        self._logger.warning("Mock stop called.")

    def set_speed_scale(self, scale: float) -> None:
        if scale <= 0.0 or scale > 1.0:
            raise ValueError("Speed scale must be within (0.0, 1.0].")
        self._speed_scale = scale
        self._logger.info("Mock speed scale set to %.3f", scale)

