from __future__ import annotations

from abc import ABC, abstractmethod

from embodiment_core.types import JointState, Pose


class JakaBackend(ABC):
    @abstractmethod
    def connect(self) -> bool:
        raise NotImplementedError

    @abstractmethod
    def get_joint_state(self) -> JointState:
        raise NotImplementedError

    @abstractmethod
    def move_joints(self, joints: list[float], blocking: bool = True) -> bool:
        raise NotImplementedError

    @abstractmethod
    def move_pose(self, pose: Pose, blocking: bool = True) -> bool:
        raise NotImplementedError

    @abstractmethod
    def stop(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def set_speed_scale(self, scale: float) -> None:
        raise NotImplementedError

