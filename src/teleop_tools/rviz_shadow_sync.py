from __future__ import annotations

from typing import Any


ARM_JOINT_NAMES = [f"joint_{index}" for index in range(1, 7)]
RVIZ_ARM_JOINT_NAMES = [f"jaka_joint_{index}" for index in range(1, 7)]


def extract_arm_joints_from_joint_state(message: Any) -> list[float] | None:
    names = list(getattr(message, "name", []))
    positions = list(getattr(message, "position", []))
    if len(positions) < 6:
        return None
    by_name = {str(name): float(value) for name, value in zip(names, positions, strict=False)}
    if all(name in by_name for name in ARM_JOINT_NAMES):
        return [by_name[name] for name in ARM_JOINT_NAMES]
    if all(name in by_name for name in RVIZ_ARM_JOINT_NAMES):
        return [by_name[name] for name in RVIZ_ARM_JOINT_NAMES]
    return [float(value) for value in positions[:6]]
