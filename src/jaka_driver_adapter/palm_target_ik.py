from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np


DEFAULT_MJCF = Path("data/sim_assets/jaka_rh56.xml")
MJCF_ARM_JOINT_NAMES = [f"jaka_joint_{index}" for index in range(1, 7)]
PALM_BODY_NAME = "rh56_R_hand_base_link"
JAKA_MINI2_JOINT_LIMITS_RAD = [
    (-2.0 * np.pi, 2.0 * np.pi),
    (np.deg2rad(-125.0), np.deg2rad(125.0)),
    (np.deg2rad(-130.0), np.deg2rad(130.0)),
    (-2.0 * np.pi, 2.0 * np.pi),
    (np.deg2rad(-120.0), np.deg2rad(120.0)),
    (-2.0 * np.pi, 2.0 * np.pi),
]
DEFAULT_JOINT_LIMIT_MARGIN_RAD = float(np.deg2rad(5.0))


def safe_joint_limits_rad(
    margin_rad: float = DEFAULT_JOINT_LIMIT_MARGIN_RAD,
) -> list[tuple[float, float]]:
    margin = max(0.0, float(margin_rad))
    return [(low + margin, high - margin) for low, high in JAKA_MINI2_JOINT_LIMITS_RAD]


def clip_joints_to_safe_limits(
    joints_rad: list[float] | np.ndarray,
    *,
    margin_rad: float = DEFAULT_JOINT_LIMIT_MARGIN_RAD,
) -> tuple[np.ndarray, list[int]]:
    clipped = np.asarray(joints_rad, dtype=np.float64).copy()
    limited: list[int] = []
    for index, (low, high) in enumerate(safe_joint_limits_rad(margin_rad)):
        before = float(clipped[index])
        clipped[index] = np.clip(before, low, high)
        if clipped[index] != before:
            limited.append(index)
    return clipped, limited


def joint_limit_margin_blockers(
    joints_rad: list[float] | np.ndarray,
    *,
    margin_rad: float = DEFAULT_JOINT_LIMIT_MARGIN_RAD,
) -> list[str]:
    blockers: list[str] = []
    for index, value in enumerate(np.asarray(joints_rad, dtype=np.float64)):
        low, high = safe_joint_limits_rad(margin_rad)[index]
        if value < low:
            blockers.append(f"joint_{index + 1}_below_safe_limit")
        elif value > high:
            blockers.append(f"joint_{index + 1}_above_safe_limit")
    return blockers


class PalmTargetIkState:
    """Track a bounded palm position target with damped least-squares IK."""

    def __init__(
        self,
        initial_arm_joints_rad: list[float],
        *,
        mjcf_path: str | Path = DEFAULT_MJCF,
        ik_gain: float = 0.65,
        ik_damping: float = 0.05,
        ik_max_step_rad: float = 0.025,
        ik_iterations: int = 4,
        target_workspace_radius_m: float = 0.0,
        joint_limit_margin_rad: float = DEFAULT_JOINT_LIMIT_MARGIN_RAD,
    ) -> None:
        if len(initial_arm_joints_rad) != 6:
            raise ValueError("initial_arm_joints_rad must contain 6 values.")
        self.model = mujoco.MjModel.from_xml_path(str(Path(mjcf_path).resolve()))
        self.data = mujoco.MjData(self.model)
        self.ik_gain = float(ik_gain)
        self.ik_damping = abs(float(ik_damping))
        self.ik_max_step_rad = abs(float(ik_max_step_rad))
        self.ik_iterations = max(1, int(ik_iterations))
        self.target_workspace_radius_m = abs(float(target_workspace_radius_m))
        self.joint_limit_margin_rad = abs(float(joint_limit_margin_rad))

        self.arm_joint_ids = np.asarray(
            [
                self._required_id(mujoco.mjtObj.mjOBJ_JOINT, name)
                for name in MJCF_ARM_JOINT_NAMES
            ],
            dtype=np.int32,
        )
        self.arm_qpos_ids = np.asarray(self.model.jnt_qposadr[self.arm_joint_ids], dtype=np.int32)
        self.arm_dof_ids = np.asarray(self.model.jnt_dofadr[self.arm_joint_ids], dtype=np.int32)
        self.palm_body_id = self._required_id(mujoco.mjtObj.mjOBJ_BODY, PALM_BODY_NAME)

        self.arm_joints_rad = np.asarray(initial_arm_joints_rad, dtype=np.float64)
        self.initial_palm_position_m = np.zeros(3, dtype=np.float64)
        self.target_palm_position_m = np.zeros(3, dtype=np.float64)
        self.target_workspace_limited = False
        self.joint_limit_limited = False
        self.limited_joint_indices_1_based: list[int] = []
        self.reset_session(initial_arm_joints_rad)

    @property
    def current_palm_position_m(self) -> np.ndarray:
        return self.data.xpos[self.palm_body_id].copy()

    @property
    def target_error_m(self) -> float:
        return float(np.linalg.norm(self.target_palm_position_m - self.current_palm_position_m))

    def apply(
        self,
        *,
        palm_velocity_m_s: list[float],
        wrist_roll_velocity_rad_s: float,
        dt: float,
    ) -> None:
        if len(palm_velocity_m_s) != 3:
            raise ValueError("palm_velocity_m_s must contain 3 values.")
        dt = max(0.0, min(float(dt), 0.1))
        self.target_palm_position_m += np.asarray(palm_velocity_m_s, dtype=np.float64) * dt
        self._clip_target_workspace()
        self.arm_joints_rad[5] += float(wrist_roll_velocity_rad_s) * dt
        self._clip_arm_joints()
        for _ in range(self.ik_iterations):
            self._solve_position_ik()
        self._forward()

    def apply_short_horizon(
        self,
        *,
        palm_velocity_m_s: list[float],
        wrist_roll_velocity_rad_s: float,
        dt: float,
        horizon_sec: float,
        max_target_offset_m: float,
    ) -> None:
        if len(palm_velocity_m_s) != 3:
            raise ValueError("palm_velocity_m_s must contain 3 values.")
        dt = max(0.0, min(float(dt), 0.1))
        horizon_sec = max(0.0, min(float(horizon_sec), 0.25))
        offset = np.asarray(palm_velocity_m_s, dtype=np.float64) * horizon_sec
        max_target_offset_m = abs(float(max_target_offset_m))
        offset_norm = float(np.linalg.norm(offset))
        if max_target_offset_m > 0.0 and offset_norm > max_target_offset_m:
            offset *= max_target_offset_m / offset_norm
        self.target_palm_position_m = self.current_palm_position_m + offset
        self._clip_target_workspace()
        self.arm_joints_rad[5] += float(wrist_roll_velocity_rad_s) * dt
        self._clip_arm_joints()
        for _ in range(self.ik_iterations):
            self._solve_position_ik()
        self._forward()

    def apply_position_target(
        self,
        *,
        palm_target_position_m: list[float],
        wrist_roll_velocity_rad_s: float,
        dt: float,
    ) -> None:
        if len(palm_target_position_m) != 3:
            raise ValueError("palm_target_position_m must contain 3 values.")
        dt = max(0.0, min(float(dt), 0.1))
        self.target_palm_position_m = np.asarray(palm_target_position_m, dtype=np.float64)
        self._clip_target_workspace()
        self.arm_joints_rad[5] += float(wrist_roll_velocity_rad_s) * dt
        self._clip_arm_joints()
        for _ in range(self.ik_iterations):
            self._solve_position_ik()
        self._forward()

    def hold_current_target(self) -> None:
        self.target_palm_position_m = self.current_palm_position_m.copy()
        self.target_workspace_limited = False

    def set_arm_joints_rad(self, joints: list[float]) -> None:
        if len(joints) != 6:
            raise ValueError("joints must contain 6 values.")
        self.arm_joints_rad[:] = joints
        self._clip_arm_joints()
        self._forward()

    def reset_session(self, joints: list[float]) -> None:
        self.set_arm_joints_rad(joints)
        self.initial_palm_position_m = self.current_palm_position_m.copy()
        self.target_palm_position_m = self.current_palm_position_m.copy()
        self.target_workspace_limited = False

    def _required_id(self, object_type: mujoco.mjtObj, name: str) -> int:
        object_id = mujoco.mj_name2id(self.model, object_type, name)
        if object_id < 0:
            raise KeyError(f"Missing MuJoCo object {name!r}.")
        return int(object_id)

    def _clip_target_workspace(self) -> None:
        offset = self.target_palm_position_m - self.initial_palm_position_m
        distance = float(np.linalg.norm(offset))
        self.target_workspace_limited = False
        if distance > self.target_workspace_radius_m > 0.0:
            self.target_palm_position_m = (
                self.initial_palm_position_m + offset * self.target_workspace_radius_m / distance
            )
            self.target_workspace_limited = True

    def _clip_arm_joints(self) -> None:
        self.arm_joints_rad, limited = clip_joints_to_safe_limits(
            self.arm_joints_rad,
            margin_rad=self.joint_limit_margin_rad,
        )
        self.joint_limit_limited = bool(limited)
        self.limited_joint_indices_1_based = [index + 1 for index in limited]

    def _forward(self) -> None:
        self.data.qpos[self.arm_qpos_ids] = self.arm_joints_rad
        self.data.qvel[self.arm_dof_ids] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def _solve_position_ik(self) -> None:
        self._forward()
        error = self.target_palm_position_m - self.current_palm_position_m
        if np.linalg.norm(error) < 1e-5:
            return
        jacp = np.zeros((3, self.model.nv), dtype=np.float64)
        jacr = np.zeros((3, self.model.nv), dtype=np.float64)
        mujoco.mj_jacBody(self.model, self.data, jacp, jacr, self.palm_body_id)
        arm_jacobian = jacp[:, self.arm_dof_ids]
        lhs = arm_jacobian @ arm_jacobian.T + (self.ik_damping**2) * np.eye(3)
        delta = arm_jacobian.T @ np.linalg.solve(lhs, self.ik_gain * error)
        self.arm_joints_rad += np.clip(delta, -self.ik_max_step_rad, self.ik_max_step_rad)
        self._clip_arm_joints()
