from __future__ import annotations

from pathlib import Path
import time

import mujoco
import numpy as np

from embodiment_core.robot_limits import (
    DEFAULT_JOINT_LIMIT_MARGIN_RAD,
    JAKA_MINI2_JOINT_LIMITS_RAD,
    safe_jaka_mini2_joint_limits_rad,
)

DEFAULT_MJCF = Path("data/sim_assets/jaka_rh56_visual_coacd.xml")
MJCF_ARM_JOINT_NAMES = [f"jaka_joint_{index}" for index in range(1, 7)]
PALM_BODY_NAME = "rh56_R_hand_base_link"
IDENTITY_QUAT_WXYZ = np.asarray([1.0, 0.0, 0.0, 0.0], dtype=np.float64)


def safe_joint_limits_rad(
    margin_rad: float = DEFAULT_JOINT_LIMIT_MARGIN_RAD,
) -> list[tuple[float, float]]:
    return list(safe_jaka_mini2_joint_limits_rad(margin_rad))


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


def _unit_quat_wxyz(values: list[float] | np.ndarray) -> np.ndarray:
    quat = np.asarray(values, dtype=np.float64)
    if quat.shape != (4,):
        raise ValueError("quaternion must contain 4 values in wxyz order.")
    norm = float(np.linalg.norm(quat))
    if norm <= 1e-9:
        return IDENTITY_QUAT_WXYZ.copy()
    return quat / norm


def _quat_conjugate_wxyz(quat: list[float] | np.ndarray) -> np.ndarray:
    q = _unit_quat_wxyz(quat)
    return np.asarray([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)


def _quat_multiply_wxyz(a: list[float] | np.ndarray, b: list[float] | np.ndarray) -> np.ndarray:
    aw, ax, ay, az = _unit_quat_wxyz(a)
    bw, bx, by, bz = _unit_quat_wxyz(b)
    return _unit_quat_wxyz(
        np.asarray(
            [
                aw * bw - ax * bx - ay * by - az * bz,
                aw * bx + ax * bw + ay * bz - az * by,
                aw * by - ax * bz + ay * bw + az * bx,
                aw * bz + ax * by - ay * bx + az * bw,
            ],
            dtype=np.float64,
        )
    )


def _quat_to_rotvec_wxyz(quat: list[float] | np.ndarray) -> np.ndarray:
    q = _unit_quat_wxyz(quat)
    vector = q[1:]
    sin_half = float(np.linalg.norm(vector))
    if sin_half <= 1e-9:
        return np.zeros(3, dtype=np.float64)
    angle = 2.0 * np.arctan2(sin_half, float(q[0]))
    if angle > np.pi:
        angle -= 2.0 * np.pi
    return vector / sin_half * angle


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
        adaptive_damping_sigma_start: float = 0.0,
        adaptive_damping_sigma_full: float = 0.0,
        adaptive_damping_max: float | None = None,
        target_workspace_radius_m: float = 0.0,
        joint_limit_margin_rad: float = DEFAULT_JOINT_LIMIT_MARGIN_RAD,
        orientation_ik_weight: float = 0.35,
    ) -> None:
        if len(initial_arm_joints_rad) != 6:
            raise ValueError("initial_arm_joints_rad must contain 6 values.")
        self.model = mujoco.MjModel.from_xml_path(str(Path(mjcf_path).resolve()))
        self.data = mujoco.MjData(self.model)
        self.ik_gain = float(ik_gain)
        self.ik_damping = abs(float(ik_damping))
        self.ik_max_step_rad = abs(float(ik_max_step_rad))
        self.ik_iterations = max(1, int(ik_iterations))
        self.adaptive_damping_sigma_start = max(
            0.0, float(adaptive_damping_sigma_start)
        )
        self.adaptive_damping_sigma_full = max(
            0.0, float(adaptive_damping_sigma_full)
        )
        self.adaptive_damping_max = (
            self.ik_damping
            if adaptive_damping_max is None
            else max(self.ik_damping, abs(float(adaptive_damping_max)))
        )
        if self.adaptive_damping_sigma_start > 0.0 and not (
            0.0 < self.adaptive_damping_sigma_full
            < self.adaptive_damping_sigma_start
        ):
            raise ValueError(
                "adaptive damping requires 0 < sigma_full < sigma_start"
            )
        self.last_effective_damping = self.ik_damping
        self.target_workspace_radius_m = abs(float(target_workspace_radius_m))
        self.joint_limit_margin_rad = abs(float(joint_limit_margin_rad))
        self.orientation_ik_weight = max(0.0, float(orientation_ik_weight))
        self.last_position_target_ik_iterations_ns = 0
        self.last_position_target_final_fk_ns = 0
        self.last_position_target_iterations_completed = 0

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
        self.target_palm_quaternion_wxyz: np.ndarray | None = None
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

    @property
    def current_palm_quaternion_wxyz(self) -> np.ndarray:
        quat = np.zeros(4, dtype=np.float64)
        mujoco.mju_mat2Quat(quat, self.data.xmat[self.palm_body_id])
        return _unit_quat_wxyz(quat)

    @property
    def target_rotation_error_rad(self) -> float | None:
        if self.target_palm_quaternion_wxyz is None:
            return None
        delta = _quat_multiply_wxyz(
            self.target_palm_quaternion_wxyz,
            _quat_conjugate_wxyz(self.current_palm_quaternion_wxyz),
        )
        return float(np.linalg.norm(_quat_to_rotvec_wxyz(delta)))

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
        self.target_palm_quaternion_wxyz = None
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
        self.target_palm_quaternion_wxyz = None
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
        palm_target_quaternion_wxyz: list[float] | None = None,
        wrist_roll_velocity_rad_s: float,
        dt: float,
        compute_deadline_ns: int | None = None,
    ) -> bool:
        if len(palm_target_position_m) != 3:
            raise ValueError("palm_target_position_m must contain 3 values.")
        dt = max(0.0, min(float(dt), 0.1))
        self.target_palm_position_m = np.asarray(palm_target_position_m, dtype=np.float64)
        self.target_palm_quaternion_wxyz = (
            None
            if palm_target_quaternion_wxyz is None
            else _unit_quat_wxyz(palm_target_quaternion_wxyz)
        )
        self._clip_target_workspace()
        self.arm_joints_rad[5] += float(wrist_roll_velocity_rad_s) * dt
        self._clip_arm_joints()
        self.last_position_target_ik_iterations_ns = 0
        self.last_position_target_final_fk_ns = 0
        self.last_position_target_iterations_completed = 0
        for _ in range(self.ik_iterations):
            if (
                compute_deadline_ns is not None
                and time.perf_counter_ns() >= compute_deadline_ns
            ):
                return False
            started_ns = time.perf_counter_ns()
            self._solve_position_ik()
            self.last_position_target_ik_iterations_ns += (
                time.perf_counter_ns() - started_ns
            )
            self.last_position_target_iterations_completed += 1
        started_ns = time.perf_counter_ns()
        self._forward()
        self.last_position_target_final_fk_ns = time.perf_counter_ns() - started_ns
        return True

    def hold_current_target(self) -> None:
        self.target_palm_position_m = self.current_palm_position_m.copy()
        self.target_palm_quaternion_wxyz = self.current_palm_quaternion_wxyz.copy()
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
        self.target_palm_quaternion_wxyz = None
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
        pos_error = self.target_palm_position_m - self.current_palm_position_m
        if self.target_palm_quaternion_wxyz is None or self.orientation_ik_weight <= 0.0:
            error = pos_error
            if np.linalg.norm(error) < 1e-5:
                return
            jacp = np.zeros((3, self.model.nv), dtype=np.float64)
            jacr = np.zeros((3, self.model.nv), dtype=np.float64)
            mujoco.mj_jacBody(self.model, self.data, jacp, jacr, self.palm_body_id)
            arm_jacobian = jacp[:, self.arm_dof_ids]
            damping = self._effective_damping(arm_jacobian)
            lhs = arm_jacobian @ arm_jacobian.T + (damping**2) * np.eye(3)
            delta = arm_jacobian.T @ np.linalg.solve(lhs, self.ik_gain * error)
            self.arm_joints_rad += np.clip(delta, -self.ik_max_step_rad, self.ik_max_step_rad)
            self._clip_arm_joints()
            return
        jacp = np.zeros((3, self.model.nv), dtype=np.float64)
        jacr = np.zeros((3, self.model.nv), dtype=np.float64)
        mujoco.mj_jacBody(self.model, self.data, jacp, jacr, self.palm_body_id)
        rot_delta = _quat_multiply_wxyz(
            self.target_palm_quaternion_wxyz,
            _quat_conjugate_wxyz(self.current_palm_quaternion_wxyz),
        )
        rot_error = _quat_to_rotvec_wxyz(rot_delta)
        weight = self.orientation_ik_weight
        error = np.concatenate([pos_error, weight * rot_error])
        if np.linalg.norm(error) < 1e-5:
            return
        arm_jacobian = np.vstack([jacp[:, self.arm_dof_ids], weight * jacr[:, self.arm_dof_ids]])
        damping = self._effective_damping(arm_jacobian)
        lhs = arm_jacobian @ arm_jacobian.T + (damping**2) * np.eye(6)
        delta = arm_jacobian.T @ np.linalg.solve(lhs, self.ik_gain * error)
        self.arm_joints_rad += np.clip(delta, -self.ik_max_step_rad, self.ik_max_step_rad)
        self._clip_arm_joints()

    def _effective_damping(self, jacobian: np.ndarray) -> float:
        """Smoothly increase DLS damping only as solver Jacobian quality falls."""

        damping = self.ik_damping
        if self.adaptive_damping_sigma_start > 0.0:
            sigma_min = float(np.linalg.svd(jacobian, compute_uv=False)[-1])
            ratio = np.clip(
                (self.adaptive_damping_sigma_start - sigma_min)
                / (
                    self.adaptive_damping_sigma_start
                    - self.adaptive_damping_sigma_full
                ),
                0.0,
                1.0,
            )
            smooth = float(ratio * ratio * (3.0 - 2.0 * ratio))
            damping += smooth * (self.adaptive_damping_max - self.ik_damping)
        self.last_effective_damping = damping
        return damping
