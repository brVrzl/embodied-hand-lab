"""Offline MuJoCo joint-reach and command pre-shape smoke harness."""

from __future__ import annotations

import copy
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Mapping

import numpy as np

from .config import BenchmarkConfig, HAND_ACTUATOR_ORDER


BENCHMARK_RESULT_SCHEMA = "embodied_lab.benchmark_result.v1"

_LIMITATIONS = [
    "Offline MuJoCo execution only; no physical JAKA, RH56, Quest, or camera is accessed.",
    "Arm success measures six simulated actuated joint positions against one bounded target.",
    "Hand pre-shape success measures the six actuator-driven MuJoCo joints, not every coupled passive joint.",
    "The test does not measure grasp acquisition, lift, object retention, tactile sensing, or sim-to-real transfer.",
    "Targets enter the MuJoCo adapter directly; this does not validate the complete Quest mapping, shared IK, or physical safety pipeline.",
]


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _relative_or_absolute(path: Path, root: Path) -> str:
    try:
        return path.relative_to(root).as_posix()
    except ValueError:
        return str(path)


def _invalid_result(
    base: dict[str, object],
    *,
    reason: str,
    action_bounds: Mapping[str, object],
) -> dict[str, object]:
    result = dict(base)
    result.update(
        {
            "status": "invalid",
            "failure_reason": reason,
            "steps_executed": 0,
            "action_bounds": dict(action_bounds),
            "metrics": {},
        }
    )
    return result


def run_mujoco_joint_reach_preshape(
    config: BenchmarkConfig,
    *,
    repository_root: str | Path,
) -> dict[str, object]:
    """Run a fresh deterministic MuJoCo plant for one fixed-horizon smoke task."""

    # These imports intentionally occur only when the offline benchmark runs.
    import mujoco

    from quest_jaka_sim import JakaMujocoSimulation, ReplayConfig
    from quest_jaka_sim.simulation import build_viewer_mjcf

    root = Path(repository_root).resolve()
    replay = ReplayConfig.load(config.replay_config_path)
    model_path = replay.mjcf_path
    if not model_path.is_absolute():
        model_path = root / model_path
    model_path = model_path.resolve()
    if not model_path.is_file():
        raise FileNotFoundError(f"MuJoCo model does not exist: {model_path}")

    rng = np.random.default_rng(config.seed)
    target_jitter = rng.uniform(
        -np.asarray(config.arm_target_jitter_rad, dtype=np.float64),
        np.asarray(config.arm_target_jitter_rad, dtype=np.float64),
    )

    with tempfile.TemporaryDirectory(prefix="embodied-lab-benchmark-") as directory:
        generated_model = build_viewer_mjcf(
            model_path, Path(directory) / "benchmark_model.xml"
        )
        simulation = JakaMujocoSimulation(replay, mjcf_path=generated_model)
        initial_arm = simulation.arm_joints_rad
        arm_target = (
            initial_arm
            + np.asarray(config.arm_target_offset_rad, dtype=np.float64)
            + target_jitter
        )
        initial_tcp = np.asarray(
            simulation.current_tcp_pose.position_m, dtype=np.float64
        )

        arm_ranges = simulation.model.jnt_range[
            simulation.arm_joint_ids
        ].astype(np.float64, copy=True)
        arm_limited = simulation.model.jnt_limited[
            simulation.arm_joint_ids
        ].astype(bool, copy=True)
        hand_ranges = simulation.hand_ctrl_ranges.astype(
            np.float64, copy=True
        )
        action_bounds: dict[str, object] = {
            "arm_model_joint_range_rad": arm_ranges.tolist(),
            "arm_joints_limited": arm_limited.tolist(),
            "hand_actuator_control_range_rad": hand_ranges.tolist(),
        }

        base: dict[str, object] = {
            "schema_version": BENCHMARK_RESULT_SCHEMA,
            "benchmark_id": config.benchmark_id,
            "benchmark_type": "mujoco_joint_reach_preshape_smoke",
            "seed": config.seed,
            "config_snapshot": {
                "benchmark": config.snapshot(repository_root=root),
                "simulation_replay": copy.deepcopy(dict(replay.raw)),
            },
            "simulation": {
                "backend": "mujoco",
                "mujoco_version": getattr(mujoco, "__version__", None),
                "replay_config_path": _relative_or_absolute(
                    config.replay_config_path, root
                ),
                "replay_config_sha256": _sha256(config.replay_config_path),
                "model_path": _relative_or_absolute(model_path, root),
                "model_sha256": _sha256(model_path),
                "model_timestep_s": float(simulation.model.opt.timestep),
                "zero_gravity": replay.zero_gravity,
                "reset_method": "fresh_JakaMujocoSimulation_instance",
            },
            "limitations": list(_LIMITATIONS),
            "sampled_action": {
                "arm_initial_rad": initial_arm.tolist(),
                "arm_target_offset_jitter_rad": target_jitter.tolist(),
                "arm_target_rad": arm_target.tolist(),
                "hand_target_rad": config.hand_target_by_name,
            },
        }

        if not bool(np.all(arm_limited)):
            return _invalid_result(
                base,
                reason="arm_model_contains_unlimited_benchmark_joint",
                action_bounds=action_bounds,
            )
        if bool(
            np.any(arm_target < arm_ranges[:, 0])
            or np.any(arm_target > arm_ranges[:, 1])
        ):
            return _invalid_result(
                base,
                reason="arm_target_out_of_model_joint_range",
                action_bounds=action_bounds,
            )
        if not simulation.hand_available:
            return _invalid_result(
                base,
                reason="six_channel_hand_actuators_unavailable",
                action_bounds=action_bounds,
            )
        hand_target = np.asarray(config.hand_target_rad, dtype=np.float64)
        if bool(
            np.any(hand_target < hand_ranges[:, 0])
            or np.any(hand_target > hand_ranges[:, 1])
        ):
            return _invalid_result(
                base,
                reason="hand_target_out_of_model_control_range",
                action_bounds=action_bounds,
            )

        hand_joint_ids = simulation.model.actuator_trnid[
            simulation.hand_actuator_ids, 0
        ]
        hand_qpos_ids = simulation.model.jnt_qposadr[hand_joint_ids]
        simulation.set_accepted_arm_joint_target(
            tuple(float(value) for value in arm_target)
        )
        simulation.set_hand_actuator_target(config.hand_target_by_name)

        peak_arm_joint_speed = np.zeros(6, dtype=np.float64)
        peak_arm_control_speed = np.zeros(6, dtype=np.float64)
        sum_squared_arm_control_speed = 0.0
        arm_control_speed_samples = 0
        maximum_tcp_displacement_m = 0.0
        maximum_contact_count = int(simulation.data.ncon)
        first_success_step: int | None = None
        previous_control = simulation.data.ctrl[
            simulation.arm_actuator_ids
        ].copy()

        for step_index in range(config.step_count):
            simulation.step(config.control_period_s)
            arm_position = simulation.arm_joints_rad
            hand_position = simulation.data.qpos[hand_qpos_ids].copy()
            arm_control = simulation.data.ctrl[
                simulation.arm_actuator_ids
            ].copy()
            control_speed = (
                arm_control - previous_control
            ) / config.control_period_s
            previous_control = arm_control
            peak_arm_control_speed = np.maximum(
                peak_arm_control_speed, np.abs(control_speed)
            )
            sum_squared_arm_control_speed += float(
                np.dot(control_speed, control_speed)
            )
            arm_control_speed_samples += control_speed.size
            peak_arm_joint_speed = np.maximum(
                peak_arm_joint_speed,
                np.abs(simulation.data.qvel[simulation.arm_dof_ids]),
            )
            tcp_displacement = float(
                np.linalg.norm(
                    np.asarray(
                        simulation.current_tcp_pose.position_m,
                        dtype=np.float64,
                    )
                    - initial_tcp
                )
            )
            maximum_tcp_displacement_m = max(
                maximum_tcp_displacement_m, tcp_displacement
            )
            maximum_contact_count = max(
                maximum_contact_count, int(simulation.data.ncon)
            )
            arm_error = float(np.max(np.abs(arm_target - arm_position)))
            hand_error = float(
                np.max(np.abs(hand_target - hand_position))
            )
            if (
                first_success_step is None
                and arm_error <= config.arm_success_tolerance_rad
                and hand_error <= config.hand_success_tolerance_rad
            ):
                first_success_step = step_index + 1

        final_arm = simulation.arm_joints_rad
        final_hand = simulation.data.qpos[hand_qpos_ids].copy()
        final_tcp = np.asarray(
            simulation.current_tcp_pose.position_m, dtype=np.float64
        )
        final_arm_error = np.abs(arm_target - final_arm)
        final_hand_error = np.abs(hand_target - final_hand)
        arm_reached = bool(
            np.max(final_arm_error) <= config.arm_success_tolerance_rad
        )
        hand_reached = bool(
            np.max(final_hand_error) <= config.hand_success_tolerance_rad
        )
        passed = arm_reached and hand_reached
        completion_time_s = (
            None
            if first_success_step is None
            else first_success_step * config.control_period_s
        )
        rms_control_speed = math.sqrt(
            sum_squared_arm_control_speed / arm_control_speed_samples
        )

        result = dict(base)
        result.update(
            {
                "status": "passed" if passed else "failed",
                "failure_reason": None
                if passed
                else "target_tolerance_not_met",
                "steps_executed": config.step_count,
                "action_bounds": action_bounds,
                "metrics": {
                    "arm_reached": arm_reached,
                    "hand_preshape_reached": hand_reached,
                    "first_combined_success_step": first_success_step,
                    "completion_time_s": completion_time_s,
                    "final_arm_joint_position_rad": final_arm.tolist(),
                    "final_arm_absolute_error_rad": final_arm_error.tolist(),
                    "final_arm_max_absolute_error_rad": float(
                        np.max(final_arm_error)
                    ),
                    "final_arm_l2_error_rad": float(
                        np.linalg.norm(final_arm_error)
                    ),
                    "peak_arm_joint_speed_rad_s": peak_arm_joint_speed.tolist(),
                    "peak_arm_control_speed_rad_s": peak_arm_control_speed.tolist(),
                    "rms_arm_control_speed_rad_s": rms_control_speed,
                    "final_hand_actuated_joint_position_rad": final_hand.tolist(),
                    "final_hand_absolute_error_rad": final_hand_error.tolist(),
                    "final_hand_max_absolute_error_rad": float(
                        np.max(final_hand_error)
                    ),
                    "final_tcp_displacement_m": float(
                        np.linalg.norm(final_tcp - initial_tcp)
                    ),
                    "maximum_tcp_displacement_m": maximum_tcp_displacement_m,
                    "maximum_contact_count": maximum_contact_count,
                    "simulated_duration_s": float(simulation.data.time),
                },
            }
        )
        return result


def write_benchmark_result(
    output_path: str | Path, result: Mapping[str, object]
) -> Path:
    """Atomically replace one strict JSON benchmark result."""

    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary_path = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(
                result,
                stream,
                allow_nan=False,
                indent=2,
                sort_keys=True,
            )
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, path)
    except BaseException:
        temporary_path.unlink(missing_ok=True)
        raise
    return path
