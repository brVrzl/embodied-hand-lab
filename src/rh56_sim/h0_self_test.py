"""Low-amplitude MuJoCo-only H0 actuator self-test for the mounted RH56DFX."""

from __future__ import annotations

from dataclasses import dataclass
import importlib
import json
import math
from pathlib import Path
import time
from typing import TextIO

import mujoco
import numpy as np

from .model import RH56_CHANNELS, Rh56SimChannel


@dataclass(frozen=True, slots=True)
class H0RunResult:
    completed: bool
    interrupted: bool
    log_path: Path
    step_count: int
    invalid_count: int
    saturation_count: int
    arm_target_unchanged: bool
    completed_channels: tuple[str, ...]
    initial_penetrating_contacts: tuple[tuple[str, str, float], ...]


@dataclass(frozen=True, slots=True)
class _BoundChannel:
    spec: Rh56SimChannel
    actuator_id: int
    joint_id: int
    qpos_id: int
    joint_range: tuple[float, float]
    ctrl_range: tuple[float, float]
    effective_range: tuple[float, float]
    neutral_ctrl: float


class Rh56H0SelfTest:
    """Exercise one semantic hand channel at a time without any I/O transport."""

    def __init__(
        self,
        *,
        model_path: Path,
        log_path: Path,
        cycle_seconds: float = 2.0,
        amplitude_scale: float = 0.15,
        repeat: int = 1,
        initial_arm_joints_rad: tuple[float, ...] | None = None,
    ) -> None:
        if not math.isfinite(cycle_seconds) or cycle_seconds <= 0.0:
            raise ValueError("cycle_seconds must be finite and positive")
        if not math.isfinite(amplitude_scale) or not 0.0 < amplitude_scale <= 0.20:
            raise ValueError("amplitude_scale must be in (0, 0.20]")
        if repeat <= 0:
            raise ValueError("repeat must be positive")

        self.model_path = model_path.resolve()
        self.log_path = log_path.resolve()
        self.cycle_seconds = float(cycle_seconds)
        self.amplitude_scale = float(amplitude_scale)
        self.repeat = int(repeat)
        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = mujoco.MjData(self.model)
        self.channels = tuple(self._bind_channel(spec) for spec in RH56_CHANNELS)
        self.arm_actuator_ids = np.asarray(
            [
                self._named_id(mujoco.mjtObj.mjOBJ_ACTUATOR, f"jaka_joint_{index}_act")
                for index in range(1, 7)
            ],
            dtype=np.int32,
        )
        self.arm_joint_ids = np.asarray(
            [
                self._named_id(mujoco.mjtObj.mjOBJ_JOINT, f"jaka_joint_{index}")
                for index in range(1, 7)
            ],
            dtype=np.int32,
        )
        if self.model.nu != 12 or len({channel.actuator_id for channel in self.channels}) != 6:
            raise RuntimeError("H0 requires exactly six arm and six unique RH56 actuators")

        if initial_arm_joints_rad is not None:
            if len(initial_arm_joints_rad) != 6 or not all(
                math.isfinite(value) for value in initial_arm_joints_rad
            ):
                raise ValueError("initial_arm_joints_rad must contain six finite values")
            arm_qpos_ids = self.model.jnt_qposadr[self.arm_joint_ids]
            self.data.qpos[arm_qpos_ids] = np.asarray(initial_arm_joints_rad, dtype=float)
            self.data.ctrl[self.arm_actuator_ids] = np.asarray(
                initial_arm_joints_rad, dtype=float
            )

        for channel in self.channels:
            self.data.qpos[channel.qpos_id] = channel.neutral_ctrl
            self.data.ctrl[channel.actuator_id] = channel.neutral_ctrl
        mujoco.mj_forward(self.model, self.data)
        self.initial_arm_target = self.data.ctrl[self.arm_actuator_ids].copy()
        self.initial_hand_target = np.asarray(
            [self.data.ctrl[channel.actuator_id] for channel in self.channels], dtype=float
        )
        self.initial_penetrating_contacts = self._penetrating_contacts()

    def _named_id(self, object_type: mujoco.mjtObj, name: str) -> int:
        object_id = mujoco.mj_name2id(self.model, object_type, name)
        if object_id < 0:
            raise RuntimeError(f"required MuJoCo object is missing: {name}")
        return int(object_id)

    def _bind_channel(self, spec: Rh56SimChannel) -> _BoundChannel:
        actuator_id = self._named_id(mujoco.mjtObj.mjOBJ_ACTUATOR, spec.actuator)
        joint_id = self._named_id(mujoco.mjtObj.mjOBJ_JOINT, spec.joint)
        driven_joint_id = int(self.model.actuator_trnid[actuator_id, 0])
        if driven_joint_id != joint_id:
            raise RuntimeError(
                f"{spec.actuator} drives joint id {driven_joint_id}, expected {spec.joint}"
            )
        if not bool(self.model.jnt_limited[joint_id]):
            raise RuntimeError(f"H0 hand joint is not range-limited: {spec.joint}")
        if not bool(self.model.actuator_ctrllimited[actuator_id]):
            raise RuntimeError(f"H0 hand actuator is not control-limited: {spec.actuator}")
        joint_range = tuple(float(value) for value in self.model.jnt_range[joint_id])
        ctrl_range = tuple(float(value) for value in self.model.actuator_ctrlrange[actuator_id])
        effective = (max(joint_range[0], ctrl_range[0]), min(joint_range[1], ctrl_range[1]))
        if not effective[0] < effective[1]:
            raise RuntimeError(f"empty joint/control intersection for {spec.canonical}")
        qpos_id = int(self.model.jnt_qposadr[joint_id])
        neutral = float(np.clip(self.data.qpos[qpos_id], *effective))
        return _BoundChannel(
            spec,
            actuator_id,
            joint_id,
            qpos_id,
            joint_range,
            ctrl_range,
            effective,
            neutral,
        )

    def mapping_rows(self) -> tuple[dict[str, object], ...]:
        return tuple(
            {
                "canonical": channel.spec.canonical,
                "actuator": channel.spec.actuator,
                "joint": channel.spec.joint,
                "protocol_index": channel.spec.protocol_index,
                "raw_index": channel.spec.raw_index,
                "direction": channel.spec.mujoco_positive_motion,
                "hardware_raw_direction": channel.spec.hardware_raw_direction,
                "joint_range": list(channel.joint_range),
                "ctrl_range": list(channel.ctrl_range),
            }
            for channel in self.channels
        )

    def clipped_target(self, canonical: str, requested_ctrl: float) -> tuple[float, bool]:
        channel = next(
            (candidate for candidate in self.channels if candidate.spec.canonical == canonical),
            None,
        )
        if channel is None:
            raise KeyError(canonical)
        clipped = float(np.clip(requested_ctrl, *channel.effective_range))
        return clipped, not math.isclose(clipped, requested_ctrl, rel_tol=0.0, abs_tol=1e-12)

    def command_vector(self, canonical: str, requested_ctrl: float) -> np.ndarray:
        vector = self.initial_hand_target.copy()
        channel_index = next(
            index
            for index, channel in enumerate(self.channels)
            if channel.spec.canonical == canonical
        )
        vector[channel_index] = self.clipped_target(canonical, requested_ctrl)[0]
        return vector

    def run(self, *, viewer: bool = False) -> H0RunResult:
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        handle = self._launch_viewer() if viewer else None
        step_count = 0
        invalid_count = 0
        saturation_count = 0
        completed_channels: list[str] = []
        interrupted = False
        started_ns = time.monotonic_ns()
        next_sync = time.monotonic()

        try:
            with self.log_path.open("w", encoding="utf-8") as log_file:
                for repeat_index in range(self.repeat):
                    for channel in self.channels:
                        if handle is not None and not handle.is_running():
                            interrupted = True
                            break
                        counts = self._run_channel(
                            channel,
                            repeat_index=repeat_index,
                            started_ns=started_ns,
                            log_file=log_file,
                            handle=handle,
                            next_sync=next_sync,
                        )
                        step_count += counts[0]
                        invalid_count += counts[1]
                        saturation_count += counts[2]
                        next_sync = counts[3]
                        if counts[4]:
                            interrupted = True
                            break
                        completed_channels.append(channel.spec.canonical)
                    if interrupted:
                        break
        except KeyboardInterrupt:
            interrupted = True
        finally:
            self._restore_neutral()
            if handle is not None:
                handle.sync()
                handle.close()

        arm_unchanged = bool(
            np.array_equal(self.data.ctrl[self.arm_actuator_ids], self.initial_arm_target)
        )
        return H0RunResult(
            completed=not interrupted and len(completed_channels) == self.repeat * len(self.channels),
            interrupted=interrupted,
            log_path=self.log_path,
            step_count=step_count,
            invalid_count=invalid_count,
            saturation_count=saturation_count,
            arm_target_unchanged=arm_unchanged,
            completed_channels=tuple(completed_channels),
            initial_penetrating_contacts=self.initial_penetrating_contacts,
        )

    def _run_channel(
        self,
        channel: _BoundChannel,
        *,
        repeat_index: int,
        started_ns: int,
        log_file: TextIO,
        handle: object | None,
        next_sync: float,
    ) -> tuple[int, int, int, float, bool]:
        low, high = channel.effective_range
        span = high - low
        amplitude = span * self.amplitude_scale
        neutral = channel.neutral_ctrl
        positive = min(high, neutral + amplitude)
        negative = max(low, neutral - amplitude)
        negative_legal = negative < neutral - 1e-12
        phases = (
            ("initial", neutral, neutral),
            ("positive", neutral, positive),
            ("return_from_positive", positive, neutral),
            (
                "negative" if negative_legal else "negative_skipped_illegal",
                neutral,
                negative,
            ),
            ("return_from_negative", negative, neutral),
        )
        phase_steps = max(1, int(round(self.cycle_seconds / 5.0 / self.model.opt.timestep)))
        steps = invalid = saturations = 0

        for phase, start, end in phases:
            for phase_step in range(phase_steps):
                if handle is not None and not handle.is_running():
                    return steps, invalid, saturations, next_sync, True
                progress = (phase_step + 1) / phase_steps
                smooth = progress * progress * (3.0 - 2.0 * progress)
                requested = start + (end - start) * smooth
                clipped, saturated = self.clipped_target(channel.spec.canonical, requested)
                hand_target = self.command_vector(channel.spec.canonical, clipped)
                for index, bound in enumerate(self.channels):
                    self.data.ctrl[bound.actuator_id] = hand_target[index]
                mujoco.mj_step(self.model, self.data)
                actual = float(self.data.qpos[channel.qpos_id])
                has_invalid = not (
                    math.isfinite(requested)
                    and math.isfinite(clipped)
                    and math.isfinite(actual)
                    and np.all(np.isfinite(self.data.qpos))
                    and np.all(np.isfinite(self.data.ctrl))
                )
                invalid += int(has_invalid)
                saturations += int(saturated)
                steps += 1
                self._write_log(
                    log_file,
                    started_ns=started_ns,
                    repeat_index=repeat_index,
                    phase=phase,
                    progress=progress,
                    channel=channel,
                    requested=requested,
                    clipped=clipped,
                    actual=actual,
                    saturation=saturated,
                    invalid=has_invalid,
                )
                if handle is not None:
                    now = time.monotonic()
                    if now >= next_sync:
                        handle.sync()
                        next_sync = now + 1.0 / 60.0
                    target_wall = started_ns / 1e9 + self.data.time
                    remaining = target_wall - time.monotonic()
                    if remaining > 0.0:
                        time.sleep(min(remaining, self.model.opt.timestep))
        return steps, invalid, saturations, next_sync, False

    def _write_log(
        self,
        log_file: TextIO,
        *,
        started_ns: int,
        repeat_index: int,
        phase: str,
        progress: float,
        channel: _BoundChannel,
        requested: float,
        clipped: float,
        actual: float,
        saturation: bool,
        invalid: bool,
    ) -> None:
        record = {
            "timestamp": float(self.data.time),
            "host_monotonic_ns": time.monotonic_ns(),
            "elapsed_wall_s": (time.monotonic_ns() - started_ns) / 1e9,
            "repeat_index": repeat_index,
            "phase": phase,
            "canonical_channel": channel.spec.canonical,
            "actuator_name": channel.spec.actuator,
            "joint_name": channel.spec.joint,
            "requested_ctrl": requested,
            "clipped_ctrl": clipped,
            "actual_qpos": actual,
            "joint_range": list(channel.joint_range),
            "ctrl_range": list(channel.ctrl_range),
            "saturation": saturation,
            "invalid_nan": invalid,
            "phase_progress": progress,
        }
        log_file.write(json.dumps(record, sort_keys=True) + "\n")

    def _restore_neutral(self) -> None:
        for index, channel in enumerate(self.channels):
            self.data.ctrl[channel.actuator_id] = self.initial_hand_target[index]
        self.data.ctrl[self.arm_actuator_ids] = self.initial_arm_target
        mujoco.mj_forward(self.model, self.data)

    def _penetrating_contacts(self) -> tuple[tuple[str, str, float], ...]:
        contacts: list[tuple[str, str, float]] = []
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            if contact.dist >= 0.0:
                continue
            left = mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom1)
            ) or f"geom_{int(contact.geom1)}"
            right = mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom2)
            ) or f"geom_{int(contact.geom2)}"
            contacts.append((left, right, float(contact.dist)))
        return tuple(contacts)

    def _launch_viewer(self) -> object:
        viewer_module = importlib.import_module("mujoco.viewer")
        handle = viewer_module.launch_passive(
            self.model,
            self.data,
            show_left_ui=False,
            show_right_ui=False,
        )
        handle.cam.azimuth = -135
        handle.cam.elevation = -25
        handle.cam.distance = 0.8
        hand_body_id = self._named_id(
            mujoco.mjtObj.mjOBJ_BODY, "rh56_R_hand_base_link"
        )
        handle.cam.lookat[:] = self.data.xpos[hand_body_id]
        return handle
