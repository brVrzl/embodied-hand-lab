from __future__ import annotations

from enum import Enum
from dataclasses import replace
import math
from pathlib import Path
from typing import Any, Callable, Mapping

from .episode import (
    CameraSample,
    CanonicalEpisodeWriter,
    CanonicalSample,
    ControlSample,
    EpisodeStatus,
    StartPrerequisites,
)
from .timeline import CanonicalClock, CausalTimeline, TimestampRegression


class CaptureState(str, Enum):
    IDLE = "IDLE"
    ARMING = "ARMING"
    REC = "REC"
    FINALIZING = "FINALIZING"
    DONE = "DONE"


class SingleEpisodeCollector:
    """Strict trigger-bounded state machine, independent of control algorithms."""

    def __init__(
        self,
        writer: CanonicalEpisodeWriter,
        *,
        # Recorder snapshot freshness is separate from robot watchdogs.  The
        # target host measured an ~83 ms producer stall, leaving a healthy
        # 30 Hz frame about 75 ms behind the next canonical slot.
        camera_max_age_ns: int = 100_000_000,
        control_max_age_ns: int = 40_000_000,
        maximum_start_delta_rad: float,
        maximum_hand_start_delta_rad: float,
    ) -> None:
        self.writer = writer
        self.camera_max_age_ns = int(camera_max_age_ns)
        self.maximum_start_delta_rad = float(maximum_start_delta_rad)
        self.maximum_hand_start_delta_rad = float(maximum_hand_start_delta_rad)
        for name, value in (
            ("maximum_start_delta_rad", self.maximum_start_delta_rad),
            ("maximum_hand_start_delta_rad", self.maximum_hand_start_delta_rad),
        ):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")
        self.state = CaptureState.IDLE
        self._state_listener: Callable[[CaptureState], None] | None = None
        self.clock = CanonicalClock(writer.dataset_fps)
        self.control = CausalTimeline[ControlSample](max_age_ns=control_max_age_ns)
        self.workspace = CausalTimeline[CameraSample](max_age_ns=camera_max_age_ns)
        self.wrist = CausalTimeline[CameraSample](max_age_ns=camera_max_age_ns)
        self._last_trigger = False
        self._control_segment_id = 0
        self._last_control_segment_mode: str | None = None
        self._trigger_press_ns: int | None = None
        self._last_camera_clock: dict[str, tuple[float, float, int, int, str, str]] = {}
        self._last_control_source_timestamps: dict[str, int] = {}
        self.result: Path | None = None

    def set_state_listener(self, listener: Callable[[CaptureState], None]) -> None:
        self._state_listener = listener
        listener(self.state)

    def _set_state(self, state: CaptureState) -> None:
        self.state = state
        if self._state_listener is not None:
            self._state_listener(state)

    def ingest_camera(self, frame: CameraSample) -> None:
        clock = (
            frame.device_rgb_timestamp_ms,
            frame.device_depth_timestamp_ms,
            frame.rgb_frame_number,
            frame.depth_frame_number,
            frame.rgb_timestamp_domain,
            frame.depth_timestamp_domain,
        )
        previous = self._last_camera_clock.get(frame.role)
        self._last_camera_clock[frame.role] = clock
        if self.state is CaptureState.REC and previous is not None:
            regressed = any(current < old for current, old in zip(clock[:4], previous[:4], strict=True))
            domain_changed = clock[4:] != previous[4:]
            if regressed or domain_changed:
                self.abort(
                    f"{frame.role}_camera_device_timestamp_or_frame_regression",
                    invalid=True,
                )
                return
        try:
            (self.workspace if frame.role == "workspace" else self.wrist).append(
                frame.host_monotonic_ns, frame
            )
            if self.state is CaptureState.REC:
                self.writer.append_raw_camera(frame)
        except TimestampRegression as exc:
            self.abort("camera_timestamp_regression", invalid=True, detail=str(exc))
        except (OSError, ValueError) as exc:
            self.abort("camera_data_write_failure", detail=str(exc))

    def ingest_control(
        self,
        sample: ControlSample,
        *,
        reference_established: bool,
        raw_records: Mapping[str, Mapping[str, Any]] | None = None,
        capture_active: bool | None = None,
    ) -> None:
        if self.state is CaptureState.DONE:
            return
        segment_mode = (
            "arm_and_hand"
            if sample.arm_trigger and sample.hand_grip
            else "arm_only"
            if sample.arm_trigger
            else "hand_only"
            if sample.hand_grip
            else "both_idle"
        )
        if self._last_control_segment_mode is not None and (
            segment_mode != self._last_control_segment_mode
        ):
            self._control_segment_id += 1
        self._last_control_segment_mode = segment_mode
        sample = replace(
            sample,
            control_segment_id=self._control_segment_id,
            control_segment_mode=segment_mode,
        )
        try:
            self.control.append(sample.host_monotonic_ns, sample)
        except TimestampRegression as exc:
            self.abort("control_timestamp_regression", invalid=True, detail=str(exc))
            return
        pressed = sample.arm_trigger if capture_active is None else bool(capture_active)
        press_edge = pressed and not self._last_trigger
        release_edge = not pressed and self._last_trigger
        self._last_trigger = pressed

        if self.state is CaptureState.IDLE and press_edge:
            self._set_state(CaptureState.ARMING)
            self._trigger_press_ns = sample.host_monotonic_ns

        if self.state is CaptureState.ARMING:
            if release_edge:
                self.result = self.writer.discard_rejected_start(
                    "trigger_released_before_reference_and_first_valid_sample"
                )
                self._set_state(CaptureState.DONE)
                return
            if pressed and self._start_if_ready(
                sample, reference_established, capture_active=pressed
            ):
                if raw_records:
                    self._append_raw_records(raw_records)
                return

        if self.state is CaptureState.REC:
            for name, timestamp_ns in (sample.source_timestamps_ns or {}).items():
                if timestamp_ns is None:
                    continue
                previous = self._last_control_source_timestamps.get(name)
                if previous is not None and timestamp_ns < previous:
                    self.abort(f"control_source_timestamp_regression:{name}", invalid=True)
                    return
                self._last_control_source_timestamps[name] = timestamp_ns
            if raw_records:
                self._append_raw_records(raw_records)
                if self.state is CaptureState.DONE:
                    return
            if sample.controller_fault:
                self.abort("controller_alarm")
                return
            if sample.tracking_hard_fault:
                self.abort("tracking_hard_fault")
                return
            if not sample.control_heartbeat_valid:
                self.abort("control_heartbeat_lost")
                return
            if release_edge:
                self._finish_completed(sample.host_monotonic_ns)
                return
            due = self.clock.due(sample.host_monotonic_ns)
            if due is not None:
                frame_index, timestamp_ns = due
                self._append_canonical(frame_index, timestamp_ns)

    def camera_fault(self, role: str, reason: str) -> None:
        if self.state is CaptureState.REC:
            self.abort(f"{role}_camera_disconnected:{reason}")

    def shutdown(self, reason: str) -> None:
        """End capture lifecycle without leaving an idle writer thread behind."""

        if self.state is CaptureState.DONE:
            return
        if self.state is CaptureState.IDLE:
            close = getattr(self.writer, "close", None)
            if callable(close):
                close()
            self._set_state(CaptureState.DONE)
            return
        self.abort(reason)

    def finish(self, reason: str, *, release_ns: int | None = None) -> None:
        """Finalize an active externally bounded session as completed."""

        if self.state is CaptureState.DONE:
            return
        if self.state is not CaptureState.REC:
            self.shutdown(reason)
            return
        self._set_state(CaptureState.FINALIZING)
        self.result = self.writer.finalize(
            EpisodeStatus.COMPLETED,
            termination_reason=reason,
            trigger_release_monotonic_ns=release_ns,
        )
        self._set_state(CaptureState.DONE)

    def abort(self, reason: str, *, invalid: bool = False, detail: str | None = None) -> None:
        if self.state is CaptureState.DONE:
            return
        if self.state in {CaptureState.IDLE, CaptureState.ARMING}:
            self.result = self.writer.discard_rejected_start(reason if detail is None else f"{reason}:{detail}")
            self._set_state(CaptureState.DONE)
            return
        self._set_state(CaptureState.FINALIZING)
        self.result = self.writer.finalize(
            EpisodeStatus.INVALID if invalid else EpisodeStatus.ABORTED,
            termination_reason=reason,
            trigger_release_monotonic_ns=None,
            report={} if detail is None else {"fault_detail": detail},
        )
        self._set_state(CaptureState.DONE)

    def _start_if_ready(
        self,
        sample: ControlSample,
        reference_established: bool,
        *,
        capture_active: bool,
    ) -> bool:
        if not reference_established or sample.accepted_arm_q is None:
            return False
        workspace = self.workspace.latest_at_or_before(sample.host_monotonic_ns)
        wrist = self.wrist.latest_at_or_before(sample.host_monotonic_ns)
        if not workspace.valid or not wrist.valid:
            return False
        assert workspace.value is not None and wrist.value is not None
        trigger_press_ns = int(self._trigger_press_ns or sample.host_monotonic_ns)
        if (
            workspace.value.host_monotonic_ns < trigger_press_ns
            or wrist.value.host_monotonic_ns < trigger_press_ns
        ):
            return False
        prerequisites = StartPrerequisites(
            trigger_press_monotonic_ns=trigger_press_ns,
            reference_established=True,
            accepted=(
                sample
                if sample.arm_trigger
                else replace(sample, arm_trigger=capture_active)
            ),
            workspace=workspace.value,
            wrist=wrist.value,
            maximum_start_delta_rad=self.maximum_start_delta_rad,
            maximum_hand_start_delta_rad=self.maximum_hand_start_delta_rad,
        )
        try:
            self.writer.begin(prerequisites, camera_max_age_ns=self.camera_max_age_ns)
            self.writer.append_raw_camera(workspace.value)
            self.writer.append_raw_camera(wrist.value)
            self.clock.start(sample.host_monotonic_ns)
            self._last_control_source_timestamps = {
                name: timestamp_ns
                for name, timestamp_ns in (sample.source_timestamps_ns or {}).items()
                if timestamp_ns is not None
            }
            self._set_state(CaptureState.REC)
            self._append_canonical(0, sample.host_monotonic_ns)
        except (OSError, ValueError) as exc:
            self.result = self.writer.discard_rejected_start(str(exc))
            self._set_state(CaptureState.DONE)
            return False
        return True

    def _append_canonical(self, frame_index: int, timestamp_ns: int) -> None:
        control = self.control.latest_at_or_before(timestamp_ns)
        workspace = self.workspace.latest_at_or_before(timestamp_ns)
        wrist = self.wrist.latest_at_or_before(timestamp_ns)
        selections = {"control": control, "workspace": workspace, "wrist": wrist}
        invalid = [name for name, selection in selections.items() if not selection.valid]
        if invalid:
            self.abort("stale_or_missing_source:" + ",".join(invalid))
            return
        assert control.value is not None and workspace.value is not None and wrist.value is not None
        if any(
            value is None
            for value in (
                control.value.accepted_arm_q,
                control.value.arm_q_measured,
                control.value.arm_dq_measured,
                control.value.tcp_pose_xyzw,
                control.value.hand_observation,
                control.value.hand_target,
            )
        ):
            self.abort("canonical_required_field_unavailable", invalid=True)
            return
        try:
            self.writer.append_sample(
                CanonicalSample(
                    frame_index=frame_index,
                    timestamp_ns=timestamp_ns,
                    control=control.value,
                    workspace=workspace.value,
                    wrist=wrist.value,
                    source_offsets_ns={
                        name: int(selection.signed_offset_ns or 0)
                        for name, selection in selections.items()
                    },
                    synchronization_valid=True,
                    nominal_slot_index=(
                        0
                        if frame_index == 0
                        else self.clock.last_nominal_slot_index
                    ),
                    missed_slots_before=(
                        0
                        if frame_index == 0
                        else self.clock.last_missed_slots_before
                    ),
                    missed_slots_after=(
                        0
                        if frame_index == 0
                        else self.clock.last_missed_slots_after
                    ),
                )
            )
        except (OSError, ValueError) as exc:
            self.abort("data_write_failure", detail=str(exc))

    def _finish_completed(self, release_ns: int) -> None:
        self._set_state(CaptureState.FINALIZING)
        self.result = self.writer.finalize(
            EpisodeStatus.COMPLETED,
            termination_reason="arm_trigger_released",
            trigger_release_monotonic_ns=release_ns,
        )
        self._set_state(CaptureState.DONE)

    def _append_raw_records(self, records: Mapping[str, Mapping[str, Any]]) -> None:
        try:
            batch = getattr(self.writer, "append_raw_batch", None)
            if callable(batch):
                batch(list(records.items()))
            else:
                for stream, record in records.items():
                    self.writer.append_raw(stream, record)
        except (OSError, ValueError) as exc:
            self.abort("raw_data_write_failure", detail=str(exc))
