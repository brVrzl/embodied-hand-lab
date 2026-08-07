from __future__ import annotations

from enum import Enum
from dataclasses import replace
from collections import deque
import math
from pathlib import Path
import time
from typing import Any, Callable, Mapping

from .episode import (
    CameraRecord,
    CanonicalEpisodeWriter,
    CanonicalSample,
    ControlSample,
    EpisodeStatus,
    StartPrerequisites,
    start_arm_target_measured_diagnostics,
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
        maximum_hand_start_delta_rad: float,
        defer_finalization: bool = False,
        camera_severe_stale_ns: int = 500_000_000,
        camera_consecutive_stale_limit: int = 15,
        camera_missing_timeout_ns: int = 1_000_000_000,
        canonical_required_field_consecutive_limit: int = 15,
        quality_min_valid_ratio: float = 1.0,
        quality_max_invalid_run: int = 0,
    ) -> None:
        self.writer = writer
        # The physical producer must not wait for queue drain, validation, or
        # final fsync after a recorder-only fault.  The hardware runtime sets
        # this flag and finalizes during outer-session cleanup; offline callers
        # retain the original synchronous behavior.
        self.defer_finalization = bool(defer_finalization)
        self.camera_max_age_ns = int(camera_max_age_ns)
        self.camera_severe_stale_ns = int(camera_severe_stale_ns)
        self.camera_consecutive_stale_limit = int(camera_consecutive_stale_limit)
        self.camera_missing_timeout_ns = int(camera_missing_timeout_ns)
        self.canonical_required_field_consecutive_limit = int(
            canonical_required_field_consecutive_limit
        )
        self.quality_min_valid_ratio = float(quality_min_valid_ratio)
        self.quality_max_invalid_run = int(quality_max_invalid_run)
        if not 0.0 <= self.quality_min_valid_ratio <= 1.0:
            raise ValueError("quality_min_valid_ratio must be within [0, 1]")
        if self.quality_max_invalid_run < 0:
            raise ValueError("quality_max_invalid_run must be non-negative")
        if self.camera_consecutive_stale_limit <= 0:
            raise ValueError("camera_consecutive_stale_limit must be positive")
        if self.camera_severe_stale_ns < self.camera_max_age_ns:
            raise ValueError("camera severe stale limit must not be below freshness limit")
        if self.camera_missing_timeout_ns < self.camera_severe_stale_ns:
            raise ValueError("camera missing timeout must not be below severe stale limit")
        if self.canonical_required_field_consecutive_limit <= 0:
            raise ValueError(
                "canonical required field consecutive limit must be positive"
            )
        self.maximum_hand_start_delta_rad = float(maximum_hand_start_delta_rad)
        if not math.isfinite(self.maximum_hand_start_delta_rad) or self.maximum_hand_start_delta_rad < 0.0:
            raise ValueError("maximum_hand_start_delta_rad must be finite and non-negative")
        self.state = CaptureState.IDLE
        self._state_listener: Callable[[CaptureState], None] | None = None
        self.clock = CanonicalClock(writer.dataset_fps)
        self.control = CausalTimeline[ControlSample](max_age_ns=control_max_age_ns)
        # Camera arrays are large and the recorder already persists every
        # frame to the raw layer.  Keep only a short causal window for
        # canonical selection instead of retaining an entire episode in RAM.
        self.workspace = CausalTimeline[CameraRecord](
            max_age_ns=camera_max_age_ns, capacity=16
        )
        self.wrist = CausalTimeline[CameraRecord](
            max_age_ns=camera_max_age_ns, capacity=16
        )
        self._last_trigger = False
        self._control_segment_id = 0
        self._last_control_segment_mode: str | None = None
        self._trigger_press_ns: int | None = None
        self._last_camera_clock: dict[str, tuple[float, float, int, int, str, str]] = {}
        self._last_control_source_timestamps: dict[str, int] = {}
        self.result: Path | None = None
        self.termination_reason: str | None = None
        self.completion_status: EpisodeStatus | None = None
        self._deferred_abort: tuple[str, bool, str | None] | None = None
        self._deferred_finish: tuple[str, int | None] | None = None
        self._deferred_discard: str | None = None
        self._deferred_rejection: str | None = None
        self._start_arm_target_measured_delta_rad: float | None = None
        self._start_arm_target_measured_max_joint_index: int | None = None
        self._camera_consecutive_stale = {"workspace": 0, "wrist": 0}
        self._camera_stale_count = {"workspace": 0, "wrist": 0}
        self._camera_drop_count = {"workspace": 0, "wrist": 0}
        self._camera_valid_count = {"workspace": 0, "wrist": 0}
        self._camera_invalid_count = {"workspace": 0, "wrist": 0}
        self._camera_invalid_run = {"workspace": 0, "wrist": 0}
        self._camera_longest_invalid_run = {"workspace": 0, "wrist": 0}
        self._camera_age_ns = {
            "workspace": deque(maxlen=4096),
            "wrist": deque(maxlen=4096),
        }
        self._canonical_durations_ns: deque[int] = deque(maxlen=4096)
        self._canonical_missing_source_count = 0
        self._canonical_total_slots = 0
        self._canonical_valid_all_sources = 0
        self._canonical_invalid_any_source = 0
        self._canonical_metadata_only_slots = 0
        self._canonical_required_field_invalid_run = 0
        self._canonical_required_field_invalid_total = 0
        self._pending_quality: deque[dict[str, Any]] = deque(maxlen=4096)
        self._quality_events_unpersisted = 0

    def set_state_listener(self, listener: Callable[[CaptureState], None]) -> None:
        self._state_listener = listener
        listener(self.state)

    def _set_state(self, state: CaptureState) -> None:
        self.state = state
        if self._state_listener is not None:
            self._state_listener(state)

    def ingest_camera(self, frame: CameraRecord, *, skipped_frames: int = 0) -> None:
        if self.state is CaptureState.DONE:
            return
        self._camera_drop_count[frame.role] += max(0, int(skipped_frames))
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
                reason = "trigger_released_before_reference_and_first_valid_sample"
                if self.defer_finalization:
                    self._deferred_rejection = reason
                else:
                    self.result = self.writer.discard_rejected_start(reason)
                self.termination_reason = reason
                self.completion_status = EpisodeStatus.INVALID
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
            self.finalize_pending()
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
        if self.defer_finalization:
            self.termination_reason = reason
            self.completion_status = EpisodeStatus.COMPLETED
            self._deferred_finish = (reason, release_ns)
            self._set_state(CaptureState.DONE)
            return
        self._set_state(CaptureState.FINALIZING)
        self.result = self.writer.finalize(
            EpisodeStatus.COMPLETED,
            termination_reason=reason,
            trigger_release_monotonic_ns=release_ns,
        )
        self.termination_reason = reason
        self.completion_status = EpisodeStatus.COMPLETED
        self._set_state(CaptureState.DONE)

    def abort(self, reason: str, *, invalid: bool = False, detail: str | None = None) -> None:
        if self.state is CaptureState.DONE:
            return
        if self.state in {CaptureState.IDLE, CaptureState.ARMING}:
            rejection = reason if detail is None else f"{reason}:{detail}"
            if self.defer_finalization:
                self._deferred_rejection = rejection
            else:
                self.result = self.writer.discard_rejected_start(rejection)
            self.termination_reason = reason
            self.completion_status = EpisodeStatus.INVALID
            self._set_state(CaptureState.DONE)
            return
        self.termination_reason = reason
        self.completion_status = EpisodeStatus.INVALID if invalid else EpisodeStatus.ABORTED
        if self.defer_finalization:
            self._deferred_abort = (reason, invalid, detail)
            self._set_state(CaptureState.DONE)
            return
        self._set_state(CaptureState.FINALIZING)
        self.result = self.writer.finalize(
            self.completion_status,
            termination_reason=reason,
            trigger_release_monotonic_ns=None,
            report={
                **self.diagnostics(),
                **({} if detail is None else {"fault_detail": detail}),
            },
        )
        self._set_state(CaptureState.DONE)

    def discard_current(self, reason: str) -> None:
        """Discard the active, not-yet-boundaried episode.

        Physical staging uses this only for outer-session shutdown.  Episodes
        already rotated by the explicit clutch boundary have already been
        finalized and are never revisited.
        """

        if self.state is CaptureState.DONE:
            return
        if self.state in {CaptureState.IDLE, CaptureState.ARMING}:
            if self.defer_finalization:
                self._deferred_rejection = reason
            else:
                self.result = self.writer.discard_rejected_start(reason)
            self.termination_reason = reason
            self.completion_status = EpisodeStatus.INVALID
            self._set_state(CaptureState.DONE)
            return
        if self.defer_finalization:
            self._deferred_discard = reason
            self.termination_reason = reason
            self.completion_status = EpisodeStatus.INVALID
            self._set_state(CaptureState.DONE)
            return
        self._set_state(CaptureState.FINALIZING)
        self.result = self.writer.discard_rejected_start(reason)
        self.termination_reason = reason
        self.completion_status = EpisodeStatus.INVALID
        self._set_state(CaptureState.DONE)

    def finalize_pending(self) -> None:
        """Finalize a deferred recorder abort outside the control loop."""

        if self._deferred_rejection is not None and self.result is None:
            reason = self._deferred_rejection
            self._deferred_rejection = None
            self.result = self.writer.discard_rejected_start(reason)
            return
        if self._deferred_finish is not None and self.result is None:
            reason, release_ns = self._deferred_finish
            self._deferred_finish = None
            self._flush_quality_events()
            self._set_state(CaptureState.FINALIZING)
            self.result = self.writer.finalize(
                EpisodeStatus.COMPLETED,
                termination_reason=reason,
                trigger_release_monotonic_ns=release_ns,
                report=self.diagnostics(),
            )
            self._set_state(CaptureState.DONE)
            return
        if self._deferred_discard is not None and self.result is None:
            reason = self._deferred_discard
            self._deferred_discard = None
            self._set_state(CaptureState.FINALIZING)
            self.result = self.writer.discard_rejected_start(reason)
            self._set_state(CaptureState.DONE)
            return
        pending = self._deferred_abort
        if pending is None or self.result is not None:
            return
        self._deferred_abort = None
        reason, invalid, detail = pending
        self._set_state(CaptureState.FINALIZING)
        self._flush_quality_events()
        self.result = self.writer.finalize(
            EpisodeStatus.INVALID if invalid else EpisodeStatus.ABORTED,
            termination_reason=reason,
            trigger_release_monotonic_ns=None,
            report={
                **self.diagnostics(),
                **({} if detail is None else {"fault_detail": detail}),
            },
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
            maximum_hand_start_delta_rad=self.maximum_hand_start_delta_rad,
        )
        try:
            self.writer.begin(prerequisites, camera_max_age_ns=self.camera_max_age_ns)
            start_diagnostics = start_arm_target_measured_diagnostics(sample)
            self._start_arm_target_measured_delta_rad = start_diagnostics[
                "start_arm_target_measured_delta_rad"
            ]
            self._start_arm_target_measured_max_joint_index = start_diagnostics[
                "start_arm_target_measured_max_joint_index"
            ]
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
        except ValueError as exc:
            if self.defer_finalization:
                self._deferred_rejection = str(exc)
            else:
                self.result = self.writer.discard_rejected_start(str(exc))
            self.termination_reason = str(exc)
            self.completion_status = EpisodeStatus.INVALID
            self._set_state(CaptureState.DONE)
            return False
        except OSError as exc:
            if self.defer_finalization:
                self._deferred_rejection = str(exc)
            else:
                self.result = self.writer.discard_rejected_start(str(exc))
            self.termination_reason = str(exc)
            self.completion_status = EpisodeStatus.INVALID
            self._set_state(CaptureState.DONE)
            return False
        return True

    def _append_canonical(self, frame_index: int, timestamp_ns: int) -> None:
        started_ns = time.perf_counter_ns()
        self._canonical_total_slots += 1
        control = self.control.latest_at_or_before(timestamp_ns)
        workspace = self.workspace.latest_at_or_before(timestamp_ns)
        wrist = self.wrist.latest_at_or_before(timestamp_ns)
        selections = {"control": control, "workspace": workspace, "wrist": wrist}
        if not control.valid:
            self._canonical_missing_source_count += 1
            self._canonical_invalid_any_source += 1
            self._canonical_metadata_only_slots += 1
            self.abort("stale_or_missing_source:control")
            return
        invalid_cameras = [
            role for role in ("workspace", "wrist") if not selections[role].valid
        ]
        if invalid_cameras:
            self._canonical_missing_source_count += 1
            self._canonical_invalid_any_source += 1
            self._canonical_metadata_only_slots += 1
            persistent = []
            quality: dict[str, Any] = {
                "record_type": "canonical_data_quality",
                "canonical_timestamp_ns": timestamp_ns,
                "nominal_slot_index": self.clock.last_nominal_slot_index,
                "metadata_only": True,
                "reason": "camera_stale_or_missing",
                "control": {
                    "host_monotonic_ns": control.value.host_monotonic_ns
                    if control.value is not None
                    else None,
                    "accepted_target_sequence": getattr(
                        control.value, "accepted_target_sequence", None
                    ),
                    "arm_action_status": getattr(
                        control.value, "arm_action_status", None
                    ),
                    "hand_target": list(
                        getattr(control.value, "hand_target", None) or ()
                    ),
                },
            }
            for role in ("workspace", "wrist"):
                selection = selections[role]
                age_ns = (
                    None
                    if selection.signed_offset_ns is None
                    else max(0, -int(selection.signed_offset_ns))
                )
                valid = selection.valid
                quality[f"{role}_valid"] = valid
                quality[f"{role}_age_ns"] = age_ns
                quality[f"{role}_frame_sequence"] = getattr(
                    selection.value, "sequence", None
                )
                quality[f"{role}_stale_reason"] = selection.reason
                if valid:
                    self._camera_consecutive_stale[role] = 0
                    self._camera_valid_count[role] += 1
                    self._camera_invalid_run[role] = 0
                else:
                    self._camera_invalid_count[role] += 1
                    self._camera_invalid_run[role] += 1
                    self._camera_longest_invalid_run[role] = max(
                        self._camera_longest_invalid_run[role],
                        self._camera_invalid_run[role],
                    )
                    self._camera_stale_count[role] += 1
                    self._camera_consecutive_stale[role] += 1
                    if age_ns is not None:
                        self._camera_age_ns[role].append(age_ns)
                    severe = age_ns is not None and age_ns >= self.camera_severe_stale_ns
                    missing = age_ns is None or age_ns >= self.camera_missing_timeout_ns
                    if self._camera_consecutive_stale[role] >= self.camera_consecutive_stale_limit and (
                        severe or missing
                    ):
                        persistent.append(role)
            try:
                self._publish_quality(quality)
            except (OSError, ValueError) as exc:
                self.abort("recording_writer_failure", detail=str(exc))
                return
            if persistent:
                self.abort("persistent_camera_acquisition_fault:" + ",".join(persistent))
            self._canonical_durations_ns.append(time.perf_counter_ns() - started_ns)
            return
        for role in ("workspace", "wrist"):
            self._camera_consecutive_stale[role] = 0
            self._camera_valid_count[role] += 1
            self._camera_invalid_run[role] = 0
            age_ns = max(0, -int(selections[role].signed_offset_ns or 0))
            self._camera_age_ns[role].append(age_ns)
        self._canonical_valid_all_sources += 1
        assert control.value is not None and workspace.value is not None and wrist.value is not None
        required_fields = {
            "accepted_arm_q": control.value.accepted_arm_q,
            "arm_q_measured": control.value.arm_q_measured,
            "arm_dq_measured": control.value.arm_dq_measured,
            "tcp_pose_xyzw": control.value.tcp_pose_xyzw,
            "hand_observation": control.value.hand_observation,
            "hand_target": control.value.hand_target,
        }
        missing_fields = [name for name, value in required_fields.items() if value is None]
        if missing_fields:
            self._canonical_required_field_invalid_run += 1
            self._canonical_required_field_invalid_total += 1
            self._canonical_metadata_only_slots += 1
            self._publish_quality(
                {
                    "record_type": "canonical_data_quality",
                    "canonical_timestamp_ns": timestamp_ns,
                    "nominal_slot_index": self.clock.last_nominal_slot_index,
                    "metadata_only": True,
                    "reason": "canonical_required_field_unavailable",
                    "missing_fields": missing_fields,
                    "control_host_monotonic_ns": control.value.host_monotonic_ns,
                }
            )
            if (
                self._canonical_required_field_invalid_run
                >= self.canonical_required_field_consecutive_limit
            ):
                self.abort(
                    "persistent_canonical_required_field_loss:"
                    + ",".join(missing_fields),
                    invalid=True,
                )
            return
        self._canonical_required_field_invalid_run = 0
        try:
            canonical_sample = CanonicalSample(
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
            accepted = self.writer.append_sample(canonical_sample)
            if accepted is False:
                # This is a recorder queue drop, not a camera acquisition
                # drop.  Keep the source counters independent.
                self._canonical_metadata_only_slots += 1
                self._publish_quality(
                    self._quality_from_sample(canonical_sample, "recorder_queue_full")
                )
        except (OSError, ValueError) as exc:
            self.abort("recording_writer_failure", detail=str(exc))
        finally:
            self._canonical_durations_ns.append(time.perf_counter_ns() - started_ns)

    def _finish_completed(self, release_ns: int) -> None:
        if self.defer_finalization:
            self.termination_reason = "arm_trigger_released"
            self.completion_status = EpisodeStatus.COMPLETED
            self._deferred_finish = ("arm_trigger_released", release_ns)
            self._set_state(CaptureState.DONE)
            return
        self._set_state(CaptureState.FINALIZING)
        self.result = self.writer.finalize(
            EpisodeStatus.COMPLETED,
            termination_reason="arm_trigger_released",
            trigger_release_monotonic_ns=release_ns,
        )
        self.termination_reason = "arm_trigger_released"
        self.completion_status = EpisodeStatus.COMPLETED
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

    def _publish_quality(self, quality: dict[str, Any]) -> None:
        try:
            if self.writer.append_raw("data_quality", quality) is not False:
                return
        except (OSError, ValueError) as exc:
            self.abort("recording_writer_failure", detail=str(exc))
            return
        self._pending_quality.append(quality)

    @staticmethod
    def _quality_from_sample(sample: CanonicalSample, reason: str) -> dict[str, Any]:
        quality: dict[str, Any] = {
            "record_type": "canonical_data_quality",
            "canonical_timestamp_ns": sample.timestamp_ns,
            "nominal_slot_index": sample.nominal_slot_index,
            "metadata_only": True,
            "reason": reason,
            "control": {
                "host_monotonic_ns": sample.control.host_monotonic_ns,
                "accepted_target_sequence": sample.control.accepted_target_sequence,
                "arm_action_status": sample.control.arm_action_status,
                "hand_target": list(sample.control.hand_target or ()),
            },
        }
        for role, camera in (("workspace", sample.workspace), ("wrist", sample.wrist)):
            quality[f"{role}_valid"] = True
            quality[f"{role}_age_ns"] = max(0, sample.timestamp_ns - camera.host_monotonic_ns)
            quality[f"{role}_frame_sequence"] = getattr(camera, "sequence", None)
            quality[f"{role}_stale_reason"] = None
        return quality

    def _flush_quality_events(self) -> None:
        pending = self._pending_quality
        self._pending_quality = deque(maxlen=pending.maxlen)
        deadline = time.monotonic() + 1.0
        while pending:
            quality = pending.popleft()
            try:
                while self.writer.append_raw("data_quality", quality) is False:
                    if time.monotonic() >= deadline:
                        self._quality_events_unpersisted += 1
                        break
                    time.sleep(0.001)
            except (OSError, ValueError):
                self._quality_events_unpersisted += 1

    def diagnostics(self) -> dict[str, Any]:
        role_quality = {}
        for role in ("workspace", "wrist"):
            valid = self._camera_valid_count[role]
            invalid = self._camera_invalid_count[role]
            role_quality[role] = {
                "valid_count": valid,
                "invalid_count": invalid,
                "valid_ratio": valid / max(valid + invalid, 1),
                "longest_invalid_run": self._camera_longest_invalid_run[role],
                "max_age_ns": max(self._camera_age_ns[role], default=0),
            }
        diagnostics = getattr(self.writer, "diagnostics", None)
        writer_metrics = diagnostics() if callable(diagnostics) else {}
        return {
            "canonical_total_slots": self._canonical_total_slots,
            "canonical_valid_all_sources": self._canonical_valid_all_sources,
            "canonical_invalid_any_source": self._canonical_invalid_any_source,
            "canonical_metadata_only_slots": self._canonical_metadata_only_slots,
            "canonical_required_field_invalid_total": self._canonical_required_field_invalid_total,
            "canonical_required_field_invalid_run": self._canonical_required_field_invalid_run,
            "canonical_required_field_consecutive_limit": self.canonical_required_field_consecutive_limit,
            "start_arm_target_measured_delta_rad": self._start_arm_target_measured_delta_rad,
            "start_arm_target_measured_max_joint_index": self._start_arm_target_measured_max_joint_index,
            "recorder_enqueued_count": writer_metrics.get("recorder_enqueued_count", 0),
            "recorder_written_count": writer_metrics.get("recorder_written_count", 0),
            "recorder_dropped_count": writer_metrics.get("recorder_dropped_count", 0),
            "queue_full_count": writer_metrics.get("queue_full_count", 0),
            "ring_reference_expired_count": writer_metrics.get("ring_reference_expired_count", 0),
            "writer_failed_count": writer_metrics.get("writer_error_count", 0),
            "quality_state": self._quality_state(writer_metrics),
            "quality_events_unpersisted": self._quality_events_unpersisted + len(self._pending_quality),
            "data_quality": {
                "workspace_stale_count": self._camera_stale_count["workspace"],
                "wrist_stale_count": self._camera_stale_count["wrist"],
                "workspace_drop_count": self._camera_drop_count["workspace"],
                "wrist_drop_count": self._camera_drop_count["wrist"],
                "workspace_frame_age_ns": _summary(self._camera_age_ns["workspace"]),
                "wrist_frame_age_ns": _summary(self._camera_age_ns["wrist"]),
                "canonical_compute_duration_ns": _summary(self._canonical_durations_ns),
                "canonical_missing_source_count": self._canonical_missing_source_count,
                **{
                    f"{role}_{name}": value
                    for role, values in role_quality.items()
                    for name, value in values.items()
                },
            }
        }

    def _quality_state(self, writer_metrics: Mapping[str, Any]) -> str:
        if writer_metrics.get("writer_error_count", 0):
            return "partial_writer_failure"
        if self.termination_reason and (
            self.termination_reason.startswith("persistent_camera")
            or "camera" in self.termination_reason
            or "writer" in self.termination_reason
            or "recording" in self.termination_reason
        ):
            if writer_metrics.get("writer_error_count", 0):
                return "partial_writer_failure"
            return "aborted_recording"
        if self.termination_reason and (
            "safety" in self.termination_reason
            or "control" in self.termination_reason
            or self.termination_reason.startswith("stale_or_missing_source:control")
        ):
            return "aborted_robot_safety"
        role_degraded = any(
            self._camera_valid_count[role]
            / max(
                self._camera_valid_count[role] + self._camera_invalid_count[role],
                1,
            )
            < self.quality_min_valid_ratio
            or self._camera_longest_invalid_run[role] > self.quality_max_invalid_run
            for role in ("workspace", "wrist")
        )
        if (
            self._canonical_invalid_any_source
            or writer_metrics.get("recorder_dropped_count", 0)
            or role_degraded
        ):
            return "completed_degraded"
        return "completed_valid"


def _summary(values: deque[int]) -> dict[str, int]:
    ordered = sorted(values)
    if not ordered:
        return {"count": 0, "p50": 0, "p95": 0, "p99": 0, "max": 0}
    last = len(ordered) - 1
    return {
        "count": len(ordered),
        "p50": ordered[round(last * 0.50)],
        "p95": ordered[round(last * 0.95)],
        "p99": ordered[round(last * 0.99)],
        "max": ordered[-1],
    }
