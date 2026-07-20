from __future__ import annotations

import json
import time
from dataclasses import replace
from pathlib import Path
from typing import IO, Any

from ..contracts import (
    ArmPoseSample,
    OperatorActionSample,
    RunGateSample,
    arm_pose_sample_from_dict,
)
from .interface import AdapterSnapshot


RECORDING_SCHEMA = "arm_pose_recording.v1"


class PoseStreamRecorder:
    """Append-only JSONL recorder outside the control deadline path."""

    def __init__(self, path: str | Path, *, metadata: dict[str, Any] | None = None) -> None:
        self.path = Path(path)
        self._file: IO[str] = self.path.open("x", encoding="utf-8")
        self._file.write(
            json.dumps(
                {
                    "record_type": "header",
                    "schema_version": RECORDING_SCHEMA,
                    "created_wall_time_ns": time.time_ns(),
                    "metadata": metadata or {},
                },
                separators=(",", ":"),
                sort_keys=True,
            )
            + "\n"
        )
        self.count = 0

    def write(self, snapshot: AdapterSnapshot) -> None:
        payload = {
            "record_type": "sample",
            "schema_version": RECORDING_SCHEMA,
            "generation": snapshot.generation,
            "connected": snapshot.connected,
            "reason": snapshot.reason,
            "pose": None if snapshot.pose is None else snapshot.pose.to_dict(),
            "run_gate": snapshot.run_gate.to_dict(),
            "operator_action": (
                None if snapshot.operator_action is None else snapshot.operator_action.to_dict()
            ),
        }
        self._file.write(json.dumps(payload, separators=(",", ":"), sort_keys=True) + "\n")
        self.count += 1

    def close(self) -> None:
        if not self._file.closed:
            self._file.flush()
            self._file.close()

    def __enter__(self) -> "PoseStreamRecorder":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()


def _run_gate_from_dict(payload: dict[str, Any]) -> RunGateSample:
    if payload.get("schema_version") != "arm_teleoperation.v1":
        raise ValueError("unsupported run-gate schema")
    return RunGateSample(
        source_id=str(payload["source_id"]),
        sequence=int(payload["sequence"]),
        local_receive_ns=int(payload["local_receive_ns"]),
        engaged=bool(payload["engaged"]),
        valid=bool(payload["valid"]),
        connection_epoch=int(payload.get("connection_epoch", 0)),
        reason=str(payload.get("reason", "")),
    )


def _operator_action_from_dict(payload: dict[str, Any] | None) -> OperatorActionSample | None:
    if payload is None:
        return None
    if payload.get("schema_version") != "arm_teleoperation.v1":
        raise ValueError("unsupported operator-action schema")
    return OperatorActionSample(
        source_id=str(payload["source_id"]),
        sequence=int(payload["sequence"]),
        local_receive_ns=int(payload["local_receive_ns"]),
        recenter_requested=bool(payload.get("recenter_requested", False)),
        stop_requested=bool(payload.get("stop_requested", False)),
        fault_reset_requested=bool(payload.get("fault_reset_requested", False)),
        valid=bool(payload.get("valid", True)),
        reason=str(payload.get("reason", "")),
    )


class ReplayPoseInput:
    """Timestamp-faithful replay that skips overdue records instead of replaying a FIFO."""

    def __init__(self, path: str | Path, *, speed: float = 1.0, start_ns: int | None = None) -> None:
        if not speed > 0.0:
            raise ValueError("replay speed must be positive")
        records: list[AdapterSnapshot] = []
        with Path(path).open("r", encoding="utf-8") as stream:
            for line in stream:
                payload = json.loads(line)
                if payload.get("record_type") == "header":
                    if payload.get("schema_version") != RECORDING_SCHEMA:
                        raise ValueError("unsupported recording schema")
                    continue
                if payload.get("record_type") != "sample":
                    raise ValueError("unknown recording record type")
                pose_payload = payload.get("pose")
                records.append(
                    AdapterSnapshot(
                        pose=None if pose_payload is None else arm_pose_sample_from_dict(pose_payload),
                        run_gate=_run_gate_from_dict(payload["run_gate"]),
                        connected=bool(payload["connected"]),
                        generation=int(payload["generation"]),
                        reason=str(payload.get("reason", "")),
                        operator_action=_operator_action_from_dict(payload.get("operator_action")),
                    )
                )
        if not records:
            raise ValueError("recording contains no samples")
        self._records = records
        self.speed = float(speed)
        self.start_ns = time.monotonic_ns() if start_ns is None else int(start_ns)
        first_pose = next((item.pose for item in records if item.pose is not None), None)
        if first_pose is None:
            raise ValueError("recording contains no pose samples")
        self._recording_start_ns = first_pose.timestamps.local_receive_ns
        self._index = 0
        self.skipped_backlog = 0

    @property
    def finished(self) -> bool:
        return self._index >= len(self._records)

    def latest(self, *, now_ns: int, after_generation: int = -1) -> AdapterSnapshot | None:
        replay_elapsed = max(0, now_ns - self.start_ns)
        recording_now = self._recording_start_ns + int(replay_elapsed * self.speed)
        newest_index: int | None = None
        starting_index = self._index
        while self._index < len(self._records):
            candidate = self._records[self._index]
            timestamp = (
                candidate.pose.timestamps.local_receive_ns
                if candidate.pose is not None
                else candidate.run_gate.local_receive_ns
            )
            if timestamp > recording_now:
                break
            newest_index = self._index
            self._index += 1
        if newest_index is None:
            return None
        consumed = self._index - starting_index
        if consumed > 1:
            self.skipped_backlog += consumed - 1
        result = self._records[newest_index]
        if result.generation <= after_generation:
            return None
        original_timestamp = (
            result.pose.timestamps.local_receive_ns
            if result.pose is not None
            else result.run_gate.local_receive_ns
        )
        replay_timestamp = self.start_ns + int(
            (original_timestamp - self._recording_start_ns) / self.speed
        )
        pose = result.pose
        if pose is not None:
            pose = replace(
                pose,
                timestamps=pose.timestamps.with_stage(
                    local_receive_ns=replay_timestamp,
                    processing_ns=replay_timestamp,
                    dispatch_ns=None,
                    robot_command_ns=None,
                    robot_state_observation_ns=None,
                ),
                sample_age_ns=max(0, now_ns - replay_timestamp),
            )
        action = result.operator_action
        if action is not None:
            action = replace(action, local_receive_ns=replay_timestamp)
        return replace(
            result,
            pose=pose,
            run_gate=replace(result.run_gate, local_receive_ns=replay_timestamp),
            operator_action=action,
        )

    def close(self) -> None:
        return None
