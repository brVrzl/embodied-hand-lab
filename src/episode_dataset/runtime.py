"""Shared dual-RealSense episode runtime assembly."""

from __future__ import annotations

from dataclasses import dataclass
import multiprocessing as mp
from pathlib import Path
import time
from typing import Any, Mapping

from embodiment_core.config import load_yaml
from vision_interface.realsense_adapter import resolve_realsense_config

from .collector import CaptureState
from .episode import SCHEMA_VERSION, file_sha256
from .process_runtime import (
    FrameReferenceDescriptor,
    ProcessCamera,
    ProcessEpisodeCollectorProxy,
    ProcessEpisodeRecorder,
    ProcessPreview,
)
from .preview import PreviewStatus, require_preview_dependencies


@dataclass(slots=True)
class EpisodeDataRuntime:
    collector: ProcessEpisodeCollectorProxy
    cameras: dict[str, ProcessCamera]
    preview: ProcessPreview | None
    recorder: ProcessEpisodeRecorder
    last_camera_timestamp_ns: dict[str, int]

    @classmethod
    def start(
        cls,
        config_path: str | Path,
        *,
        episode_root: str | Path | None,
        task_name: str,
        operator: str,
        control_config_path: str | Path,
        maximum_start_delta_rad: float,
        metadata: Mapping[str, Any],
        schema_version: str = SCHEMA_VERSION,
        preview_enabled: bool = False,
        forbidden_cpu: int | None = None,
    ) -> "EpisodeDataRuntime":
        if preview_enabled:
            require_preview_dependencies()
        config_path = Path(config_path)
        data_config = load_yaml(config_path)
        cameras_config = data_config.get("cameras")
        if not isinstance(cameras_config, dict) or set(cameras_config) != {
            "workspace",
            "wrist",
        }:
            raise ValueError(
                "episode data config must define exactly workspace and wrist cameras"
            )
        resolved = {
            role: resolve_realsense_config(data_config, camera_name=role)
            for role in ("workspace", "wrist")
        }
        serials = {role: str(resolved[role].get("serial", "")) for role in resolved}
        if any(not serial or serial.startswith("REPLACE_") for serial in serials.values()):
            raise ValueError("both D435 roles require explicit non-placeholder serial numbers")
        if len(set(serials.values())) != 2:
            raise ValueError("workspace and wrist must bind different RealSense serial numbers")

        dataset = data_config.get("dataset", {})
        if not isinstance(dataset, dict):
            raise ValueError("episode data config dataset must be a mapping")
        ring_capacity = int(dataset.get("camera_ring_capacity", 16))
        context = mp.get_context("spawn")
        workers = {
            role: ProcessCamera(
                role,
                resolved[role],
                capacity=ring_capacity,
                forbidden_cpu=forbidden_cpu,
                context=context,
            )
            for role in ("workspace", "wrist")
        }
        recorder: ProcessEpisodeRecorder | None = None
        preview: ProcessPreview | None = None
        try:
            camera_timeout_s = max(
                float(resolved[role].get("timeout_ms", 5000)) / 1000.0 + 2.0
                for role in resolved
            )
            for worker in workers.values():
                worker.start(camera_timeout_s)
            overflow_policy = dataset.get("recorder_overflow_policy", "drop_newest")
            if overflow_policy != "drop_newest":
                raise ValueError("recorder_overflow_policy must be drop_newest")
            calibration = data_config.get("calibration", {})
            if not isinstance(calibration, dict):
                raise ValueError("episode data config calibration must be a mapping")
            control_config_path = Path(control_config_path)
            recorder = ProcessEpisodeRecorder(
                context=context,
                ring_specs={role: worker.ring_spec for role, worker in workers.items()},
                episode_root=episode_root,
                task_name=task_name,
                operator=operator,
                control_config_path=control_config_path,
                maximum_start_delta_rad=maximum_start_delta_rad,
                metadata={
                    "camera_serials": serials,
                    "calibration_files": calibration.get("snapshot_files", []),
                    "calibration_snapshot": {
                        "files": [],
                        "version": calibration.get("version"),
                    },
                    "control_config": {
                        "path": str(control_config_path.resolve()),
                        "sha256": file_sha256(control_config_path),
                    },
                    **dict(metadata),
                },
                dataset=dataset,
                camera_profiles={role: workers[role].profile_metadata() for role in workers},
                forbidden_cpu=forbidden_cpu,
                schema_version=schema_version,
            )
            collector = recorder.start(timeout_s=8.0)
            if preview_enabled:
                preview = ProcessPreview(
                    context=context,
                    ring_specs={role: worker.ring_spec for role, worker in workers.items()},
                    refresh_hz=float(dataset.get("preview_max_fps", 10.0)),
                    forbidden_cpu=forbidden_cpu,
                )
                preview.start()
            return cls(
                collector=collector,
                cameras=workers,
                preview=preview,
                recorder=recorder,
                last_camera_timestamp_ns={"workspace": -1, "wrist": -1},
            )
        except BaseException:
            if preview is not None:
                preview.stop()
            if recorder is not None:
                recorder.stop()
            for worker in workers.values():
                worker.stop()
            raise

    def ingest_cameras(self) -> None:
        if self.collector.state is CaptureState.DONE:
            return
        for role, camera in self.cameras.items():
            if camera.error is not None:
                self.collector.camera_fault(role, str(camera.error))
                continue
            frame, skipped = camera.latest_after(self.last_camera_timestamp_ns[role])
            if frame is not None:
                self.collector.ingest_camera(frame, skipped_frames=skipped)
                self.last_camera_timestamp_ns[role] = frame.host_monotonic_ns

    def update_preview(self, *, arm_trigger: bool, hand_grip: bool) -> None:
        if self.preview is None:
            return
        workspace = self.cameras["workspace"].latest
        wrist = self.cameras["wrist"].latest
        self.preview.update(
            None if workspace is None else FrameReferenceDescriptor.from_reference(workspace),
            None if wrist is None else FrameReferenceDescriptor.from_reference(wrist),
            PreviewStatus(
                state=self.collector.state,
                temporary_id=self.collector.writer.temporary_id,
                episode_start_ns=None,
                arm_trigger=arm_trigger,
                hand_grip=hand_grip,
                recording_frame_count=self.collector.writer.sample_count,
            ),
        )

    def close(self) -> None:
        if self.preview is not None:
            self.preview.stop()
        self.recorder.stop()
        for camera in self.cameras.values():
            camera.stop()
