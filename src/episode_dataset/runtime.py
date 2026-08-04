"""Shared dual-RealSense episode runtime assembly."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import time
from typing import Any, Mapping

from embodiment_core.config import load_yaml
from vision_interface.realsense_adapter import RealSenseCamera, resolve_realsense_config

from .async_writer import AsyncEpisodeWriter
from .camera import AsyncRGBDCamera
from .collector import CaptureState, SingleEpisodeCollector
from .episode import CanonicalEpisodeWriter, SCHEMA_VERSION, file_sha256
from .preview import AsyncDualCameraPreview, PreviewStatus, require_preview_dependencies


@dataclass(slots=True)
class EpisodeDataRuntime:
    collector: SingleEpisodeCollector
    cameras: dict[str, AsyncRGBDCamera]
    preview: AsyncDualCameraPreview | None
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

        workers = {
            role: AsyncRGBDCamera(
                role,
                lambda role=role: RealSenseCamera(resolved[role]),
                queue_capacity=int(
                    data_config.get("dataset", {}).get("camera_ring_capacity", 16)
                ),
                # RealSenseCamera copies the SDK buffers before returning an
                # RGBDFrame.  Do not copy those full RGB-D arrays a second
                # time on the capture thread.
                copy_arrays=False,
            )
            for role in ("workspace", "wrist")
        }
        for worker in workers.values():
            worker.start()
        writer: AsyncEpisodeWriter | None = None
        preview: AsyncDualCameraPreview | None = None
        deadline = time.monotonic() + max(
            float(resolved[role].get("timeout_ms", 5000)) / 1000.0 + 2.0
            for role in resolved
        )
        try:
            while time.monotonic() < deadline:
                errors = {
                    role: worker.error
                    for role, worker in workers.items()
                    if worker.error is not None
                }
                if errors:
                    raise RuntimeError(f"RealSense startup failed: {errors}")
                if all(worker.latest() is not None for worker in workers.values()):
                    break
                time.sleep(0.01)
            else:
                raise RuntimeError(
                    "dual RealSense startup timed out before fresh frames arrived"
                )

            dataset = data_config.get("dataset", {})
            if not isinstance(dataset, dict):
                raise ValueError("episode data config dataset must be a mapping")
            overflow_policy = dataset.get("recorder_overflow_policy", "drop_newest")
            if overflow_policy != "drop_newest":
                raise ValueError("recorder_overflow_policy must be drop_newest")
            calibration = data_config.get("calibration", {})
            if not isinstance(calibration, dict):
                raise ValueError("episode data config calibration must be a mapping")
            control_config_path = Path(control_config_path)
            ring_capacity = int(dataset.get("camera_ring_capacity", 16))
            requested_queue_capacity = int(dataset.get("recorder_queue_capacity", 64))
            # Strategy D: references are intentionally lossy/latest-only.  Do
            # not allow more queued references than a ring can retain; excess
            # producer work is rejected explicitly as recorder backpressure.
            effective_queue_capacity = min(requested_queue_capacity, ring_capacity)
            writer = AsyncEpisodeWriter(
                CanonicalEpisodeWriter(
                    episode_root or dataset.get("root", "data/episodes"),
                    task_name=task_name,
                    operator=operator,
                    dataset_fps=int(dataset.get("fps", 30)),
                    schema_version=schema_version,
                    metadata={
                        "camera_serials": serials,
                        "camera_profiles": {
                            role: workers[role].profile_metadata() for role in workers
                        },
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
                ),
                capacity=effective_queue_capacity,
                batch_size=int(dataset.get("writer_batch_size", 8)),
                flush_interval_s=float(dataset.get("writer_flush_interval_s", 1.0)),
                shutdown_timeout_s=float(dataset.get("writer_shutdown_timeout_s", 5.0)),
                require_frame_references=True,
            )
            collector = SingleEpisodeCollector(
                writer,
                camera_max_age_ns=round(
                    float(dataset.get("camera_max_age_ms", 100.0)) * 1e6
                ),
                control_max_age_ns=round(
                    float(dataset.get("control_max_age_ms", 40.0)) * 1e6
                ),
                maximum_start_delta_rad=maximum_start_delta_rad,
                maximum_hand_start_delta_rad=float(
                    dataset.get("hand_start_tolerance_rad", 0.05)
                ),
                defer_finalization=True,
                camera_severe_stale_ns=round(
                    float(dataset.get("camera_severe_stale_limit_ms", 500.0)) * 1e6
                ),
                camera_consecutive_stale_limit=int(
                    dataset.get("camera_consecutive_stale_limit", 15)
                ),
                camera_missing_timeout_ns=round(
                    float(dataset.get("camera_missing_timeout_ms", 1000.0)) * 1e6
                ),
                quality_min_valid_ratio=float(
                    dataset.get("quality_min_valid_ratio", 1.0)
                ),
                quality_max_invalid_run=int(
                    dataset.get("quality_max_invalid_run", 0)
                ),
            )
            writer.set_final_metadata_provider(
                lambda: {
                    "camera_profiles": {
                        role: workers[role].profile_metadata() for role in workers
                    },
                    **collector.diagnostics(),
                }
            )
            if preview_enabled:
                preview = AsyncDualCameraPreview(
                    workers["workspace"],
                    workers["wrist"],
                    PreviewStatus(
                        state=collector.state,
                        temporary_id=writer.temporary_id,
                        episode_start_ns=None,
                        arm_trigger=False,
                        hand_grip=False,
                        recording_frame_count=0,
                    ),
                    refresh_hz=float(dataset.get("preview_max_fps", 10.0)),
                )
                preview.start()
                collector.set_state_listener(preview.set_capture_state)
            return cls(
                collector=collector,
                cameras=workers,
                preview=preview,
                last_camera_timestamp_ns={"workspace": -1, "wrist": -1},
            )
        except BaseException:
            if preview is not None:
                preview.stop()
            if writer is not None and writer.start_monotonic_ns is None:
                writer.close()
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
        self.preview.update(
            PreviewStatus(
                state=self.collector.state,
                temporary_id=self.collector.writer.temporary_id,
                episode_start_ns=self.collector.writer.start_monotonic_ns,
                arm_trigger=arm_trigger,
                hand_grip=hand_grip,
                recording_frame_count=self.collector.writer.sample_count,
            )
        )

    def close(self) -> None:
        if self.preview is not None:
            self.preview.stop()
        for camera in self.cameras.values():
            camera.stop()
