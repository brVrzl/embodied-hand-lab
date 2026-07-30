from __future__ import annotations

import json
from pathlib import Path
import sys
import types

import numpy as np
import pytest

from episode_dataset.collector import CaptureState, SingleEpisodeCollector
from episode_dataset.async_writer import AsyncEpisodeWriter
from episode_dataset.episode import CameraSample, CanonicalEpisodeWriter, ControlSample
from episode_dataset.exporters import export_act_hdf5, export_lerobot_v3
from episode_dataset.timeline import CanonicalClock, CausalTimeline, TimestampRegression
from vision_interface.realsense_adapter import choose_closest_profile


def _camera(role: str, timestamp_ns: int, frame_number: int) -> CameraSample:
    return CameraSample(
        role=role,
        host_monotonic_ns=timestamp_ns,
        rgb=np.full((8, 10, 3), frame_number, dtype=np.uint8),
        depth_raw=np.full((8, 10), 1000 + frame_number, dtype=np.uint16),
        depth_aligned_to_rgb=np.full((8, 10), 1100 + frame_number, dtype=np.uint16),
        device_rgb_timestamp_ms=timestamp_ns / 1e6,
        device_depth_timestamp_ms=timestamp_ns / 1e6 + 0.1,
        rgb_frame_number=frame_number,
        depth_frame_number=frame_number,
        rgb_timestamp_domain="hardware_clock",
        depth_timestamp_domain="hardware_clock",
    )


def _control(timestamp_ns: int, *, trigger: bool, q: float = 0.1) -> ControlSample:
    return ControlSample(
        host_monotonic_ns=timestamp_ns,
        accepted_arm_q=(q,) * 6,
        arm_q_measured=(0.1,) * 6,
        arm_dq_measured=(0.0,) * 6,
        tcp_pose_xyzw=(0.3, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0),
        hand_observation=(0.2,) * 6,
        hand_source="measured",
        hand_target=(0.2,) * 6,
        arm_trigger=trigger,
        hand_grip=True,
        accepted_target_sequence=1,
        reference_generation=1,
        source_timestamps_ns={"quest": timestamp_ns - 1_000_000},
    )


def _collector(tmp_path: Path) -> SingleEpisodeCollector:
    writer = CanonicalEpisodeWriter(tmp_path, task_name="pick", operator="tester")
    return SingleEpisodeCollector(
        writer,
        camera_max_age_ns=33_333_334,
        control_max_age_ns=20_000_000,
        maximum_start_delta_rad=0.02,
        maximum_hand_start_delta_rad=0.02,
    )


def test_causal_timeline_never_selects_future_and_marks_stale() -> None:
    timeline = CausalTimeline[str](max_age_ns=10)
    timeline.append(100, "past")
    timeline.append(120, "future")
    selected = timeline.latest_at_or_before(115)
    assert selected.value == "past"
    assert selected.signed_offset_ns == -15
    assert selected.stale
    assert not selected.valid
    try:
        timeline.append(119, "backwards")
    except TimestampRegression:
        pass
    else:
        raise AssertionError("timestamp regression was accepted")


def test_canonical_clock_skips_deadlines_without_catchup_or_index_gap() -> None:
    clock = CanonicalClock(30)
    assert clock.start(100) == (0, 100)
    first = clock.due(100 + 5 * clock.period_ns)
    assert first == (1, 100 + clock.period_ns)
    assert clock.due(100 + 5 * clock.period_ns) is None
    assert clock.due(100 + 6 * clock.period_ns) == (2, 100 + 6 * clock.period_ns)


def test_trigger_bounds_exactly_one_episode_and_excludes_idle(
    tmp_path: Path, monkeypatch
) -> None:
    collector = _collector(tmp_path)
    base = 1_000_000_000
    collector.ingest_camera(_camera("workspace", base - 1_000_000, 1))
    collector.ingest_camera(_camera("wrist", base - 1_000_000, 1))
    collector.ingest_control(_control(base - 20_000_000, trigger=False), reference_established=False)
    assert collector.state is CaptureState.IDLE

    collector.ingest_control(_control(base, trigger=True), reference_established=True)
    assert collector.state is CaptureState.ARMING
    collector.ingest_camera(_camera("workspace", base + 1_000_000, 2))
    collector.ingest_camera(_camera("wrist", base + 1_000_000, 2))
    collector.ingest_control(
        _control(base + 17_000_000, trigger=True, q=0.101), reference_established=True
    )
    assert collector.state is CaptureState.REC
    collector.ingest_camera(_camera("workspace", base + 31_000_000, 3))
    collector.ingest_camera(_camera("wrist", base + 31_000_000, 3))
    collector.ingest_control(
        _control(base + 34_000_000, trigger=True, q=0.102), reference_established=True
    )
    collector.ingest_control(
        _control(base + 51_000_000, trigger=True, q=0.103), reference_established=True
    )
    collector.ingest_control(
        _control(base + 67_000_000, trigger=False, q=0.103), reference_established=True
    )
    assert collector.state is CaptureState.DONE
    assert collector.result is not None and collector.result.is_dir()
    metadata = json.loads((collector.result / "metadata.json").read_text(encoding="utf-8"))
    assert metadata["completion_status"] == "completed"
    assert metadata["sample_count"] == 2
    rows = [
        json.loads(line)
        for line in (collector.result / "canonical" / "samples.jsonl")
        .read_text(encoding="utf-8")
        .splitlines()
    ]
    assert rows[0]["timestamp"] == 0.0
    assert rows[0]["action"]["arm_q_target"] == [0.101] * 6
    assert rows[0]["timing"]["signed_offsets_ns"]["workspace"] == -16_000_000
    assert rows[-1]["timestamp_host_monotonic_ns"] < base + 67_000_000
    assert not list(tmp_path.glob("*.partial"))
    class FakeLeRobotDataset:
        created = None

        def __init__(self, root: Path) -> None:
            self.root = root
            self.frames = []
            self.saved = False
            self.finalized = False

        @classmethod
        def create(cls, **kwargs):
            root = Path(kwargs["root"])
            (root / "meta").mkdir(parents=True)
            cls.created = cls(root)
            cls.created.create_kwargs = kwargs
            return cls.created

        def add_frame(self, frame):
            self.frames.append(frame)

        def save_episode(self):
            self.saved = True

        def finalize(self):
            self.finalized = True

        def has_pending_frames(self):
            return False

    lerobot = types.ModuleType("lerobot")
    datasets = types.ModuleType("lerobot.datasets")
    module = types.ModuleType("lerobot.datasets.lerobot_dataset")
    module.LeRobotDataset = FakeLeRobotDataset
    monkeypatch.setitem(sys.modules, "lerobot", lerobot)
    monkeypatch.setitem(sys.modules, "lerobot.datasets", datasets)
    monkeypatch.setitem(sys.modules, "lerobot.datasets.lerobot_dataset", module)
    lerobot_path = export_lerobot_v3(
        collector.result, tmp_path / "lerobot", repo_id="local/test"
    )
    fake = FakeLeRobotDataset.created
    assert fake is not None and fake.saved and fake.finalized
    assert len(fake.frames) == 2
    assert fake.frames[0]["action"].shape == (12,)
    assert fake.frames[0]["observation.state"].shape == (25,)
    sidecar = json.loads(
        (lerobot_path / "meta" / "embodied_lab_depth_sidecar.json").read_text(
            encoding="utf-8"
        )
    )
    assert sidecar["official_lerobot_feature"] is False
    assert len(list((lerobot_path / "depth_sidecar" / "workspace" / "depth_raw").glob("*.npy"))) == 2


def test_act_hdf5_export_uses_optional_dependency(tmp_path: Path) -> None:
    h5py = pytest.importorskip(
        "h5py", reason="ACT export requires the dataset-export extra"
    )
    collector = _collector(tmp_path)
    base = 1_500_000_000
    collector.ingest_camera(_camera("workspace", base, 1))
    collector.ingest_camera(_camera("wrist", base, 1))
    collector.ingest_control(_control(base, trigger=True), reference_established=True)
    collector.ingest_control(
        _control(base + 17_000_000, trigger=True, q=0.101),
        reference_established=True,
    )
    collector.ingest_control(
        _control(base + 34_000_000, trigger=False, q=0.101),
        reference_established=True,
    )
    assert collector.result is not None
    act_path = export_act_hdf5(collector.result, tmp_path / "episode.hdf5")
    with h5py.File(act_path, "r") as dataset:
        assert dataset["action"].shape == (1, 12)
        assert dataset["observations/qpos"].shape == (1, 12)
        assert dataset["observations/qvel"].shape == (1, 6)
        assert dataset["observations/images/workspace"].shape == (1, 8, 10, 3)
        assert dataset["observations/depth/wrist"].dtype == np.uint16
        assert dataset.attrs["success_label"] == "unlabeled"


def test_dataset_modules_import_without_optional_dependencies(monkeypatch) -> None:
    import builtins
    import importlib

    original_import = builtins.__import__

    def guarded_import(name, *args, **kwargs):
        if name.split(".", 1)[0] in {"cv2", "h5py", "lerobot"}:
            raise ModuleNotFoundError(name)
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", guarded_import)
    preview = importlib.reload(importlib.import_module("episode_dataset.preview"))
    importlib.reload(sys.modules["episode_dataset.exporters"])
    tool_spec = importlib.util.spec_from_file_location(
        "quest_jaka_mujoco_sim_optional_dependency_test",
        Path("tools/quest_jaka_mujoco_sim.py"),
    )
    assert tool_spec is not None and tool_spec.loader is not None
    tool_module = importlib.util.module_from_spec(tool_spec)
    tool_spec.loader.exec_module(tool_module)
    with pytest.raises(RuntimeError, match="episode preview requires OpenCV"):
        preview.require_preview_dependencies()


def test_continuity_failure_writes_report_without_partial_episode(tmp_path: Path) -> None:
    collector = _collector(tmp_path)
    base = 2_000_000_000
    collector.ingest_camera(_camera("workspace", base, 1))
    collector.ingest_camera(_camera("wrist", base, 1))
    collector.ingest_control(_control(base, trigger=True, q=0.5), reference_established=True)
    assert collector.state is CaptureState.DONE
    assert collector.result is not None and collector.result.name.startswith("rejected-start-")
    assert not list(tmp_path.glob("episode-*"))
    assert not list(tmp_path.glob("*.partial"))


def test_async_writer_drains_before_atomic_finalize(tmp_path: Path) -> None:
    writer = AsyncEpisodeWriter(
        CanonicalEpisodeWriter(tmp_path, task_name="pick", operator="tester")
    )
    collector = SingleEpisodeCollector(
        writer,
        camera_max_age_ns=33_333_334,
        control_max_age_ns=20_000_000,
        maximum_start_delta_rad=0.02,
        maximum_hand_start_delta_rad=0.02,
    )
    base = 2_500_000_000
    collector.ingest_camera(_camera("workspace", base, 1))
    collector.ingest_camera(_camera("wrist", base, 1))
    collector.ingest_control(_control(base, trigger=True), reference_established=True)
    collector.ingest_control(_control(base + 10_000_000, trigger=False), reference_established=True)
    assert collector.state is CaptureState.DONE
    metadata = json.loads((collector.result / "metadata.json").read_text(encoding="utf-8"))
    assert metadata["completion_status"] == "completed"
    assert metadata["sample_count"] == 1
    assert (collector.result / "canonical" / "frames" / "workspace" / "rgb" / "000000.npy").is_file()


def test_camera_staleness_aborts_instead_of_copying_old_frame(tmp_path: Path) -> None:
    collector = _collector(tmp_path)
    base = 3_000_000_000
    collector.ingest_camera(_camera("workspace", base, 1))
    collector.ingest_camera(_camera("wrist", base, 1))
    collector.ingest_control(_control(base, trigger=True), reference_established=True)
    collector.ingest_control(_control(base + 17_000_000, trigger=True), reference_established=True)
    collector.ingest_control(_control(base + 34_000_000, trigger=True), reference_established=True)
    collector.ingest_control(_control(base + 51_000_000, trigger=True), reference_established=True)
    collector.ingest_control(_control(base + 67_000_000, trigger=True), reference_established=True)
    assert collector.state is CaptureState.DONE
    metadata = json.loads((collector.result / "metadata.json").read_text(encoding="utf-8"))
    assert metadata["completion_status"] == "aborted"
    assert metadata["termination_reason"].startswith("stale_or_missing_source")


def test_device_timestamp_regression_invalidates_recording(tmp_path: Path) -> None:
    collector = _collector(tmp_path)
    base = 3_500_000_000
    collector.ingest_camera(_camera("workspace", base, 10))
    collector.ingest_camera(_camera("wrist", base, 10))
    collector.ingest_control(_control(base, trigger=True), reference_established=True)
    collector.ingest_camera(_camera("workspace", base + 1_000_000, 9))
    assert collector.state is CaptureState.DONE
    metadata = json.loads((collector.result / "metadata.json").read_text(encoding="utf-8"))
    assert metadata["completion_status"] == "invalid"
    assert "device_timestamp_or_frame_regression" in metadata["termination_reason"]


def test_profile_selection_prefers_30hz_then_nearest_resolution() -> None:
    profiles = [(1280, 720, 15), (848, 480, 30), (640, 360, 30), (640, 480, 15)]
    assert choose_closest_profile(profiles, width=640, height=480, fps=30) == (640, 360, 30)
