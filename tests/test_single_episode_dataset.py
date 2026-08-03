from __future__ import annotations

import hashlib
import json
from pathlib import Path
import shutil
import sys
import tomllib
import types

import numpy as np
import pytest

from episode_dataset.collector import CaptureState, SingleEpisodeCollector
from episode_dataset.async_writer import AsyncEpisodeWriter
from episode_dataset.cli import build_parser as build_dataset_parser
from episode_dataset.episode import (
    CameraSample,
    CanonicalEpisodeWriter,
    ControlSample,
    PHYSICAL_SCHEMA_VERSION,
    StartPrerequisites,
)
from episode_dataset.exporters import (
    _validate_lerobot_video_shape,
    export_act_hdf5,
    export_lerobot_v3,
)
from episode_dataset.manifest import build_dataset_manifest, compute_train_statistics
from episode_dataset.inspection import inspect_episode, write_inspection_plot
from episode_dataset.timeline import CanonicalClock, CausalTimeline, TimestampRegression
from episode_dataset.validation import validate_episode
from vision_interface.realsense_adapter import choose_closest_profile


def test_dataset_export_extra_installs_lerobot_dataset_dependencies() -> None:
    project = tomllib.loads(Path("pyproject.toml").read_text(encoding="utf-8"))
    requirements = project["project"]["optional-dependencies"]["dataset-export"]
    assert any(
        requirement.startswith("lerobot[dataset]") for requirement in requirements
    )


def test_lerobot_export_rejects_video_width_that_hangs_default_encoder() -> None:
    with pytest.raises(ValueError, match="width must be at least 32px"):
        _validate_lerobot_video_shape(
            "workspace", np.zeros((8, 10, 3), dtype=np.uint8)
        )
    _validate_lerobot_video_shape(
        "workspace", np.zeros((8, 32, 3), dtype=np.uint8)
    )


def _camera(
    role: str, timestamp_ns: int, frame_number: int, *, width: int = 10
) -> CameraSample:
    return CameraSample(
        role=role,
        host_monotonic_ns=timestamp_ns,
        rgb=np.full((8, width, 3), frame_number, dtype=np.uint8),
        depth_raw=np.full((8, width), 1000 + frame_number, dtype=np.uint16),
        depth_aligned_to_rgb=np.full((8, width), 1100 + frame_number, dtype=np.uint16),
        depth_scale_m=0.001,
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


def _collector(
    tmp_path: Path,
    *,
    fps: int = 30,
    simulation_only: bool = False,
) -> SingleEpisodeCollector:
    writer = CanonicalEpisodeWriter(
        tmp_path,
        task_name="pick",
        operator="tester",
        dataset_fps=fps,
        metadata={"simulation_only": simulation_only},
    )
    return SingleEpisodeCollector(
        writer,
        camera_max_age_ns=33_333_334,
        control_max_age_ns=20_000_000,
        maximum_start_delta_rad=0.02,
        maximum_hand_start_delta_rad=0.02,
    )


def _set_success_label(episode: Path, label: str) -> None:
    metadata_path = episode / "metadata.json"
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    metadata["success_label"] = label
    metadata_path.write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
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
    assert clock.last_nominal_slot_index == 1
    assert clock.last_missed_slots_before == 0
    assert clock.last_missed_slots_after == 4
    assert clock.due(100 + 5 * clock.period_ns) is None
    assert clock.due(100 + 6 * clock.period_ns) == (2, 100 + 6 * clock.period_ns)
    assert clock.last_nominal_slot_index == 6
    assert clock.last_missed_slots_before == 4
    assert clock.total_missed_slots == 4


def test_physical_v2_preserves_normalized_hand_unit(tmp_path: Path) -> None:
    writer = CanonicalEpisodeWriter(
        tmp_path,
        task_name="pick",
        operator="tester",
        schema_version=PHYSICAL_SCHEMA_VERSION,
        metadata={"units": {"hand": "normalized_closure_0_to_1"}},
    )
    collector = SingleEpisodeCollector(
        writer,
        camera_max_age_ns=33_333_334,
        control_max_age_ns=20_000_000,
        maximum_start_delta_rad=0.02,
        maximum_hand_start_delta_rad=0.02,
    )
    base = 1_000_000_000
    collector.ingest_control(_control(base, trigger=True), reference_established=True)
    collector.ingest_camera(_camera("workspace", base + 1_000_000, 1, width=32))
    collector.ingest_camera(_camera("wrist", base + 1_000_000, 1, width=32))
    collector.ingest_control(
        _control(base + 2_000_000, trigger=True), reference_established=True
    )
    collector.ingest_control(
        _control(base + 3_000_000, trigger=False),
        reference_established=True,
        capture_active=True,
    )
    assert collector.state is CaptureState.REC
    collector.writer.append_raw(
        "jaka_state",
        {
            "read_host_monotonic_ns": base + 1_000_000,
            "record_host_monotonic_ns": base + 2_000_000,
            "command_host_monotonic_ns": base + 1_000_000,
            "accepted_joint_target_rad": [0.1] * 6,
            "measured_joint_position_rad": [0.1] * 6,
            "estimated_joint_velocity_rad_s": [0.0] * 6,
            "commanded_tcp_pose_xyzw": [0.3, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0],
        },
    )
    collector.writer.append_raw(
        "rh56_feedback",
        {
            "action": {"hand_target": [0.2] * 6},
            "hand_command_timestamp": base + 1_000_000,
            "hand_feedback_timestamp": base + 1_000_000,
            "hand_feedback_register_timestamps_ns": {
                name: base + 1_000_000
                for name in ("ANGLE_ACT", "CURRENT", "FORCE_ACT", "ERROR", "STATUS")
            },
            "rh56_registers": {
                name: [0.0] * 6
                for name in ("ANGLE_ACT", "CURRENT", "FORCE_ACT", "ERROR", "STATUS")
            },
        },
    )
    collector.writer.append_raw(
        "rh56_feedback",
        {
            "action": {"hand_target": None},
            "hand_command_timestamp": None,
            "hand_feedback_timestamp": base + 2_000_000,
            "hand_feedback_register_timestamps_ns": {
                name: base + 2_000_000
                for name in ("ANGLE_ACT", "CURRENT", "FORCE_ACT", "ERROR", "STATUS")
            },
            "rh56_registers": {
                name: [0.0] * 6
                for name in ("ANGLE_ACT", "CURRENT", "FORCE_ACT", "ERROR", "STATUS")
            },
        },
    )
    collector.finish("duration_complete")

    assert collector.result is not None
    report = validate_episode(collector.result, deep=True)
    assert report["schema_version"] == PHYSICAL_SCHEMA_VERSION
    assert report["valid"]


def test_trigger_bounds_exactly_one_episode_and_excludes_idle(
    tmp_path: Path, monkeypatch
) -> None:
    collector = _collector(tmp_path, fps=25)
    base = 1_000_000_000
    collector.ingest_camera(_camera("workspace", base - 1_000_000, 1, width=32))
    collector.ingest_camera(_camera("wrist", base - 1_000_000, 1, width=32))
    collector.ingest_control(_control(base - 20_000_000, trigger=False), reference_established=False)
    assert collector.state is CaptureState.IDLE

    collector.ingest_control(_control(base, trigger=True), reference_established=True)
    assert collector.state is CaptureState.ARMING
    collector.ingest_camera(_camera("workspace", base + 1_000_000, 2, width=32))
    collector.ingest_camera(_camera("wrist", base + 1_000_000, 2, width=32))
    collector.ingest_control(
        _control(base + 17_000_000, trigger=True, q=0.101), reference_established=True
    )
    assert collector.state is CaptureState.REC
    collector.ingest_camera(_camera("workspace", base + 41_000_000, 3, width=32))
    collector.ingest_camera(_camera("wrist", base + 41_000_000, 3, width=32))
    collector.ingest_control(
        _control(base + 40_000_000, trigger=True, q=0.102), reference_established=True
    )
    collector.ingest_control(
        _control(base + 58_000_000, trigger=True, q=0.103), reference_established=True
    )
    collector.ingest_control(
        _control(base + 80_000_000, trigger=False, q=0.103), reference_established=True
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
    unlabeled_report = validate_episode(collector.result)
    assert unlabeled_report["valid"]
    assert not unlabeled_report["training_eligible"]
    assert unlabeled_report["success_label"] == "unlabeled"
    _set_success_label(collector.result, "success")

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
    assert fake.create_kwargs["fps"] == 25
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

    class FailingLeRobotDataset:
        @classmethod
        def create(cls, **kwargs):
            Path(kwargs["root"]).mkdir(parents=True)
            raise RuntimeError("synthetic LeRobot create failure")

    module.LeRobotDataset = FailingLeRobotDataset
    with pytest.raises(RuntimeError, match="synthetic LeRobot create failure"):
        export_lerobot_v3(
            collector.result, tmp_path / "failed-lerobot", repo_id="local/test"
        )
    assert not list(tmp_path.glob(".failed-lerobot.*.partial"))


def test_act_hdf5_export_uses_optional_dependency(tmp_path: Path) -> None:
    h5py = pytest.importorskip(
        "h5py", reason="ACT export requires the dataset-export extra"
    )
    collector = _collector(tmp_path, simulation_only=True)
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
    with pytest.raises(ValueError, match="explicitly labeled success/failure"):
        export_act_hdf5(collector.result, tmp_path / "unlabeled.hdf5")
    _set_success_label(collector.result, "success")
    act_path = export_act_hdf5(collector.result, tmp_path / "episode.hdf5")
    with h5py.File(act_path, "r") as dataset:
        assert dataset["action"].shape == (1, 12)
        assert dataset["observations/qpos"].shape == (1, 12)
        assert dataset["observations/qvel"].shape == (1, 6)
        assert dataset["observations/images/workspace"].shape == (1, 8, 10, 3)
        assert dataset["observations/depth/wrist"].dtype == np.uint16
        assert dataset.attrs["success_label"] == "success"
        assert bool(dataset.attrs["sim"]) is True


def test_episode_inspection_summarizes_actions_and_timing(tmp_path: Path) -> None:
    episode = _complete_one_sample_episode(tmp_path, 1_750_000_000)
    report = inspect_episode(episode)

    assert report["inspection_available"] is True
    assert report["sample_count"] == 1
    assert report["arm_action_status_counts"] == {"accepted": 1}
    assert report["arm_command_state_error_rad"]["maximum"] == pytest.approx(0.0)
    assert report["source_offset_summary"]["workspace"]["maximum_ms"] == 0.0
    plot = write_inspection_plot(episode, tmp_path / "inspection.png")
    assert plot.is_file() and plot.stat().st_size > 0


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


def test_begin_failure_removes_owned_partial_staging(
    tmp_path: Path, monkeypatch
) -> None:
    calibration = tmp_path / "calibration.json"
    calibration.write_text("{}\n", encoding="utf-8")
    writer = CanonicalEpisodeWriter(
        tmp_path / "episodes",
        task_name="pick",
        operator="tester",
        metadata={"calibration_files": [calibration]},
    )
    base = 2_200_000_000
    prerequisites = StartPrerequisites(
        trigger_press_monotonic_ns=base,
        reference_established=True,
        accepted=_control(base, trigger=True),
        workspace=_camera("workspace", base, 1),
        wrist=_camera("wrist", base, 1),
        maximum_start_delta_rad=0.02,
        maximum_hand_start_delta_rad=0.02,
    )

    first_snapshot = tmp_path / "first" / "calibration.json"
    second_snapshot = tmp_path / "second" / "calibration.json"
    first_snapshot.parent.mkdir()
    second_snapshot.parent.mkdir()
    first_snapshot.write_text('{"camera": "workspace"}\n', encoding="utf-8")
    second_snapshot.write_text('{"camera": "wrist"}\n', encoding="utf-8")
    duplicate_writer = CanonicalEpisodeWriter(
        tmp_path / "duplicates",
        task_name="pick",
        operator="tester",
        metadata={"calibration_files": [first_snapshot, second_snapshot]},
    )
    with pytest.raises(ValueError, match="basenames must be unique"):
        duplicate_writer.begin(
            prerequisites, camera_max_age_ns=33_333_334
        )
    assert not duplicate_writer.partial_dir.exists()

    def fail_copy(*_args, **_kwargs):
        raise OSError("synthetic calibration copy failure")

    monkeypatch.setattr("episode_dataset.episode.shutil.copy2", fail_copy)
    with pytest.raises(OSError, match="synthetic"):
        writer.begin(prerequisites, camera_max_age_ns=33_333_334)
    report = writer.discard_rejected_start("calibration_copy_failed")
    assert report.is_file()
    assert not writer.partial_dir.exists()


def _complete_one_sample_episode(root: Path, base: int) -> Path:
    collector = _collector(root)
    collector.ingest_camera(_camera("workspace", base, 1))
    collector.ingest_camera(_camera("wrist", base, 1))
    collector.ingest_control(
        _control(base, trigger=True), reference_established=True
    )
    collector.ingest_control(
        _control(base + 10_000_000, trigger=False),
        reference_established=True,
    )
    assert collector.result is not None and collector.result.is_dir()
    return collector.result


def test_timing_gap_is_recorded_and_excluded_from_training(tmp_path: Path) -> None:
    collector = _collector(tmp_path)
    base = 2_300_000_000
    collector.ingest_camera(_camera("workspace", base, 1))
    collector.ingest_camera(_camera("wrist", base, 1))
    collector.ingest_control(
        _control(base, trigger=True), reference_established=True
    )
    collector.ingest_camera(_camera("workspace", base + 30_000_000, 2))
    collector.ingest_camera(_camera("wrist", base + 30_000_000, 2))
    collector.ingest_control(
        _control(base + 30_000_000, trigger=True),
        reference_established=True,
    )
    collector.ingest_control(
        _control(base + 200_000_000, trigger=True),
        reference_established=True,
    )
    collector.ingest_control(
        _control(base + 201_000_000, trigger=False),
        reference_established=True,
    )
    assert collector.result is not None
    rows = [
        json.loads(line)
        for line in (
            collector.result / "canonical" / "samples.jsonl"
        ).read_text(encoding="utf-8").splitlines()
    ]
    assert rows[1]["timing"]["missed_slots_after"] > 0
    metadata = json.loads(
        (collector.result / "metadata.json").read_text(encoding="utf-8")
    )
    assert metadata["canonical_missed_slot_count"] == rows[1]["timing"][
        "missed_slots_after"
    ]
    _set_success_label(collector.result, "success")
    report = validate_episode(collector.result)
    assert report["valid"]
    assert not report["training_eligible"]
    assert report["quality"]["canonical_missed_slot_count"] > 0


def test_deep_validator_rejects_corrupt_camera_array(tmp_path: Path) -> None:
    episode = _complete_one_sample_episode(tmp_path, 2_400_000_000)
    _set_success_label(episode, "success")
    rgb = episode / "canonical/frames/workspace/rgb/000000.npy"
    np.save(rgb, np.zeros((8, 10, 3), dtype=np.float32), allow_pickle=False)
    report = validate_episode(episode)
    assert not report["valid"]
    assert any("expected uint8/3D" in error for error in report["errors"])
    manifest_path = build_dataset_manifest(tmp_path, tmp_path / "manifest.json")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["deep_validation"] is True
    assert manifest["split_counts"]["excluded"] == 1
    assert manifest["episodes"][0]["training_eligible"] is False
    assert any(
        "expected uint8/3D" in error
        for error in manifest["episodes"][0]["validation_errors"]
    )


def test_manifest_excludes_every_duplicate_uuid_occurrence(tmp_path: Path) -> None:
    episode = _complete_one_sample_episode(tmp_path, 2_425_000_000)
    _set_success_label(episode, "success")
    duplicate = tmp_path / "episode-duplicate"
    shutil.copytree(episode, duplicate)

    manifest_path = build_dataset_manifest(tmp_path, tmp_path / "manifest.json")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["split_counts"] == {
        "train": 0,
        "validation": 0,
        "test": 0,
        "excluded": 2,
    }
    assert all(
        entry["duplicate_or_missing_uuid"] is True
        and entry["training_eligible"] is False
        and entry["split"] == "excluded"
        for entry in manifest["episodes"]
    )

    duplicate_metadata = duplicate / "metadata.json"
    metadata = json.loads(duplicate_metadata.read_text(encoding="utf-8"))
    metadata["episode_uuid"] = "not-a-uuid"
    duplicate_metadata.write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    report = validate_episode(duplicate, deep=False)
    assert not report["valid"]
    assert any("canonical UUID string" in error for error in report["errors"])


def test_manifest_split_and_train_only_statistics_are_episode_level(
    tmp_path: Path,
) -> None:
    episode = _complete_one_sample_episode(tmp_path, 2_450_000_000)
    parser = build_dataset_parser()
    default_arguments = parser.parse_args(
        ["manifest", str(tmp_path), str(tmp_path / "default.json")]
    )
    fast_arguments = parser.parse_args(
        ["manifest", str(tmp_path), str(tmp_path / "fast.json"), "--fast"]
    )
    assert default_arguments.fast is False
    assert fast_arguments.fast is True

    metadata = json.loads((episode / "metadata.json").read_text(encoding="utf-8"))
    episode_uuid = metadata["episode_uuid"]
    seed = next(
        f"test-seed-{index}"
        for index in range(100)
        if int.from_bytes(
            hashlib.sha256(
                f"test-seed-{index}:{episode_uuid}".encode("utf-8")
            ).digest()[:8],
            "big",
        )
        / float(1 << 64)
        < 0.8
    )
    unlabeled_manifest_path = build_dataset_manifest(
        tmp_path,
        tmp_path / "unlabeled-manifest.json",
        seed=seed,
        deep_validation=True,
    )
    unlabeled_manifest = json.loads(
        unlabeled_manifest_path.read_text(encoding="utf-8")
    )
    assert unlabeled_manifest["split_counts"]["excluded"] == 1
    assert unlabeled_manifest["episodes"][0]["success_label"] == "unlabeled"
    assert unlabeled_manifest["episodes"][0]["training_eligible"] is False

    _set_success_label(episode, "ambiguous")
    invalid_label_report = validate_episode(episode, deep=False)
    assert not invalid_label_report["valid"]
    assert any(
        "metadata.success_label" in error
        for error in invalid_label_report["errors"]
    )

    _set_success_label(episode, "failure")
    fast_manifest_path = build_dataset_manifest(
        tmp_path,
        tmp_path / "fast-manifest.json",
        seed=seed,
        deep_validation=False,
    )
    fast_manifest = json.loads(fast_manifest_path.read_text(encoding="utf-8"))
    assert fast_manifest["deep_validation"] is False
    assert fast_manifest["split_counts"]["excluded"] == 1
    assert any(
        "inventory" in warning
        for warning in fast_manifest["episodes"][0]["validation_warnings"]
    )
    with pytest.raises(ValueError, match="deep-validation manifest"):
        compute_train_statistics(
            fast_manifest_path, tmp_path / "fast-statistics.json"
        )

    _set_success_label(episode, "success")
    manifest_path = build_dataset_manifest(
        tmp_path, tmp_path / "manifest.json", seed=seed, deep_validation=True
    )
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["split_counts"]["train"] == 1
    assert manifest["episodes"][0]["path"] == episode.name
    statistics_path = compute_train_statistics(
        manifest_path, tmp_path / "statistics.json"
    )
    statistics = json.loads(statistics_path.read_text(encoding="utf-8"))
    assert statistics["observation_state"]["count"] == 1
    assert statistics["action"]["count"] == 1
    assert statistics["episodes_used"] == [episode_uuid]

    canonical_index = episode / "canonical" / "samples.jsonl"
    canonical_index.write_text(
        canonical_index.read_text(encoding="utf-8").rstrip("\n") + " \n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="canonical_index_sha256 changed"):
        compute_train_statistics(
            manifest_path, tmp_path / "stale-statistics.json"
        )


def test_manifest_and_act_export_exclude_reviewed_failures(tmp_path: Path) -> None:
    episode = _complete_one_sample_episode(tmp_path, 2_475_000_000)
    _set_success_label(episode, "failure")
    report = validate_episode(episode, deep=True)
    assert report["training_eligible"] is True

    manifest_path = build_dataset_manifest(
        tmp_path, tmp_path / "manifest.json", deep_validation=True
    )
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["split_counts"]["excluded"] == 1
    assert manifest["episodes"][0]["training_eligible"] is False
    assert any(
        "failure episode is excluded" in warning
        for warning in manifest["episodes"][0]["validation_warnings"]
    )
    with pytest.raises(ValueError, match="requires success_label='success'"):
        export_act_hdf5(episode, tmp_path / "failure.hdf5")


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


def test_collector_shutdown_closes_idle_and_rejects_arming_writer(
    tmp_path: Path,
) -> None:
    idle_writer = AsyncEpisodeWriter(
        CanonicalEpisodeWriter(
            tmp_path / "idle", task_name="pick", operator="tester"
        )
    )
    idle = SingleEpisodeCollector(
        idle_writer,
        maximum_start_delta_rad=0.02,
        maximum_hand_start_delta_rad=0.02,
    )
    idle.shutdown("capture_loop_ended")
    assert idle.state is CaptureState.DONE
    assert idle.result is None
    assert not list((tmp_path / "idle").glob("rejected-start-*.json"))
    with pytest.raises(OSError, match="closed"):
        idle_writer.append_raw("test", {})

    arming_writer = AsyncEpisodeWriter(
        CanonicalEpisodeWriter(
            tmp_path / "arming", task_name="pick", operator="tester"
        )
    )
    arming = SingleEpisodeCollector(
        arming_writer,
        maximum_start_delta_rad=0.02,
        maximum_hand_start_delta_rad=0.02,
    )
    arming.ingest_control(
        _control(2_600_000_000, trigger=True),
        reference_established=False,
    )
    assert arming.state is CaptureState.ARMING
    arming.shutdown("operator_interrupt")
    assert arming.state is CaptureState.DONE
    assert arming.result is not None
    rejection = json.loads(arming.result.read_text(encoding="utf-8"))
    assert rejection["termination_reason"] == "operator_interrupt"
    with pytest.raises(OSError, match="closed"):
        arming_writer.append_raw("test", {})

    with pytest.raises(ValueError, match="finite and non-negative"):
        SingleEpisodeCollector(
            CanonicalEpisodeWriter(
                tmp_path / "invalid", task_name="pick", operator="tester"
            ),
            maximum_start_delta_rad=float("nan"),
            maximum_hand_start_delta_rad=0.02,
        )


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


def test_default_recorder_control_age_accepts_observed_producer_jitter(
    tmp_path: Path,
) -> None:
    collector = SingleEpisodeCollector(
        CanonicalEpisodeWriter(tmp_path, task_name="pick", operator="tester"),
        maximum_start_delta_rad=0.02,
        maximum_hand_start_delta_rad=0.02,
    )
    base = 3_250_000_000
    collector.ingest_camera(_camera("workspace", base, 1))
    collector.ingest_camera(_camera("wrist", base, 1))
    collector.ingest_control(
        _control(base, trigger=True), reference_established=True
    )
    # At the 33.33 ms canonical slot this new producer row is still in the
    # future, so the selector must reuse the 33.33 ms old row. The target host
    # produced a measured maximum interval of 30.6 ms during physical capture.
    collector.ingest_control(
        _control(base + 34_000_000, trigger=True), reference_established=True
    )
    assert collector.state is CaptureState.REC


def test_default_recorder_camera_age_accepts_observed_producer_stall(
    tmp_path: Path,
) -> None:
    collector = SingleEpisodeCollector(
        CanonicalEpisodeWriter(tmp_path, task_name="pick", operator="tester"),
        maximum_start_delta_rad=0.02,
        maximum_hand_start_delta_rad=0.02,
    )
    base = 3_300_000_000
    # A healthy 30 Hz frame can be about 72 ms behind the next causal slot
    # after the measured ~83 ms outer-loop stall on the target host.
    collector.ingest_camera(_camera("workspace", base + 25_000_000, 1))
    collector.ingest_camera(_camera("wrist", base + 25_000_000, 1))
    collector.ingest_control(
        _control(base, trigger=True), reference_established=True
    )
    collector.ingest_control(
        _control(base + 30_000_000, trigger=True), reference_established=True
    )
    collector.ingest_control(
        _control(base + 70_000_000, trigger=True), reference_established=True
    )
    collector.ingest_control(
        _control(base + 105_000_000, trigger=True), reference_established=True
    )
    assert collector.state is CaptureState.REC


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
