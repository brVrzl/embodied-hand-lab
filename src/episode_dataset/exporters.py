from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
from types import ModuleType
from typing import Any
import uuid

import numpy as np

from .episode import ACTION_ORDER, OBSERVATION_STATE_ORDER, file_sha256
from .validation import load_canonical_rows, validate_episode


PROVENANCE_CODE = {"unavailable": 0, "commanded": 1, "estimated": 2, "measured": 3}
ACTION_STATUS_CODE = {"accepted": 0, "held_rejected": 1}
TIMING_NAMES = (
    "control",
    "accepted_action_host",
    "quest_host_receive",
    "quest",
    "workspace",
    "wrist",
)
INVALID_INT64 = np.iinfo(np.int64).min
MIN_LEROBOT_VIDEO_WIDTH_PX = 32


def _require_cv2() -> ModuleType:
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError(
            "exporting non-NPY episode images requires OpenCV; "
            "install embodied-lab[dataset-export]"
        ) from exc
    return cv2


def _training_episode(
    episode_dir: Path,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    report = validate_episode(episode_dir, deep=False)
    if not report["valid"]:
        raise ValueError(
            "episode integrity validation failed: "
            + "; ".join(report["errors"])
        )
    if not report["training_eligible"]:
        raise ValueError(
            "episode is not training eligible; it must be completed, non-empty, "
            "explicitly labeled success/failure, and contain no compressed "
            "canonical timing gaps"
        )
    metadata = json.loads(
        (episode_dir / "metadata.json").read_text(encoding="utf-8")
    )
    rows, errors = load_canonical_rows(episode_dir)
    if errors:
        raise ValueError("cannot read canonical rows: " + "; ".join(errors))
    return metadata, rows


def _rgb(episode_dir: Path, relative: str) -> np.ndarray:
    if relative.endswith(".npy"):
        image = np.load(episode_dir / relative, allow_pickle=False)
        if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
            raise OSError(f"failed to read lossless uint8 RGB {relative}")
        return image
    cv2 = _require_cv2()
    image = cv2.imread(str(episode_dir / relative), cv2.IMREAD_COLOR)
    if image is None:
        raise OSError(f"failed to read {relative}")
    return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)


def _depth(episode_dir: Path, relative: str) -> np.ndarray:
    if relative.endswith(".npy"):
        image = np.load(episode_dir / relative, allow_pickle=False)
        if image.dtype != np.uint16 or image.ndim != 2:
            raise OSError(f"failed to read lossless uint16 depth {relative}")
        return image
    cv2 = _require_cv2()
    image = cv2.imread(str(episode_dir / relative), cv2.IMREAD_UNCHANGED)
    if image is None or image.dtype != np.uint16 or image.ndim != 2:
        raise OSError(f"failed to read lossless uint16 depth {relative}")
    return image


def observation_vector(row: dict[str, Any]) -> np.ndarray:
    state = row["observation"]["state"]
    return np.asarray(
        state["arm_q_measured"]
        + state["arm_dq_measured"]
        + state["tcp_pose"]
        + state["hand"],
        dtype=np.float32,
    )


def action_vector(row: dict[str, Any]) -> np.ndarray:
    action = row["action"]
    return np.asarray(action["arm_q_target"] + action["hand_target"], dtype=np.float32)


def _validate_lerobot_video_shape(name: str, image: np.ndarray) -> None:
    """Reject frames that the official default SVT-AV1 encoder cannot finish."""

    width = int(image.shape[1])
    if width < MIN_LEROBOT_VIDEO_WIDTH_PX:
        raise ValueError(
            f"{name} RGB width {width}px is unsupported by the default LeRobot "
            f"SVT-AV1 encoder; width must be at least {MIN_LEROBOT_VIDEO_WIDTH_PX}px"
        )


def export_act_hdf5(episode: str | Path, output: str | Path) -> Path:
    """Export one canonical episode using an ACT-style RGB/state/action layout.

    Lossless depth is an explicit extension under ``/observations/depth``. The
    canonical episode remains authoritative and is not changed by this export.
    The 12-D project view still needs an adapter for upstream ACT reference
    code that assumes the ALOHA 14-D embodiment.
    """

    try:
        import h5py
    except ImportError as exc:
        raise RuntimeError("ACT HDF5 export requires the optional h5py package") from exc

    episode_dir = Path(episode).resolve()
    output_path = Path(output).resolve()
    metadata, rows = _training_episode(episode_dir)
    if output_path.exists():
        raise FileExistsError(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = output_path.with_name(
        f".{output_path.name}.{uuid.uuid4().hex}.partial"
    )
    first = rows[0]
    first_workspace_rgb = _rgb(
        episode_dir, first["observation"]["images"]["workspace"]["rgb"]
    )
    first_wrist_rgb = _rgb(
        episode_dir, first["observation"]["images"]["wrist"]["rgb"]
    )
    first_workspace_depth = _depth(
        episode_dir, first["observation"]["images"]["workspace"]["depth_raw"]
    )
    first_wrist_depth = _depth(
        episode_dir, first["observation"]["images"]["wrist"]["depth_raw"]
    )
    count = len(rows)
    try:
        with h5py.File(temporary, "x") as root:
            root.attrs["sim"] = bool(metadata.get("simulation_only", False))
            root.attrs["episode_uuid"] = metadata["episode_uuid"]
            root.attrs["completion_status"] = metadata["completion_status"]
            root.attrs["success_label"] = metadata["success_label"]
            root.attrs["valid"] = True
            root.attrs["dataset_fps"] = int(metadata["dataset_fps"])
            root.attrs["canonical_schema_version"] = metadata["schema_version"]
            root.attrs["action_order"] = json.dumps(ACTION_ORDER)
            root.attrs["qpos_order"] = json.dumps(
                [f"arm_q_measured.{name}" for name in ACTION_ORDER[:6]]
                + [f"hand.{name}" for name in ACTION_ORDER[6:]]
            )
            root.attrs["qvel_order"] = json.dumps(
                [f"arm_dq_measured.{name}" for name in ACTION_ORDER[:6]]
            )
            root.attrs["units"] = json.dumps(metadata["units"], sort_keys=True)
            observations = root.create_group("observations")
            qpos = observations.create_dataset(
                "qpos", shape=(count, 12), dtype=np.float32
            )
            qvel = observations.create_dataset(
                "qvel", shape=(count, 6), dtype=np.float32
            )
            images = observations.create_group("images")
            workspace_rgb = images.create_dataset(
                "workspace",
                shape=(count, *first_workspace_rgb.shape),
                dtype=np.uint8,
                chunks=(1, *first_workspace_rgb.shape),
                compression="gzip",
            )
            wrist_rgb = images.create_dataset(
                "wrist",
                shape=(count, *first_wrist_rgb.shape),
                dtype=np.uint8,
                chunks=(1, *first_wrist_rgb.shape),
                compression="gzip",
            )
            depth = observations.create_group("depth")
            depth.attrs["extension"] = "embodied_lab.lossless_depth_uint16.v1"
            workspace_depth = depth.create_dataset(
                "workspace",
                shape=(count, *first_workspace_depth.shape),
                dtype=np.uint16,
                chunks=(1, *first_workspace_depth.shape),
                compression="gzip",
            )
            wrist_depth = depth.create_dataset(
                "wrist",
                shape=(count, *first_wrist_depth.shape),
                dtype=np.uint16,
                chunks=(1, *first_wrist_depth.shape),
                compression="gzip",
            )
            actions = root.create_dataset(
                "action", shape=(count, len(ACTION_ORDER)), dtype=np.float32
            )
            timestamps = root.create_dataset(
                "timestamps", shape=(count,), dtype=np.float64
            )
            timing_valid = root.create_dataset(
                "timing_valid", shape=(count,), dtype=np.bool_
            )
            action_status = root.create_dataset(
                "arm_action_status", shape=(count,), dtype=np.int8
            )
            for index, row in enumerate(rows):
                state = row["observation"]["state"]
                qpos[index] = np.asarray(
                    state["arm_q_measured"] + state["hand"], dtype=np.float32
                )
                qvel[index] = np.asarray(
                    state["arm_dq_measured"], dtype=np.float32
                )
                actions[index] = action_vector(row)
                timestamps[index] = float(row["timestamp"])
                timing_valid[index] = bool(
                    row.get("timing", {}).get("timing_valid", True)
                )
                action_status[index] = ACTION_STATUS_CODE[
                    row["action"]["arm_status"]
                ]
                workspace_rgb[index] = (
                    first_workspace_rgb
                    if index == 0
                    else _rgb(
                        episode_dir,
                        row["observation"]["images"]["workspace"]["rgb"],
                    )
                )
                wrist_rgb[index] = (
                    first_wrist_rgb
                    if index == 0
                    else _rgb(
                        episode_dir,
                        row["observation"]["images"]["wrist"]["rgb"],
                    )
                )
                workspace_depth[index] = (
                    first_workspace_depth
                    if index == 0
                    else _depth(
                        episode_dir,
                        row["observation"]["images"]["workspace"]["depth_raw"],
                    )
                )
                wrist_depth[index] = (
                    first_wrist_depth
                    if index == 0
                    else _depth(
                        episode_dir,
                        row["observation"]["images"]["wrist"]["depth_raw"],
                    )
                )
            root.flush()
        os.replace(temporary, output_path)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise
    return output_path


def export_lerobot_v3(episode: str | Path, output_root: str | Path, *, repo_id: str) -> Path:
    """Create a one-episode official LeRobot v3 dataset.

    RGB and low-dimensional features use the installed official SDK. Lossless
    raw depth stays in the episode sidecar and is indexed by frame_index; it is
    deliberately not passed through LeRobot's quantized depth-video encoder.
    """

    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
    except ImportError as exc:
        raise RuntimeError(
            "LeRobot v3 export requires an installed current official lerobot package; "
            "the legacy lerobot.common namespace is not accepted"
        ) from exc

    episode_dir = Path(episode).resolve()
    output = Path(output_root).resolve()
    metadata, rows = _training_episode(episode_dir)
    if output.exists():
        raise FileExistsError(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    staging = output.with_name(f".{output.name}.{uuid.uuid4().hex}.partial")
    workspace_first = _rgb(
        episode_dir, rows[0]["observation"]["images"]["workspace"]["rgb"]
    )
    wrist_first = _rgb(
        episode_dir, rows[0]["observation"]["images"]["wrist"]["rgb"]
    )
    _validate_lerobot_video_shape("workspace", workspace_first)
    _validate_lerobot_video_shape("wrist", wrist_first)
    workspace_height, workspace_width = workspace_first.shape[:2]
    wrist_height, wrist_width = wrist_first.shape[:2]
    features = {
        "observation.images.workspace.rgb": {
            "dtype": "video",
            "shape": (workspace_height, workspace_width, 3),
            "names": ["height", "width", "channel"],
        },
        "observation.images.wrist.rgb": {
            "dtype": "video",
            "shape": (wrist_height, wrist_width, 3),
            "names": ["height", "width", "channel"],
        },
        "observation.state": {
            "dtype": "float32",
            "shape": (len(OBSERVATION_STATE_ORDER),),
            "names": list(OBSERVATION_STATE_ORDER),
        },
        "action": {
            "dtype": "float32",
            "shape": (len(ACTION_ORDER),),
            "names": list(ACTION_ORDER),
        },
        "observation.arm_trigger": {"dtype": "bool", "shape": (1,), "names": ["pressed"]},
        "observation.hand_grip": {"dtype": "bool", "shape": (1,), "names": ["pressed"]},
        "observation.provenance": {
            "dtype": "int64",
            "shape": (4,),
            "names": ["arm_q", "arm_dq", "tcp_pose", "hand"],
        },
        "action.status": {"dtype": "int64", "shape": (1,), "names": ["status"]},
        "timing.source_timestamp_ns": {
            "dtype": "int64",
            "shape": (len(TIMING_NAMES),),
            "names": list(TIMING_NAMES),
        },
        "timing.signed_offset_ns": {
            "dtype": "int64",
            "shape": (len(TIMING_NAMES),),
            "names": list(TIMING_NAMES),
        },
        "timing.synchronization_valid": {"dtype": "bool", "shape": (1,), "names": ["valid"]},
    }
    dataset = None
    try:
        dataset = LeRobotDataset.create(
            repo_id=repo_id,
            fps=int(metadata["dataset_fps"]),
            root=staging,
            robot_type="jaka_mini2_rh56dfx",
            features=features,
            use_videos=True,
            batch_encoding_size=1,
            image_writer_threads=4,
        )
        for row in rows:
            state = row["observation"]["state"]
            sources = row["timing"]["source_timestamps_ns"]
            offsets = row["timing"]["signed_offsets_ns"]
            dataset.add_frame(
                {
                    "observation.images.workspace.rgb": _rgb(
                        episode_dir, row["observation"]["images"]["workspace"]["rgb"]
                    ),
                    "observation.images.wrist.rgb": _rgb(
                        episode_dir, row["observation"]["images"]["wrist"]["rgb"]
                    ),
                    "observation.state": observation_vector(row),
                    "action": action_vector(row),
                    "observation.arm_trigger": np.asarray([state["arm_trigger"]], dtype=np.bool_),
                    "observation.hand_grip": np.asarray([state["hand_grip"]], dtype=np.bool_),
                    "observation.provenance": np.asarray(
                        [
                            PROVENANCE_CODE[state["arm_q_source"]],
                            PROVENANCE_CODE[state["arm_dq_source"]],
                            PROVENANCE_CODE[state["tcp_pose_source"]],
                            PROVENANCE_CODE[state["hand_source"]],
                        ],
                        dtype=np.int64,
                    ),
                    "action.status": np.asarray(
                        [ACTION_STATUS_CODE[row["action"]["arm_status"]]], dtype=np.int64
                    ),
                    "timing.source_timestamp_ns": np.asarray(
                        [
                            INVALID_INT64 if sources.get(name) is None else sources[name]
                            for name in TIMING_NAMES
                        ],
                        dtype=np.int64,
                    ),
                    "timing.signed_offset_ns": np.asarray(
                        [
                            INVALID_INT64 if offsets.get(name) is None else offsets[name]
                            for name in TIMING_NAMES
                        ],
                        dtype=np.int64,
                    ),
                    "timing.synchronization_valid": np.asarray(
                        [bool(row["timing"].get("timing_valid", True))],
                        dtype=np.bool_,
                    ),
                    "task": metadata["task_name"],
                }
            )
        dataset.save_episode()
        dataset.finalize()
    except BaseException:
        if dataset is not None:
            try:
                if dataset.has_pending_frames():
                    dataset.clear_episode_buffer(delete_images=True)
                dataset.finalize()
            except Exception:
                # Cleanup is best-effort; retain the original export failure.
                pass
        if staging.is_dir():
            shutil.rmtree(staging)
        raise
    sidecar = {
        "schema_version": "embodied_lab.lerobot_v3_depth_sidecar.v1",
        "official_lerobot_feature": False,
        "reason": "lossless raw uint16 is retained; LeRobot depth video quantizes to 12 bits",
        "episode_uuid": metadata["episode_uuid"],
        "dataset_fps": int(metadata["dataset_fps"]),
        "index": "frame_index",
        "source": "depth_sidecar/{workspace,wrist}/depth_raw/{frame_index:06d}.npy",
        "source_episode_sha256": file_sha256(episode_dir / "canonical" / "samples.jsonl"),
        "provenance_codes": PROVENANCE_CODE,
        "action_status_codes": ACTION_STATUS_CODE,
        "timing_names": list(TIMING_NAMES),
        "invalid_int64_sentinel": int(INVALID_INT64),
    }
    try:
        sidecar_root = staging / "depth_sidecar"
        for role in ("workspace", "wrist"):
            role_root = sidecar_root / role
            role_root.mkdir(parents=True, exist_ok=False)
            shutil.copytree(
                episode_dir / "canonical" / "frames" / role / "depth_raw",
                role_root / "depth_raw",
            )
            aligned = (
                episode_dir
                / "canonical"
                / "frames"
                / role
                / "depth_aligned_to_rgb"
            )
            if any(aligned.iterdir()):
                shutil.copytree(aligned, role_root / "depth_aligned_to_rgb")
        shutil.copy2(
            episode_dir / "canonical" / "samples.jsonl",
            sidecar_root / "index.jsonl",
        )
        (staging / "meta" / "embodied_lab_depth_sidecar.json").write_text(
            json.dumps(sidecar, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        staging.replace(output)
    except BaseException:
        if staging.is_dir():
            shutil.rmtree(staging)
        raise
    return output
