from __future__ import annotations

import json
from pathlib import Path
import shutil
from types import ModuleType
from typing import Any

import numpy as np

from .episode import ACTION_ORDER, OBSERVATION_STATE_ORDER, file_sha256


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


def _require_cv2() -> ModuleType:
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError(
            "exporting non-NPY episode images requires OpenCV; "
            "install embodied-lab[dataset-export]"
        ) from exc
    return cv2


def _rows(episode_dir: Path) -> list[dict[str, Any]]:
    path = episode_dir / "canonical" / "samples.jsonl"
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]


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


def export_act_hdf5(episode: str | Path, output: str | Path) -> Path:
    """Export one canonical episode using standard ACT RGB/state/action paths.

    Lossless depth is an explicit extension under ``/observations/depth``. The
    canonical episode remains authoritative and is not changed by this export.
    """

    try:
        import h5py
    except ImportError as exc:
        raise RuntimeError("ACT HDF5 export requires the optional h5py package") from exc

    episode_dir = Path(episode).resolve()
    output_path = Path(output).resolve()
    rows = _rows(episode_dir)
    if not rows:
        raise ValueError("cannot export an empty episode")
    metadata = json.loads((episode_dir / "metadata.json").read_text(encoding="utf-8"))
    workspace_rgb = np.stack(
        [_rgb(episode_dir, row["observation"]["images"]["workspace"]["rgb"]) for row in rows]
    )
    wrist_rgb = np.stack(
        [_rgb(episode_dir, row["observation"]["images"]["wrist"]["rgb"]) for row in rows]
    )
    workspace_depth = np.stack(
        [_depth(episode_dir, row["observation"]["images"]["workspace"]["depth_raw"]) for row in rows]
    )
    wrist_depth = np.stack(
        [_depth(episode_dir, row["observation"]["images"]["wrist"]["depth_raw"]) for row in rows]
    )
    qpos = np.asarray(
        [row["observation"]["state"]["arm_q_measured"] + row["observation"]["state"]["hand"] for row in rows],
        dtype=np.float32,
    )
    qvel = np.asarray(
        [row["observation"]["state"]["arm_dq_measured"] for row in rows], dtype=np.float32
    )
    actions = np.stack([action_vector(row) for row in rows])
    timestamps = np.asarray([row["timestamp"] for row in rows], dtype=np.float64)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with h5py.File(output_path, "x") as root:
        root.attrs["sim"] = False
        root.attrs["episode_uuid"] = metadata["episode_uuid"]
        root.attrs["completion_status"] = metadata["completion_status"]
        root.attrs["success_label"] = metadata["success_label"]
        root.attrs["valid"] = metadata["completion_status"] == "completed"
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
        observations.create_dataset("qpos", data=qpos)
        observations.create_dataset("qvel", data=qvel)
        images = observations.create_group("images")
        images.create_dataset("workspace", data=workspace_rgb, compression="gzip")
        images.create_dataset("wrist", data=wrist_rgb, compression="gzip")
        depth = observations.create_group("depth")
        depth.attrs["extension"] = "embodied_lab.lossless_depth_uint16.v1"
        depth.create_dataset("workspace", data=workspace_depth, compression="gzip")
        depth.create_dataset("wrist", data=wrist_depth, compression="gzip")
        root.create_dataset("action", data=actions)
        root.create_dataset("timestamps", data=timestamps)
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
    rows = _rows(episode_dir)
    if not rows:
        raise ValueError("cannot export an empty episode")
    workspace_first = _rgb(
        episode_dir, rows[0]["observation"]["images"]["workspace"]["rgb"]
    )
    wrist_first = _rgb(
        episode_dir, rows[0]["observation"]["images"]["wrist"]["rgb"]
    )
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
    dataset = LeRobotDataset.create(
        repo_id=repo_id,
        fps=30,
        root=output,
        robot_type="jaka_mini2_rh56dfx",
        features=features,
        use_videos=True,
        batch_encoding_size=1,
        image_writer_threads=4,
    )
    try:
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
                    "timing.synchronization_valid": np.asarray([True], dtype=np.bool_),
                    "task": json.loads((episode_dir / "metadata.json").read_text(encoding="utf-8"))[
                        "task_name"
                    ],
                }
            )
        dataset.save_episode()
        dataset.finalize()
    except Exception:
        if dataset.has_pending_frames():
            dataset.clear_episode_buffer(delete_images=True)
        dataset.finalize()
        raise
    sidecar = {
        "schema_version": "embodied_lab.lerobot_v3_depth_sidecar.v1",
        "official_lerobot_feature": False,
        "reason": "lossless raw uint16 is retained; LeRobot depth video quantizes to 12 bits",
        "episode_uuid": json.loads((episode_dir / "metadata.json").read_text(encoding="utf-8"))[
            "episode_uuid"
        ],
        "index": "frame_index",
        "source": "depth_sidecar/{workspace,wrist}/depth_raw/{frame_index:06d}.npy",
        "source_episode_sha256": file_sha256(episode_dir / "canonical" / "samples.jsonl"),
        "provenance_codes": PROVENANCE_CODE,
        "action_status_codes": ACTION_STATUS_CODE,
        "timing_names": list(TIMING_NAMES),
        "invalid_int64_sentinel": int(INVALID_INT64),
    }
    sidecar_root = output / "depth_sidecar"
    for role in ("workspace", "wrist"):
        role_root = sidecar_root / role
        role_root.mkdir(parents=True, exist_ok=False)
        shutil.copytree(
            episode_dir / "canonical" / "frames" / role / "depth_raw",
            role_root / "depth_raw",
        )
        aligned = episode_dir / "canonical" / "frames" / role / "depth_aligned_to_rgb"
        if any(aligned.iterdir()):
            shutil.copytree(aligned, role_root / "depth_aligned_to_rgb")
    shutil.copy2(episode_dir / "canonical" / "samples.jsonl", sidecar_root / "index.jsonl")
    (output / "meta" / "embodied_lab_depth_sidecar.json").write_text(
        json.dumps(sidecar, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return output
