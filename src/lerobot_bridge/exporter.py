from __future__ import annotations

import json
from pathlib import Path


def export_to_lerobot_stub(episodes_root: str | Path, export_root: str | Path) -> Path:
    episodes_root = Path(episodes_root).resolve()
    export_root = Path(export_root).resolve()
    meta_dir = export_root / "meta"
    train_dir = export_root / "train"
    meta_dir.mkdir(parents=True, exist_ok=True)
    train_dir.mkdir(parents=True, exist_ok=True)

    info = {
        "format": "lerobot_jsonl_compat_v0.1",
        "note": "JSONL compatibility preview; final LeRobot v3 export should write Parquet, MP4 shards, and meta/*.jsonl metadata.",
        "source_root": str(episodes_root),
        "indexing": {
            "index": "global int64 row index",
            "episode_index": "contiguous int64 episode id within this export",
            "frame_index": "zero-based int64 frame id within episode",
            "task_index": "contiguous int64 task id within this export",
        },
        "state_mapping": {
            "arm_joint_states": "observation.state.arm_joints",
            "hand_states": "observation.state.hand",
            "dog_states": "observation.state.mobile_base",
        },
        "action_mapping": {
            "action": "action",
        },
    }
    (meta_dir / "info.json").write_text(json.dumps(info, indent=2), encoding="utf-8")

    samples_path = train_dir / "samples.jsonl"
    if samples_path.exists():
        samples_path.unlink()
    task_to_index: dict[str, int] = {}
    global_index = 0
    for episode_index, episode_dir in enumerate(sorted(episodes_root.glob("episode_*"))):
        metadata_path = episode_dir / "metadata.json"
        steps_path = episode_dir / "steps.jsonl"
        if not metadata_path.exists() or not steps_path.exists():
            continue
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        task_name = metadata["task_name"]
        if task_name not in task_to_index:
            task_to_index[task_name] = len(task_to_index)
        task_index = task_to_index[task_name]
        step_lines = [line for line in steps_path.read_text(encoding="utf-8").splitlines() if line.strip()]
        for index, line in enumerate(step_lines):
            if not line.strip():
                continue
            step = json.loads(line)
            is_last = index == len(step_lines) - 1
            reward = 1.0 if is_last and metadata.get("success") else 0.0
            sample = {
                "index": global_index,
                "episode_index": episode_index,
                "episode_id": metadata["episode_id"],
                "frame_index": index,
                "task_index": task_index,
                "task": metadata["task_name"],
                "instruction": metadata["natural_language_instruction"],
                "timestamp": step["timestamp"],
                "observation": {
                    "images": {
                        "rgb": step["rgb_path"],
                    },
                    "depth": {
                        "depth": step["depth_path"],
                    },
                    "state": {
                        "arm_joints": step["arm_joint_states"],
                        "arm_ee_pose": step["arm_ee_pose"],
                        "hand": step["hand_states"],
                        "mobile_base": step["dog_states"],
                    },
                },
                "action": step["action"],
                "reward": reward,
                "discount": 0.0 if is_last else 1.0,
                "is_first": index == 0,
                "is_last": is_last,
                "is_terminal": is_last,
                "done": is_last,
            }
            with samples_path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(sample) + "\n")
            global_index += 1
    return export_root
