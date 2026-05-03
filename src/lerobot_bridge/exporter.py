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
        "format": "lerobot_stub_v1",
        "source_root": str(episodes_root),
        "state_mapping": {
            "arm_joint_states": "observation.state.arm_joints",
            "hand_states": "observation.state.hand",
            "dog_states": "observation.state.base",
        },
        "action_mapping": {
            "action": "action",
        },
    }
    (meta_dir / "info.json").write_text(json.dumps(info, indent=2), encoding="utf-8")

    samples_path = train_dir / "samples.jsonl"
    if samples_path.exists():
        samples_path.unlink()
    for episode_dir in sorted(episodes_root.glob("episode_*")):
        metadata_path = episode_dir / "metadata.json"
        steps_path = episode_dir / "steps.jsonl"
        if not metadata_path.exists() or not steps_path.exists():
            continue
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        step_lines = [line for line in steps_path.read_text(encoding="utf-8").splitlines() if line.strip()]
        for index, line in enumerate(step_lines):
            if not line.strip():
                continue
            step = json.loads(line)
            sample = {
                "episode_index": metadata["episode_id"],
                "frame_index": index,
                "task": metadata["task_name"],
                "instruction": metadata["natural_language_instruction"],
                "observation": {
                    "image": step["rgb_path"],
                    "depth": step["depth_path"],
                    "state": {
                        "arm_joints": step["arm_joint_states"],
                        "arm_ee_pose": step["arm_ee_pose"],
                        "hand": step["hand_states"],
                        "base": step["dog_states"],
                    },
                },
                "action": step["action"],
                "reward": 1.0 if metadata.get("success") else 0.0,
                "done": index == len(step_lines) - 1,
            }
            with samples_path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(sample) + "\n")
    return export_root
