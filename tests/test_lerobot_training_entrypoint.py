from __future__ import annotations

import json
from pathlib import Path

import yaml


ROOT = Path(__file__).parents[1]
STATE_NAMES = (
    "jaka_joint_1",
    "jaka_joint_2",
    "jaka_joint_3",
    "jaka_joint_4",
    "jaka_joint_5",
    "jaka_joint_6",
    "rh56_index",
    "rh56_middle",
    "rh56_ring",
    "rh56_pinky",
    "rh56_thumb_close",
    "rh56_thumb_lateral",
)


def _config() -> dict:
    return json.loads(
        (ROOT / "configs/training/lerobot/act_physical_bottle.json").read_text(
            encoding="utf-8"
        )
    )


def test_current_act_config_is_the_nominal52_formal_baseline() -> None:
    config = _config()
    assert config["policy"]["chunk_size"] == 60
    assert config["policy"]["n_action_steps"] == 1
    assert config["policy"]["pretrained_backbone_weights"] == (
        "ResNet18_Weights.IMAGENET1K_V1"
    )
    assert config["dataset"]["root"].endswith(
        "/physical_bottle_v4_nominal52/lerobot/act_strong_view"
    )
    assert config["batch_size"] == 16
    assert config["steps"] == 100_000
    assert config["eval_steps"] == 10_000


def test_canonical_task_config_owns_split_and_both_views() -> None:
    task = yaml.safe_load(
        (ROOT / "configs/training/physical_bottle.yaml").read_text(
            encoding="utf-8"
        )
    )
    assert task["task"]["prompt"] == "Pick up the bottle and place it on the cardboard box."
    assert task["training"]["validation_source_episodes"] == [
        87, 88, 108, 109, 138, 139, 140, 141, 142, 172, 173, 174
    ]
    assert task["training"]["expected"] == {
        "train_trajectories": 37,
        "validation_trajectories": 15,
        "train_rows": 23802,
        "validation_rows": 9309,
    }
    assert task["views"]["act"]["dataset_root"].endswith("/physical_bottle_v4_nominal52/act")
    assert task["views"]["act_force"]["dataset_root"].endswith(
        "/physical_bottle_v4_nominal52/act_force"
    )


def test_entrypoint_is_pinned_offline_and_current_only() -> None:
    script = (ROOT / "scripts/train_physical_bottle_lerobot.sh").read_text(
        encoding="utf-8"
    )
    assert "jaka-lerobot-dev:snapshot-before-raw-mount" in script
    assert "EXPECTED_IMAGE_ID" in script
    assert "--network none" in script
    assert "python -m lerobot.scripts.lerobot_train" in script
    assert "--dataset-config" in script
    assert "act_physical_bottle.json" in script
    assert "physical_bottle_mixed" not in script
    assert "source/master" not in script
    assert tuple(STATE_NAMES) == tuple(
        task_state_name
        for task_state_name in (
            "jaka_joint_1",
            "jaka_joint_2",
            "jaka_joint_3",
            "jaka_joint_4",
            "jaka_joint_5",
            "jaka_joint_6",
            "rh56_index",
            "rh56_middle",
            "rh56_ring",
            "rh56_pinky",
            "rh56_thumb_close",
            "rh56_thumb_lateral",
        )
    )
