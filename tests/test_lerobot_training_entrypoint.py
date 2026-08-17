from __future__ import annotations

import json
from pathlib import Path


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
ACTION_NAMES = (
    "jaka_target_1",
    "jaka_target_2",
    "jaka_target_3",
    "jaka_target_4",
    "jaka_target_5",
    "jaka_target_6",
    "rh56_target_index",
    "rh56_target_middle",
    "rh56_target_ring",
    "rh56_target_pinky",
    "rh56_target_thumb_close",
    "rh56_target_thumb_lateral",
)


def _config(name: str) -> dict:
    return json.loads(
        (ROOT / "configs/training/lerobot" / name).read_text(encoding="utf-8")
    )


def test_repo_configs_keep_matched_absolute_action_contract() -> None:
    act = _config("act_physical_bottle_mixed.json")
    force = _config("act_force_physical_bottle_mixed.json")

    assert act["policy"]["chunk_size"] == force["policy"]["chunk_size"] == 16
    assert act["policy"]["n_action_steps"] == force["policy"]["n_action_steps"] == 16
    assert act["policy"]["temporal_ensemble_coeff"] is None
    assert force["policy"]["temporal_ensemble_coeff"] is None
    assert act["policy"]["pretrained_backbone_weights"] is None
    assert force["policy"]["pretrained_backbone_weights"] is None
    assert act["dataset"]["root"].endswith("/lerobot/act_view")
    assert force["dataset"]["root"].endswith("/lerobot/act_force_view")
    assert act["output_dir"].endswith("/physical_bottle_v2/act_run")
    assert force["output_dir"].endswith("/physical_bottle_v2/act_force_run")
    assert act["policy"]["normalization_mapping"] == force["policy"]["normalization_mapping"]


def test_tool_declares_exact_state_action_order_and_force_mapping() -> None:
    tool = (ROOT / "tools/lerobot_physical_bottle.py").read_text(encoding="utf-8")
    assert '"jaka_joint_6"' in tool
    assert '"jaka_target_6"' in tool
    assert 'FORCE_KEY = "observation.environment_state"' in tool
    assert '"force_units": "rh56_force_act_raw_count"' in tool
    assert '"action_semantics": "absolute_native_target"' in tool
    assert tool.index('"jaka_joint_6"') < tool.index('"rh56_index"')
    assert tool.index('"jaka_target_6"') < tool.index('"rh56_target_index"')


def test_entrypoint_is_pinned_and_offline_only() -> None:
    script = (ROOT / "scripts/train_physical_bottle_lerobot.sh").read_text(
        encoding="utf-8"
    )
    assert "jaka-lerobot-dev:snapshot-before-raw-mount" in script
    assert "EXPECTED_IMAGE_ID" in script
    assert "--network none" in script
    assert "python -m lerobot.scripts.lerobot_train" in script
    assert "physical_bottle_mixed" in script
    assert "source/master" not in script
    assert tuple(STATE_NAMES) == tuple(_config("act_physical_bottle_mixed.json").get("state_order", STATE_NAMES))
    assert len(ACTION_NAMES) == 12


def test_val4_configs_use_matched_episode_level_eval_split() -> None:
    act = _config("act_physical_bottle_mixed_val4.json")
    force = _config("act_force_physical_bottle_mixed_val4.json")
    assert act["dataset"]["eval_split"] == force["dataset"]["eval_split"] == 0.16
    assert act["eval_steps"] == force["eval_steps"] == 200
    assert act["dataset"]["root"].endswith("/lerobot/act_val4_view")
    assert force["dataset"]["root"].endswith("/lerobot/act_force_val4_view")
    assert act["output_dir"].endswith("/physical_bottle_v2/act_val4_run")
    assert force["output_dir"].endswith("/physical_bottle_v2/act_force_val4_run")


def test_val4_manifest_holds_out_the_requested_good_demonstrations() -> None:
    import yaml

    config = yaml.safe_load(
        (ROOT / "configs/training/physical_bottle_mixed_val4.yaml").read_text(
            encoding="utf-8"
        )
    )
    assert config["validation_source_episodes"] == [89, 98, 114, 116]
    assert config["eval_split"] == 0.16


def test_nominal52_strong_config_is_current_formal_baseline() -> None:
    strong = _config("act_physical_bottle_nominal52_strong.json")
    script = (ROOT / "scripts/train_physical_bottle_lerobot.sh").read_text(encoding="utf-8")

    assert strong["policy"]["chunk_size"] == 60
    assert strong["policy"]["pretrained_backbone_weights"] == (
        "ResNet18_Weights.IMAGENET1K_V1"
    )
    assert strong["batch_size"] == 16
    assert strong["steps"] == 100_000
    assert strong["dataset"]["root"].endswith("/physical_bottle_v4_nominal52/lerobot/act_strong_view")
    assert "strong-act" in script
    assert "act_physical_bottle_nominal52_strong.json" in script
    assert "clean-scratch" not in script
