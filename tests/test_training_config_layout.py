from __future__ import annotations

import json
from pathlib import Path

import yaml


ROOT = Path(__file__).parents[1]


def test_training_configs_are_separated_by_owner() -> None:
    shared_path = ROOT / "configs/training/shared/physical_bottle.yaml"
    act_view_path = ROOT / "configs/training/act/physical_bottle.yaml"
    act_trainer_path = ROOT / "configs/training/act/lerobot.json"
    pi05_path = ROOT / "configs/training/pi05/physical_bottle.yaml"
    lock_path = ROOT / "configs/training/pi05/openpi.lock.json"

    shared = yaml.safe_load(shared_path.read_text(encoding="utf-8"))
    act_view = yaml.safe_load(act_view_path.read_text(encoding="utf-8"))
    act_trainer = json.loads(act_trainer_path.read_text(encoding="utf-8"))
    pi05 = yaml.safe_load(pi05_path.read_text(encoding="utf-8"))
    lock = json.loads(lock_path.read_text(encoding="utf-8"))

    assert shared["task"]["prompt"] == "Pick up the bottle and place it on the cardboard box."
    assert act_view["dataset_config"] == "../shared/physical_bottle.yaml"
    assert act_trainer["policy"]["chunk_size"] == 60
    assert pi05["task_config"] == "../shared/physical_bottle.yaml"
    assert pi05["model"]["base_checkpoint"] == "pi05_base"
    assert pi05["model"]["action_horizon"] == 16
    assert lock["config"] == "configs/training/pi05/physical_bottle.yaml"
    assert lock["runner"] == "training/pi05/scripts/openpi_runner.py"
