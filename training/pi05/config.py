"""Load the repository-owned π0.5 task and training configuration."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
CONFIG_PATH = REPOSITORY_ROOT / "configs/training/pi05/physical_bottle.yaml"


def _load_yaml(path: Path) -> dict[str, Any]:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError(f"configuration must be a mapping: {path}")
    return document


def _load_documents() -> tuple[dict[str, Any], dict[str, Any]]:
    pi05 = _load_yaml(CONFIG_PATH)
    if pi05.get("schema_version") != "embodied_lab.pi05_training_config.v1":
        raise ValueError(f"unsupported π0.5 config schema: {CONFIG_PATH}")
    task_path = (CONFIG_PATH.parent / str(pi05["task_config"])).resolve()
    task = _load_yaml(task_path)
    if task.get("schema_version") != "embodied_lab.physical_bottle_collection_config.v1":
        raise ValueError(f"unsupported shared task config schema: {task_path}")
    return pi05, task


PI05_CONFIG, TASK_CONFIG = _load_documents()
TASK_PROMPT = str(TASK_CONFIG["task"]["prompt"])
MODEL_CONFIG = dict(PI05_CONFIG["model"])
DATA_CONFIG = dict(PI05_CONFIG["data"])
TRAINING_CONFIG = dict(PI05_CONFIG["training"])
VALIDATION_CONFIG = dict(PI05_CONFIG["validation"])

BASE_CHECKPOINT = str(MODEL_CONFIG["base_checkpoint"])
STATE_DIM = int(MODEL_CONFIG["state_dim"])
ACTION_DIM = int(MODEL_CONFIG["command_action_dim"])
MODEL_ACTION_DIM = int(MODEL_CONFIG["internal_action_dim"])
ACTION_HORIZON = int(MODEL_CONFIG["action_horizon"])
MODEL_IMAGE_KEYS = tuple(str(value) for value in MODEL_CONFIG["image_keys"])
DATASET_REPO_ID = str(DATA_CONFIG["train_repo_id"])
VALIDATION_REPO_ID = str(DATA_CONFIG["validation_repo_id"])

if len(MODEL_IMAGE_KEYS) != 3:
    raise ValueError("π0.5 expects three model image slots")
if ACTION_DIM != 12 or STATE_DIM != 12 or MODEL_ACTION_DIM != 32:
    raise ValueError("the current π0.5 RH56 contract must be 12/12 with internal width 32")
if ACTION_HORIZON < 1:
    raise ValueError("π0.5 action_horizon must be positive")
