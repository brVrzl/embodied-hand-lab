"""Thin openpi/π robot-data adapter.

The local environment currently has no openpi installation.  This module
therefore keeps the repository-specific mapping explicit and dependency-free;
an installed openpi runner can consume the returned dictionary through its
normal LeRobot input transform without changing openpi core.
"""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np

from .training_views import ActDatasetAdapter, absolute_native_action


OPENPI_INPUT_MAPPING = {
    "images": {
        "workspace": "observation.images.workspace",
        "wrist": "observation.images.wrist",
    },
    "state": "observation.state",
    "task": "task",
    "action": "action",
}


def to_openpi_example(sample: Mapping[str, Any]) -> dict[str, Any]:
    """Map a repository ACT example to the initial openpi robot contract."""

    observation = sample["observation"]
    return {
        "observation": {
            "images": {
                "workspace": observation["images"]["workspace"],
                "wrist": observation["images"]["wrist"],
            },
            "state": np.asarray(observation["state"], dtype=np.float32),
        },
        "task": str(sample["task"]),
        "action": absolute_native_action(sample["action"][0]),
        "action_chunk": np.asarray(sample["action"], dtype=np.float32),
        "action_mask": np.asarray(sample["action_mask"], dtype=bool),
        "action_semantics": "absolute_native_target",
    }

def smoke_openpi_dataset(dataset_root: str, *, split: str = "train", action_horizon: int = 16) -> dict[str, Any]:
    dataset = ActDatasetAdapter(dataset_root, split=split, action_horizon=action_horizon, force=False)
    if not len(dataset):
        raise ValueError(f"split {split!r} has no rows")
    sample = to_openpi_example(dataset[0])
    try:
        import openpi  # type: ignore[import-not-found]
        status = "package_available_mapping_checked"
        version = getattr(openpi, "__version__", None)
    except ImportError:
        status = "skeleton_only_package_not_installed"
        version = None
    return {
        "status": status,
        "openpi_version": version,
        "mapping": OPENPI_INPUT_MAPPING,
        "image_shapes": {name: list(value.shape) for name, value in sample["observation"]["images"].items()},
        "state_shape": list(sample["observation"]["state"].shape),
        "action_shape": list(sample["action_chunk"].shape),
        "task": sample["task"],
        "force_in_baseline": False,
        "action_semantics": "absolute_native_target",
        "normalization_stats": "stats/openpi_stats.json",
    }
