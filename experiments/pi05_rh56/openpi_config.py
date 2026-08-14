"""Pinned OpenPI configuration and transforms for the JAKA Mini2/RH56 baseline.

The imports in this file intentionally target the checked-out upstream OpenPI
tree.  It is loaded inside the pinned Thor container, not by the repository's
ACT environment.
"""

from __future__ import annotations

import dataclasses
from pathlib import Path
from typing import Any

import numpy as np

from openpi.models import pi0_config
from openpi.training import config as openpi_config
from openpi.training import optimizer
from openpi.training import weight_loaders
from openpi.training.config import ModelTransformFactory
from openpi.transforms import DataTransformFn, Group, RepackTransform


TASK_PROMPT = "Pick up the bottle and place it on the cardboard box."
STATE_DIM = 12
ACTION_DIM = 12
MODEL_ACTION_DIM = 32
ACTION_HORIZON = 16
DATASET_REPO_ID = "local/pi05_rh56_train"
MODEL_IMAGE_KEYS = ("base_0_rgb", "left_wrist_0_rgb", "right_wrist_0_rgb")


def _as_numpy(value: Any) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach().cpu().numpy()
    return np.asarray(value)


def _image_to_hwc_rgb(value: Any, name: str) -> np.ndarray:
    image = _as_numpy(value)
    if image.ndim != 3:
        raise ValueError(f"{name} image must be rank 3, got {image.shape}")
    if image.shape[0] == 3 and image.shape[-1] != 3:
        image = np.transpose(image, (1, 2, 0))
    if image.shape[-1] != 3:
        raise ValueError(f"{name} image must have three RGB channels, got {image.shape}")
    if np.issubdtype(image.dtype, np.floating):
        if not np.isfinite(image).all():
            raise ValueError(f"{name} image contains NaN/Inf")
        if float(np.min(image)) >= 0.0 and float(np.max(image)) <= 1.0:
            image = np.rint(image * 255.0)
        elif float(np.min(image)) >= 0.0 and float(np.max(image)) <= 255.0:
            image = np.rint(image)
        else:
            raise ValueError(f"{name} floating image has an unexpected range")
        image = image.astype(np.uint8)
    if image.dtype != np.uint8:
        if not np.issubdtype(image.dtype, np.integer):
            raise ValueError(f"{name} image dtype must be uint8 or numeric, got {image.dtype}")
        image = np.clip(image, 0, 255).astype(np.uint8)
    if image.shape != (480, 640, 3):
        raise ValueError(f"{name} image must be 640x480 RGB before model resize, got {image.shape}")
    return np.ascontiguousarray(image)


@dataclasses.dataclass(frozen=True)
class Rh56Inputs(DataTransformFn):
    """Map the audited native sample into the current OpenPI model contract."""

    def __call__(self, data: dict[str, Any]) -> dict[str, Any]:
        if set(data) < {"images", "state", "actions", "prompt"}:
            raise ValueError(f"Missing required input keys: {set(data)}")
        prompt = data["prompt"]
        if not isinstance(prompt, str):
            prompt = prompt.item()
        if prompt != TASK_PROMPT:
            raise ValueError(f"Unexpected task prompt: {prompt!r}")

        source_images = data["images"]
        if set(source_images) != {"workspace", "wrist"}:
            raise ValueError(f"Expected workspace and wrist images, got {set(source_images)}")
        workspace = _image_to_hwc_rgb(source_images["workspace"], "workspace")
        wrist = _image_to_hwc_rgb(source_images["wrist"], "wrist")
        state = _as_numpy(data["state"]).astype(np.float32, copy=False)
        actions = _as_numpy(data["actions"]).astype(np.float32, copy=False)
        if state.shape != (STATE_DIM,):
            raise ValueError(f"State must have shape ({STATE_DIM},), got {state.shape}")
        if actions.ndim != 2 or actions.shape[-1] != ACTION_DIM:
            raise ValueError(f"Actions must have shape (horizon, {ACTION_DIM}), got {actions.shape}")
        if not np.isfinite(state).all() or not np.isfinite(actions).all():
            raise ValueError("State/actions contain NaN/Inf")

        # OpenPI's current pi0.5 model has three named image slots. The
        # project has two proven camera streams; the third slot is explicitly
        # masked, rather than pretending that a third camera exists.
        right_wrist = np.zeros_like(workspace)
        return {
            "image": {
                "base_0_rgb": workspace,
                "left_wrist_0_rgb": wrist,
                "right_wrist_0_rgb": right_wrist,
            },
            "image_mask": {
                "base_0_rgb": np.asarray(True, dtype=bool),
                "left_wrist_0_rgb": np.asarray(True, dtype=bool),
                "right_wrist_0_rgb": np.asarray(False, dtype=bool),
            },
            "state": state,
            "actions": actions,
            "prompt": np.asarray(TASK_PROMPT),
        }


@dataclasses.dataclass(frozen=True)
class Rh56Outputs(DataTransformFn):
    """Keep model action outputs in the native absolute arm+RH56 contract."""

    def __call__(self, data: dict[str, Any]) -> dict[str, Any]:
        if "actions" in data:
            actions = _as_numpy(data["actions"])
            if actions.shape[-1] == MODEL_ACTION_DIM:
                actions = actions[..., :ACTION_DIM]
                data = {**data, "actions": actions}
            if actions.shape[-1] != ACTION_DIM:
                raise ValueError(f"Model output action dimension is {actions.shape[-1]}, expected {ACTION_DIM}")
            if not np.isfinite(actions).all():
                raise ValueError("Model output contains NaN/Inf")
        return data


@dataclasses.dataclass(frozen=True)
class Rh56DataConfig(openpi_config.DataConfigFactory):
    repo_id: str = DATASET_REPO_ID
    assets: openpi_config.AssetsConfig = dataclasses.field(
        default_factory=lambda: openpi_config.AssetsConfig(asset_id=DATASET_REPO_ID)
    )

    def create(self, assets_dirs: Path, model_config: pi0_config.Pi0Config) -> openpi_config.DataConfig:
        base = self.create_base_config(assets_dirs, model_config)
        return dataclasses.replace(
            base,
            repack_transforms=Group(
                inputs=[
                    RepackTransform(
                        {
                            "images": {
                                "workspace": "observation.images.workspace",
                                "wrist": "observation.images.wrist",
                            },
                            "state": "observation.state",
                            "actions": "action",
                            "task_index": "task_index",
                            "prompt": "prompt",
                        }
                    )
                ]
            ),
            data_transforms=Group(inputs=[Rh56Inputs()], outputs=[Rh56Outputs()]),
            model_transforms=ModelTransformFactory(default_prompt=TASK_PROMPT)(model_config),
            action_sequence_keys=("action",),
            prompt_from_task=True,
        )


def make_config(
    *,
    experiment_root: Path,
    exp_name: str,
    num_train_steps: int,
    batch_size: int = 4,
    num_workers: int = 0,
    save_interval: int = 100,
    log_interval: int = 10,
    keep_period: int = 500,
    seed: int = 20260814,
    resume: bool = False,
    overwrite: bool = False,
) -> openpi_config.TrainConfig:
    """Create the one experiment config used by norm-stats, smoke, and train."""

    model = pi0_config.Pi0Config(
        pi05=True,
        # pi05_base's released action projections are 32-wide. The audited
        # 12-dimensional command is padded by OpenPI's standard
        # PadStatesAndActions transform immediately before the model and is
        # sliced back to 12 by Rh56Outputs.
        action_dim=MODEL_ACTION_DIM,
        action_horizon=ACTION_HORIZON,
        paligemma_variant="gemma_2b_lora",
        action_expert_variant="gemma_300m_lora",
    )
    return openpi_config.TrainConfig(
        name="pi05_rh56",
        project_name="embodied_lab",
        exp_name=exp_name,
        model=model,
        data=Rh56DataConfig(),
        weight_loader=weight_loaders.CheckpointWeightLoader("gs://openpi-assets/checkpoints/pi05_base/params"),
        assets_base_dir=str(experiment_root / "assets"),
        checkpoint_base_dir=str(experiment_root / "checkpoints"),
        seed=seed,
        batch_size=batch_size,
        num_workers=num_workers,
        num_train_steps=num_train_steps,
        log_interval=log_interval,
        save_interval=save_interval,
        keep_period=keep_period,
        overwrite=overwrite,
        resume=resume,
        wandb_enabled=False,
        freeze_filter=model.get_freeze_filter(),
        lr_schedule=optimizer.CosineDecaySchedule(
            warmup_steps=100,
            peak_lr=1e-5,
            decay_steps=max(num_train_steps, 1000),
            decay_lr=1e-6,
        ),
        optimizer=optimizer.AdamW(clip_gradient_norm=1.0),
        ema_decay=None,
        policy_metadata={
            "task": TASK_PROMPT,
            "action_semantics": "absolute_native_target",
            "state_dim": STATE_DIM,
            "action_dim": ACTION_DIM,
            "action_horizon": ACTION_HORIZON,
            "image_keys": list(MODEL_IMAGE_KEYS),
            "force_enabled": False,
            "base_checkpoint": "pi05_base",
            "lora": True,
        },
    )


def config_summary(config: openpi_config.TrainConfig) -> dict[str, Any]:
    model = config.model
    return {
        "name": config.name,
        "exp_name": config.exp_name,
        "task": TASK_PROMPT,
        "base_checkpoint": "pi05_base",
        "model_type": model.model_type.value,
        "pi05": model.pi05,
        # The native command sent by this project is 12-dimensional. OpenPI's
        # released pi05_base projections require a 32-wide internal tensor;
        # keeping both names visible prevents the padding from being mistaken
        # for a change to the robot action contract.
        "action_dim": ACTION_DIM,
        "internal_model_action_dim": model.action_dim,
        "command_action_dim": ACTION_DIM,
        "model_action_dim": MODEL_ACTION_DIM,
        "action_horizon": model.action_horizon,
        "state_dim": STATE_DIM,
        "image_keys": list(MODEL_IMAGE_KEYS),
        "batch_size": config.batch_size,
        "effective_batch_size": config.batch_size,
        "num_workers": config.num_workers,
        "learning_rate": dataclasses.asdict(config.lr_schedule),
        "optimizer": dataclasses.asdict(config.optimizer),
        "lora": {
            "paligemma_variant": model.paligemma_variant,
            "action_expert_variant": model.action_expert_variant,
            "freeze_filter": str(config.freeze_filter),
        },
        "seed": config.seed,
        "save_interval": config.save_interval,
        "num_train_steps": config.num_train_steps,
        "upstream_commit": "15a9616a00943ada6c20a0f158e3adb39df2ccac",
    }
