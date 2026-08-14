"""Run staged validation and training against the pinned upstream OpenPI tree."""

from __future__ import annotations

import argparse
import dataclasses
import json
import os
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

EXPERIMENT_DIR = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = EXPERIMENT_DIR.parents[1]
if str(EXPERIMENT_DIR) not in sys.path:
    sys.path.insert(0, str(EXPERIMENT_DIR))

from openpi_adapter import (  # noqa: E402
    ACTION_DIM,
    ACTION_HORIZON,
    STATE_DIM,
    TASK_PROMPT,
    build_audited_manifest,
    build_openpi_view,
    validate_derived_view,
)
from openpi_config import config_summary, make_config  # noqa: E402


def _json_default(value: Any) -> Any:
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(type(value).__name__)


def _write_report(root: Path, name: str, report: dict[str, Any]) -> None:
    path = root / "reports" / f"{name}.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2, sort_keys=True, default=_json_default) + "\n")


def _set_dataset_home(root: Path) -> None:
    dataset_home = root / "lerobot_home_v2"
    dataset_home.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("HF_LEROBOT_HOME", str(dataset_home))


def _config(args: argparse.Namespace, *, resume: bool = False, steps: int | None = None):
    root = args.experiment_root.resolve()
    return make_config(
        experiment_root=root,
        exp_name=args.exp_name,
        num_train_steps=steps if steps is not None else args.steps,
        batch_size=args.batch_size,
        num_workers=args.num_workers,
        save_interval=args.save_interval,
        log_interval=args.log_interval,
        keep_period=args.keep_period,
        seed=args.seed,
        resume=resume,
        overwrite=False,
    )


def _register(config: Any) -> None:
    # The upstream scripts intentionally use the config registry. Registering
    # one in-process object keeps norm-stat, smoke, and train behavior aligned.
    import openpi.training.config as registry

    if not any(item.name == config.name for item in registry._CONFIGS):
        registry._CONFIGS.append(config)
    registry._CONFIGS_DICT[config.name] = config


def _openpi_scripts():
    # /app is the pinned upstream OpenPI checkout in the Thor container.
    if "/app" not in sys.path:
        sys.path.insert(0, "/app")
    _patch_flax_state_map_for_thor()
    _patch_orbax_restore_for_thor()
    from scripts import compute_norm_stats, train

    return compute_norm_stats, train


def _patch_flax_state_map_for_thor() -> None:
    """Work around Flax 0.12's unhashable NNX variable values on Thor.

    The pinned OpenPI checkout calls ``set(FlatState)`` while selecting frozen
    variables. In the image's Flax 0.12.2, FlatState entries contain unhashable
    ``Param`` values. The keys are the intended identity, so this runtime-only
    compatibility shim makes that selection explicit without modifying the
    read-only upstream checkout or changing the LoRA freeze filter.
    """

    import openpi.shared.nnx_utils as nnx_utils

    if getattr(nnx_utils.state_map, "_embodied_lab_thor_compat", False):
        return

    def state_map(state: Any, filter_fn: Any, fn: Any) -> Any:
        filtered_keys = {key for key, _ in state.filter(filter_fn).flat_state()}
        return state.map(lambda key, value: fn(value) if key in filtered_keys else value)

    state_map._embodied_lab_thor_compat = True
    nnx_utils.state_map = state_map


def _patch_orbax_restore_for_thor() -> None:
    """Adapt OpenPI's metadata access to the Orbax version in the Thor image.

    The pinned OpenPI source indexes ``PyTreeCheckpointer.metadata()`` as a
    dictionary.  Orbax 0.11.39 returns ``StepMetadata`` instead, with the
    restore tree under ``item_metadata.tree``.  This preserves OpenPI's
    restore arguments and only changes that version-specific metadata lookup;
    no checkpoint contents or parameter mapping are changed.
    """

    import flax.traverse_util
    import jax
    import orbax.checkpoint as ocp
    import openpi.models.model as model_lib

    if getattr(model_lib.restore_params, "_embodied_lab_thor_compat", False):
        return

    original_restore_params = model_lib.restore_params

    def restore_params(params_path, *, restore_type=jax.Array, dtype=None, sharding=None):
        if str(params_path).startswith("gs://"):
            return original_restore_params(
                params_path, restore_type=restore_type, dtype=dtype, sharding=sharding
            )

        local_path = Path(params_path).resolve()
        with ocp.PyTreeCheckpointer() as ckptr:
            metadata = ckptr.metadata(local_path)
            if isinstance(metadata, dict):
                return original_restore_params(
                    local_path, restore_type=restore_type, dtype=dtype, sharding=sharding
                )
            item = {"params": metadata.item_metadata.tree["params"]}
            if restore_type is jax.Array and sharding is None:
                mesh = jax.sharding.Mesh(jax.devices(), ("x",))
                sharding = jax.sharding.NamedSharding(mesh, jax.sharding.PartitionSpec())
            params = ckptr.restore(
                local_path,
                ocp.args.PyTreeRestore(
                    item=item,
                    restore_args=jax.tree.map(
                        lambda _: ocp.ArrayRestoreArgs(
                            sharding=sharding, restore_type=restore_type, dtype=dtype
                        ),
                        item,
                    ),
                ),
            )["params"]

        flat_params = flax.traverse_util.flatten_dict(params)
        if all(kp[-1] == "value" for kp in flat_params):
            flat_params = {kp[:-1]: value for kp, value in flat_params.items()}
        return flax.traverse_util.unflatten_dict(flat_params)

    restore_params._embodied_lab_thor_compat = True
    model_lib.restore_params = restore_params


def cmd_manifest(args: argparse.Namespace) -> dict[str, Any]:
    return build_audited_manifest(args.source_master, args.output)


def cmd_build_view(args: argparse.Namespace) -> dict[str, Any]:
    return build_openpi_view(args.source_master, args.view_root, args.manifest, split_name=args.split)


def cmd_validate_view(args: argparse.Namespace) -> dict[str, Any]:
    return validate_derived_view(args.view_root, args.manifest)


def cmd_norm_stats(args: argparse.Namespace) -> dict[str, Any]:
    _set_dataset_home(args.experiment_root)
    config = _config(args)
    _register(config)
    compute_norm_stats, _ = _openpi_scripts()
    compute_norm_stats.main(config.name)
    norm_path = config.assets_dirs / config.data.repo_id / "norm_stats.json"
    if not norm_path.exists():
        raise FileNotFoundError(norm_path)
    stats = json.loads(norm_path.read_text())["norm_stats"]
    report: dict[str, Any] = {
        "status": "computed",
        "path": norm_path,
        "task": TASK_PROMPT,
        "state_dim": STATE_DIM,
        "action_dim": ACTION_DIM,
        "action_horizon": ACTION_HORIZON,
        "stats": stats,
        "checks": {},
    }
    for key in ("state", "actions"):
        values = stats[key]
        for field in ("mean", "std", "q01", "q99"):
            array = np.asarray(values[field], dtype=np.float64)
            report["checks"][f"{key}.{field}"] = {
                "dim": int(array.size),
                "finite": bool(np.isfinite(array).all()),
                "min": float(np.min(array)),
                "max": float(np.max(array)),
            }
        std = np.asarray(values["std"], dtype=np.float64)
        q01 = np.asarray(values["q01"], dtype=np.float64)
        q99 = np.asarray(values["q99"], dtype=np.float64)
        if std.size != ACTION_DIM or np.any(std <= 1e-8) or not np.isfinite(std).all():
            raise ValueError(f"Pathological {key} normalization std: {std}")
        if np.any(q99 <= q01) or not np.isfinite(q01).all() or not np.isfinite(q99).all():
            raise ValueError(f"Pathological {key} normalization quantiles")
    _write_report(args.experiment_root, "normalization", report)
    return report


def cmd_config_summary(args: argparse.Namespace) -> dict[str, Any]:
    """Write the resolved training configuration without initializing weights."""

    config = _config(args)
    report = config_summary(config)
    report["experiment_root"] = args.experiment_root.resolve()
    report["checkpoint_dir"] = config.checkpoint_dir
    report["assets_dir"] = config.assets_dirs
    _write_report(args.experiment_root, "config_summary", report)
    return report


def cmd_data_smoke(args: argparse.Namespace) -> dict[str, Any]:
    _set_dataset_home(args.experiment_root)
    config = _config(args)
    _register(config)
    import jax
    from openpi.models import model as model_lib
    from openpi.training import data_loader
    from openpi.training import sharding

    loader = data_loader.create_data_loader(
        config,
        sharding=jax.sharding.NamedSharding(
            jax.sharding.Mesh(jax.devices(), ("B",)), jax.sharding.PartitionSpec("B")
        ),
        shuffle=False,
        num_batches=3,
    )
    batches = []
    for _, (observation, actions) in zip(range(3), loader, strict=True):
        if actions.shape[-1] != 32:
            raise ValueError(f"Internal model action width is {actions.shape[-1]}, expected 32")
        if observation.state.shape[-1] != 32:
            raise ValueError(f"Internal model state width is {observation.state.shape[-1]}, expected 32")
        if not bool(np.isfinite(np.asarray(actions)).all()) or not bool(np.isfinite(np.asarray(observation.state)).all()):
            raise ValueError("Transformed batch has NaN/Inf")
        batches.append(
            {
                "images": {key: list(value.shape) for key, value in observation.images.items()},
                "image_masks": {key: list(value.shape) for key, value in observation.image_masks.items()},
                "state_shape": list(observation.state.shape),
                "actions_shape": list(actions.shape),
                "actions_finite": bool(np.isfinite(np.asarray(actions)).all()),
            }
        )
    expected = {"base_0_rgb", "left_wrist_0_rgb", "right_wrist_0_rgb"}
    if set(batches[0]["images"]) != expected:
        raise ValueError(f"Unexpected model image keys: {batches[0]['images']}")
    report = {
        "status": "passed",
        "task": TASK_PROMPT,
        "audited_episodes": 52,
        "training_episodes": 37,
        "source_action_dim": ACTION_DIM,
        "model_action_dim": 32,
        "action_horizon": ACTION_HORIZON,
        "batches": batches,
        "jax_devices": [str(device) for device in jax.devices()],
        "backend": jax.default_backend(),
        "thor_flax_state_map_compatibility": True,
        "unused_import_check": model_lib.__name__,
        "sharding": str(sharding.make_mesh(config.fsdp_devices)),
    }
    _write_report(args.experiment_root, "data_smoke", report)
    return report


def _count_state(state: Any) -> int:
    import jax

    return int(sum(np.prod(np.asarray(value).shape, dtype=np.int64) for value in jax.tree.leaves(state.to_pure_dict())))


def cmd_model_smoke(args: argparse.Namespace) -> dict[str, Any]:
    _set_dataset_home(args.experiment_root)
    config = _config(args, steps=max(args.steps, 2))
    _register(config)
    import functools
    import jax
    from openpi.training import data_loader, sharding
    _, train_script = _openpi_scripts()

    mesh = sharding.make_mesh(config.fsdp_devices)
    data_sharding = jax.sharding.NamedSharding(mesh, jax.sharding.PartitionSpec(sharding.DATA_AXIS))
    replicated_sharding = jax.sharding.NamedSharding(mesh, jax.sharding.PartitionSpec())
    loader = data_loader.create_data_loader(config, sharding=data_sharding, shuffle=True, num_batches=2)
    batch = next(iter(loader))
    train_state, train_state_sharding = train_script.init_train_state(
        config, jax.random.key(config.seed + 1), mesh, resume=False
    )
    jax.block_until_ready(train_state)
    ptrain_step = jax.jit(
        functools.partial(train_script.train_step, config),
        in_shardings=(replicated_sharding, train_state_sharding, data_sharding),
        out_shardings=(train_state_sharding, replicated_sharding),
        donate_argnums=(1,),
    )
    start = time.monotonic()
    train_state, info = ptrain_step(jax.random.key(config.seed + 2), train_state, batch)
    jax.block_until_ready(info)
    loss = float(np.asarray(info["loss"]))
    if not np.isfinite(loss):
        raise ValueError(f"Non-finite smoke loss: {loss}")
    params = train_state.params
    trainable = _count_state(params.filter(config.trainable_filter))
    frozen = _count_state(params.filter(config.freeze_filter))
    report = {
        "status": "passed",
        "duration_s": time.monotonic() - start,
        "loss": loss,
        "grad_norm": float(np.asarray(info["grad_norm"])),
        "param_norm": float(np.asarray(info["param_norm"])),
        "trainable_parameters": trainable,
        "frozen_parameters": frozen,
        "total_parameters": trainable + frozen,
        "optimizer_step_after_update": int(train_state.step),
        "jax_devices": [str(device) for device in jax.devices()],
        "backend": jax.default_backend(),
        "lora_variants": {
            "paligemma": config.model.paligemma_variant,
            "action_expert": config.model.action_expert_variant,
        },
    }
    _write_report(args.experiment_root, "model_smoke", report)
    return report


def _checkpoint_steps(path: Path) -> list[int]:
    if not path.exists():
        return []
    result = []
    for child in path.iterdir():
        if child.is_dir() and child.name.isdigit():
            result.append(int(child.name))
    return sorted(result)


def cmd_checkpoint_smoke(args: argparse.Namespace) -> dict[str, Any]:
    _set_dataset_home(args.experiment_root)
    _, train_script = _openpi_scripts()
    pilot_name = f"{args.exp_name}_pilot_resume"
    first = make_config(
        experiment_root=args.experiment_root,
        exp_name=pilot_name,
        num_train_steps=2,
        batch_size=args.batch_size,
        num_workers=args.num_workers,
        save_interval=1,
        log_interval=1,
        keep_period=10,
        seed=args.seed,
        resume=False,
        overwrite=False,
    )
    _register(first)
    train_script.main(first)
    first_steps = _checkpoint_steps(first.checkpoint_dir)
    if not first_steps:
        raise RuntimeError(f"Pilot did not write a checkpoint under {first.checkpoint_dir}")
    resumed = dataclasses.replace(first, num_train_steps=4, resume=True)
    train_script.main(resumed)
    final_steps = _checkpoint_steps(resumed.checkpoint_dir)
    if not final_steps or max(final_steps) <= max(first_steps):
        raise RuntimeError(f"Pilot resume did not advance checkpoints: {first_steps} -> {final_steps}")
    report = {
        "status": "passed",
        "pilot_checkpoint_dir": first.checkpoint_dir,
        "steps_after_first_process": first_steps,
        "steps_after_resume": final_steps,
        "resume_started_from": max(first_steps),
        "resume_advanced_to": max(final_steps),
        "overwrite": False,
    }
    _write_report(args.experiment_root, "checkpoint_resume_smoke", report)
    return report


def cmd_train(args: argparse.Namespace) -> dict[str, Any]:
    _set_dataset_home(args.experiment_root)
    _, train_script = _openpi_scripts()
    config = _config(args, resume=args.resume)
    _register(config)
    train_script.main(config)
    report = {
        "status": "completed_process",
        "config": config_summary(config),
        "checkpoint_dir": config.checkpoint_dir,
        "resume": args.resume,
    }
    _write_report(args.experiment_root, f"train_exit_{args.exp_name}", report)
    return report


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("manifest", "build-view", "validate-view", "norm-stats", "config-summary", "data-smoke", "model-smoke", "checkpoint-smoke", "train"))
    parser.add_argument("--experiment-root", type=Path, default=REPOSITORY_ROOT / "outputs/training/pi05_rh56")
    parser.add_argument("--source-master", type=Path, default=REPOSITORY_ROOT / "data/training/physical_bottle_v4_nominal52/act/master")
    parser.add_argument("--manifest", type=Path, default=REPOSITORY_ROOT / "outputs/training/pi05_rh56/audited_manifest.json")
    parser.add_argument("--output", type=Path, default=REPOSITORY_ROOT / "outputs/training/pi05_rh56/audited_manifest.json")
    parser.add_argument("--view-root", type=Path, default=REPOSITORY_ROOT / "outputs/training/pi05_rh56/lerobot_home_v2/local/pi05_rh56_train")
    parser.add_argument("--split", choices=("train", "val"), default="train")
    parser.add_argument("--exp-name", default="weekend")
    parser.add_argument("--steps", type=int, default=20_000)
    parser.add_argument("--batch-size", type=int, default=4)
    parser.add_argument("--num-workers", type=int, default=0)
    parser.add_argument("--save-interval", type=int, default=100)
    parser.add_argument("--log-interval", type=int, default=10)
    parser.add_argument("--keep-period", type=int, default=500)
    parser.add_argument("--seed", type=int, default=20260814)
    parser.add_argument("--resume", action="store_true")
    return parser


def main() -> None:
    args = _parser().parse_args()
    if args.command == "manifest":
        result = cmd_manifest(args)
    elif args.command == "build-view":
        result = cmd_build_view(args)
    elif args.command == "validate-view":
        result = cmd_validate_view(args)
    elif args.command == "norm-stats":
        result = cmd_norm_stats(args)
    elif args.command == "config-summary":
        result = cmd_config_summary(args)
    elif args.command == "data-smoke":
        result = cmd_data_smoke(args)
    elif args.command == "model-smoke":
        result = cmd_model_smoke(args)
    elif args.command == "checkpoint-smoke":
        result = cmd_checkpoint_smoke(args)
    else:
        result = cmd_train(args)
    print(json.dumps(result, indent=2, sort_keys=True, default=_json_default))


if __name__ == "__main__":
    main()
