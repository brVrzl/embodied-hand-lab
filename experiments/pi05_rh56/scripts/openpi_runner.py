"""Run staged validation and training against the pinned upstream OpenPI tree."""

from __future__ import annotations

import argparse
import dataclasses
import gc
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
from openpi_config import DATASET_REPO_ID, config_summary, make_config  # noqa: E402


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


def _config(
    args: argparse.Namespace,
    *,
    resume: bool = False,
    steps: int | None = None,
    dataset_repo_id: str | None = None,
    norm_stats_repo_id: str | None = None,
):
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
        dataset_repo_id=dataset_repo_id or args.dataset_repo_id,
        norm_stats_repo_id=norm_stats_repo_id or DATASET_REPO_ID,
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


def _checkpoint_paths(checkpoint_dir: Path) -> list[tuple[int, Path]]:
    """Return only complete numeric checkpoint directories, in step order."""

    result = []
    for step in _checkpoint_steps(checkpoint_dir):
        path = checkpoint_dir / str(step)
        if (path / "params").is_dir() and (path / "train_state").is_dir():
            result.append((step, path))
    return result


def _parse_requested_steps(value: str, available: list[tuple[int, Path]]) -> list[int]:
    if not value.strip():
        return [step for step, _ in available]
    requested = []
    available_steps = {step for step, _ in available}
    for token in value.split(","):
        token = token.strip()
        if not token:
            continue
        step = int(token)
        if step not in available_steps:
            raise ValueError(f"Requested checkpoint {step} is not a complete checkpoint")
        requested.append(step)
    if not requested:
        raise ValueError("--checkpoint-steps did not contain a checkpoint step")
    return sorted(set(requested))


def _load_val_batches(config: Any, args: argparse.Namespace) -> tuple[list[tuple[Any, Any]], dict[str, Any]]:
    """Build one deterministic, transformed validation probe for all checkpoints."""

    import json as stdlib_json
    from openpi.models import model as model_lib
    from openpi.training import data_loader

    data_config = config.data.create(config.assets_dirs, config.model)
    dataset = data_loader.create_torch_dataset(data_config, config.model.action_horizon, config.model)
    transformed = data_loader.transform_dataset(dataset, data_config)
    episodes_path = args.val_view_root / "meta" / "episodes.jsonl"
    episodes = [stdlib_json.loads(line) for line in episodes_path.read_text().splitlines() if line.strip()]
    if not episodes:
        raise ValueError(f"No validation episodes in {episodes_path}")

    sample_indices: list[int] = []
    episode_sample_counts: dict[str, int] = {}
    episode_offset = 0
    if args.full_val:
        sample_indices = list(range(len(transformed)))
    else:
        if args.samples_per_episode < 1:
            raise ValueError("--samples-per-episode must be positive")
        for episode in episodes:
            length = int(episode["length"])
            max_start = max(0, length - config.model.action_horizon)
            starts = np.unique(np.linspace(0, max_start, args.samples_per_episode, dtype=np.int64)).tolist()
            sample_indices.extend(episode_offset + int(start) for start in starts)
            episode_sample_counts[str(episode["episode_index"])] = len(starts)
            episode_offset += length
        if episode_offset != len(transformed):
            raise ValueError(
                f"Episode frame total {episode_offset} does not match dataset length {len(transformed)}"
            )

    usable_count = len(sample_indices) - (len(sample_indices) % config.batch_size)
    if usable_count == 0:
        raise ValueError(f"Validation probe has fewer than one full batch: {len(sample_indices)} samples")
    sample_indices = sample_indices[:usable_count]

    batches = []
    for offset in range(0, usable_count, config.batch_size):
        samples = [transformed[index] for index in sample_indices[offset : offset + config.batch_size]]
        batch = data_loader._collate_fn(samples)
        observation = model_lib.Observation.from_dict(batch)
        actions = batch["actions"]
        if actions.shape[-1] != 32 or observation.state.shape[-1] != 32:
            raise ValueError(
                f"Unexpected transformed dimensions: state={observation.state.shape}, actions={actions.shape}"
            )
        if not np.isfinite(np.asarray(actions)).all() or not np.isfinite(np.asarray(observation.state)).all():
            raise ValueError("Validation batch contains NaN/Inf")
        batches.append((observation, actions))

    metadata = {
        "repo_id": config.data.repo_id,
        "normalization_repo_id": DATASET_REPO_ID,
        "episode_count": len(episodes),
        "dataset_frames": len(transformed),
        "sample_count": usable_count,
        "batch_count": len(batches),
        "batch_size": config.batch_size,
        "action_horizon": config.model.action_horizon,
        "full_val": args.full_val,
        "samples_per_episode": None if args.full_val else args.samples_per_episode,
        "episode_sample_counts": episode_sample_counts,
        "image_keys": sorted(batches[0][0].images),
        "state_shape": list(batches[0][0].state.shape),
        "actions_shape": list(batches[0][1].shape),
    }
    return batches, metadata


def cmd_val_eval(args: argparse.Namespace) -> dict[str, Any]:
    """Compare checkpoints using deterministic OpenPI flow-matching val loss."""

    _set_dataset_home(args.experiment_root)
    _openpi_scripts()
    config = _config(
        args,
        dataset_repo_id=args.val_repo_id,
        norm_stats_repo_id=DATASET_REPO_ID,
    )
    _register(config)
    available = _checkpoint_paths(config.checkpoint_dir)
    requested_steps = _parse_requested_steps(args.checkpoint_steps, available)
    batches, batch_metadata = _load_val_batches(config, args)

    import jax
    import jax.numpy as jnp
    from openpi.models import model as model_lib
    from openpi.shared import nnx_utils

    report_path = args.experiment_root / "reports" / f"{args.report_name}.json"
    prior_results: dict[str, dict[str, Any]] = {}
    if report_path.exists():
        try:
            prior = json.loads(report_path.read_text())
            if (
                prior.get("protocol", {}).get("full_val") == args.full_val
                and prior.get("protocol", {}).get("samples_per_episode")
                == (None if args.full_val else args.samples_per_episode)
                and prior.get("protocol", {}).get("val_repo_id") == args.val_repo_id
            ):
                prior_results = {
                    str(item["checkpoint_step"]): item
                    for item in prior.get("results", [])
                    if item.get("status") == "passed"
                }
        except (OSError, KeyError, TypeError, ValueError):
            prior_results = {}

    base_key = jax.random.key(args.seed + 100_003)
    results = []
    checkpoint_by_step = dict(available)
    for step in requested_steps:
        if str(step) in prior_results:
            result = prior_results[str(step)]
            results.append(result)
            print(json.dumps({"reused": True, **result}, sort_keys=True), flush=True)
            continue

        checkpoint_path = checkpoint_by_step[step]
        started = time.monotonic()
        print(f"[val-eval] loading checkpoint step={step} path={checkpoint_path}", flush=True)
        try:
            params = model_lib.restore_params(checkpoint_path / "params", dtype=jnp.bfloat16)
            model = config.model.load(params)
            loss_fn = nnx_utils.module_jit(model.compute_loss)
            losses = []
            for batch_index, (observation, actions) in enumerate(batches):
                loss = loss_fn(
                    jax.random.fold_in(base_key, batch_index),
                    observation,
                    actions,
                )
                jax.block_until_ready(loss)
                loss_values = np.asarray(jax.device_get(loss), dtype=np.float64).reshape(-1)
                if not np.isfinite(loss_values).all():
                    raise ValueError(f"Non-finite loss at batch {batch_index}: {loss_values}")
                losses.append(loss_values)
            flat_loss = np.concatenate(losses)
            result = {
                "status": "passed",
                "checkpoint_step": step,
                "checkpoint_path": str(checkpoint_path.relative_to(args.experiment_root)),
                "loss_mean": float(np.mean(flat_loss)),
                "loss_std": float(np.std(flat_loss)),
                "loss_min": float(np.min(flat_loss)),
                "loss_max": float(np.max(flat_loss)),
                "loss_values": int(flat_loss.size),
                "duration_s": time.monotonic() - started,
            }
            print(json.dumps(result, sort_keys=True, default=_json_default), flush=True)
        except Exception as exc:  # Record a bad/corrupt checkpoint and continue the sweep.
            result = {
                "status": "failed",
                "checkpoint_step": step,
                "checkpoint_path": str(checkpoint_path.relative_to(args.experiment_root)),
                "error": f"{type(exc).__name__}: {exc}",
                "duration_s": time.monotonic() - started,
            }
            print(json.dumps(result, sort_keys=True, default=_json_default), flush=True)
        results.append(result)
        report = {
            "status": "in_progress",
            "task": TASK_PROMPT,
            "metric": "mean_flow_matching_loss",
            "seed": args.seed + 100_003,
            "protocol": {"full_val": args.full_val, "samples_per_episode": None if args.full_val else args.samples_per_episode, "val_repo_id": args.val_repo_id},
            "batch": batch_metadata,
            "results": sorted(results, key=lambda item: int(item["checkpoint_step"])),
            "updated_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        }
        _write_report(args.experiment_root, args.report_name, report)
        params = None
        model = None
        loss_fn = None
        gc.collect()

    passed = [item for item in results if item.get("status") == "passed"]
    if not passed:
        raise RuntimeError("No checkpoint completed validation")
    best = min(passed, key=lambda item: float(item["loss_mean"]))
    report = {
        "status": "passed",
        "task": TASK_PROMPT,
        "metric": "mean_flow_matching_loss",
        "seed": args.seed + 100_003,
        "protocol": {"full_val": args.full_val, "samples_per_episode": None if args.full_val else args.samples_per_episode, "val_repo_id": args.val_repo_id},
        "batch": batch_metadata,
        "results": sorted(results, key=lambda item: int(item["checkpoint_step"])),
        "best_checkpoint_step": int(best["checkpoint_step"]),
        "best_loss_mean": float(best["loss_mean"]),
        "updated_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    }
    _write_report(args.experiment_root, args.report_name, report)
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
    parser.add_argument("command", choices=("manifest", "build-view", "validate-view", "norm-stats", "config-summary", "data-smoke", "model-smoke", "checkpoint-smoke", "val-eval", "train"))
    parser.add_argument("--experiment-root", type=Path, default=REPOSITORY_ROOT / "outputs/training/pi05_rh56")
    parser.add_argument("--source-master", type=Path, default=REPOSITORY_ROOT / "data/training/physical_bottle_v4_nominal52/act/master")
    parser.add_argument("--manifest", type=Path, default=REPOSITORY_ROOT / "outputs/training/pi05_rh56/audited_manifest.json")
    parser.add_argument("--output", type=Path, default=REPOSITORY_ROOT / "outputs/training/pi05_rh56/audited_manifest.json")
    parser.add_argument("--view-root", type=Path, default=REPOSITORY_ROOT / "outputs/training/pi05_rh56/lerobot_home_v2/local/pi05_rh56_train")
    parser.add_argument("--dataset-repo-id", default=DATASET_REPO_ID)
    parser.add_argument("--val-repo-id", default="local/pi05_rh56_val")
    parser.add_argument("--val-view-root", type=Path, default=REPOSITORY_ROOT / "outputs/training/pi05_rh56/lerobot_home_v2/local/pi05_rh56_val")
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
    parser.add_argument("--checkpoint-steps", default="", help="Comma-separated checkpoint steps; empty means all complete checkpoints")
    parser.add_argument("--samples-per-episode", type=int, default=8)
    parser.add_argument("--full-val", action="store_true", help="Evaluate every validation frame instead of a fixed probe")
    parser.add_argument("--report-name", default="val_checkpoint_eval")
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
    elif args.command == "val-eval":
        result = cmd_val_eval(args)
    else:
        result = cmd_train(args)
    print(json.dumps(result, indent=2, sort_keys=True, default=_json_default))


if __name__ == "__main__":
    main()
