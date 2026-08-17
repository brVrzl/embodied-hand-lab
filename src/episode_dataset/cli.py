"""Offline validation, inspection, indexing, statistics, and export."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import yaml

from .exporters import export_act_hdf5, export_lerobot_v3
from .inspection import inspect_episode, play_episode, write_inspection_plot
from .lerobot_staging import (
    materialize_staging_episode,
    set_staging_review,
    stage_review_html,
)
from .manifest import build_dataset_manifest, compute_train_statistics
from .synchronization import synchronize_staging_episode
from .training_materialization import (
    materialize_training_dataset,
    validate_training_dataset,
)
from .training_views import smoke_act_dataset
from .openpi_adapter import smoke_openpi_dataset
from .physical_bottle_materialization import (
    audit_physical_bottle,
    materialize_physical_bottle,
    validate_physical_bottle,
)
from .validation import validate_episode, validation_exit_code


def build_parser(*, prog: str | None = None) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog=prog, description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)

    validate = commands.add_parser(
        "validate", help="validate one or more finalized episode directories"
    )
    validate.add_argument("episodes", type=Path, nargs="+")
    validate.add_argument(
        "--fast",
        action="store_true",
        help="skip NPY payload and raw JSONL reads",
    )
    validate.add_argument("--output", type=Path, help="optional JSON report")

    inspect = commands.add_parser(
        "inspect",
        help="summarize one episode and optionally plot or play it locally",
    )
    inspect.add_argument("episode", type=Path)
    inspect.add_argument("--output", type=Path, help="optional JSON summary")
    inspect.add_argument("--plot", type=Path, help="write a state/action/timing PNG")
    inspect.add_argument(
        "--playback",
        action="store_true",
        help="open an offline RGB/depth playback window",
    )
    inspect.add_argument("--playback-rate", type=float, default=1.0)

    label = commands.add_parser(
        "label", help="assign reviewed task outcome metadata to one episode"
    )
    label.add_argument("episode", type=Path)
    label.add_argument("--success", choices=("success", "failure"), required=True)
    label.add_argument("--failure-stage")
    label.add_argument("--notes", default="")

    manifest = commands.add_parser(
        "manifest",
        help="build deterministic episode-level train/validation/test splits",
    )
    manifest.add_argument("dataset_root", type=Path)
    manifest.add_argument("output", type=Path)
    manifest.add_argument("--seed", default="embodied-lab-v1")
    manifest.add_argument("--train-fraction", type=float, default=0.8)
    manifest.add_argument("--validation-fraction", type=float, default=0.1)
    manifest.add_argument(
        "--fast",
        action="store_true",
        help=(
            "skip NPY payload and raw JSONL reads; inventory-only episodes "
            "are excluded from every training split"
        ),
    )

    statistics = commands.add_parser(
        "statistics", help="compute train-only normalization statistics"
    )
    statistics.add_argument("manifest", type=Path)
    statistics.add_argument("output", type=Path)

    export = commands.add_parser(
        "export", help="export one training-eligible canonical episode"
    )
    export.add_argument("episode", type=Path)
    formats = export.add_subparsers(dest="format", required=True)
    act = formats.add_parser("act-hdf5")
    act.add_argument("output", type=Path)
    lerobot = formats.add_parser("lerobot-v3")
    lerobot.add_argument("output", type=Path)
    lerobot.add_argument("--repo-id", required=True)

    review_staging = commands.add_parser(
        "review-staging", help="write a local HTML review page for a staging episode"
    )
    review_staging.add_argument("dataset_root", type=Path)
    review_staging.add_argument("episode")
    review_staging.add_argument("--output", type=Path)

    approve_staging = commands.add_parser(
        "approve-staging", help="record the human review decision for staging data"
    )
    approve_staging.add_argument("dataset_root", type=Path)
    approve_staging.add_argument("episode")
    approve_staging.add_argument("--status", choices=("approved", "rejected"), required=True)
    approve_staging.add_argument("--notes", default="")

    convert_staging = commands.add_parser(
        "convert-staging", help="convert an approved staging episode to Parquet"
    )
    convert_staging.add_argument("dataset_root", type=Path)
    convert_staging.add_argument("episode")
    convert_staging.add_argument("output_root", type=Path)

    sync_staging = commands.add_parser(
        "sync-staging",
        help="check causal 30 Hz synchronization for one staging episode",
    )
    sync_staging.add_argument("dataset_root", type=Path)
    sync_staging.add_argument("episode")
    sync_staging.add_argument("--camera-tolerance-ms", type=float, default=100.0)
    sync_staging.add_argument("--fps", type=int)
    sync_staging.add_argument("--output", type=Path)

    materialize_training = commands.add_parser(
        "materialize-training",
        help="materialize an immutable physical episode manifest into one training master",
    )
    materialize_training.add_argument("--config", type=Path, required=True)
    materialize_training.add_argument(
        "--replace",
        action="store_true",
        help="replace only an existing generated training root with a different fingerprint",
    )

    validate_training = commands.add_parser(
        "validate-training",
        help="validate a generated physical training master and its videos",
    )
    validate_training.add_argument("dataset_root", type=Path)
    validate_training.add_argument("--output", type=Path)

    act_smoke = commands.add_parser(
        "act-smoke",
        help="load representative samples through the local ACT view adapter",
    )
    act_smoke.add_argument("--config", type=Path, required=True)

    act_force_smoke = commands.add_parser(
        "act-force-smoke",
        help="load representative samples through the local ACT+Force view adapter",
    )
    act_force_smoke.add_argument("--config", type=Path, required=True)

    openpi_smoke = commands.add_parser(
        "openpi-smoke",
        help="dry-run the thin repository-specific openpi data mapping",
    )
    openpi_smoke.add_argument("--config", type=Path, required=True)

    audit_physical = commands.add_parser(
        "audit-physical-bottle",
        help="audit reviewed physical bottle episodes and logical segmentation",
    )
    audit_physical.add_argument("--config", type=Path, required=True)

    materialize_physical = commands.add_parser(
        "materialize-physical-bottle",
        help="materialize matched ACT and ACT+Force physical bottle views",
    )
    materialize_physical.add_argument("--config", type=Path, required=True)
    materialize_physical.add_argument("--replace", action="store_true")

    validate_physical = commands.add_parser(
        "validate-physical-bottle",
        help="validate matched physical bottle training views",
    )
    validate_physical.add_argument("dataset_root", type=Path)
    validate_physical.add_argument("--output", type=Path)
    return parser


def _write_report(path: Path, payload: object) -> None:
    path = path.resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def _label_episode(
    episode: Path, *, success: str, failure_stage: str | None, notes: str
) -> Path:
    episode = episode.resolve()
    metadata_path = episode / "metadata.json"
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    if metadata.get("finalized") is not True:
        raise ValueError("only a finalized episode can be labeled")
    if success == "success" and failure_stage is not None:
        raise ValueError("--failure-stage is only valid with --success failure")
    if success == "failure" and not failure_stage:
        raise ValueError("failed episode requires --failure-stage")
    metadata["success_label"] = success
    metadata["failure_stage"] = failure_stage
    metadata["notes"] = notes
    _write_report(metadata_path, metadata)
    return metadata_path


def _load_policy_view_config(
    config_path: Path, *, view_name: str | None = None
) -> tuple[Path, dict[str, object]]:
    config_path = config_path.resolve()
    value = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("policy view config must be a mapping")
    dataset_config_path = config_path
    if "dataset_root" not in value:
        dataset_config_value = value.get("dataset_config")
        if not isinstance(dataset_config_value, str):
            raise ValueError(
                "policy view config requires dataset_root or dataset_config"
            )
        dataset_config_path = (config_path.parent / dataset_config_value).resolve()
        dataset_config = yaml.safe_load(
            dataset_config_path.read_text(encoding="utf-8")
        )
        if not isinstance(dataset_config, dict):
            raise ValueError("dataset_config must contain a mapping")
        selected_view = view_name or str(value.get("view", "act"))
        views = dataset_config.get("views")
        if not isinstance(views, dict) or not isinstance(views.get(selected_view), dict):
            raise ValueError(f"dataset_config has no policy view {selected_view!r}")
        merged = dict(views[selected_view])
        merged.update(
            {
                key: item
                for key, item in value.items()
                if key not in {"schema_version", "dataset_config", "view"}
            }
        )
        merged["view"] = selected_view
        value = merged
    dataset_value = value.get("dataset_root")
    if not isinstance(dataset_value, str):
        raise ValueError("resolved policy view config requires dataset_root")
    dataset_root = Path(dataset_value)
    if not dataset_root.is_absolute():
        dataset_root = (dataset_config_path.parent / dataset_root).resolve()
    return dataset_root, value


def main(argv: list[str] | None = None, *, prog: str | None = None) -> int:
    args = build_parser(prog=prog).parse_args(argv)
    if args.command == "validate":
        reports = [
            validate_episode(path, deep=not args.fast) for path in args.episodes
        ]
        payload: object = reports[0] if len(reports) == 1 else reports
        if args.output is not None:
            _write_report(args.output, payload)
        print(json.dumps(payload, indent=2, sort_keys=True))
        return validation_exit_code(reports)
    if args.command == "inspect":
        payload = inspect_episode(args.episode)
        if args.plot is not None and payload["inspection_available"]:
            payload["plot"] = str(
                write_inspection_plot(args.episode, args.plot)
            )
        if args.output is not None:
            _write_report(args.output, payload)
        print(json.dumps(payload, indent=2, sort_keys=True))
        if args.playback and payload["inspection_available"]:
            play_episode(args.episode, playback_rate=args.playback_rate)
        return 0 if payload["validation"]["valid"] else 1
    if args.command == "label":
        result = _label_episode(
            args.episode,
            success=args.success,
            failure_stage=args.failure_stage,
            notes=args.notes,
        )
        print(result)
        return 0
    if args.command == "review-staging":
        result = stage_review_html(
            args.dataset_root, args.episode, output=args.output
        )
        print(result)
        return 0
    if args.command == "approve-staging":
        result = set_staging_review(
            args.dataset_root,
            args.episode,
            status=args.status,
            notes=args.notes,
        )
        print(result)
        return 0
    if args.command == "convert-staging":
        result = materialize_staging_episode(
            args.dataset_root, args.episode, args.output_root
        )
        print(result)
        return 0
    if args.command == "sync-staging":
        result = synchronize_staging_episode(
            args.dataset_root,
            args.episode,
            camera_tolerance_ms=args.camera_tolerance_ms,
            fps=args.fps,
        )
        if args.output is not None:
            _write_report(args.output, result)
            print(args.output)
        else:
            print(json.dumps(result, indent=2, sort_keys=True))
        return 0
    if args.command == "materialize-training":
        result = materialize_training_dataset(args.config, replace=args.replace)
        payload = {
            "output_root": str(result.output_root),
            "report_path": None if result.report_path is None else str(result.report_path),
            "reused": result.reused,
            "summary": result.summary,
        }
        print(json.dumps(payload, indent=2, sort_keys=True))
        return 0
    if args.command == "validate-training":
        result = validate_training_dataset(args.dataset_root)
        if args.output is not None:
            _write_report(args.output, result)
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0 if result["status"] == "passed" else 1
    if args.command == "audit-physical-bottle":
        result = audit_physical_bottle(args.config)
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0
    if args.command == "materialize-physical-bottle":
        result = materialize_physical_bottle(args.config, replace=args.replace)
        payload = {
            "output_root": str(result.output_root),
            "reused": result.reused,
            "summary": result.summary,
        }
        print(json.dumps(payload, indent=2, sort_keys=True))
        return 0
    if args.command == "validate-physical-bottle":
        result = validate_physical_bottle(args.dataset_root)
        if args.output is not None:
            _write_report(args.output, result)
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0 if result["status"] == "passed" else 1
    if args.command in {"act-smoke", "act-force-smoke", "openpi-smoke"}:
        view_name = "act_force" if args.command == "act-force-smoke" else None
        dataset_root, view_config = _load_policy_view_config(
            args.config, view_name=view_name
        )
        split = str(view_config.get("split", "train"))
        horizon = int(view_config.get("action_horizon", 16))
        if args.command == "act-smoke":
            result = smoke_act_dataset(dataset_root, split=split, action_horizon=horizon)
        elif args.command == "act-force-smoke":
            result = smoke_act_dataset(dataset_root, split=split, action_horizon=horizon, force=True)
        else:
            result = smoke_openpi_dataset(dataset_root, split=split, action_horizon=horizon)
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0
    if args.command == "manifest":
        result = build_dataset_manifest(
            args.dataset_root,
            args.output,
            seed=args.seed,
            train_fraction=args.train_fraction,
            validation_fraction=args.validation_fraction,
            deep_validation=not args.fast,
        )
    elif args.command == "statistics":
        result = compute_train_statistics(args.manifest, args.output)
    elif args.format == "act-hdf5":
        result = export_act_hdf5(args.episode, args.output)
    else:
        result = export_lerobot_v3(
            args.episode, args.output, repo_id=args.repo_id
        )
    print(result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
