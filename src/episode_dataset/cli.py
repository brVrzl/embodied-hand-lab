"""Offline validation, inspection, indexing, statistics, and export."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .exporters import export_act_hdf5, export_lerobot_v3
from .inspection import inspect_episode, play_episode, write_inspection_plot
from .manifest import build_dataset_manifest, compute_train_statistics
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
