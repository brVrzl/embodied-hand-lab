#!/usr/bin/env python3
"""Run session-grouped offline probes on materialized RH56 FORCE_ACT data."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from research.exploration.force_information import (
    ProbeConfig,
    compact_markdown,
    load_force_episodes,
    run_information_probe,
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("configs/experiments/icra2027_force_information.yaml"),
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    configuration = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    if configuration.get("schema_version") != "embodied_lab.force_information_experiment.v1":
        raise SystemExit("unsupported experiment config schema")
    dataset = configuration["dataset"]
    master = Path(dataset["master"])
    metadata = Path(dataset["source_metadata_root"])
    materialization = json.loads(
        Path(dataset["materialization_manifest"]).read_text(encoding="utf-8")
    )
    expected_fingerprint = str(dataset["expected_materialization_fingerprint"])
    if materialization.get("fingerprint") != expected_fingerprint:
        raise SystemExit("materialization fingerprint does not match the experiment config")

    probe = ProbeConfig(**configuration["probe"])
    episodes = load_force_episodes(master, metadata)
    cohort = configuration.get("cohort")
    if not isinstance(cohort, dict) or not isinstance(cohort.get("label"), str):
        raise SystemExit("experiment config requires a labeled cohort")
    include = cohort.get("source_episode_include")
    if include is not None:
        requested = {int(value) for value in include}
        episodes = [
            episode for episode in episodes if episode.source_episode_index in requested
        ]
        present = {episode.source_episode_index for episode in episodes}
        if present != requested:
            raise SystemExit(
                f"cohort source identity mismatch: missing={sorted(requested - present)}"
            )
    result = run_information_probe(episodes, probe)
    result["cohort"] = cohort
    result["identity"] = {
        "materialization_fingerprint": expected_fingerprint,
        "materialization_manifest": str(Path(dataset["materialization_manifest"])),
        "materialization_manifest_sha256": _sha256(
            Path(dataset["materialization_manifest"])
        ),
        "experiment_config": str(args.config),
        "experiment_config_sha256": _sha256(args.config),
    }
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / "results.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    (args.output / "summary.md").write_text(compact_markdown(result), encoding="utf-8")
    print(json.dumps({"output": str(args.output), "identity": result["identity"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
