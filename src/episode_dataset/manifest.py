from __future__ import annotations

from collections import Counter
import hashlib
import json
from pathlib import Path
from typing import Any, Iterable, Mapping

import numpy as np

from .episode import ACTION_ORDER, OBSERVATION_STATE_ORDER, SCHEMA_VERSION, file_sha256
from .validation import load_canonical_rows, validate_episode


MANIFEST_SCHEMA_VERSION = "embodied_lab.dataset_manifest.v1"
STATISTICS_SCHEMA_VERSION = "embodied_lab.normalization_statistics.v1"


def _write_json_atomic(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(
            payload,
            indent=2,
            sort_keys=True,
            ensure_ascii=False,
            allow_nan=False,
        )
        + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def _split_for_episode(
    episode_uuid: str,
    *,
    seed: str,
    train_fraction: float,
    validation_fraction: float,
) -> str:
    digest = hashlib.sha256(f"{seed}:{episode_uuid}".encode("utf-8")).digest()
    value = int.from_bytes(digest[:8], "big") / float(1 << 64)
    if value < train_fraction:
        return "train"
    if value < train_fraction + validation_fraction:
        return "validation"
    return "test"


def _metadata(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def build_dataset_manifest(
    dataset_root: str | Path,
    output: str | Path,
    *,
    seed: str = "embodied-lab-v1",
    train_fraction: float = 0.8,
    validation_fraction: float = 0.1,
    deep_validation: bool = True,
) -> Path:
    """Build an episode-level split manifest.

    The hash input is the episode UUID, so frames from one demonstration can
    never leak across splits. Invalid, aborted, timing-gapped, unlabeled, and
    duplicate/missing-UUID archives remain visible with split ``excluded``.
    A fast inventory never assigns a training split because it has not read
    and validated the payloads.
    """

    if not (
        0.0 < train_fraction < 1.0
        and 0.0 <= validation_fraction < 1.0
        and train_fraction + validation_fraction < 1.0
    ):
        raise ValueError(
            "fractions require 0 < train < 1, validation >= 0, and "
            "train + validation < 1"
        )
    root = Path(dataset_root).resolve()
    if not root.is_dir():
        raise FileNotFoundError(f"dataset root does not exist: {root}")
    entries: list[dict[str, Any]] = []
    for episode_dir in sorted(
        path
        for path in root.iterdir()
        if path.is_dir() and path.name.startswith("episode-")
    ):
        report = validate_episode(episode_dir, deep=deep_validation)
        metadata_path = episode_dir / "metadata.json"
        metadata: dict[str, Any] = {}
        if metadata_path.is_file():
            try:
                metadata = _metadata(metadata_path)
            except (OSError, ValueError, json.JSONDecodeError):
                pass
        episode_uuid = metadata.get("episode_uuid")
        eligible = bool(
            deep_validation
            and report["training_eligible"]
            and metadata.get("success_label") == "success"
        )
        validation_warnings = list(report.get("warnings", []))
        if not deep_validation:
            validation_warnings.append(
                "fast inventory did not validate payloads; training split is excluded"
            )
        elif report["training_eligible"] and metadata.get("success_label") == "failure":
            validation_warnings.append(
                "reviewed failure episode is excluded from behavior-cloning splits"
            )
        entries.append(
            {
                "episode_uuid": episode_uuid,
                "path": episode_dir.relative_to(root).as_posix(),
                "schema_version": metadata.get("schema_version"),
                "completion_status": metadata.get("completion_status"),
                "training_eligible": eligible,
                "split": "excluded",
                "task_name": metadata.get("task_name"),
                "object_id": metadata.get("object_id"),
                "operator": metadata.get("operator"),
                "success_label": metadata.get("success_label"),
                "simulation_only": metadata.get("simulation_only"),
                "sample_count": report.get("sample_count"),
                "duration_s": metadata.get("duration_s"),
                "dataset_fps": metadata.get("dataset_fps"),
                "canonical_missed_slot_count": report.get("quality", {}).get(
                    "canonical_missed_slot_count"
                ),
                "calibration_version": metadata.get(
                    "calibration_snapshot", {}
                ).get("version")
                if isinstance(metadata.get("calibration_snapshot"), dict)
                else None,
                "metadata_sha256": (
                    file_sha256(metadata_path) if metadata_path.is_file() else None
                ),
                "canonical_index_sha256": (
                    file_sha256(episode_dir / "canonical" / "samples.jsonl")
                    if (episode_dir / "canonical" / "samples.jsonl").is_file()
                    else None
                ),
                "validation_errors": report.get("errors", []),
                "validation_warnings": validation_warnings,
                "duplicate_or_missing_uuid": not isinstance(episode_uuid, str),
            }
        )
    uuid_counts = Counter(
        entry["episode_uuid"]
        for entry in entries
        if isinstance(entry["episode_uuid"], str)
    )
    for entry in entries:
        episode_uuid = entry["episode_uuid"]
        duplicate_or_missing = (
            not isinstance(episode_uuid, str) or uuid_counts[episode_uuid] != 1
        )
        entry["duplicate_or_missing_uuid"] = duplicate_or_missing
        if duplicate_or_missing:
            entry["training_eligible"] = False
            if isinstance(episode_uuid, str):
                entry["validation_warnings"].append(
                    "episode UUID occurs more than once in this dataset; "
                    "every occurrence is excluded"
                )
        elif entry["training_eligible"]:
            entry["split"] = _split_for_episode(
                episode_uuid,
                seed=seed,
                train_fraction=train_fraction,
                validation_fraction=validation_fraction,
            )
    counts = {
        name: sum(entry["split"] == name for entry in entries)
        for name in ("train", "validation", "test", "excluded")
    }
    payload = {
        "schema_version": MANIFEST_SCHEMA_VERSION,
        "canonical_episode_schema": SCHEMA_VERSION,
        "dataset_root": str(root),
        "deep_validation": deep_validation,
        "split_policy": {
            "unit": "episode",
            "hash": "sha256",
            "seed": seed,
            "train_fraction": train_fraction,
            "validation_fraction": validation_fraction,
            "test_fraction": 1.0 - train_fraction - validation_fraction,
            "warning": (
                "Small datasets may have an empty validation or test split; "
                "inspect counts before training."
            ),
        },
        "action_order": list(ACTION_ORDER),
        "observation_state_order": list(OBSERVATION_STATE_ORDER),
        "episode_count": len(entries),
        "split_counts": counts,
        "episodes": entries,
    }
    output_path = Path(output).resolve()
    _write_json_atomic(output_path, payload)
    return output_path


class _OnlineMoments:
    def __init__(self, width: int) -> None:
        self.count = 0
        self.mean = np.zeros(width, dtype=np.float64)
        self.m2 = np.zeros(width, dtype=np.float64)
        self.minimum = np.full(width, np.inf, dtype=np.float64)
        self.maximum = np.full(width, -np.inf, dtype=np.float64)

    def update(self, values: Iterable[float]) -> None:
        vector = np.asarray(tuple(values), dtype=np.float64)
        if vector.shape != self.mean.shape or not np.all(np.isfinite(vector)):
            raise ValueError("normalization input has invalid shape or non-finite value")
        self.count += 1
        delta = vector - self.mean
        self.mean += delta / self.count
        self.m2 += delta * (vector - self.mean)
        self.minimum = np.minimum(self.minimum, vector)
        self.maximum = np.maximum(self.maximum, vector)

    def payload(self) -> dict[str, Any]:
        if self.count == 0:
            raise ValueError("cannot compute statistics from zero samples")
        variance = self.m2 / self.count
        return {
            "count": self.count,
            "mean": self.mean.tolist(),
            "std": np.sqrt(np.maximum(variance, 0.0)).tolist(),
            "minimum": self.minimum.tolist(),
            "maximum": self.maximum.tolist(),
        }


def compute_train_statistics(
    manifest: str | Path,
    output: str | Path,
) -> Path:
    """Compute state/action moments from only the manifest's train episodes."""

    manifest_path = Path(manifest).resolve()
    payload = _metadata(manifest_path)
    if payload.get("schema_version") != MANIFEST_SCHEMA_VERSION:
        raise ValueError("unsupported dataset manifest schema")
    if payload.get("deep_validation") is not True:
        raise ValueError(
            "normalization statistics require a deep-validation manifest"
        )
    dataset_root_value = payload.get("dataset_root")
    if not isinstance(dataset_root_value, str):
        raise ValueError("dataset manifest has no valid dataset_root")
    dataset_root = Path(dataset_root_value).resolve()
    state = _OnlineMoments(len(OBSERVATION_STATE_ORDER))
    action = _OnlineMoments(len(ACTION_ORDER))
    episodes_used: list[str] = []
    for entry in payload.get("episodes", []):
        if not isinstance(entry, dict) or entry.get("split") != "train":
            continue
        if entry.get("training_eligible") is not True:
            raise ValueError("manifest train entry is not marked training eligible")
        relative_path = entry.get("path")
        if not isinstance(relative_path, str):
            raise ValueError("manifest train entry has no valid episode path")
        episode_dir = (dataset_root / relative_path).resolve()
        try:
            episode_dir.relative_to(dataset_root)
        except ValueError as exc:
            raise ValueError(
                f"manifest train episode escapes dataset root: {relative_path}"
            ) from exc
        for field, relative_file in (
            ("metadata_sha256", Path("metadata.json")),
            ("canonical_index_sha256", Path("canonical") / "samples.jsonl"),
        ):
            expected_hash = entry.get(field)
            source_path = episode_dir / relative_file
            if not isinstance(expected_hash, str) or not source_path.is_file():
                raise ValueError(
                    f"manifest train episode lacks {field}: {episode_dir}"
                )
            if file_sha256(source_path) != expected_hash:
                raise ValueError(
                    f"manifest train episode {field} changed: {episode_dir}"
                )
        report = validate_episode(episode_dir, deep=False)
        if not report["training_eligible"]:
            raise ValueError(
                f"manifest train episode is no longer eligible: {episode_dir}"
            )
        if report["episode_uuid"] != entry.get("episode_uuid"):
            raise ValueError(
                f"manifest train episode UUID changed: {episode_dir}"
            )
        rows, errors = load_canonical_rows(episode_dir)
        if errors:
            raise ValueError(f"cannot read {episode_dir}: {errors}")
        for row in rows:
            observation = row["observation"]["state"]
            state.update(
                observation["arm_q_measured"]
                + observation["arm_dq_measured"]
                + observation["tcp_pose"]
                + observation["hand"]
            )
            actions = row["action"]
            action.update(actions["arm_q_target"] + actions["hand_target"])
        episodes_used.append(str(entry["episode_uuid"]))
    statistics = {
        "schema_version": STATISTICS_SCHEMA_VERSION,
        "manifest": str(manifest_path),
        "manifest_sha256": file_sha256(manifest_path),
        "split": "train",
        "episodes_used": episodes_used,
        "observation_state_order": list(OBSERVATION_STATE_ORDER),
        "action_order": list(ACTION_ORDER),
        "observation_state": state.payload(),
        "action": action.payload(),
        "zero_std_policy": (
            "training adapters must replace zero standard deviations with 1 "
            "and record the affected fields"
        ),
    }
    output_path = Path(output).resolve()
    _write_json_atomic(output_path, statistics)
    return output_path
