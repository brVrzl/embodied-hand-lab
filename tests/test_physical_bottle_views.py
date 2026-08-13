from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest
import yaml

from episode_dataset.physical_bottle_materialization import _validate_split_groups
from episode_dataset.training_materialization import materialize_training_dataset
from episode_dataset.training_views import ActDatasetAdapter, ActForceDatasetAdapter

from test_training_materialization import _config, _synthetic_staging


def test_act_force_keeps_invalid_force_rows_matched(tmp_path: Path) -> None:
    source = tmp_path / "raw"
    row_path, _ = _synthetic_staging(source)
    rows = [json.loads(line) for line in row_path.read_text(encoding="utf-8").splitlines()]
    timing = rows[1]["timing"]
    timing["rh56_force_act_valid"] = False
    timing["source_validity"]["rh56_force_act"] = False
    rows[1]["timing"] = timing
    row_path.write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in rows),
        encoding="utf-8",
    )
    result = materialize_training_dataset(_config(tmp_path, source, tmp_path / "training"))
    act = ActDatasetAdapter(result.output_root, action_horizon=4)
    force = ActForceDatasetAdapter(result.output_root, action_horizon=4)

    assert len(act) == len(force) == 3
    force_rows = [force[index] for index in range(len(force))]
    assert any(sample["observation"]["force_valid"] is False for sample in force_rows)


def test_nominal16_manifest_keeps_corrected_demonstrations_separate() -> None:
    config = yaml.safe_load(
        (Path(__file__).parents[1] / "configs/training/physical_bottle_v2_nominal16.yaml").read_text(
            encoding="utf-8"
        )
    )
    segments = {value["id"]: value for value in config["segments"]}

    assert len([value for value in segments.values() if value.get("include")]) == 16
    assert (segments["ep099_a"]["start_frame"], segments["ep099_a"]["end_frame"]) == (0, 965)
    assert (segments["ep099_b"]["start_frame"], segments["ep099_b"]["end_frame"]) == (1335, 2348)
    assert (segments["ep102_a"]["start_frame"], segments["ep102_a"]["end_frame"]) == (0, 877)
    assert (segments["ep102_b"]["start_frame"], segments["ep102_b"]["end_frame"]) == (1209, 1934)
    assert segments["ep099_a"]["end_frame"] < segments["ep099_b"]["start_frame"]
    assert segments["ep102_a"]["end_frame"] < segments["ep102_b"]["start_frame"]
    assert segments["ep102_a"]["include"] is False
    assert segments["ep102_b"]["include"] is True

    split_for = {
        segment: split
        for split, values in config["splits"].items()
        for segment in values
    }
    assert split_for["ep099_a"] == split_for["ep099_b"] == "train"


def test_corrected_merged_episode_chunks_cannot_cross_reset_interval(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def row(source_frame: int, action_value: float) -> dict:
        return {
            "index": source_frame,
            "segment_id": 0,
            "raw_frame_index": source_frame,
            "frame_index": source_frame,
            "timestamp_ns": source_frame * 33_333_333,
            "observation.state": [0.0] * 12,
            "action": [action_value] * 12,
            "task": "test",
        }

    first = [row(964, 0.1), row(965, 0.2)]
    second = [row(1335, 0.8), row(1336, 0.9)]
    dataset = ActDatasetAdapter.__new__(ActDatasetAdapter)
    dataset.action_horizon = 16
    dataset.force_enabled = False
    dataset._episodes = {0: first, 1: second}
    ref_type = type("Ref", (), {})
    dataset._refs = []
    for episode_index, rows in dataset._episodes.items():
        for value in rows:
            ref = ref_type()
            ref.episode_index = episode_index
            ref.row = value
            ref.video_root = Path(".")
            dataset._refs.append(ref)
    monkeypatch.setattr(
        ActDatasetAdapter,
        "_read_image",
        lambda *_args: np.zeros((3, 1, 1), dtype=np.uint8),
    )

    last_first = dataset[1]
    first_second = dataset[2]
    assert last_first["action_mask"].tolist() == [True] + [False] * 15
    assert last_first["action"][0, 0] == pytest.approx(0.2)
    assert first_second["action"][0, 0] == pytest.approx(0.8)


def test_split_group_contract_rejects_session_leakage() -> None:
    config = {
        "split_groups": [
            {"id": "source_99", "segments": ["ep099_a", "ep099_b"]},
        ]
    }
    indices = {"ep099_a": 0, "ep099_b": 1}
    with pytest.raises(ValueError, match="leaks across splits"):
        _validate_split_groups(
            config,
            segment_to_index=indices,
            split_names_by_segment={"ep099_a": "train", "ep099_b": "val"},
        )


def test_nominal33_manifest_adds_only_reviewed_task_trimmed_trajectories() -> None:
    config = yaml.safe_load(
        (Path(__file__).parents[1] / "configs/training/physical_bottle_v3_nominal33.yaml").read_text(
            encoding="utf-8"
        )
    )
    segments = {value["id"]: value for value in config["segments"]}
    included = {name for name, value in segments.items() if value.get("include")}
    new_included = {
        "ep121", "ep131", "ep133", "ep134", "ep137", "ep138", "ep139",
        "ep140", "ep141", "ep142", "ep143", "ep144", "ep145", "ep146",
        "ep147", "ep148", "ep149",
    }

    assert len(included) == 33
    assert new_included <= included
    assert not ({f"ep{value}" for value in range(122, 130)} | {"ep130", "ep132", "ep135", "ep136"}) & included
    assert segments["ep130"]["reason"] == "missing_complete_payload"
    assert segments["ep135"]["classification"] == "REVIEW_REQUIRED"
    assert segments["ep136"]["classification"] == "REVIEW_REQUIRED"
    assert (segments["ep121"]["start_frame"], segments["ep121"]["end_frame"]) == (60, 672)
    assert (segments["ep149"]["start_frame"], segments["ep149"]["end_frame"]) == (1, 460)

    split_for = {
        segment: split
        for split, values in config["splits"].items()
        for segment in values
    }
    for group in config["split_groups"]:
        assert len({split_for[segment] for segment in group["segments"]}) == 1
    assert {split_for[f"ep{value}"] for value in range(138, 143)} == {"val"}
