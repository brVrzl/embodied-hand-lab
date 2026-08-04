from __future__ import annotations

from dataclasses import FrozenInstanceError
import json
from pathlib import Path
import subprocess
import sys

import pytest

from training_infra import (
    DistributedContext,
    GlobalBatchConfig,
    write_rank_zero_json,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
SMOKE_TOOL = REPOSITORY_ROOT / "tools" / "distributed_smoke_test.py"
RANK_ENV_KEYS = (
    "LOCAL_RANK",
    "RANK",
    "WORLD_SIZE",
    "MASTER_ADDR",
    "MASTER_PORT",
)


def _single_process_environment(monkeypatch: pytest.MonkeyPatch) -> None:
    for name in RANK_ENV_KEYS:
        monkeypatch.delenv(name, raising=False)


def test_distributed_context_parses_complete_environment_and_is_immutable() -> None:
    context = DistributedContext.from_environ(
        {"LOCAL_RANK": "1", "RANK": "5", "WORLD_SIZE": "8"}
    )
    assert context.as_dict() == {
        "local_rank": 1,
        "rank": 5,
        "world_size": 8,
        "is_distributed": True,
    }
    assert not context.is_rank_zero
    with pytest.raises(FrozenInstanceError):
        context.rank = 0  # type: ignore[misc]


@pytest.mark.parametrize(
    ("environment", "message"),
    [
        ({"RANK": "0", "WORLD_SIZE": "1"}, "LOCAL_RANK"),
        (
            {"LOCAL_RANK": "zero", "RANK": "0", "WORLD_SIZE": "1"},
            "LOCAL_RANK must be an integer",
        ),
        (
            {"LOCAL_RANK": "0", "RANK": "2", "WORLD_SIZE": "2"},
            "must be smaller than world_size",
        ),
        (
            {"LOCAL_RANK": "0", "RANK": "0", "WORLD_SIZE": "0"},
            "world_size must be positive",
        ),
    ],
)
def test_distributed_context_rejects_ambiguous_or_invalid_environment(
    environment: dict[str, str], message: str
) -> None:
    with pytest.raises(ValueError, match=message):
        DistributedContext.from_environ(environment)


def test_global_batch_formula_names_every_scaling_factor() -> None:
    layout = GlobalBatchConfig(
        per_device_batch_size=8,
        gpu_count_per_node=4,
        node_count=2,
        gradient_accumulation_steps=3,
    )
    assert layout.world_size == 8
    assert layout.global_batch_size == 192

    with pytest.raises(ValueError, match="per_device_batch_size"):
        GlobalBatchConfig(per_device_batch_size=0, gpu_count_per_node=1)
    with pytest.raises(ValueError, match="gradient_accumulation_steps"):
        GlobalBatchConfig(
            per_device_batch_size=1,
            gpu_count_per_node=1,
            gradient_accumulation_steps=True,
        )


def test_rank_zero_json_is_atomic_and_nonzero_rank_has_no_side_effect(
    tmp_path: Path,
) -> None:
    output = tmp_path / "nested" / "result.json"
    nonzero = DistributedContext(local_rank=1, rank=1, world_size=2)
    assert write_rank_zero_json(output, {"not_serializable": object()}, nonzero) is None
    assert not output.parent.exists()

    rank_zero = DistributedContext(local_rank=0, rank=0, world_size=2)
    assert write_rank_zero_json(output, {"attempt": 1}, rank_zero) == output
    assert json.loads(output.read_text(encoding="utf-8")) == {"attempt": 1}
    assert write_rank_zero_json(output, {"attempt": 2}, rank_zero) == output
    assert json.loads(output.read_text(encoding="utf-8")) == {"attempt": 2}
    with pytest.raises(TypeError):
        write_rank_zero_json(output, {"not_serializable": object()}, rank_zero)
    assert json.loads(output.read_text(encoding="utf-8")) == {"attempt": 2}
    assert not list(output.parent.glob(f".{output.name}.*.tmp"))


def test_help_and_check_do_not_require_eager_torch_import(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _single_process_environment(monkeypatch)
    help_result = subprocess.run(
        [sys.executable, str(SMOKE_TOOL), "--help"],
        cwd=REPOSITORY_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    assert help_result.returncode == 0
    assert "--check" in help_result.stdout
    assert "--result-json" in help_result.stdout
    assert "DistributedSampler" in help_result.stdout

    check_result = subprocess.run(
        [sys.executable, str(SMOKE_TOOL), "--check"],
        cwd=REPOSITORY_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    assert check_result.returncode in {0, 1}
    report = json.loads(check_result.stdout)
    assert report["context"]["world_size"] == 1
    assert report["status"] in {"ready", "unavailable"}
    assert "Traceback" not in check_result.stderr
