from __future__ import annotations

import json
from pathlib import Path

import pytest

from benchmarking import (
    write_benchmark_result,
)
from benchmarking.cli import build_parser


ROOT = Path(__file__).resolve().parents[1]


def test_cli_has_repository_local_default_output() -> None:
    args = build_parser().parse_args([])
    assert args.output == ROOT / "build" / "validation" / "benchmark.json"


def test_result_writer_is_atomic_and_failed_serialization_preserves_old_result(
    tmp_path: Path,
) -> None:
    output = tmp_path / "results" / "smoke.json"
    assert write_benchmark_result(output, {"status": "passed"}) == output
    with pytest.raises(TypeError):
        write_benchmark_result(
            output, {"status": "passed", "invalid": object()}
        )
    assert json.loads(output.read_text(encoding="utf-8")) == {
        "status": "passed"
    }
    assert not list(output.parent.glob(f".{output.name}.*.tmp"))
