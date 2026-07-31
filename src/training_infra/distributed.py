"""Pure-Python distributed configuration and rank-zero output helpers."""

from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
import tempfile
from typing import Mapping


_DISTRIBUTED_ENV_KEYS = ("LOCAL_RANK", "RANK", "WORLD_SIZE")


def _non_negative_int(name: str, raw_value: str) -> int:
    try:
        value = int(raw_value)
    except ValueError as exc:
        raise ValueError(f"{name} must be an integer, got {raw_value!r}") from exc
    if value < 0:
        raise ValueError(f"{name} must be non-negative, got {value}")
    return value


def _positive_int(name: str, value: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{name} must be a positive integer, got {value!r}")
    return value


@dataclass(frozen=True)
class DistributedContext:
    """Immutable identity supplied by ``torchrun`` or a single-process default."""

    local_rank: int
    rank: int
    world_size: int

    def __post_init__(self) -> None:
        for name, value in (
            ("local_rank", self.local_rank),
            ("rank", self.rank),
            ("world_size", self.world_size),
        ):
            if isinstance(value, bool) or not isinstance(value, int):
                raise ValueError(f"{name} must be an integer, got {value!r}")
        if self.local_rank < 0:
            raise ValueError(f"local_rank must be non-negative, got {self.local_rank}")
        if self.rank < 0:
            raise ValueError(f"rank must be non-negative, got {self.rank}")
        if self.world_size < 1:
            raise ValueError(f"world_size must be positive, got {self.world_size}")
        if self.rank >= self.world_size:
            raise ValueError(
                f"rank {self.rank} must be smaller than world_size {self.world_size}"
            )
        if self.local_rank >= self.world_size:
            raise ValueError(
                "local_rank cannot be greater than or equal to world_size: "
                f"{self.local_rank} >= {self.world_size}"
            )

    @classmethod
    def from_environ(
        cls, environ: Mapping[str, str] | None = None
    ) -> "DistributedContext":
        """Parse the complete torchrun rank triplet or use the local default.

        A partially populated environment is rejected because silently assuming
        one of the ranks can make multiple processes write the same artifact.
        """

        source = os.environ if environ is None else environ
        present = [key for key in _DISTRIBUTED_ENV_KEYS if key in source]
        if not present:
            return cls(local_rank=0, rank=0, world_size=1)
        missing = [key for key in _DISTRIBUTED_ENV_KEYS if key not in source]
        if missing:
            raise ValueError(
                "incomplete distributed environment: missing "
                + ", ".join(missing)
                + "; set LOCAL_RANK, RANK, and WORLD_SIZE together"
            )
        return cls(
            local_rank=_non_negative_int("LOCAL_RANK", source["LOCAL_RANK"]),
            rank=_non_negative_int("RANK", source["RANK"]),
            world_size=_non_negative_int("WORLD_SIZE", source["WORLD_SIZE"]),
        )

    @property
    def is_distributed(self) -> bool:
        return self.world_size > 1

    @property
    def is_rank_zero(self) -> bool:
        return self.rank == 0

    def as_dict(self) -> dict[str, int | bool]:
        return {
            "local_rank": self.local_rank,
            "rank": self.rank,
            "world_size": self.world_size,
            "is_distributed": self.is_distributed,
        }


@dataclass(frozen=True)
class GlobalBatchConfig:
    """Explicit global-batch calculation for data-parallel training.

    ``global_batch_size = per_device_batch_size * gpu_count_per_node
    * node_count * gradient_accumulation_steps``.
    """

    per_device_batch_size: int
    gpu_count_per_node: int
    node_count: int = 1
    gradient_accumulation_steps: int = 1

    def __post_init__(self) -> None:
        for name in (
            "per_device_batch_size",
            "gpu_count_per_node",
            "node_count",
            "gradient_accumulation_steps",
        ):
            _positive_int(name, getattr(self, name))

    @property
    def world_size(self) -> int:
        return self.gpu_count_per_node * self.node_count

    @property
    def global_batch_size(self) -> int:
        return (
            self.per_device_batch_size
            * self.gpu_count_per_node
            * self.node_count
            * self.gradient_accumulation_steps
        )


def write_rank_zero_json(
    output_path: Path | str,
    payload: Mapping[str, object],
    context: DistributedContext,
) -> Path | None:
    """Atomically replace a JSON result from rank zero only.

    Non-zero ranks return before creating the output directory or serializing
    the payload. The temporary file is written in the destination directory so
    ``os.replace`` remains an atomic same-filesystem operation.
    """

    if not context.is_rank_zero:
        return None

    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary_path = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, path)
    except BaseException:
        temporary_path.unlink(missing_ok=True)
        raise
    return path
