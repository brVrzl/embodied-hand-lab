"""Small, framework-neutral contracts for future training entry points."""

from .distributed import (
    DistributedContext,
    GlobalBatchConfig,
    write_rank_zero_json,
)

__all__ = [
    "DistributedContext",
    "GlobalBatchConfig",
    "write_rank_zero_json",
]
