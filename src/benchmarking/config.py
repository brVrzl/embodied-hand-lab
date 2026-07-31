"""Strict configuration for the bounded MuJoCo smoke benchmark."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
from typing import Any, Mapping

from embodiment_core.config import load_yaml


BENCHMARK_CONFIG_SCHEMA = "embodied_lab.mujoco_joint_reach_preshape.v1"
BENCHMARK_TASK = "mujoco_joint_reach_preshape_smoke"
HAND_ACTUATOR_ORDER = (
    "thumb_lateral",
    "thumb_close",
    "index",
    "middle",
    "ring",
    "pinky",
)


class BenchmarkConfigurationError(ValueError):
    """Raised when the benchmark configuration is incomplete or ambiguous."""


def _mapping(value: object, field: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise BenchmarkConfigurationError(f"{field} must be a mapping")
    return value


def _reject_unknown(
    values: Mapping[str, Any], allowed: set[str], field: str
) -> None:
    unknown = sorted(set(values) - allowed)
    if unknown:
        raise BenchmarkConfigurationError(
            f"{field} contains unsupported field(s): {', '.join(unknown)}"
        )


def _finite_positive(value: object, field: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise BenchmarkConfigurationError(f"{field} must be numeric") from exc
    if not math.isfinite(result) or result <= 0.0:
        raise BenchmarkConfigurationError(f"{field} must be finite and positive")
    return result


def _six_vector(
    value: object, field: str, *, non_negative: bool = False
) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != 6:
        raise BenchmarkConfigurationError(f"{field} must contain exactly six values")
    result = tuple(float(element) for element in value)
    if not all(math.isfinite(element) for element in result):
        raise BenchmarkConfigurationError(f"{field} must contain finite values")
    if non_negative and any(element < 0.0 for element in result):
        raise BenchmarkConfigurationError(f"{field} must be non-negative")
    return result


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _display_path(path: Path, repository_root: Path) -> str:
    try:
        return path.relative_to(repository_root).as_posix()
    except ValueError:
        return str(path)


@dataclass(frozen=True, slots=True)
class BenchmarkConfig:
    """Effective benchmark configuration after strict YAML validation."""

    benchmark_id: str
    seed: int
    replay_config_path: Path
    duration_s: float
    control_period_s: float
    arm_target_offset_rad: tuple[float, ...]
    arm_target_jitter_rad: tuple[float, ...]
    arm_success_tolerance_rad: float
    hand_target_rad: tuple[float, ...]
    hand_success_tolerance_rad: float
    source_path: Path
    source_sha256: str

    def __post_init__(self) -> None:
        if not self.benchmark_id.strip():
            raise BenchmarkConfigurationError("benchmark_id must not be empty")
        if isinstance(self.seed, bool) or not isinstance(self.seed, int):
            raise BenchmarkConfigurationError("seed must be an integer")
        if not 0 <= self.seed < 2**64:
            raise BenchmarkConfigurationError("seed must be in the uint64 range")
        if not self.replay_config_path.is_file():
            raise BenchmarkConfigurationError(
                f"simulation replay config does not exist: {self.replay_config_path}"
            )
        if self.step_count < 1:
            raise BenchmarkConfigurationError(
                "duration_s must cover at least one control period"
            )

    @classmethod
    def load(
        cls,
        path: str | Path,
        *,
        repository_root: str | Path,
    ) -> "BenchmarkConfig":
        root = Path(repository_root).resolve()
        source = Path(path).resolve()
        raw = load_yaml(source)
        _reject_unknown(
            raw,
            {"schema_version", "benchmark_id", "seed", "simulation", "task"},
            "benchmark config",
        )
        if raw.get("schema_version") != BENCHMARK_CONFIG_SCHEMA:
            raise BenchmarkConfigurationError(
                f"schema_version must be {BENCHMARK_CONFIG_SCHEMA!r}"
            )

        benchmark_id = raw.get("benchmark_id")
        if not isinstance(benchmark_id, str):
            raise BenchmarkConfigurationError("benchmark_id must be a string")
        seed = raw.get("seed")
        if isinstance(seed, bool) or not isinstance(seed, int):
            raise BenchmarkConfigurationError("seed must be an integer")

        simulation = _mapping(raw.get("simulation"), "simulation")
        _reject_unknown(simulation, {"replay_config_path"}, "simulation")
        replay_path_raw = simulation.get("replay_config_path")
        if not isinstance(replay_path_raw, str) or not replay_path_raw.strip():
            raise BenchmarkConfigurationError(
                "simulation.replay_config_path must be a non-empty string"
            )
        replay_path = Path(replay_path_raw)
        if not replay_path.is_absolute():
            replay_path = root / replay_path
        replay_path = replay_path.resolve()

        task = _mapping(raw.get("task"), "task")
        _reject_unknown(
            task,
            {"type", "duration_s", "control_period_s", "arm", "hand"},
            "task",
        )
        if task.get("type") != BENCHMARK_TASK:
            raise BenchmarkConfigurationError(
                f"task.type must be {BENCHMARK_TASK!r}"
            )
        duration_s = _finite_positive(task.get("duration_s"), "task.duration_s")
        control_period_s = _finite_positive(
            task.get("control_period_s"), "task.control_period_s"
        )
        raw_steps = duration_s / control_period_s
        if not math.isclose(raw_steps, round(raw_steps), rel_tol=0.0, abs_tol=1e-9):
            raise BenchmarkConfigurationError(
                "task.duration_s must be an integer multiple of control_period_s"
            )

        arm = _mapping(task.get("arm"), "task.arm")
        _reject_unknown(
            arm,
            {
                "target_offset_rad",
                "target_jitter_rad",
                "success_tolerance_rad",
            },
            "task.arm",
        )
        arm_offset = _six_vector(
            arm.get("target_offset_rad"), "task.arm.target_offset_rad"
        )
        arm_jitter = _six_vector(
            arm.get("target_jitter_rad"),
            "task.arm.target_jitter_rad",
            non_negative=True,
        )
        arm_tolerance = _finite_positive(
            arm.get("success_tolerance_rad"),
            "task.arm.success_tolerance_rad",
        )

        hand = _mapping(task.get("hand"), "task.hand")
        _reject_unknown(
            hand, {"target_rad", "success_tolerance_rad"}, "task.hand"
        )
        hand_target_mapping = _mapping(
            hand.get("target_rad"), "task.hand.target_rad"
        )
        if set(hand_target_mapping) != set(HAND_ACTUATOR_ORDER):
            missing = sorted(set(HAND_ACTUATOR_ORDER) - set(hand_target_mapping))
            extra = sorted(set(hand_target_mapping) - set(HAND_ACTUATOR_ORDER))
            details = []
            if missing:
                details.append("missing " + ", ".join(missing))
            if extra:
                details.append("unsupported " + ", ".join(extra))
            raise BenchmarkConfigurationError(
                "task.hand.target_rad must name the six simulated actuators"
                + (": " + "; ".join(details) if details else "")
            )
        hand_target = tuple(
            float(hand_target_mapping[name]) for name in HAND_ACTUATOR_ORDER
        )
        if not all(math.isfinite(value) for value in hand_target):
            raise BenchmarkConfigurationError(
                "task.hand.target_rad must contain finite values"
            )
        hand_tolerance = _finite_positive(
            hand.get("success_tolerance_rad"),
            "task.hand.success_tolerance_rad",
        )

        return cls(
            benchmark_id=benchmark_id,
            seed=seed,
            replay_config_path=replay_path,
            duration_s=duration_s,
            control_period_s=control_period_s,
            arm_target_offset_rad=arm_offset,
            arm_target_jitter_rad=arm_jitter,
            arm_success_tolerance_rad=arm_tolerance,
            hand_target_rad=hand_target,
            hand_success_tolerance_rad=hand_tolerance,
            source_path=source,
            source_sha256=_sha256(source),
        )

    @property
    def step_count(self) -> int:
        return int(round(self.duration_s / self.control_period_s))

    @property
    def hand_target_by_name(self) -> dict[str, float]:
        return dict(zip(HAND_ACTUATOR_ORDER, self.hand_target_rad, strict=True))

    def snapshot(self, *, repository_root: str | Path) -> dict[str, object]:
        root = Path(repository_root).resolve()
        return {
            "schema_version": BENCHMARK_CONFIG_SCHEMA,
            "benchmark_id": self.benchmark_id,
            "seed": self.seed,
            "source_path": _display_path(self.source_path, root),
            "source_sha256": self.source_sha256,
            "simulation": {
                "replay_config_path": _display_path(
                    self.replay_config_path, root
                ),
            },
            "task": {
                "type": BENCHMARK_TASK,
                "duration_s": self.duration_s,
                "control_period_s": self.control_period_s,
                "step_count": self.step_count,
                "arm": {
                    "target_offset_rad": list(self.arm_target_offset_rad),
                    "target_jitter_rad": list(self.arm_target_jitter_rad),
                    "success_tolerance_rad": self.arm_success_tolerance_rad,
                },
                "hand": {
                    "target_rad": self.hand_target_by_name,
                    "success_tolerance_rad": self.hand_success_tolerance_rad,
                },
            },
        }
