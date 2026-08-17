"""Small, dependency-free validation of a LeRobot ACT checkpoint contract."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any, Mapping


@dataclass(frozen=True)
class ActCheckpointContract:
    """The input/output contract needed by an ACT inference client.

    This deliberately reads only LeRobot's saved ``config.json``.  Loading the
    policy and its processors remains the responsibility of the pinned
    LeRobot runtime, but rollout code can fail before opening hardware when a
    checkpoint is incompatible.
    """

    checkpoint: Path
    input_features: dict[str, dict[str, Any]]
    image_keys: tuple[str, ...]
    state_key: str
    environment_state_key: str | None
    action_dim: int
    chunk_size: int

    @classmethod
    def from_checkpoint(cls, checkpoint: Path) -> "ActCheckpointContract":
        checkpoint = Path(checkpoint).resolve()
        config_path = checkpoint / "config.json"
        if not config_path.is_file():
            raise FileNotFoundError(f"ACT checkpoint config is missing: {config_path}")
        try:
            config = json.loads(config_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError as exc:
            raise ValueError(f"invalid ACT checkpoint config: {config_path}") from exc
        if config.get("type") not in (None, "act"):
            raise ValueError(f"checkpoint is not an ACT policy: type={config.get('type')!r}")
        raw_inputs = config.get("input_features")
        if not isinstance(raw_inputs, Mapping):
            raise ValueError("ACT checkpoint input_features must be a mapping")
        input_features: dict[str, dict[str, Any]] = {}
        for key, value in raw_inputs.items():
            if not isinstance(key, str) or not isinstance(value, Mapping):
                raise ValueError("malformed ACT input feature")
            shape = value.get("shape")
            if not isinstance(shape, list) or not all(isinstance(v, int) for v in shape):
                raise ValueError(f"ACT feature {key!r} has no integer shape")
            input_features[key] = dict(value)

        image_keys = tuple(
            key for key, feature in input_features.items()
            if str(feature.get("type", "")).upper() == "VISUAL"
            or key.startswith("observation.images.")
        )
        if len(image_keys) != 2:
            raise ValueError(
                "ACT rollout requires exactly two visual inputs; "
                f"checkpoint declares {list(image_keys)!r}"
            )
        wrist_keys = [key for key in image_keys if key.rsplit(".", 1)[-1].lower() == "wrist"]
        if len(wrist_keys) != 1:
            raise ValueError(
                "ACT rollout requires one image key ending in '.wrist'; "
                f"checkpoint declares {list(image_keys)!r}"
            )
        for key in image_keys:
            if input_features[key]["shape"] != [3, 240, 320]:
                raise ValueError(
                    f"unsupported live ACT image shape for {key}: "
                    f"{input_features[key]['shape']!r}; expected [3,240,320]"
                )

        state_candidates = [
            key for key, feature in input_features.items()
            if str(feature.get("type", "")).upper() == "STATE"
            or key == "observation.state"
        ]
        if state_candidates != ["observation.state"]:
            raise ValueError(
                "ACT rollout requires observation.state as its sole state input; "
                f"checkpoint declares {state_candidates!r}"
            )
        if input_features["observation.state"]["shape"] != [12]:
            raise ValueError(
                "physical bottle ACT requires observation.state shape [12], "
                f"got {input_features['observation.state']['shape']!r}"
            )

        environment_keys = [
            key for key, feature in input_features.items()
            if key == "observation.environment_state"
            or str(feature.get("type", "")).upper() == "ENVIRONMENT"
        ]
        if len(environment_keys) > 1:
            raise ValueError(f"multiple environment-state inputs are unsupported: {environment_keys}")
        environment_state_key = environment_keys[0] if environment_keys else None
        if environment_state_key is not None and input_features[environment_state_key]["shape"] != [6]:
            raise ValueError(
                "physical bottle environment state must have shape [6], "
                f"got {input_features[environment_state_key]['shape']!r}"
            )

        raw_outputs = config.get("output_features")
        if not isinstance(raw_outputs, Mapping) or not isinstance(raw_outputs.get("action"), Mapping):
            raise ValueError("ACT checkpoint must declare output_features.action")
        action_shape = raw_outputs["action"].get("shape")
        if action_shape != [12]:
            raise ValueError(f"physical bottle ACT requires action shape [12], got {action_shape!r}")
        chunk_size = config.get("chunk_size")
        if not isinstance(chunk_size, int) or chunk_size < 1:
            raise ValueError(f"invalid ACT chunk_size: {chunk_size!r}")
        return cls(
            checkpoint=checkpoint,
            input_features=input_features,
            image_keys=image_keys,
            state_key="observation.state",
            environment_state_key=environment_state_key,
            action_dim=12,
            chunk_size=chunk_size,
        )

    @property
    def requires_environment_state(self) -> bool:
        return self.environment_state_key is not None

    @property
    def workspace_image_key(self) -> str:
        return next(key for key in self.image_keys if key != self.wrist_image_key)

    @property
    def wrist_image_key(self) -> str:
        return next(key for key in self.image_keys if key.rsplit(".", 1)[-1].lower() == "wrist")

    def validate_observation(self, observation: Mapping[str, Any]) -> None:
        import numpy as np

        required = set(self.image_keys) | {self.state_key}
        if self.environment_state_key is not None:
            required.add(self.environment_state_key)
        missing = sorted(required - set(observation))
        if missing:
            raise ValueError(f"ACT observation is missing {missing}")
        for key in self.image_keys:
            value = observation[key]
            if not isinstance(value, np.ndarray) or tuple(value.shape) != (3, 240, 320):
                raise ValueError(f"{key} must be a CHW [3,240,320] ndarray")
            if value.dtype != np.float32 or not np.isfinite(value).all():
                raise ValueError(f"{key} must be finite float32")
        state = observation[self.state_key]
        if not isinstance(state, np.ndarray) or tuple(state.shape) != (12,):
            raise ValueError("observation.state must be [12]")
        if state.dtype != np.float32 or not np.isfinite(state).all():
            raise ValueError("observation.state must be finite float32")
        if self.environment_state_key is not None:
            environment = observation[self.environment_state_key]
            if not isinstance(environment, np.ndarray) or tuple(environment.shape) != (6,):
                raise ValueError("observation.environment_state must be [6]")
            if environment.dtype != np.float32 or not np.isfinite(environment).all():
                raise ValueError("observation.environment_state must be finite float32")

    def summary(self) -> dict[str, Any]:
        return {
            "checkpoint": str(self.checkpoint),
            "image_keys": list(self.image_keys),
            "workspace_image_key": self.workspace_image_key,
            "wrist_image_key": self.wrist_image_key,
            "state_key": self.state_key,
            "environment_state_key": self.environment_state_key,
            "action_dim": self.action_dim,
            "chunk_size": self.chunk_size,
            "input_shapes": {key: value["shape"] for key, value in self.input_features.items()},
        }
