#!/usr/bin/env python3
"""Inference-only π0.5-DROID websocket client with no physical command path."""

from __future__ import annotations

import argparse
import dataclasses
import datetime as dt
import json
import pathlib
import time
from typing import Any

import cv2
import numpy as np
from openpi_client import image_tools
from openpi_client import websocket_client_policy

from camera_probe import SCENE_CAMERA, WRIST_CAMERA, capture_synchronized, save_probe


OPENPI_COMMIT = "15a9616a00943ada6c20a0f158e3adb39df2ccac"
MODEL_CONFIG = "pi05_droid"
CHECKPOINT = "gs://openpi-assets/checkpoints/pi05_droid"
EXPECTED_ACTION_SHAPE = (15, 8)
DEFAULT_PROMPTS = (
    "pick up the water bottle",
    "move toward the water bottle",
    "place the water bottle in the tray",
)


@dataclasses.dataclass(frozen=True)
class DroidState:
    joint_position: np.ndarray
    gripper_position: np.ndarray
    source: str
    source_timestamp_ns: int | None


def load_droid_state(path: pathlib.Path | None, *, synthetic: bool) -> DroidState:
    if synthetic:
        if path is not None:
            raise ValueError("Use either --state-json or --synthetic-state, not both")
        return DroidState(
            joint_position=np.zeros(7, dtype=np.float32),
            gripper_position=np.zeros(1, dtype=np.float32),
            source="explicit_synthetic_zero_state",
            source_timestamp_ns=None,
        )
    if path is None:
        raise ValueError("--state-json is required unless --synthetic-state is explicit")
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("schema") != "openpi.pi05_droid_state.v1":
        raise ValueError("state JSON schema must be openpi.pi05_droid_state.v1")
    joints = np.asarray(payload["joint_position"], dtype=np.float32)
    gripper = np.atleast_1d(np.asarray(payload["gripper_position"], dtype=np.float32))
    if joints.shape != (7,):
        raise ValueError(
            f"π0.5-DROID requires seven Franka joint positions; received {joints.shape}. "
            "JAKA mini2 has six joints and must not be silently padded or remapped."
        )
    if gripper.shape != (1,):
        raise ValueError(f"π0.5-DROID requires one gripper position; received {gripper.shape}")
    if not np.all(np.isfinite(joints)) or not np.all(np.isfinite(gripper)):
        raise ValueError("State contains non-finite values")
    return DroidState(
        joint_position=joints,
        gripper_position=gripper,
        source=str(payload.get("source", path.resolve())),
        source_timestamp_ns=payload.get("timestamp_ns"),
    )


def build_observation(
    scene_bgr: np.ndarray,
    wrist_bgr: np.ndarray,
    state: DroidState,
    prompt: str,
) -> dict[str, Any]:
    # OpenCV supplies BGR; OpenPI DROID expects HWC uint8 RGB. Rotation is
    # already applied by camera_probe according to the verified installation.
    scene_rgb = cv2.cvtColor(scene_bgr, cv2.COLOR_BGR2RGB)
    wrist_rgb = cv2.cvtColor(wrist_bgr, cv2.COLOR_BGR2RGB)
    return {
        "observation/exterior_image_1_left": image_tools.resize_with_pad(scene_rgb, 224, 224),
        "observation/wrist_image_left": image_tools.resize_with_pad(wrist_rgb, 224, 224),
        "observation/joint_position": state.joint_position.copy(),
        "observation/gripper_position": state.gripper_position.copy(),
        "prompt": prompt,
    }


def validate_actions(actions: Any) -> np.ndarray:
    array = np.asarray(actions)
    if array.shape != EXPECTED_ACTION_SHAPE:
        raise ValueError(f"Expected action chunk {EXPECTED_ACTION_SHAPE}, received {array.shape}")
    if not np.issubdtype(array.dtype, np.number) or not np.all(np.isfinite(array)):
        raise ValueError("Predicted action chunk must be finite and numeric")
    return array


def _jsonable_observation_summary(observation: dict[str, Any], state: DroidState) -> dict[str, Any]:
    return {
        "keys": sorted(observation),
        "scene_image_shape": list(observation["observation/exterior_image_1_left"].shape),
        "scene_image_dtype": str(observation["observation/exterior_image_1_left"].dtype),
        "wrist_image_shape": list(observation["observation/wrist_image_left"].shape),
        "wrist_image_dtype": str(observation["observation/wrist_image_left"].dtype),
        "joint_position_shape": list(observation["observation/joint_position"].shape),
        "gripper_position_shape": list(observation["observation/gripper_position"].shape),
        "state_source": state.source,
        "state_timestamp_ns": state.source_timestamp_ns,
    }


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--state-json", type=pathlib.Path)
    parser.add_argument("--synthetic-state", action="store_true")
    parser.add_argument("--no-query", action="store_true", help="Construct and validate observations only")
    parser.add_argument("--prompt", action="append", dest="prompts")
    parser.add_argument("--capture-duration-s", type=float, default=1.5)
    parser.add_argument("--output-dir", type=pathlib.Path)
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    state = load_droid_state(args.state_json, synthetic=args.synthetic_state)
    prompts = tuple(args.prompts or DEFAULT_PROMPTS)
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = args.output_dir or pathlib.Path(__file__).parent / "artifacts" / f"shadow_{stamp}"
    camera_report, frames = capture_synchronized(duration_s=args.capture_duration_s)
    save_probe(output_dir, camera_report, frames)

    observations = [build_observation(frames["scene"], frames["wrist"], state, prompt) for prompt in prompts]
    run_metadata = {
        "schema": "embodied_lab.pi05_shadow_run.v1",
        "openpi_commit": OPENPI_COMMIT,
        "model_config": MODEL_CONFIG,
        "checkpoint": CHECKPOINT,
        "mode": "observation_only" if args.no_query else "websocket_shadow_inference",
        "physical_execution": False,
        "scene_camera": dataclasses.asdict(SCENE_CAMERA),
        "wrist_camera": dataclasses.asdict(WRIST_CAMERA),
        "observation": _jsonable_observation_summary(observations[0], state),
        "expected_action_shape": list(EXPECTED_ACTION_SHAPE),
        "action_semantics": "7 DROID/Franka joint velocities + 1 gripper position; never mapped to JAKA/RH56",
        "prompts": list(prompts),
    }
    (output_dir / "run_metadata.json").write_text(json.dumps(run_metadata, indent=2) + "\n", encoding="utf-8")

    if args.no_query:
        print(json.dumps({**run_metadata, "output_dir": str(output_dir.resolve())}, indent=2))
        return

    client = websocket_client_policy.WebsocketClientPolicy(args.host, args.port)
    log_path = output_dir / "predictions.jsonl"
    with log_path.open("x", encoding="utf-8") as log_file:
        for prompt, observation in zip(prompts, observations, strict=True):
            started_ns = time.monotonic_ns()
            response = client.infer(observation)
            ended_ns = time.monotonic_ns()
            actions = validate_actions(response["actions"])
            record = {
                "schema": "embodied_lab.pi05_shadow_prediction.v1",
                "prompt": prompt,
                "request_started_monotonic_ns": started_ns,
                "response_received_monotonic_ns": ended_ns,
                "round_trip_ms": (ended_ns - started_ns) / 1_000_000,
                "action_shape": list(actions.shape),
                "action_dtype": str(actions.dtype),
                "action_min": float(actions.min()),
                "action_max": float(actions.max()),
                "actions": actions.tolist(),
                "physical_execution": False,
            }
            log_file.write(json.dumps(record) + "\n")
            log_file.flush()
            print(json.dumps({key: value for key, value in record.items() if key != "actions"}))


if __name__ == "__main__":
    main()
