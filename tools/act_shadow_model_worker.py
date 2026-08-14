#!/usr/bin/env python3
"""Command-disabled LeRobot ACT inference worker over a local Unix socket."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import pickle
import socket
import struct
import time
from typing import Any

import numpy as np

from embodiment_core.act_contract import ActCheckpointContract


WORKSPACE_KEY = "observation.images.workspace"
WRIST_KEY = "observation.images.wrist"
STATE_KEY = "observation.state"
ENVIRONMENT_STATE_KEY = "observation.environment_state"


def _recv_exact(connection: socket.socket, size: int) -> bytes:
    chunks = bytearray()
    while len(chunks) < size:
        chunk = connection.recv(size - len(chunks))
        if not chunk:
            raise EOFError("shadow client disconnected")
        chunks.extend(chunk)
    return bytes(chunks)


def _receive(connection: socket.socket) -> Any:
    size = struct.unpack("!Q", _recv_exact(connection, 8))[0]
    if size > 16 * 1024 * 1024:
        raise ValueError("shadow request exceeds 16 MiB")
    return pickle.loads(_recv_exact(connection, size))


def _send(connection: socket.socket, value: Any) -> None:
    payload = pickle.dumps(value, protocol=5)
    connection.sendall(struct.pack("!Q", len(payload)) + payload)


def _validate_observation(
    request: dict[str, Any], *, requires_environment_state: bool,
    contract: ActCheckpointContract | None = None,
) -> None:
    if contract is not None:
        contract.validate_observation(request)
        return
    for key in (WORKSPACE_KEY, WRIST_KEY):
        value = request[key]
        if not isinstance(value, np.ndarray) or value.shape != (3, 240, 320):
            raise ValueError(f"{key} must be a CHW [3,240,320] ndarray")
        if value.dtype != np.float32 or not np.isfinite(value).all():
            raise ValueError(f"{key} must be finite float32")
    state = request[STATE_KEY]
    if not isinstance(state, np.ndarray) or state.shape != (12,):
        raise ValueError("observation.state must be [12]")
    if state.dtype != np.float32 or not np.isfinite(state).all():
        raise ValueError("observation.state must be finite float32")
    if requires_environment_state:
        environment_state = request.get(ENVIRONMENT_STATE_KEY)
        if not isinstance(environment_state, np.ndarray) or environment_state.shape != (6,):
            raise ValueError("observation.environment_state must be [6]")
        if environment_state.dtype != np.float32 or not np.isfinite(environment_state).all():
            raise ValueError("observation.environment_state must be finite float32")
    elif ENVIRONMENT_STATE_KEY in request:
        raise ValueError("standard ACT checkpoint must not receive environment state")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--socket", type=Path, required=True)
    parser.add_argument("--ready-file", type=Path, required=True)
    parser.add_argument("--summary", type=Path, required=True)
    args = parser.parse_args()

    import torch
    from lerobot.configs import PreTrainedConfig
    from lerobot.policies.act.modeling_act import ACTPolicy
    from lerobot.policies.factory import make_pre_post_processors

    checkpoint = args.checkpoint.resolve()
    contract = ActCheckpointContract.from_checkpoint(checkpoint)
    config = PreTrainedConfig.from_pretrained(checkpoint)
    requires_environment_state = contract.requires_environment_state
    if not torch.cuda.is_available():
        raise RuntimeError("Thor CUDA is required for the shadow benchmark")
    config.device = "cuda"
    policy = ACTPolicy.from_pretrained(checkpoint, config=config).to(config.device)
    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=config,
        pretrained_path=str(checkpoint),
        preprocessor_overrides={"device_processor": {"device": config.device}},
    )
    policy.eval()
    torch.manual_seed(0)
    torch.cuda.reset_peak_memory_stats()

    args.socket.parent.mkdir(parents=True, exist_ok=True)
    args.ready_file.parent.mkdir(parents=True, exist_ok=True)
    args.summary.parent.mkdir(parents=True, exist_ok=True)
    if args.socket.exists():
        args.socket.unlink()
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(str(args.socket))
    server.listen(1)
    args.ready_file.write_text("ready\n", encoding="utf-8")
    query_count = 0
    inference_failures = 0
    started = time.monotonic()
    try:
        connection, _ = server.accept()
        with connection, torch.inference_mode():
            while True:
                request = _receive(connection)
                if request == {"command": "stop"}:
                    _send(connection, {"stopped": True})
                    break
                request_started_ns = time.perf_counter_ns()
                try:
                    _validate_observation(
                        request,
                        requires_environment_state=requires_environment_state,
                        contract=contract,
                    )
                    batch = {
                        key: torch.from_numpy(request[key])
                        for key in (*contract.image_keys, contract.state_key)
                    }
                    if contract.environment_state_key is not None:
                        batch[contract.environment_state_key] = torch.from_numpy(
                            request[contract.environment_state_key]
                        )
                    torch.cuda.synchronize()
                    preprocessing_started_ns = time.perf_counter_ns()
                    processed = preprocessor(batch)
                    torch.cuda.synchronize()
                    preprocessing_ended_ns = time.perf_counter_ns()
                    prediction = policy.predict_action_chunk(processed)
                    torch.cuda.synchronize()
                    inference_ended_ns = time.perf_counter_ns()
                    native = postprocessor(prediction)
                    torch.cuda.synchronize()
                    postprocessing_ended_ns = time.perf_counter_ns()
                    output = native.detach().cpu().numpy()
                    if output.shape == (1, contract.chunk_size, contract.action_dim):
                        output = output[0]
                    if output.shape != (contract.chunk_size, contract.action_dim):
                        raise ValueError(f"unexpected ACT output shape {output.shape}")
                    if not np.isfinite(output).all():
                        raise ValueError("ACT output contains a non-finite value")
                    query_count += 1
                    _send(
                        connection,
                        {
                            "prediction": output.astype(np.float32, copy=False),
                            "timing_ms": {
                                "checkpoint_preprocessing": (
                                    preprocessing_ended_ns - preprocessing_started_ns
                                )
                                / 1e6,
                                "act_inference": (
                                    inference_ended_ns - preprocessing_ended_ns
                                )
                                / 1e6,
                                "checkpoint_postprocessing": (
                                    postprocessing_ended_ns - inference_ended_ns
                                )
                                / 1e6,
                                "worker_total": (
                                    postprocessing_ended_ns - request_started_ns
                                )
                                / 1e6,
                            },
                        },
                    )
                except BaseException as exc:
                    inference_failures += 1
                    _send(connection, {"error": f"{type(exc).__name__}: {exc}"})
    finally:
        server.close()
        if args.socket.exists():
            args.socket.unlink()
        summary = {
            "schema_version": "act_shadow_model_worker.v1",
            "checkpoint": str(checkpoint),
            "checkpoint_contract": contract.summary(),
            "lerobot_version": __import__("lerobot").__version__,
            "torch_version": torch.__version__,
            "cuda_device": torch.cuda.get_device_name(0),
            "chunk_size": int(config.chunk_size),
            "n_action_steps": int(config.n_action_steps),
            "query_count_including_warmup_and_determinism": query_count,
            "inference_failures": inference_failures,
            "elapsed_sec": time.monotonic() - started,
            "gpu_peak_memory_allocated_bytes": torch.cuda.max_memory_allocated(),
            "gpu_peak_memory_reserved_bytes": torch.cuda.max_memory_reserved(),
            "normalization_loaded_from_checkpoint": True,
            "command_api_present": False,
            "environment_state_input": requires_environment_state,
            "pid": os.getpid(),
        }
        args.summary.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
