"""Lazy-PyTorch distributed communication smoke test implementation."""

from __future__ import annotations

from contextlib import contextmanager
from datetime import timedelta
import json
import os
from pathlib import Path
import socket
import tempfile
from types import ModuleType
from typing import Iterator, Literal

from .distributed import DistributedContext, write_rank_zero_json


DeviceChoice = Literal["auto", "cpu", "cuda"]
BackendChoice = Literal["auto", "gloo", "nccl"]


class TorchUnavailableError(RuntimeError):
    """Raised when the optional PyTorch distributed runtime cannot be used."""


def _load_torch() -> ModuleType:
    try:
        import torch
    except Exception as exc:
        raise TorchUnavailableError(
            "PyTorch could not be imported. Install a training environment with "
            f"a compatible torch build before running the smoke test "
            f"({type(exc).__name__}: {exc})."
        ) from exc
    return torch


def probe_torch(context: DistributedContext) -> dict[str, object]:
    """Return an import-safe capability report without initializing a process group."""

    report: dict[str, object] = {
        "context": context.as_dict(),
        "torch_importable": False,
        "distributed_available": False,
        "gloo_available": False,
        "nccl_available": False,
        "cuda_available": False,
        "cuda_device_count": 0,
    }
    try:
        torch = _load_torch()
    except TorchUnavailableError as exc:
        report["status"] = "unavailable"
        report["reason"] = str(exc)
        return report

    distributed = getattr(torch, "distributed", None)
    distributed_available = bool(
        distributed is not None and distributed.is_available()
    )
    cuda_available = bool(torch.cuda.is_available())
    gloo_available = bool(
        distributed_available
        and getattr(distributed, "is_gloo_available", lambda: False)()
    )
    nccl_available = bool(
        distributed_available
        and getattr(distributed, "is_nccl_available", lambda: False)()
    )
    runnable = gloo_available or (cuda_available and nccl_available)
    report.update(
        {
            "status": "ready" if runnable else "unavailable",
            "torch_importable": True,
            "torch_version": str(torch.__version__),
            "torch_cuda_version": getattr(torch.version, "cuda", None),
            "distributed_available": distributed_available,
            "gloo_available": gloo_available,
            "nccl_available": nccl_available,
            "cuda_available": cuda_available,
            "cuda_device_count": int(torch.cuda.device_count())
            if cuda_available
            else 0,
        }
    )
    if not distributed_available:
        report["reason"] = "This PyTorch build has no distributed support."
    elif not runnable:
        report["reason"] = (
            "No runnable backend was found: Gloo is unavailable and no usable "
            "CUDA/NCCL combination is present."
        )
    return report


def _select_runtime(
    torch: ModuleType,
    context: DistributedContext,
    *,
    device_choice: DeviceChoice,
    backend_choice: BackendChoice,
) -> tuple[str, str, int]:
    distributed = torch.distributed
    if not distributed.is_available():
        raise TorchUnavailableError("This PyTorch build has no distributed support.")

    if device_choice == "auto":
        use_cuda = (
            backend_choice != "gloo"
            and torch.cuda.is_available()
            and distributed.is_nccl_available()
        )
        device_type = "cuda" if use_cuda else "cpu"
    else:
        device_type = device_choice

    if backend_choice == "auto":
        backend = "nccl" if device_type == "cuda" else "gloo"
    else:
        backend = backend_choice

    if device_type == "cuda":
        if not torch.cuda.is_available():
            raise TorchUnavailableError(
                "CUDA was requested, but torch.cuda.is_available() is false."
            )
        device_count = int(torch.cuda.device_count())
        if context.local_rank >= device_count:
            raise ValueError(
                f"LOCAL_RANK={context.local_rank} has no visible CUDA device; "
                f"torch sees {device_count} device(s). Check CUDA_VISIBLE_DEVICES."
            )
        if backend != "nccl":
            raise ValueError(
                "The smoke test supports CUDA with NCCL only; use "
                "--device cpu --backend gloo for a CPU communication check."
            )
        if not distributed.is_nccl_available():
            raise TorchUnavailableError(
                "NCCL was requested, but this PyTorch build has no NCCL support."
            )
        torch.cuda.set_device(context.local_rank)
        return device_type, backend, context.local_rank

    if backend != "gloo":
        raise ValueError("NCCL requires CUDA; use --backend gloo with --device cpu.")
    if not distributed.is_gloo_available():
        raise TorchUnavailableError(
            "Gloo was requested, but this PyTorch build has no Gloo support."
        )
    return device_type, backend, -1


@contextmanager
def _rendezvous(context: DistributedContext) -> Iterator[str]:
    master_address = os.environ.get("MASTER_ADDR")
    master_port = os.environ.get("MASTER_PORT")
    if bool(master_address) != bool(master_port):
        raise ValueError("MASTER_ADDR and MASTER_PORT must be set together.")
    if master_address and master_port:
        yield "env://"
        return
    if context.world_size != 1:
        raise ValueError(
            "Multi-process launch requires torchrun rendezvous variables "
            "MASTER_ADDR and MASTER_PORT."
        )
    with tempfile.TemporaryDirectory(prefix="embodied-lab-dist-") as directory:
        yield (Path(directory) / "rendezvous").resolve().as_uri()


@contextmanager
def _single_process_gloo_interface(
    context: DistributedContext, backend: str
) -> Iterator[str | None]:
    """Avoid hostname lookup for a local-only Gloo diagnostic.

    Multi-process jobs retain the operator-selected interface because choosing
    loopback there would silently break multi-node communication.
    """

    configured = os.environ.get("GLOO_SOCKET_IFNAME")
    if configured or backend != "gloo" or context.world_size != 1:
        yield configured
        return

    interface_names = {name for _, name in socket.if_nameindex()}
    loopback = next(
        (candidate for candidate in ("lo", "lo0") if candidate in interface_names),
        None,
    )
    if loopback is None:
        yield None
        return

    os.environ["GLOO_SOCKET_IFNAME"] = loopback
    try:
        yield loopback
    finally:
        os.environ.pop("GLOO_SOCKET_IFNAME", None)


def _sampler_evidence(
    torch: ModuleType,
    context: DistributedContext,
    *,
    device: object,
    dataset_size: int,
) -> dict[str, object]:
    if dataset_size < context.world_size:
        raise ValueError(
            f"sampler_size {dataset_size} must be at least WORLD_SIZE "
            f"{context.world_size}"
        )
    from torch.utils.data import DistributedSampler

    sampler = DistributedSampler(
        range(dataset_size),
        num_replicas=context.world_size,
        rank=context.rank,
        shuffle=False,
        drop_last=True,
    )
    sampler.set_epoch(0)
    local_indices = list(iter(sampler))
    local_tensor = torch.tensor(local_indices, dtype=torch.int64, device=device)
    gathered = [torch.empty_like(local_tensor) for _ in range(context.world_size)]
    torch.distributed.all_gather(gathered, local_tensor)
    shards = [tensor.cpu().tolist() for tensor in gathered]
    flattened = [index for shard in shards for index in shard]
    return {
        "dataset_size": dataset_size,
        "drop_last": True,
        "samples_per_rank": len(local_indices),
        "shards": {str(rank): shard for rank, shard in enumerate(shards)},
        "disjoint": len(flattened) == len(set(flattened)),
        "covered_sample_count": len(flattened),
        "dropped_sample_count": dataset_size - len(flattened),
    }


def run_distributed_smoke(
    context: DistributedContext,
    *,
    device_choice: DeviceChoice = "auto",
    backend_choice: BackendChoice = "auto",
    sampler_size: int = 16,
    timeout_seconds: int = 60,
    result_json: Path | None = None,
) -> dict[str, object]:
    """Exercise the minimum collectives needed by a future DDP entry point."""

    if sampler_size <= 0:
        raise ValueError(f"sampler_size must be positive, got {sampler_size}")
    if timeout_seconds <= 0:
        raise ValueError(f"timeout_seconds must be positive, got {timeout_seconds}")

    torch = _load_torch()
    device_type, backend, device_index = _select_runtime(
        torch,
        context,
        device_choice=device_choice,
        backend_choice=backend_choice,
    )
    device = torch.device(
        f"cuda:{device_index}" if device_type == "cuda" else "cpu"
    )
    distributed = torch.distributed
    if distributed.is_initialized():
        raise RuntimeError(
            "A process group is already initialized; run this smoke test as a "
            "standalone process."
        )

    with (
        _rendezvous(context) as init_method,
        _single_process_gloo_interface(context, backend) as gloo_interface,
    ):
        initialized = False
        try:
            distributed.init_process_group(
                backend=backend,
                init_method=init_method,
                rank=context.rank,
                world_size=context.world_size,
                timeout=timedelta(seconds=timeout_seconds),
            )
            initialized = True
            identity = {
                "rank": context.rank,
                "local_rank": context.local_rank,
                "device_type": device_type,
                "device_index": device_index,
                "hostname": socket.gethostname(),
                "pid": os.getpid(),
            }
            print(json.dumps({"rank_mapping": identity}, sort_keys=True), flush=True)

            mapping_tensor = torch.tensor(
                [context.rank, context.local_rank, device_index],
                dtype=torch.int64,
                device=device,
            )
            gathered_mappings = [
                torch.empty_like(mapping_tensor) for _ in range(context.world_size)
            ]
            distributed.all_gather(gathered_mappings, mapping_tensor)
            rank_mappings = [
                {
                    "rank": int(values[0]),
                    "local_rank": int(values[1]),
                    "device_type": device_type,
                    "device_index": int(values[2]),
                }
                for values in (
                    mapping.cpu().tolist() for mapping in gathered_mappings
                )
            ]

            reduced = torch.tensor(
                [float(context.rank + 1)], dtype=torch.float64, device=device
            )
            distributed.all_reduce(reduced)
            observed_sum = float(reduced.cpu().item())
            expected_sum = context.world_size * (context.world_size + 1) / 2
            all_reduce_passed = observed_sum == expected_sum

            sampler = _sampler_evidence(
                torch, context, device=device, dataset_size=sampler_size
            )
            sampler_disjoint = bool(sampler["disjoint"])
            result: dict[str, object] = {
                "status": "passed",
                "context": context.as_dict(),
                "backend": backend,
                "device_type": device_type,
                "gloo_socket_ifname": gloo_interface,
                "rank_mappings": rank_mappings,
                "all_reduce": {
                    "input": "rank + 1",
                    "observed_sum": observed_sum,
                    "expected_sum": expected_sum,
                    "passed": all_reduce_passed,
                },
                "sampler": sampler,
                "torch_version": str(torch.__version__),
                "torch_cuda_version": getattr(torch.version, "cuda", None),
            }
            if not all_reduce_passed or not sampler_disjoint:
                raise RuntimeError(
                    "Distributed collective or sampler partition validation failed."
                )

            distributed.barrier()
            if result_json is not None:
                write_rank_zero_json(result_json, result, context)
            distributed.barrier()
            return result
        finally:
            if initialized and distributed.is_initialized():
                distributed.destroy_process_group()
