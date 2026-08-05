"""Bounded physical dual-D435 camera-only lifecycle validation."""

from __future__ import annotations

import argparse
import json
import multiprocessing as mp
from multiprocessing import shared_memory
from pathlib import Path
import sys
import time

from embodiment_core.config import load_yaml
from episode_dataset.process_runtime import ProcessCamera
from vision_interface.realsense_adapter import resolve_realsense_config


def _rss_kb(pid: int | None) -> int:
    if pid is None:
        return 0
    try:
        for line in Path(f"/proc/{pid}/status").read_text(encoding="utf-8").splitlines():
            if line.startswith("VmRSS:"):
                return int(line.split()[1])
    except (FileNotFoundError, OSError, ValueError):
        return 0
    return 0


def _shared_memory_absent(names: list[str | None]) -> bool:
    for name in names:
        if name is None:
            continue
        try:
            handle = shared_memory.SharedMemory(name=name)
        except FileNotFoundError:
            continue
        handle.close()
        return False
    return True


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path("data/local/dual_d435_episode.yaml"))
    parser.add_argument("--duration-sec", type=float, default=20.0)
    parser.add_argument("--native-control-cpu", type=int, default=6)
    args = parser.parse_args()
    if not 0.0 < args.duration_sec <= 60.0:
        raise SystemExit("--duration-sec must be within (0, 60]")

    config = load_yaml(args.config)
    context = mp.get_context("spawn")
    cameras: dict[str, ProcessCamera] = {}
    ring_names: list[str | None] = []
    counts = {"workspace": 0, "wrist": 0}
    last_sequence = {"workspace": -1, "wrist": -1}
    last_host_timestamp = {"workspace": -1, "wrist": -1}
    rss_peak_kb = {"workspace": 0, "wrist": 0}
    errors: list[str] = []
    started_ns = time.monotonic_ns()

    try:
        for role in ("workspace", "wrist"):
            resolved = resolve_realsense_config(config, camera_name=role)
            camera = ProcessCamera(
                role,
                resolved,
                capacity=int(config["dataset"].get("camera_ring_capacity", 16)),
                forbidden_cpu=args.native_control_cpu,
                context=context,
            )
            cameras[role] = camera
            ring_names.extend(
                [
                    camera.ring_spec.header_name,
                    camera.ring_spec.rgb_name,
                    camera.ring_spec.depth_name,
                    camera.ring_spec.aligned_depth_name,
                ]
            )

        for camera in cameras.values():
            camera.start(timeout_s=8.0)

        live_profiles = {
            role: camera.profile_metadata() for role, camera in cameras.items()
        }
        deadline = time.monotonic() + args.duration_sec
        while time.monotonic() < deadline:
            for role, camera in cameras.items():
                rss_peak_kb[role] = max(rss_peak_kb[role], _rss_kb(camera._process.pid))
                if camera.error is not None:
                    errors.append(f"{role}: {camera.error}")
                    continue
                reference, _ = camera.latest_after(last_host_timestamp[role])
                if reference is None:
                    continue
                if reference.sequence <= last_sequence[role]:
                    errors.append(
                        f"{role}: non-monotonic sequence {reference.sequence} "
                        f"after {last_sequence[role]}"
                    )
                try:
                    snapshot = reference.snapshot()
                except BaseException as exc:
                    errors.append(f"{role}: frame snapshot failed: {type(exc).__name__}: {exc}")
                else:
                    if snapshot.ring_sequence != reference.sequence:
                        errors.append(f"{role}: half-written ring sequence")
                last_sequence[role] = reference.sequence
                last_host_timestamp[role] = reference.host_monotonic_ns
                counts[role] += 1
            if errors:
                break
            time.sleep(0.01)
    except BaseException as exc:
        errors.append(f"probe: {type(exc).__name__}: {exc}")

    final_diagnostics: dict[str, dict[str, object]] = {}
    shutdown_started_ns = time.monotonic_ns()
    for role, camera in cameras.items():
        try:
            final_diagnostics[role] = camera.stop(timeout_s=3.0)
        except BaseException as exc:
            errors.append(f"{role} cleanup: {type(exc).__name__}: {exc}")
    shutdown_ms = (time.monotonic_ns() - shutdown_started_ns) / 1e6

    for role, camera in cameras.items():
        final = final_diagnostics.get(role, {})
        placement = final.get("placement")
        if not isinstance(placement, dict):
            errors.append(f"{role}: missing final placement diagnostics")
        else:
            if args.native_control_cpu in placement.get("affinity", []):
                errors.append(f"{role}: native control CPU in camera affinity")
            if placement.get("scheduler_policy_name") == "SCHED_FIFO":
                errors.append(f"{role}: camera child unexpectedly SCHED_FIFO")
        if camera._process.is_alive():
            errors.append(f"{role}: camera child still alive after stop")

    result = {
        "stage": "phase1_camera_only",
        "duration_s": (time.monotonic_ns() - started_ns) / 1e9,
        "shutdown_time_ms": shutdown_ms,
        "frame_counts": counts,
        "last_sequence": last_sequence,
        "live_profiles": live_profiles,
        "final_diagnostics": final_diagnostics,
        "camera_rss_peak_kb": rss_peak_kb,
        "shared_memory_absent": _shared_memory_absent(ring_names),
        "errors": errors,
    }
    if not result["shared_memory_absent"]:
        errors.append("camera shared memory remains after cleanup")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if not errors else 1


if __name__ == "__main__":
    sys.exit(main())
