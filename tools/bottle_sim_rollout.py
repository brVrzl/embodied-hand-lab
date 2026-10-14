"""Offline synchronous RGB/proprio bottle evaluation. Never imports device drivers."""
from __future__ import annotations

import argparse
import csv
import json
import subprocess
import time
import traceback
from pathlib import Path

import cv2
import numpy as np

from embodied_bottle_sim.scene import BottleScene
from bottle_policy_worker import BottlePolicyClient


class Stages:
    """Ordered physical events, including contact-free stable release."""

    def __init__(self, initial):
        self.initial = np.asarray(initial["bottle_pos"])
        self.events = {k: None for k in ("approach", "grasp_lift", "transport", "place", "release", "final_success")}
        self.stable_since = None

    def update(self, s):
        t = float(s["time"])
        p = np.asarray(s["bottle_pos"])
        contact = int(s["hand_contacts"]) > 0
        if s["hand_distance"] < .10:
            self.events["approach"] = self.events["approach"] or t
        if self.events["approach"] is not None and contact and p[2] > self.initial[2] + .04:
            self.events["grasp_lift"] = self.events["grasp_lift"] or t
        if self.events["grasp_lift"] is not None and np.linalg.norm(p[:2] - self.initial[:2]) > .08:
            self.events["transport"] = self.events["transport"] or t
        if self.events["transport"] is not None and s["on_box"]:
            self.events["place"] = self.events["place"] or t
        # Four long fingers must actually open; no contact for stable release.
        opened = bool(np.max(np.asarray(s["state"])[6:10]) < .35)
        released = self.events["place"] is not None and s["on_box"] and not contact and opened
        if released:
            self.events["release"] = self.events["release"] or t
        velocity = np.asarray(s["bottle_velocity"])
        stable = released and np.linalg.norm(velocity[:3]) < .025 and np.linalg.norm(velocity[3:]) < .25
        if stable:
            if self.stable_since is None:
                self.stable_since = t
            if t - self.stable_since >= 1.:
                self.events["final_success"] = self.events["final_success"] or t
        else:
            self.stable_since = None
            self.events["final_success"] = None


def jsonable(x):
    if isinstance(x, np.ndarray):
        return x.tolist()
    if isinstance(x, np.generic):
        return x.item()
    raise TypeError(type(x).__name__)


def append_json(path, obj):
    with path.open("a") as f:
        f.write(json.dumps(obj, default=jsonable, allow_nan=False) + "\n")
        f.flush()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--model", choices=["act", "pi05", "dp"], required=True)
    p.add_argument("--source-root", type=Path, default=Path("/home/thor/projects/embodied_lab"))
    p.add_argument("--instances", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--seconds", type=float, default=40.)
    p.add_argument("--chunk", type=int, default=8)
    p.add_argument("--phase", choices=["development", "nominal", "perturbed"], default="development")
    p.add_argument("--acceptance", type=Path)
    p.add_argument("--video-count", type=int, default=1)
    args = p.parse_args()
    if args.phase != "development":
        if not args.acceptance or not json.loads(args.acceptance.read_text())["expert_slowrelease"]["full_task_success"]:
            p.error("Formal evaluation requires independent physical task acceptance")
    args.output.mkdir(parents=True, exist_ok=False)
    worktree = Path(__file__).resolve().parents[1]
    instances = json.loads(args.instances.read_text())
    if isinstance(instances, dict):
        instances = instances[args.phase] if args.phase in instances else instances["instances"]
    metadata = vars(args) | {"code_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=worktree, text=True).strip(), "control_hz": 30, "observation_source": "current_simulation_only"}
    (args.output / "run.json").write_text(json.dumps(metadata, default=str, indent=2))
    with BottlePolicyClient(model=args.model, source_root=args.source_root, worktree=worktree, log_dir=args.output) as policy:
        scene = BottleScene()
        for i, instance in enumerate(instances):
            attempt = f"{args.model}_{args.phase}_{i:03d}"
            result = {"attempt": attempt, "model": args.model, "phase": args.phase, "environment_seed": instance.get("environment_seed", i), "model_seed": instance.get("model_seed", 20261014), "instance": instance, "status": "started"}
            append_json(args.output / "attempts.jsonl", result)
            started = time.monotonic()
            latencies, raw_chunks, executed, raw_actions, snapshots = [], [], [], [], []
            video = None
            stages = None
            try:
                scene.reset(instance)
                policy.reset(result["model_seed"])
                stages = Stages(scene.snapshot())
                previous_obs = None
                for tick in range(round(args.seconds * 30)):
                    if tick % args.chunk == 0:
                        obs = scene.observe()
                        response = (policy.infer_history(previous_obs or obs, obs) if args.model == "dp"
                                    else policy.infer(obs["state"], obs["workspace"], obs["wrist"]))
                        chunk = np.asarray(response["prediction"])
                        if chunk.ndim != 2 or chunk.shape[1] != 12 or len(chunk) < args.chunk or not np.isfinite(chunk).all():
                            raise ValueError(f"Invalid native future chunk: {chunk.shape}")
                        raw_chunks.append(chunk)
                        latencies.append(float(response["latency_ms"]))
                    if args.model == "dp" and tick % args.chunk == args.chunk - 1:
                        previous_obs = scene.observe()
                    raw = chunk[tick % args.chunk]
                    snapshot = scene.step(raw)
                    raw_actions.append(raw.copy())
                    executed.append(np.asarray(scene.last_action["executed"]).copy())
                    snapshots.append(snapshot)
                    stages.update(snapshot)
                    if i < args.video_count and tick % 3 == 0:
                        frame = np.concatenate([scene.render("workspace"), scene.render("wrist")], axis=1)
                        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                        label = f"{attempt} | t={snapshot['time']:.1f}s | {args.phase} | success={stages.events['final_success'] is not None}"
                        cv2.putText(frame, label, (12, 25), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 0, 255), 2)
                        if video is None:
                            video = cv2.VideoWriter(str(args.output / f"{attempt}.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 10, (frame.shape[1], frame.shape[0]))
                            if not video.isOpened():
                                raise RuntimeError("VideoWriter failed")
                        video.write(frame)
                    if not np.isfinite(snapshot["state"]).all() or np.linalg.norm(snapshot["bottle_velocity"]) > 20:
                        result["status"] = "physical_instability"
                        break
                else:
                    result["status"] = "success" if stages.events["final_success"] is not None else "model_task_timeout"
            except KeyboardInterrupt:
                result.update(status="technical_interruption", error="Operator interrupted this attempt; retained")
                raise
            except Exception as exc:
                result.update(status="technical_error", error=f"{type(exc).__name__}: {exc}")
                (args.output / f"{attempt}.error.txt").write_text(traceback.format_exc())
            finally:
                if video is not None:
                    video.release()
                wall = time.monotonic() - started
                result.update(wall_seconds=wall, ticks=len(executed), queries=len(latencies), sim_seconds=len(executed) / 30, real_time_factor=len(executed) / 30 / wall, warm_latency_ms_mean=float(np.mean(latencies[1:])) if len(latencies) > 1 else None, first_latency_ms=latencies[0] if latencies else None, stage_times=stages.events if stages else {})
                if executed:
                    raw, final = np.asarray(raw_actions), np.asarray(executed)
                    projected = np.abs(raw - final) > 1e-7
                    result.update(projected_action_fraction=float(np.mean(np.any(projected, axis=1))), projected_dimension_fraction=float(np.mean(projected)), raw_hand_out_of_domain_fraction=float(np.mean((raw[:, 6:] < 0) | (raw[:, 6:] > 1))), tracking_arm_rmse=float(np.sqrt(np.mean((np.asarray([s["state"][:6] for s in snapshots]) - final[:, :6]) ** 2))))
                    result["max_projection_per_dimension"] = np.max(np.abs(raw - final), axis=0).tolist()
                    result["final_bottle_pos"] = snapshots[-1]["bottle_pos"]
                    result["max_bottle_lift_m"] = float(np.max(np.asarray([s["bottle_pos"] for s in snapshots])[:, 2]) - stages.initial[2])
                    result["hand_contact_ticks"] = sum(s["hand_contacts"] > 0 for s in snapshots)
                    np.savez_compressed(args.output / f"{attempt}.npz", raw_actions=raw, executed_actions=final, raw_chunks=np.asarray(raw_chunks), latency_ms=latencies)
                    (args.output / f"{attempt}.trace.json").write_text(json.dumps(snapshots, default=jsonable))
                append_json(args.output / "episodes.jsonl", result)
                print(json.dumps(result, default=jsonable), flush=True)
        scene.close()
    rows = [json.loads(line) for line in (args.output / "episodes.jsonl").read_text().splitlines()]
    fields = ["attempt", "model", "phase", "status", "ticks", "queries", "sim_seconds", "wall_seconds", "real_time_factor", "warm_latency_ms_mean", "projected_action_fraction", "raw_hand_out_of_domain_fraction", "tracking_arm_rmse"]
    with (args.output / "episodes.csv").open("w") as f:
        writer = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


if __name__ == "__main__":
    main()
