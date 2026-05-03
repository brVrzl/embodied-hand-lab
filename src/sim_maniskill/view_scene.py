from __future__ import annotations

import argparse
import signal
import time
from pathlib import Path

from embodiment_core.config import load_yaml

from .recorder import create_env_from_config


def main() -> None:
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    parser = argparse.ArgumentParser(description="Open a persistent ManiSkill scene viewer.")
    parser.add_argument("--config", default="configs/sim/maniskill_jaka_rh56_scene_preview.yaml")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument(
        "--step-zero",
        action="store_true",
        help="Continuously step zero actions while the viewer is open.",
    )
    args = parser.parse_args()

    config = load_yaml(Path(args.config))
    config = {
        "env": dict(config.get("env", {})),
        "task": dict(config.get("task", {})),
        "robot": dict(config.get("robot", {})),
        "recording": dict(config.get("recording", {})),
        "logging": dict(config.get("logging", {})),
    }
    config["env"]["render_mode"] = "human"
    config["env"].pop("render_backend", None)

    env = create_env_from_config(config)
    try:
        env.reset(seed=args.seed)
        unwrapped = getattr(env, "unwrapped", env)
        viewer = unwrapped.render_human()
        sleep_s = 0.0 if args.fps <= 0 else 1.0 / args.fps
        zero_action = None
        print("Viewer is open. Close the window or press Ctrl+C to exit.")
        if args.step_zero:
            zero_action = env.action_space.sample()
            if hasattr(zero_action, "fill"):
                zero_action.fill(0)

        while not viewer.closed:
            if args.step_zero:
                env.step(zero_action)
            unwrapped.render_human()
            if sleep_s > 0:
                time.sleep(sleep_s)
    finally:
        env.close()


if __name__ == "__main__":
    main()
