from __future__ import annotations

import argparse
from pathlib import Path

from embodiment_core.config import load_yaml

from .recorder import run_from_config


def main() -> None:
    parser = argparse.ArgumentParser(description="Record ManiSkill simulation episodes.")
    parser.add_argument("--config", default="configs/sim/maniskill_pick_cube.yaml")
    parser.add_argument("--episodes", type=int, default=None)
    parser.add_argument("--max-steps", type=int, default=None)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--policy", choices=["random", "zero"], default=None)
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--export-dir", default=None)
    parser.add_argument("--env-id", default=None)
    parser.add_argument("--task-name", default=None)
    parser.add_argument("--instruction", default=None)
    args = parser.parse_args()

    config = load_yaml(Path(args.config))
    overrides = {
        "episodes": args.episodes,
        "max_steps": args.max_steps,
        "seed": args.seed,
        "policy": args.policy,
        "output_dir": args.output_dir,
        "export_dir": args.export_dir,
        "env_id": args.env_id,
        "task_name": args.task_name,
        "instruction": args.instruction,
    }
    result = run_from_config(config, overrides=overrides)
    print(f"Recorded episodes to: {result['episodes_root']}")
    print(f"Structured export written to: {result['export_dir']}")


if __name__ == "__main__":
    main()
