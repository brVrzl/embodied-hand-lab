from __future__ import annotations

import argparse
import json
from pathlib import Path

from embodiment_core.config import load_yaml
from sim_maniskill.recorder import create_env_from_config
from sim_maniskill.task_assessment import assess_pickcube_jaka_rh56_scene, format_assessment_markdown


def evaluate(config_path: str | Path, out_dir: str | Path, *, seed: int = 0) -> dict:
    config = load_yaml(config_path)
    env = create_env_from_config(config)
    try:
        env.reset(seed=seed)
        summary = env.unwrapped.get_scene_summary()
        assessment = assess_pickcube_jaka_rh56_scene(summary)
    finally:
        env.close()

    out = Path(out_dir).resolve()
    out.mkdir(parents=True, exist_ok=True)
    (out / "jaka_rh56_sim_task_assessment.json").write_text(
        json.dumps(assessment, indent=2),
        encoding="utf-8",
    )
    (out / "jaka_rh56_sim_task_assessment.md").write_text(
        format_assessment_markdown(assessment),
        encoding="utf-8",
    )
    return assessment


def main() -> None:
    parser = argparse.ArgumentParser(description="Evaluate whether PickCubeJakaRH56-v1 is suitable for the current sim pipeline.")
    parser.add_argument("--config", default="configs/sim/maniskill_jaka_rh56_pick_cube_state.yaml")
    parser.add_argument("--out-dir", default="data/reports/jaka_rh56_sim_task_assessment")
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()
    assessment = evaluate(args.config, args.out_dir, seed=args.seed)
    print(json.dumps(assessment, indent=2))


if __name__ == "__main__":
    main()
