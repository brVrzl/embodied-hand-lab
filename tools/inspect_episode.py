from __future__ import annotations

import argparse
import json
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description="Inspect a recorded episode.")
    parser.add_argument("episode_dir")
    args = parser.parse_args()
    episode_dir = Path(args.episode_dir).resolve()
    metadata = json.loads((episode_dir / "metadata.json").read_text(encoding="utf-8"))
    steps = (episode_dir / "steps.jsonl").read_text(encoding="utf-8").splitlines()
    print(json.dumps({"metadata": metadata, "step_count": len([s for s in steps if s.strip()])}, indent=2))


if __name__ == "__main__":
    main()

