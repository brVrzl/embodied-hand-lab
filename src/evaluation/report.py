from __future__ import annotations

import argparse
import json
from collections import Counter, defaultdict
from pathlib import Path


def build_report(episodes_root: str | Path) -> str:
    episodes_root = Path(episodes_root).resolve()
    by_task: dict[str, list[dict]] = defaultdict(list)
    for metadata_path in sorted(episodes_root.glob("episode_*/metadata.json")):
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        by_task[metadata["task_name"]].append(metadata)

    lines = ["# Evaluation Report", ""]
    for task_name, rows in sorted(by_task.items()):
        total = len(rows)
        success_count = sum(1 for row in rows if row.get("success") is True)
        avg_duration = sum(row.get("duration_sec", 0.0) for row in rows) / max(total, 1)
        reasons = Counter(row.get("failure_reason", "") for row in rows if row.get("success") is False)
        lines.append(f"## {task_name}")
        lines.append(f"- episodes: {total}")
        lines.append(f"- success_rate: {success_count / max(total, 1):.2%}")
        lines.append(f"- average_duration_sec: {avg_duration:.2f}")
        lines.append(f"- failure_reasons: {dict(reasons)}")
        lines.append("")
    return "\n".join(lines).strip() + "\n"


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate a markdown evaluation report.")
    parser.add_argument("--episodes-root", default="data/episodes")
    parser.add_argument("--output", default="data/exports/evaluation_report.md")
    args = parser.parse_args()

    report = build_report(args.episodes_root)
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(report, encoding="utf-8")
    print(f"Markdown report written to: {output_path}")


if __name__ == "__main__":
    main()

