#!/usr/bin/env python3
"""Summarize independently labelled Quest hand-calibration captures."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Any

import numpy as np

from analyze_rh56_retarget_log import FINGER_ROWS, _hts_features


REQUIRED_LABELS = (
    "open",
    "fist",
    "thumb_open",
    "thumb_neutral",
    "thumb_opposed",
    "index_pinch",
    "middle_pinch",
    "tripod",
)
DISTANCE_FEATURES = (
    "thumb_index_distance_palm",
    "thumb_middle_distance_palm",
    "index_middle_distance_palm",
)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Analyze separately labelled Quest calibration captures using only "
            "the stable tail of each file."
        )
    )
    parser.add_argument(
        "--sample",
        action="append",
        required=True,
        metavar="LABEL=HTS_JSONL",
        help="Repeat once for every required labelled pose.",
    )
    parser.add_argument("--stable-tail-sec", type=float, default=1.0)
    parser.add_argument("--output", required=True)
    return parser


def _parse_samples(values: list[str]) -> dict[str, Path]:
    result: dict[str, Path] = {}
    for value in values:
        label, separator, raw_path = value.partition("=")
        if not separator or label not in REQUIRED_LABELS or not raw_path:
            raise ValueError(f"invalid labelled sample {value!r}")
        if label in result:
            raise ValueError(f"duplicate labelled sample {label!r}")
        result[label] = Path(raw_path)
    missing = sorted(set(REQUIRED_LABELS) - set(result))
    if missing:
        raise ValueError(f"missing labelled samples: {', '.join(missing)}")
    return result


def _stable_tail(rows: list[dict[str, Any]], seconds: float) -> list[dict[str, Any]]:
    if not rows:
        return []
    cutoff = int(rows[-1]["monotonic_ns"]) - int(seconds * 1e9)
    return [row for row in rows if int(row["monotonic_ns"]) >= cutoff]


def _summary(values: list[float]) -> tuple[float, float, float]:
    array = np.asarray(values, dtype=np.float64)
    return (
        float(np.percentile(array, 5)),
        float(np.median(array)),
        float(np.percentile(array, 95)),
    )


def _cell(values: list[float]) -> str:
    low, median, high = _summary(values)
    return f"{median:.4f} [{low:.4f}, {high:.4f}]"


def _render(samples: dict[str, Path], tail_sec: float) -> str:
    stable: dict[str, list[dict[str, Any]]] = {}
    total: dict[str, int] = {}
    for label, path in samples.items():
        rows = _hts_features(path)
        total[label] = len(rows)
        stable[label] = _stable_tail(rows, tail_sec)
        if len(stable[label]) < 20:
            raise ValueError(f"{label} has fewer than 20 valid stable-tail frames")

    lines = [
        "# RH56 real Quest calibration capture analysis",
        "",
        "Validation level: labelled Quest-only physical hand-tracking capture; no RH56 or JAKA command path was opened.",
        "",
        f"Each statistic is median [p05, p95] over the final {tail_sec:g} s of its independently labelled file.",
        "",
        "| pose | stable / total frames | index curl | middle curl | ring curl | pinky curl | thumb close curl | thumb across-palm |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for label in REQUIRED_LABELS:
        rows = stable[label]
        curl_cells = [
            _cell([float(row["fingers"][name]["combined_curl"]) for row in rows])
            for name in FINGER_ROWS
        ]
        lateral = _cell([float(row["thumb_raw_across_palm"]) for row in rows])
        lines.append(
            f"| {label} | {len(rows)} / {total[label]} | "
            + " | ".join((*curl_cells, lateral))
            + " |"
        )

    lines += [
        "",
        "## Pinch geometry",
        "",
        "Distances are divided by `distance(wrist, middle MCP)`, matching the production fingertip-distance normalization.",
        "",
        "| pose | thumb-index | thumb-middle | index-middle |",
        "|---|---:|---:|---:|",
    ]
    for label in REQUIRED_LABELS:
        rows = stable[label]
        lines.append(
            f"| {label} | "
            + " | ".join(
                _cell([float(row[feature]) for row in rows])
                for feature in DISTANCE_FEATURES
            )
            + " |"
        )

    open_rows = stable["open"]
    fist_rows = stable["fist"]
    finger_names = tuple(FINGER_ROWS)[:4]
    finger_open = [
        _summary([float(row["fingers"][name]["combined_curl"]) for row in open_rows])[1]
        for name in finger_names
    ]
    finger_closed = [
        _summary([float(row["fingers"][name]["combined_curl"]) for row in fist_rows])[1]
        for name in finger_names
    ]
    if any(
        not math.isfinite(open_value)
        or not math.isfinite(closed_value)
        or closed_value - open_value < 0.1
        for open_value, closed_value in zip(finger_open, finger_closed, strict=True)
    ):
        raise ValueError("labelled open/fist feature direction or span is invalid")
    lateral = {
        label: _summary(
            [float(row["thumb_raw_across_palm"]) for row in stable[label]]
        )[1]
        for label in ("thumb_open", "thumb_neutral", "thumb_opposed")
    }
    if not lateral["thumb_open"] < lateral["thumb_neutral"] < lateral["thumb_opposed"]:
        raise ValueError("labelled thumb feature direction is invalid")

    lines += [
        "",
        "## Measured endpoint medians",
        "",
        f"- finger open combined curl: `[{', '.join(f'{value:.6f}' for value in finger_open)}]`",
        f"- finger closed combined curl: `[{', '.join(f'{value:.6f}' for value in finger_closed)}]`",
        f"- thumb open / neutral / opposed across-palm: `{lateral['thumb_open']:.6f} / {lateral['thumb_neutral']:.6f} / {lateral['thumb_opposed']:.6f}`",
        "- positive thumb feature direction is index-MCP toward pinky-MCP, matching increasing RH56 thumb_lateral opposition.",
        "",
        "No RH56 pinch pose is inferred from these human features; actuator poses still require separate bounded physical contact validation.",
        "",
        "## Inputs",
        "",
    ]
    lines.extend(f"- `{label}`: `{path}`" for label, path in samples.items())
    return "\n".join(lines) + "\n"


def main() -> None:
    args = _parser().parse_args()
    if not math.isfinite(args.stable_tail_sec) or args.stable_tail_sec <= 0.0:
        raise ValueError("--stable-tail-sec must be positive")
    samples = _parse_samples(args.sample)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(_render(samples, args.stable_tail_sec), encoding="utf-8")


if __name__ == "__main__":
    main()
