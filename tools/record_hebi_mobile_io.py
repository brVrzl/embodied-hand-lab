from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from embodiment_core.config import load_yaml
from teleop_tools.hebi_mobile_io import HebiMobileIOClient


def main() -> None:
    parser = argparse.ArgumentParser(description="Record HEBI Mobile I/O snapshots to JSONL.")
    parser.add_argument("--config", default="configs/teleop/hebi_mobile_io_jaka_rh56.yaml")
    parser.add_argument("--duration-sec", type=float, default=30.0)
    parser.add_argument("--hz", type=float, default=30.0)
    parser.add_argument("--output", default="data/teleop/hebi_mobile_io.jsonl")
    args = parser.parse_args()

    config = load_yaml(args.config)
    hebi_cfg = config.get("hebi", {})
    client = HebiMobileIOClient(
        family=str(hebi_cfg.get("family", "HEBI")),
        name=str(hebi_cfg.get("name", "mobileIO")),
        lookup_wait_sec=float(hebi_cfg.get("lookup_wait_sec", 2.0)),
        setup_ui=bool(hebi_cfg.get("setup_ui", True)),
        max_stale_feedback_sec=float(hebi_cfg.get("max_stale_feedback_sec", 0.25)),
    )
    client.connect()
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    start = time.time()
    period = 1.0 / max(float(args.hz), 1e-6)
    with output.open("w", encoding="utf-8") as stream:
        while time.time() - start < args.duration_sec:
            now = time.time()
            snapshot = client.read(timeout_ms=float(hebi_cfg.get("read_timeout_ms", 10.0)))
            stream.write(
                json.dumps(
                    {
                        "timestamp_sec": now,
                        "elapsed_sec": now - start,
                        "snapshot": snapshot.to_dict(elapsed_sec=now - start),
                    },
                    ensure_ascii=False,
                )
                + "\n"
            )
            stream.flush()
            time.sleep(period)


if __name__ == "__main__":
    main()
