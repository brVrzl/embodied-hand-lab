from __future__ import annotations

import argparse
import json
import time

from embodiment_core.config import load_yaml
from teleop_tools.hebi_mobile_io import HebiMobileIOClient


def main() -> None:
    parser = argparse.ArgumentParser(description="Check HEBI Mobile I/O AR pose and buttons.")
    parser.add_argument("--config", default="configs/teleop/hebi_mobile_io_jaka_rh56.yaml")
    parser.add_argument("--duration-sec", type=float, default=5.0)
    parser.add_argument("--hz", type=float, default=10.0)
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
    start = time.time()
    period = 1.0 / max(float(args.hz), 1e-6)
    samples = 0
    while time.time() - start < args.duration_sec:
        snapshot = client.read(timeout_ms=float(hebi_cfg.get("read_timeout_ms", 10.0)))
        print(json.dumps(snapshot.to_dict(elapsed_sec=time.time() - start), ensure_ascii=False))
        samples += 1
        time.sleep(period)
    print(json.dumps({"samples": samples, "duration_sec": args.duration_sec}, ensure_ascii=False))


if __name__ == "__main__":
    main()
