from __future__ import annotations

import argparse
import time

import numpy as np

from embodiment_core.config import load_yaml
from teleop_tools.hebi_mobile_io import HebiMobileIOClient, quat_conjugate_wxyz, rotate_vector_wxyz


AXES = {
    "+X": [1.0, 0.0, 0.0],
    "-X": [-1.0, 0.0, 0.0],
    "+Y": [0.0, 1.0, 0.0],
    "-Y": [0.0, -1.0, 0.0],
    "+Z": [0.0, 0.0, 1.0],
    "-Z": [0.0, 0.0, -1.0],
}


def _phone_to_world_quat(quaternion_wxyz: list[float], convention: str) -> np.ndarray:
    if convention == "world-to-phone":
        return quat_conjugate_wxyz(quaternion_wxyz)
    return np.asarray(quaternion_wxyz, dtype=np.float64)


def _raw_to_wxyz(raw_quat: list[float], order: str) -> list[float]:
    q = np.asarray(raw_quat, dtype=np.float64)
    if q.shape != (4,):
        return [1.0, 0.0, 0.0, 0.0]
    if order == "xyzw":
        return [float(q[3]), float(q[0]), float(q[1]), float(q[2])]
    return [float(q[0]), float(q[1]), float(q[2]), float(q[3])]


def _format_axis_line(quaternion_wxyz: list[float], convention: str) -> str:
    quat = _phone_to_world_quat(quaternion_wxyz, convention)
    values = {
        name: float(rotate_vector_wxyz(quat, axis)[2])
        for name, axis in AXES.items()
    }
    best_level = min(values.items(), key=lambda item: abs(item[1]))
    return (
        " ".join(f"{name}:{value:+.2f}" for name, value in values.items())
        + f" | most_horizontal={best_level[0]}:{best_level[1]:+.2f}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Live-print HEBI phone axis vertical components.")
    parser.add_argument("--config", default="configs/teleop/hebi_mobile_io_jaka_rh56.yaml")
    parser.add_argument(
        "--convention",
        choices=["body-to-world", "world-to-phone"],
        default="body-to-world",
    )
    parser.add_argument("--order", choices=["xyzw", "wxyz", "both"], default="wxyz")
    parser.add_argument("--hz", type=float, default=5.0)
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
    print(
        "Watch z components. Back-camera normal should be: screen up ~= -1, upright ~= 0, screen down ~= +1."
    )
    print("Press Ctrl-C to stop.")
    period = 1.0 / max(float(args.hz), 1e-6)
    while True:
        snapshot = client.read(timeout_ms=float(hebi_cfg.get("read_timeout_ms", 10.0)))
        if not snapshot.valid:
            print(f"invalid: {snapshot.reason}")
            time.sleep(period)
            continue
        if args.order == "both":
            raw_quat = snapshot.raw_inputs.get("orientation_raw")
            if raw_quat is None:
                print(_format_axis_line(snapshot.quaternion_wxyz, args.convention))
            else:
                print(
                    "xyzw "
                    + _format_axis_line(_raw_to_wxyz(raw_quat, "xyzw"), args.convention)
                    + " || wxyz "
                    + _format_axis_line(_raw_to_wxyz(raw_quat, "wxyz"), args.convention)
                )
        else:
            print(_format_axis_line(snapshot.quaternion_wxyz, args.convention))
        time.sleep(period)


if __name__ == "__main__":
    main()
