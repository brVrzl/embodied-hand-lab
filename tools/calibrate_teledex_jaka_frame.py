from __future__ import annotations

import argparse
import json
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from embodiment_core.config import load_yaml
from teleop_tools.teledex_calibration import (
    CALIBRATION_SCHEMA_VERSION,
    fit_phone_to_robot_rotation,
    load_teledex_calibration,
)
from teleop_tools.teledex_phone import TeleDexPhoneClient


DEFAULT_CONFIG = "configs/teleop/teledex_jaka_arm.yaml"


def _write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2, ensure_ascii=False)
        stream.write("\n")
    temporary.replace(path)


def _capture_mean_position(
    client: TeleDexPhoneClient,
    *,
    duration_sec: float,
    min_unique_samples: int,
) -> tuple[np.ndarray, int]:
    deadline = time.monotonic() + max(0.1, float(duration_sec))
    positions: list[list[float]] = []
    last_sequence = -1
    while time.monotonic() < deadline:
        snapshot = client.read()
        sequence = int(snapshot.raw_inputs.get("sequence", -1))
        if snapshot.valid and sequence != last_sequence:
            positions.append(snapshot.position_m)
            last_sequence = sequence
        time.sleep(0.005)
    if len(positions) < int(min_unique_samples):
        raise RuntimeError(
            f"Only {len(positions)} unique TeleDex frames were captured; need at least "
            f"{min_unique_samples}. Check the phone connection and retry."
        )
    return np.mean(np.asarray(positions, dtype=np.float64), axis=0), len(positions)


def _confirm_for_real(path: Path, *, verified: bool) -> None:
    if not verified:
        raise SystemExit(
            "Refusing confirmation. First verify +X/-X/+Y/-Y/+Z/-Z in shadow, then add "
            "--i-verified-shadow-six-directions."
        )
    payload = load_teledex_calibration(path)
    payload["real_motion_confirmed"] = True
    payload["shadow_six_direction_verified_at"] = datetime.now(timezone.utc).isoformat()
    _write_json(path, payload)
    print(f"Marked TeleDex/JAKA frame calibration as real-motion confirmed: {path}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Fit TeleDex phone-world to JAKA-base rotation from three guided translations."
    )
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--output", default=None)
    parser.add_argument("--sample-sec", type=float, default=0.60)
    parser.add_argument("--min-unique-samples", type=int, default=8)
    parser.add_argument("--min-displacement-m", type=float, default=0.06)
    parser.add_argument("--confirm-for-real", action="store_true")
    parser.add_argument("--i-verified-shadow-six-directions", action="store_true")
    args = parser.parse_args()

    config = load_yaml(args.config)
    calibration_cfg = config.get("calibration", {})
    mapping_mode = str(calibration_cfg.get("mapping_mode", "continuous_so3"))
    output_path = Path(args.output or calibration_cfg.get("file", "configs/teleop/teledex_jaka_arm_calibration.json"))
    if args.confirm_for_real:
        _confirm_for_real(output_path, verified=args.i_verified_shadow_six_directions)
        return

    teledex_cfg = config.get("teledex", {})
    client = TeleDexPhoneClient(
        port=int(teledex_cfg.get("port", 8888)),
        show_qr=bool(teledex_cfg.get("show_qr", True)),
        debug=bool(teledex_cfg.get("debug", False)),
        max_stale_feedback_sec=float(teledex_cfg.get("max_stale_feedback_sec", 0.20)),
        server_start_timeout_sec=float(teledex_cfg.get("server_start_timeout_sec", 3.0)),
        deadman_field=str(teledex_cfg.get("deadman_field", "button")),
    )
    captures: dict[str, list[float]] = {}
    sample_counts: dict[str, dict[str, int]] = {}
    try:
        client.connect()
        print(f"TeleDex server ready at {client.address}; connect the iPhone app.")
        input("Keep the robot disabled. When live phone pose is available, press Enter to continue... ")
        if not client.read().valid:
            raise RuntimeError("No fresh TeleDex pose is available after confirmation.")
        for axis in ("x", "y", "z"):
            input(
                f"Return the phone to the same comfortable origin. Hold still, then press Enter "
                f"to capture the +{axis.upper()} baseline... "
            )
            origin, origin_count = _capture_mean_position(
                client,
                duration_sec=args.sample_sec,
                min_unique_samples=args.min_unique_samples,
            )
            input(
                f"Translate the phone at least {args.min_displacement_m:.2f} m in the physical "
                f"JAKA-base +{axis.upper()} direction without rotating it. Hold still, then press Enter... "
            )
            target, target_count = _capture_mean_position(
                client,
                duration_sec=args.sample_sec,
                min_unique_samples=args.min_unique_samples,
            )
            captures[axis] = (target - origin).astype(float).tolist()
            sample_counts[axis] = {"origin": origin_count, "target": target_count}
            print(f"Captured raw phone displacement for robot +{axis.upper()}: {captures[axis]}")
    finally:
        client.close()

    rotation, quality = fit_phone_to_robot_rotation(
        captures,
        min_displacement_m=args.min_displacement_m,
        mapping_mode=mapping_mode,
    )
    payload = {
        "schema_version": CALIBRATION_SCHEMA_VERSION,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "source": "guided_teledex_translation_capture",
        "mapping_mode": mapping_mode,
        "phone_to_robot_rotation_matrix": rotation.astype(float).tolist(),
        "raw_phone_displacements_for_robot_positive_axes_m": captures,
        "sample_counts": sample_counts,
        "quality": quality,
        "real_motion_confirmed": False,
        "confirmation_requirement": "Verify +X/-X/+Y/-Y/+Z/-Z in TeleDex shadow.",
    }
    _write_json(output_path, payload)
    print(f"Wrote unconfirmed TeleDex/JAKA calibration: {output_path}")
    print(
        "Next run the TeleDex shadow and verify all six directions. Only then run this tool "
        "with --confirm-for-real --i-verified-shadow-six-directions."
    )


if __name__ == "__main__":
    main()
