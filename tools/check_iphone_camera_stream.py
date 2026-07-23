from __future__ import annotations

import argparse
import json
import time

from teleop_tools.iphone_hand import parse_camera_source


def main() -> None:
    parser = argparse.ArgumentParser(description="Check an iPhone IP camera/OpenCV stream.")
    parser.add_argument(
        "--source",
        required=True,
        help="OpenCV camera index or URL. Supply credentials through local configuration, never source control.",
    )
    parser.add_argument("--frames", type=int, default=30)
    args = parser.parse_args()
    try:
        import cv2  # type: ignore
    except Exception as exc:
        raise RuntimeError("opencv-python is required for iPhone camera checks.") from exc
    cap = cv2.VideoCapture(parse_camera_source(args.source))
    if not cap.isOpened():
        raise SystemExit(f"Could not open camera source: {args.source}")
    start = time.time()
    ok = 0
    shape = None
    for _ in range(max(1, args.frames)):
        success, frame = cap.read()
        if success:
            ok += 1
            shape = list(frame.shape)
    cap.release()
    print(json.dumps({"frames_ok": ok, "frames_requested": args.frames, "shape": shape, "elapsed_sec": time.time() - start}, ensure_ascii=False))


if __name__ == "__main__":
    main()
