from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from teleop_tools.iphone_hand import (
    IPHONE_CAMERA_URL,
    apply_retarget_safety,
    build_landmark_payload,
    build_rh56_target_payload,
    parse_camera_source,
    retarget_mediapipe_landmarks_to_rh56,
)


def _landmarks_from_mediapipe(result: object, width: int, height: int) -> tuple[list[dict[str, float]], str, float] | None:
    hands = getattr(result, "multi_hand_landmarks", None)
    if not hands:
        return None
    handedness = "unknown"
    score = 0.0
    multi_handedness = getattr(result, "multi_handedness", None)
    if multi_handedness:
        classification = multi_handedness[0].classification[0]
        handedness = str(classification.label)
        score = float(classification.score)
    landmarks = [
        {"x": float(item.x), "y": float(item.y), "z": float(item.z)}
        for item in hands[0].landmark
    ]
    return landmarks, handedness, score


def _smooth_target(
    previous: list[float] | None,
    target: list[float],
    *,
    alpha: float,
    max_close: float,
) -> list[float]:
    if previous is None:
        return [float(max(0.0, min(max_close, value))) for value in target]
    blend = float(max(0.0, min(1.0, alpha)))
    return [
        float(max(0.0, min(max_close, old + blend * (new - old))))
        for old, new in zip(previous, target, strict=True)
    ]


def main() -> None:
    parser = argparse.ArgumentParser(description="Retarget iPhone IP-camera MediaPipe hand landmarks to RH56 commands.")
    parser.add_argument("--source", default=IPHONE_CAMERA_URL)
    parser.add_argument("--realsense-serial", default=None, help="Use RealSense RGB frames from this serial instead of --source.")
    parser.add_argument("--realsense-width", type=int, default=640)
    parser.add_argument("--realsense-height", type=int, default=480)
    parser.add_argument("--realsense-fps", type=int, default=30)
    parser.add_argument("--jsonl-out", default="data/teleop/iphone_rh56_hand_teleop.jsonl")
    parser.add_argument("--max-close", type=float, default=0.85)
    parser.add_argument("--delta-limit", type=float, default=0.05)
    parser.add_argument("--smoothing-alpha", type=float, default=1.0, help="Low-pass target_norm before rate limiting; 1 disables smoothing.")
    parser.add_argument("--thumb-mode", default="rh56_task")
    parser.add_argument("--max-frames", type=int, default=0, help="Stop after this many camera frames; 0 runs until interrupted.")
    parser.add_argument("--ros2", action="store_true", help="Publish /rh56/command_angles JSON commands.")
    parser.add_argument("--topic", default="/rh56/command_angles")
    args = parser.parse_args()

    try:
        import cv2  # type: ignore
        import mediapipe as mp  # type: ignore
    except Exception as exc:
        raise RuntimeError("opencv-python and mediapipe are required for iPhone hand teleop.") from exc

    publisher = None
    rclpy = None
    node = None
    if args.ros2:
        try:
            import rclpy as _rclpy  # type: ignore
            from rclpy.node import Node  # type: ignore
            from std_msgs.msg import String  # type: ignore
        except Exception as exc:
            raise RuntimeError("ROS2 Python packages are required. Source ROS2 first, e.g. scripts/source_ros2.sh.") from exc
        rclpy = _rclpy
        rclpy.init()
        node = Node("iphone_mediapipe_rh56_hand_teleop")
        publisher = node.create_publisher(String, args.topic, 10)

    cap = None
    realsense = None
    frame_id = "iphone_ip_camera"
    if args.realsense_serial:
        from vision_interface.realsense_adapter import RealSenseCamera

        frame_id = f"realsense_{args.realsense_serial}_color"
        realsense = RealSenseCamera(
            {
                "serial": args.realsense_serial,
                "width": args.realsense_width,
                "height": args.realsense_height,
                "fps": args.realsense_fps,
                "frame_id": frame_id,
            }
        )
    else:
        cap = cv2.VideoCapture(parse_camera_source(args.source))
        if not cap.isOpened():
            raise SystemExit(f"Could not open camera source: {args.source}")
    output = Path(args.jsonl_out)
    output.parent.mkdir(parents=True, exist_ok=True)
    previous: list[float] | None = None
    smoothed: list[float] | None = None
    hands = mp.solutions.hands.Hands(max_num_hands=1, min_detection_confidence=0.6, min_tracking_confidence=0.5)
    frame_count = 0

    try:
        with output.open("a", encoding="utf-8") as stream:
            while True:
                if realsense is not None:
                    rgb = realsense.capture().rgb
                    image_shape = rgb.shape
                else:
                    assert cap is not None
                    ok, frame = cap.read()
                    if not ok:
                        time.sleep(0.02)
                        continue
                    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    image_shape = frame.shape
                result = hands.process(rgb)
                parsed = _landmarks_from_mediapipe(result, rgb.shape[1], rgb.shape[0])
                now = time.time()
                if parsed is None:
                    target_norm = _smooth_target(smoothed, [0.0] * 6, alpha=args.smoothing_alpha, max_close=args.max_close)
                    smoothed = target_norm
                    previous = apply_retarget_safety(previous, target_norm, delta_limit=args.delta_limit, max_close=args.max_close)
                    payload = {"timestamp": now, "hand_detected": False, "target_norm": previous}
                else:
                    landmarks, handedness, score = parsed
                    retarget = retarget_mediapipe_landmarks_to_rh56(landmarks, max_close=args.max_close, thumb_mode=args.thumb_mode)
                    target_norm = _smooth_target(smoothed, retarget.target_norm, alpha=args.smoothing_alpha, max_close=args.max_close)
                    smoothed = target_norm
                    previous = apply_retarget_safety(previous, target_norm, delta_limit=args.delta_limit, max_close=args.max_close)
                    landmark_payload = build_landmark_payload(
                        timestamp=now,
                        frame_id=frame_id,
                        handedness=handedness,
                        score=score,
                        landmarks=landmarks,
                        image_shape=image_shape,
                    )
                    target_payload = build_rh56_target_payload(
                        timestamp=now,
                        frame_id=frame_id,
                        retarget=retarget,
                        safe_target_norm=previous,
                        hand_detected=True,
                    )
                    payload = {"landmarks": landmark_payload, "target": target_payload}
                    if publisher is not None:
                        from std_msgs.msg import String  # type: ignore
                        msg = String()
                        msg.data = json.dumps(
                            {
                                "values": target_payload["raw_canonical"],
                                "unit": "rh56_angle_raw_0_1000",
                                "order": "canonical",
                            },
                            ensure_ascii=False,
                        )
                        publisher.publish(msg)
                stream.write(json.dumps(payload, ensure_ascii=False) + "\n")
                stream.flush()
                frame_count += 1
                if args.max_frames > 0 and frame_count >= args.max_frames:
                    break
                if rclpy is not None:
                    rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        if cap is not None:
            cap.release()
        if realsense is not None:
            realsense.close()
        hands.close()
        if node is not None:
            node.destroy_node()
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
