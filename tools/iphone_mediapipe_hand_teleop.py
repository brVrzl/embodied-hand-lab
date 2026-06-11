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


def main() -> None:
    parser = argparse.ArgumentParser(description="Retarget iPhone IP-camera MediaPipe hand landmarks to RH56 commands.")
    parser.add_argument("--source", default=IPHONE_CAMERA_URL)
    parser.add_argument("--jsonl-out", default="data/teleop/iphone_rh56_hand_teleop.jsonl")
    parser.add_argument("--max-close", type=float, default=0.85)
    parser.add_argument("--delta-limit", type=float, default=0.05)
    parser.add_argument("--thumb-mode", default="rh56_task")
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
            raise RuntimeError("ROS2 Python packages are required. Source ROS2 Humble first.") from exc
        rclpy = _rclpy
        rclpy.init()
        node = Node("iphone_mediapipe_rh56_hand_teleop")
        publisher = node.create_publisher(String, args.topic, 10)

    cap = cv2.VideoCapture(parse_camera_source(args.source))
    if not cap.isOpened():
        raise SystemExit(f"Could not open camera source: {args.source}")
    output = Path(args.jsonl_out)
    output.parent.mkdir(parents=True, exist_ok=True)
    previous: list[float] | None = None
    hands = mp.solutions.hands.Hands(max_num_hands=1, min_detection_confidence=0.6, min_tracking_confidence=0.5)

    try:
        with output.open("a", encoding="utf-8") as stream:
            while True:
                ok, frame = cap.read()
                if not ok:
                    time.sleep(0.02)
                    continue
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                result = hands.process(rgb)
                parsed = _landmarks_from_mediapipe(result, frame.shape[1], frame.shape[0])
                now = time.time()
                if parsed is None:
                    previous = apply_retarget_safety(previous, [0.0] * 6, delta_limit=args.delta_limit, max_close=args.max_close)
                    payload = {"timestamp": now, "hand_detected": False, "target_norm": previous}
                else:
                    landmarks, handedness, score = parsed
                    retarget = retarget_mediapipe_landmarks_to_rh56(landmarks, max_close=args.max_close, thumb_mode=args.thumb_mode)
                    previous = apply_retarget_safety(previous, retarget.target_norm, delta_limit=args.delta_limit, max_close=args.max_close)
                    landmark_payload = build_landmark_payload(
                        timestamp=now,
                        frame_id="iphone_ip_camera",
                        handedness=handedness,
                        score=score,
                        landmarks=landmarks,
                        image_shape=frame.shape,
                    )
                    target_payload = build_rh56_target_payload(
                        timestamp=now,
                        frame_id="iphone_ip_camera",
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
                                "values": target_payload["target_raw_count"],
                                "unit": "rh56_angle_raw_0_1000",
                                "order": "canonical",
                            },
                            ensure_ascii=False,
                        )
                        publisher.publish(msg)
                stream.write(json.dumps(payload, ensure_ascii=False) + "\n")
                stream.flush()
                if rclpy is not None:
                    rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        hands.close()
        if node is not None:
            node.destroy_node()
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
