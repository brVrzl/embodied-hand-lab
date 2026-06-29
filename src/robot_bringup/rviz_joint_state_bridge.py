from __future__ import annotations

import argparse
import json
import time
from typing import Any

from rh56_driver.hand_schema import CANONICAL_HAND_ORDER


ARM_JOINT_NAMES = [f"jaka_joint_{index}" for index in range(1, 7)]
RH56_JOINT_MAX_RAD = {
    "index": 1.70,
    "middle": 1.68,
    "ring": 1.70,
    "pinky": 1.70,
    "thumb_close": 0.50,
    "thumb_lateral": 1.10,
}
RH56_URDF_JOINTS = {
    "index": "rh56_R_index_MCP_joint",
    "middle": "rh56_R_middle_MCP_joint",
    "ring": "rh56_R_ring_MCP_joint",
    "pinky": "rh56_R_pinky_MCP_joint",
    "thumb_close": "rh56_R_thumb_MCP_joint2",
    "thumb_lateral": "rh56_R_thumb_MCP_joint1",
}


def map_arm_joint_names(names: list[str]) -> list[str]:
    return [
        f"jaka_joint_{name.removeprefix('joint_')}" if name.startswith("joint_") else name
        for name in names
    ]


def rh56_counts_to_urdf_joint_state(counts: list[float]) -> dict[str, float]:
    if len(counts) != len(CANONICAL_HAND_ORDER):
        raise ValueError(f"Expected 6 RH56 counts, got {len(counts)}.")
    return {
        RH56_URDF_JOINTS[name]: max(0.0, min(1.0, (1000.0 - float(count)) / 1000.0))
        * RH56_JOINT_MAX_RAD[name]
        for name, count in zip(CANONICAL_HAND_ORDER, counts, strict=True)
    }


def run_rviz_joint_state_bridge(
    *,
    arm_topic: str = "/jaka/joint_states",
    hand_topic: str = "/rh56/state",
    shadow_arm_topic: str = "/teleop/shadow_joint_states",
    shadow_hand_topic: str = "/teleop/shadow_rh56_state",
    shadow_timeout_sec: float = 0.25,
    output_topic: str = "/joint_states",
) -> None:
    try:
        import rclpy  # type: ignore
        from rclpy.node import Node  # type: ignore
        from sensor_msgs.msg import JointState  # type: ignore
        from std_msgs.msg import String  # type: ignore
    except Exception as exc:
        raise RuntimeError("ROS2 Python packages are required. Source ROS2 first, e.g. scripts/source_ros2.sh.") from exc

    class RvizJointStateBridge(Node):  # type: ignore[misc]
        def __init__(self) -> None:
            super().__init__("jaka_rh56_rviz_joint_state_bridge")
            self.arm_names = list(ARM_JOINT_NAMES)
            self.arm_positions = [0.0] * len(self.arm_names)
            self.hand_positions = rh56_counts_to_urdf_joint_state([1000.0] * 6)
            self.shadow_arm_names: list[str] = []
            self.shadow_arm_positions: list[float] = []
            self.shadow_arm_timestamp = 0.0
            self.shadow_hand_positions: dict[str, float] = {}
            self.shadow_hand_timestamp = 0.0
            self.publisher = self.create_publisher(JointState, output_topic, 10)
            self.create_subscription(JointState, arm_topic, self._on_arm, 10)
            self.create_subscription(String, hand_topic, self._on_hand, 10)
            self.create_subscription(JointState, shadow_arm_topic, self._on_shadow_arm, 10)
            self.create_subscription(String, shadow_hand_topic, self._on_shadow_hand, 10)
            self.create_timer(0.02, self._publish)

        def _on_arm(self, message: Any) -> None:
            self.arm_names = map_arm_joint_names(list(message.name))
            self.arm_positions = [float(value) for value in message.position]

        def _on_hand(self, message: Any) -> None:
            try:
                payload = json.loads(str(message.data))
                counts = payload["finger_state"]["angle_count_0_1000"]
                self.hand_positions = rh56_counts_to_urdf_joint_state(counts)
            except Exception as exc:
                self.get_logger().error(f"RH56 RViz state conversion failed: {exc}")

        def _on_shadow_arm(self, message: Any) -> None:
            self.shadow_arm_names = map_arm_joint_names(list(message.name))
            self.shadow_arm_positions = [float(value) for value in message.position]
            self.shadow_arm_timestamp = time.monotonic()

        def _on_shadow_hand(self, message: Any) -> None:
            try:
                payload = json.loads(str(message.data))
                counts = payload["finger_state"]["angle_count_0_1000"]
                self.shadow_hand_positions = rh56_counts_to_urdf_joint_state(counts)
                self.shadow_hand_timestamp = time.monotonic()
            except Exception as exc:
                self.get_logger().error(f"RH56 RViz shadow conversion failed: {exc}")

        def _publish(self) -> None:
            now = time.monotonic()
            arm_names = self.arm_names
            arm_positions = self.arm_positions
            hand_positions = self.hand_positions
            if now - self.shadow_arm_timestamp <= shadow_timeout_sec:
                arm_names = self.shadow_arm_names
                arm_positions = self.shadow_arm_positions
            if now - self.shadow_hand_timestamp <= shadow_timeout_sec:
                hand_positions = self.shadow_hand_positions
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = arm_names + list(hand_positions)
            message.position = arm_positions + list(hand_positions.values())
            try:
                self.publisher.publish(message)
            except Exception:
                if rclpy.ok():
                    raise

    rclpy.init()
    node = RvizJointStateBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Fuse JAKA and RH56 feedback into RViz /joint_states.")
    parser.add_argument("--arm-topic", default="/jaka/joint_states")
    parser.add_argument("--hand-topic", default="/rh56/state")
    parser.add_argument("--shadow-arm-topic", default="/teleop/shadow_joint_states")
    parser.add_argument("--shadow-hand-topic", default="/teleop/shadow_rh56_state")
    parser.add_argument("--shadow-timeout-sec", type=float, default=0.25)
    parser.add_argument("--output-topic", default="/joint_states")
    args = parser.parse_args(argv)
    run_rviz_joint_state_bridge(
        arm_topic=args.arm_topic,
        hand_topic=args.hand_topic,
        shadow_arm_topic=args.shadow_arm_topic,
        shadow_hand_topic=args.shadow_hand_topic,
        shadow_timeout_sec=args.shadow_timeout_sec,
        output_topic=args.output_topic,
    )


if __name__ == "__main__":
    main()
