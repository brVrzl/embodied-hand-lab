from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Any

from embodiment_core.config import load_yaml
from teleop_tools.hebi_mobile_io import HebiMobileIOSnapshot
from teleop_tools.hebi_rviz_shadow import (
    _actual_palm_pose_from_state,
    _make_shadow_ik_checker,
    _shadow_state_from_config,
)
from teleop_tools.pose_teleop_config import relative_pose_config_from_mapping
from teleop_tools.relative_pose_lag_follow import RelativePoseLagFollower
from teleop_tools.rviz_shadow_sync import ARM_JOINT_NAMES, extract_arm_joints_from_joint_state
from teleop_tools.teledex_calibration import calibration_from_project_config
from teleop_tools.teledex_phone import TeleDexPhoneClient


DEFAULT_CONFIG = "configs/teleop/teledex_jaka_arm.yaml"


def run_teledex_rviz_shadow_node(
    *,
    config_path: str | Path = DEFAULT_CONFIG,
    initial_arm_joints_rad: list[float] | None = None,
    real_arm_joint_topic: str = "/jaka/joint_states",
    real_arm_sync_timeout_sec: float = 1.0,
) -> None:
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        from visualization_msgs.msg import Marker
    except Exception as exc:
        raise RuntimeError(
            "ROS2 Python packages are required. Source ROS2 first, e.g. scripts/source_ros2.sh."
        ) from exc

    config = load_yaml(config_path)
    teledex_cfg = config.get("teledex", {})
    topics = config.get("topics", {})
    calibration, calibration_reason = calibration_from_project_config(config)
    calibration_matrix = (
        None if calibration is None else calibration["phone_to_robot_rotation_matrix"]
    )
    shadow_cfg = config.get("shadow", {})
    allow_without_deadman = bool(shadow_cfg.get("allow_without_deadman", True))
    ik_check_state = _shadow_state_from_config(config, initial_arm_joints_rad)

    class TeleDexRvizShadowNode(Node):  # type: ignore[misc]
        def __init__(self) -> None:
            super().__init__("jaka_teledex_rviz_shadow")
            self.client = TeleDexPhoneClient(
                port=int(teledex_cfg.get("port", 8888)),
                show_qr=bool(teledex_cfg.get("show_qr", True)),
                debug=bool(teledex_cfg.get("debug", False)),
                max_stale_feedback_sec=float(teledex_cfg.get("max_stale_feedback_sec", 0.20)),
                server_start_timeout_sec=float(teledex_cfg.get("server_start_timeout_sec", 3.0)),
                deadman_field=str(teledex_cfg.get("deadman_field", "button")),
                precision_scale=float(teledex_cfg.get("precision_scale", 1.0)),
            )
            self.client.connect()
            self.ik_state = ik_check_state
            self.follower = RelativePoseLagFollower(
                relative_pose_config_from_mapping(
                    config,
                    phone_to_robot_rotation_matrix=calibration_matrix,
                ),
                ik_checker=_make_shadow_ik_checker(self.ik_state),
            )
            self.shadow_pub = self.create_publisher(
                JointState,
                topics.get("shadow_joint_states", "/teleop/shadow_joint_states"),
                10,
            )
            self.status_pub = self.create_publisher(
                String,
                topics.get("shadow_status", "/teleop/shadow_status"),
                10,
            )
            self.pose_pub = self.create_publisher(
                String,
                topics.get("teledex_pose", "/teleop/teledex/pose"),
                10,
            )
            self.target_pub = self.create_publisher(
                Marker,
                topics.get("shadow_palm_target", "/teleop/shadow_palm_target"),
                10,
            )
            self.real_arm_joints: list[float] | None = None
            self.last_real_arm_time = 0.0
            self.create_subscription(
                JointState,
                real_arm_joint_topic,
                self._on_real_arm_joint_state,
                10,
            )
            self.create_timer(
                1.0 / max(float(teledex_cfg.get("publish_hz", 30.0)), 1e-6),
                self._publish,
            )
            self.get_logger().warning(
                f"TeleDex RViz shadow ready at {self.client.address}; calibration={calibration_reason}. "
                "Button A is the configured hold-to-run control (shadow may override it). "
                "No hardware command topic is published."
            )

        def _on_real_arm_joint_state(self, message: Any) -> None:
            joints = extract_arm_joints_from_joint_state(message)
            if joints is not None:
                self.real_arm_joints = joints
                self.last_real_arm_time = time.time()

        @staticmethod
        def _publish_json(publisher: Any, payload: dict[str, Any]) -> None:
            message = String()
            message.data = json.dumps(payload, ensure_ascii=False)
            publisher.publish(message)

        def _publish(self) -> None:
            phone_snapshot = self.client.read()
            snapshot = phone_snapshot
            if allow_without_deadman and phone_snapshot.valid:
                snapshot = HebiMobileIOSnapshot(
                    timestamp_sec=phone_snapshot.timestamp_sec,
                    position_m=phone_snapshot.position_m,
                    quaternion_wxyz=phone_snapshot.quaternion_wxyz,
                    raw_inputs={**phone_snapshot.raw_inputs, "b1": True, "shadow_deadman_override": True},
                    valid=True,
                    reason=phone_snapshot.reason,
                )
            q_current = self.real_arm_joints or self.ik_state.arm_joints_rad.tolist()
            actual = _actual_palm_pose_from_state(self.ik_state)
            output = self.follower.step(
                snapshot,
                actual,
                q_current,
                timestamp_sec=time.time(),
            )
            if output.command_deadman and output.palm_target_position_m is not None:
                self.ik_state.set_arm_joints_rad(output.log.get("q_cmd") or q_current)
            joint_message = JointState()
            joint_message.header.stamp = self.get_clock().now().to_msg()
            joint_message.name = list(ARM_JOINT_NAMES)
            joint_message.position = self.ik_state.arm_joints_rad.tolist()
            self.shadow_pub.publish(joint_message)
            self._publish_json(
                self.pose_pub,
                {
                    "schema_version": "teledex_phone_pose_v0.1",
                    "snapshot": snapshot.to_dict(),
                },
            )
            status = dict(output.log)
            status.update(
                {
                    "mode": "teledex_rviz_shadow_palm_target_ik",
                    "hardware_commands_published": False,
                    "real_arm_sync_active": time.time() - self.last_real_arm_time <= real_arm_sync_timeout_sec,
                    "calibration": calibration_reason,
                    "calibration_real_motion_confirmed": bool(
                        calibration and calibration.get("real_motion_confirmed", False)
                    ),
                    "shadow_deadman_override": allow_without_deadman,
                    "deadman": snapshot.enabled,
                }
            )
            self._publish_json(self.status_pub, status)
            target = output.palm_target_position_m
            if target is not None:
                marker = Marker()
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.header.frame_id = "jaka_Link_0"
                marker.ns = "teledex_shadow"
                marker.id = 0
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = float(target[0])
                marker.pose.position.y = float(target[1])
                marker.pose.position.z = float(target[2])
                marker.pose.orientation.w = 1.0
                marker.scale.x = marker.scale.y = marker.scale.z = 0.04
                marker.color.r = 0.1
                marker.color.g = 0.8
                marker.color.b = 0.3
                marker.color.a = 0.9
                self.target_pub.publish(marker)

        def destroy_node(self) -> bool:
            self.client.close()
            return super().destroy_node()

    rclpy.init()
    node = TeleDexRvizShadowNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description="Preview TeleDex arm pose control in RViz without hardware commands."
    )
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--initial-joints", nargs=6, type=float, default=None)
    parser.add_argument("--real-arm-joint-topic", default="/jaka/joint_states")
    parser.add_argument("--real-arm-sync-timeout-sec", type=float, default=1.0)
    args = parser.parse_args(argv)
    run_teledex_rviz_shadow_node(
        config_path=args.config,
        initial_arm_joints_rad=args.initial_joints,
        real_arm_joint_topic=args.real_arm_joint_topic,
        real_arm_sync_timeout_sec=args.real_arm_sync_timeout_sec,
    )


if __name__ == "__main__":
    main()
