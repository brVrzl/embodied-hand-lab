from __future__ import annotations

import argparse
import json
import math
import signal
import time
from pathlib import Path
from typing import Any

from embodiment_core.config import load_yaml
from teleop_tools.hebi_mobile_io import HebiMobileIOClient
from teleop_tools.relative_pose_lag_follow import (
    RelativePoseLagFollower,
    RelativePoseLagFollowConfig,
    TcpPose,
)
from teleop_tools.rviz_shadow_sync import extract_arm_joints_from_joint_state


def _relative_config_from_config(config: dict[str, Any]) -> RelativePoseLagFollowConfig:
    relative_cfg = config.get("relative_pose_lag_follow", {})
    direction_cfg = config.get("direction_calibration", {})
    return RelativePoseLagFollowConfig(
        target_response_mode=str(relative_cfg.get("target_response_mode", "direct")),
        position_scale=float(relative_cfg.get("position_scale", 1.0)),
        max_step_position_m=float(relative_cfg.get("max_step_position_m", 0.01)),
        max_step_rotation_rad=float(
            relative_cfg.get(
                "max_step_rotation_rad",
                math.radians(float(relative_cfg.get("max_step_rotation_deg", 2.0))),
            )
        ),
        max_target_lead_m=float(relative_cfg.get("max_target_lead_m", 0.08)),
        workspace_min_m=tuple(
            float(v) for v in relative_cfg.get("workspace_min_m", [-1.0, -1.0, -1.0])
        ),
        workspace_max_m=tuple(
            float(v) for v in relative_cfg.get("workspace_max_m", [1.0, 1.0, 1.0])
        ),
        max_pos_tracking_error_warn_m=float(relative_cfg.get("max_pos_tracking_error_warn_m", 0.03)),
        max_pos_tracking_error_pause_m=float(relative_cfg.get("max_pos_tracking_error_pause_m", 0.08)),
        max_q_tracking_error_pause_rad=float(relative_cfg.get("max_q_tracking_error_pause_rad", 0.25)),
        min_warn_time_scale=float(relative_cfg.get("min_warn_time_scale", 0.25)),
        phone_translation_deadband_m=float(relative_cfg.get("phone_translation_deadband_m", 0.003)),
        phone_rotation_deadband_rad=float(
            relative_cfg.get(
                "phone_rotation_deadband_rad",
                math.radians(float(relative_cfg.get("phone_rotation_deadband_deg", 1.0))),
            )
        ),
        phone_jump_reject_translation_m=float(relative_cfg.get("phone_jump_reject_translation_m", 0.25)),
        phone_jump_reject_rotation_rad=float(
            relative_cfg.get(
                "phone_jump_reject_rotation_rad",
                math.radians(float(relative_cfg.get("phone_jump_reject_rotation_deg", 45.0))),
            )
        ),
        phone_still_translation_m=float(relative_cfg.get("phone_still_translation_m", 0.002)),
        phone_still_rotation_rad=float(
            relative_cfg.get(
                "phone_still_rotation_rad",
                math.radians(float(relative_cfg.get("phone_still_rotation_deg", 0.5))),
            )
        ),
        phone_still_min_sec=float(relative_cfg.get("phone_still_min_sec", 0.0)),
        phone_still_freeze_tracking_error_m=float(
            relative_cfg.get("phone_still_freeze_tracking_error_m", 0.03)
        ),
        freeze_when_phone_still=bool(relative_cfg.get("freeze_when_phone_still", True)),
        target_filter_time_constant_sec=float(relative_cfg.get("target_filter_time_constant_sec", 0.10)),
        max_target_velocity_m_s=float(relative_cfg.get("max_target_velocity_m_s", 0.02)),
        max_target_acceleration_m_s2=float(relative_cfg.get("max_target_acceleration_m_s2", 0.0)),
        max_target_jump_m=float(relative_cfg.get("max_target_jump_m", 0.05)),
        target_update_deadband_m=float(relative_cfg.get("target_update_deadband_m", 0.0)),
        target_update_release_m=float(relative_cfg.get("target_update_release_m", 0.0)),
        reanchor_requires_deadman_release=bool(
            relative_cfg.get("reanchor_requires_deadman_release", False)
        ),
        orientation_control_enabled=bool(relative_cfg.get("orientation_control_enabled", False)),
        orientation_mapping_mode=str(relative_cfg.get("orientation_mapping_mode", "relative")),
        phone_back_camera_axis=tuple(
            float(v) for v in relative_cfg.get("phone_back_camera_axis", [0.0, 0.0, -1.0])
        ),
        phone_quaternion_convention=str(
            relative_cfg.get("phone_quaternion_convention", "body-to-world")
        ),
        orientation_scale=float(relative_cfg.get("orientation_scale", 1.0)),
        phone_to_robot_orientation_axis_map=direction_cfg.get("phone_to_robot_orientation"),
        phone_to_robot_axis_map=direction_cfg.get("phone_to_robot"),
    )


def _tcp_pose_from_pose_stamped(message: Any) -> TcpPose:
    position = message.pose.position
    orientation = message.pose.orientation
    return TcpPose(
        [float(position.x), float(position.y), float(position.z)],
        [float(orientation.w), float(orientation.x), float(orientation.y), float(orientation.z)],
    )


def run_node(*, config_path: str, jsonl_out: str | None = None) -> None:
    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
    except Exception as exc:
        raise RuntimeError("ROS2 Python packages are required. Source ROS2 first, e.g. scripts/source_ros2.sh.") from exc

    config = load_yaml(config_path)
    hebi_cfg = config.get("hebi", {})
    topics = config.get("topics", {})
    feedback_cfg = config.get("feedback", {})

    class HebiRealArmTeleopNode(Node):  # type: ignore[misc]
        def __init__(self) -> None:
            super().__init__("jaka_rh56_hebi_real_arm_teleop")
            self.client = HebiMobileIOClient(
                family=str(hebi_cfg.get("family", "HEBI")),
                name=str(hebi_cfg.get("name", "mobileIO")),
                lookup_wait_sec=float(hebi_cfg.get("lookup_wait_sec", 2.0)),
                setup_ui=bool(hebi_cfg.get("setup_ui", True)),
                max_stale_feedback_sec=float(hebi_cfg.get("max_stale_feedback_sec", 0.25)),
            )
            self.client.connect()
            self.follower = RelativePoseLagFollower(_relative_config_from_config(config))
            self.pub = self.create_publisher(String, topics.get("arm_palm_target_jog", "/jaka/teleop_palm_target_jog"), 10)
            self.pose_pub = self.create_publisher(String, topics.get("hebi_pose", "/teleop/hebi_mobile_io/pose"), 10)
            self.status_pub = self.create_publisher(String, topics.get("shadow_status", "/teleop/hebi_real_arm_status"), 10)
            self.latest_tcp_pose: TcpPose | None = None
            self.latest_arm_joints: list[float] | None = None
            self.latest_feedback_time_sec = 0.0
            self.feedback_timeout_sec = float(feedback_cfg.get("timeout_sec", 0.5))
            self.create_subscription(
                PoseStamped,
                topics.get("jaka_tcp_pose", "/jaka/tcp_pose"),
                self._on_tcp_pose,
                10,
            )
            self.create_subscription(
                JointState,
                topics.get("jaka_joint_states", "/jaka/joint_states"),
                self._on_joint_state,
                10,
            )
            self.log_handle = None
            if jsonl_out:
                path = Path(jsonl_out)
                path.parent.mkdir(parents=True, exist_ok=True)
                self.log_handle = path.open("a", encoding="utf-8")
            self.create_timer(1.0 / max(float(hebi_cfg.get("publish_hz", 30.0)), 1e-6), self._tick)
            self.get_logger().warning(
                "HEBI real arm teleop publishes direct relative palm targets. "
                "B1 anchors phone pose to current /jaka/tcp_pose; keep real bridge safety limits enabled."
            )

        def _on_tcp_pose(self, message: Any) -> None:
            self.latest_tcp_pose = _tcp_pose_from_pose_stamped(message)
            self.latest_feedback_time_sec = time.time()

        def _on_joint_state(self, message: Any) -> None:
            joints = extract_arm_joints_from_joint_state(message)
            if joints is not None:
                self.latest_arm_joints = joints
                self.latest_feedback_time_sec = time.time()

        def _publish_json(self, publisher: Any, payload: dict[str, Any]) -> None:
            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=False)
            try:
                publisher.publish(msg)
            except Exception:
                if rclpy.ok():
                    raise

        def _tick(self) -> None:
            snapshot = self.client.read(timeout_ms=float(hebi_cfg.get("read_timeout_ms", 10.0)))
            now = time.time()
            feedback_fresh = (
                self.latest_tcp_pose is not None
                and self.latest_arm_joints is not None
                and now - self.latest_feedback_time_sec <= self.feedback_timeout_sec
            )
            follower_log: dict[str, Any]
            if feedback_fresh:
                assert self.latest_tcp_pose is not None
                assert self.latest_arm_joints is not None
                output = self.follower.step(
                    snapshot,
                    self.latest_tcp_pose,
                    self.latest_arm_joints,
                    timestamp_sec=now,
                )
                deadman = output.command_deadman
                follower_log = dict(output.log)
                hold_current = bool(deadman and follower_log.get("still_freeze", False))
                palm_target_position_m = (
                    None
                    if hold_current or not deadman
                    else output.palm_target_position_m
                )
                palm_target_quaternion_wxyz = (
                    None
                    if hold_current or not deadman
                    else output.palm_target_quaternion_wxyz
                )
                wrist_roll_velocity_rad_s = output.wrist_roll_velocity_rad_s if deadman else 0.0
            else:
                self.follower.reset()
                deadman = False
                hold_current = False
                palm_target_position_m = None
                palm_target_quaternion_wxyz = None
                wrist_roll_velocity_rad_s = 0.0
                follower_log = {
                    "command_deadman": False,
                    "reason": "waiting_for_fresh_jaka_feedback",
                    "feedback_fresh": False,
                    "has_tcp_pose": self.latest_tcp_pose is not None,
                    "has_arm_joints": self.latest_arm_joints is not None,
                }
            command = {
                "schema_version": "hebi_mobile_io_palm_target_jog_v0.1",
                "timestamp_sec": now,
                "source": "hebi_mobile_io",
                "deadman": deadman,
                "hold_current": hold_current,
                "palm_velocity_m_s": [0.0, 0.0, 0.0],
                "wrist_roll_velocity_rad_s": wrist_roll_velocity_rad_s,
                "palm_target_position_m": palm_target_position_m,
                "palm_target_quaternion_wxyz": palm_target_quaternion_wxyz,
            }
            self._publish_json(self.pub, command)
            self._publish_json(
                self.pose_pub,
                {
                    "snapshot": snapshot.to_dict(),
                    "action": command,
                    "follower": follower_log,
                },
            )
            self._publish_json(
                self.status_pub,
                {
                    "snapshot_valid": snapshot.valid,
                    "deadman": deadman,
                    "hold_current": hold_current,
                    "feedback_fresh": feedback_fresh,
                    "target_response_mode": self.follower.config.target_response_mode,
                    "reason": follower_log.get("reason", "ok"),
                },
            )
            if self.log_handle is not None:
                self.log_handle.write(
                    json.dumps(
                        {
                            "snapshot": snapshot.to_dict(),
                            "command": command,
                            "follower": follower_log,
                        },
                        ensure_ascii=False,
                    )
                    + "\n"
                )
                self.log_handle.flush()
            if bool(snapshot.raw_inputs.get("b8", False)):
                raise KeyboardInterrupt

        def destroy_node(self) -> bool:
            if self.log_handle is not None:
                self.log_handle.close()
                self.log_handle = None
            return super().destroy_node()

    signal.signal(signal.SIGTERM, lambda _signum, _frame: (_ for _ in ()).throw(KeyboardInterrupt()))
    rclpy.init()
    node = HebiRealArmTeleopNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser(description="Publish HEBI Mobile I/O palm-target jog commands for the real JAKA bridge.")
    parser.add_argument("--config", default="configs/teleop/hebi_mobile_io_jaka_rh56.yaml")
    parser.add_argument("--jsonl-out", default=None)
    parser.add_argument("--enable-motion", action="store_true", help="Compatibility flag; motion is still gated by B1 and the real bridge.")
    parser.add_argument("--teleop-mode", default="direct")
    parser.add_argument("--teleop-profile", default="practical")
    args = parser.parse_args()
    del args.enable_motion, args.teleop_mode, args.teleop_profile
    run_node(config_path=args.config, jsonl_out=args.jsonl_out)


if __name__ == "__main__":
    main()
