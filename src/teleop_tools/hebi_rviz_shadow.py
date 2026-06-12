from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from typing import Any

from embodiment_core.config import load_yaml
from jaka_driver_adapter.palm_target_ik import DEFAULT_MJCF, PalmTargetIkState
from teleop_tools.hebi_mobile_io import HebiMobileIOClient
from teleop_tools.relative_pose_lag_follow import (
    IDENTITY_QUAT_WXYZ,
    IkCheckResult,
    RelativePoseLagFollower,
    RelativePoseLagFollowConfig,
    TcpPose,
)
from teleop_tools.rviz_shadow_sync import ARM_JOINT_NAMES, extract_arm_joints_from_joint_state


DEFAULT_CONFIG = "configs/teleop/hebi_mobile_io_jaka_rh56.yaml"


def _shadow_state_from_config(config: dict[str, Any], initial_joints: list[float] | None) -> PalmTargetIkState:
    shadow_cfg = config.get("shadow", {})
    return PalmTargetIkState(
        mjcf_path=shadow_cfg.get("mjcf_path", DEFAULT_MJCF),
        initial_arm_joints_rad=initial_joints or shadow_cfg.get("initial_arm_joints_rad", [0.0] * 6),
        ik_gain=float(shadow_cfg.get("ik_gain", 0.7)),
        ik_damping=float(shadow_cfg.get("ik_damping", 0.05)),
        ik_max_step_rad=float(shadow_cfg.get("ik_max_step_rad", 0.08)),
        ik_iterations=int(shadow_cfg.get("ik_iterations", 20)),
        target_workspace_radius_m=float(shadow_cfg.get("target_workspace_radius_m", 0.08)),
    )


def _relative_config_from_config(config: dict[str, Any]) -> RelativePoseLagFollowConfig:
    relative_cfg = config.get("relative_pose_lag_follow", {})
    direction_cfg = config.get("direction_calibration", {})
    return RelativePoseLagFollowConfig(
        target_response_mode=str(relative_cfg.get("target_response_mode", "lag_follow")),
        position_scale=float(relative_cfg.get("position_scale", 1.0)),
        max_step_position_m=float(relative_cfg.get("max_step_position_m", 0.01)),
        max_step_rotation_rad=float(relative_cfg.get("max_step_rotation_rad", math.radians(float(relative_cfg.get("max_step_rotation_deg", 2.0))))),
        max_target_lead_m=float(relative_cfg.get("max_target_lead_m", 0.08)),
        workspace_min_m=tuple(float(v) for v in relative_cfg.get("workspace_min_m", [-1.0, -1.0, -1.0])),
        workspace_max_m=tuple(float(v) for v in relative_cfg.get("workspace_max_m", [1.0, 1.0, 1.0])),
        max_pos_tracking_error_warn_m=float(relative_cfg.get("max_pos_tracking_error_warn_m", 0.03)),
        max_pos_tracking_error_pause_m=float(relative_cfg.get("max_pos_tracking_error_pause_m", 0.08)),
        phone_translation_deadband_m=float(relative_cfg.get("phone_translation_deadband_m", 0.003)),
        phone_rotation_deadband_rad=float(relative_cfg.get("phone_rotation_deadband_rad", math.radians(float(relative_cfg.get("phone_rotation_deadband_deg", 1.0))))),
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
        phone_still_freeze_tracking_error_m=float(relative_cfg.get("phone_still_freeze_tracking_error_m", 0.03)),
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
        phone_to_robot_axis_map=direction_cfg.get("phone_to_robot"),
    )


def _actual_palm_pose_from_state(state: PalmTargetIkState) -> TcpPose:
    return TcpPose(
        state.current_palm_position_m.astype(float).tolist(),
        list(IDENTITY_QUAT_WXYZ),
    )


def _make_shadow_ik_checker(ik_state: PalmTargetIkState):
    def _check_relative_target_ik(pose: TcpPose, q_current: list[float]) -> IkCheckResult:
        previous_joints = ik_state.arm_joints_rad.copy()
        previous_target = ik_state.target_palm_position_m.copy()
        ik_state.set_arm_joints_rad(q_current)
        ik_state.apply_position_target(
            palm_target_position_m=pose.position_m,
            wrist_roll_velocity_rad_s=0.0,
            dt=0.0,
        )
        q_cmd = ik_state.arm_joints_rad.tolist()
        ik_state.set_arm_joints_rad(previous_joints.tolist())
        ik_state.target_palm_position_m = previous_target
        if not all(math.isfinite(value) for value in q_cmd):
            return IkCheckResult(False, None, True, "ik_nonfinite_q")
        return IkCheckResult(True, [float(value) for value in q_cmd])

    return _check_relative_target_ik


def run_hebi_rviz_shadow_node(
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
        raise RuntimeError("ROS2 Python packages are required. Source ROS2 Humble first.") from exc

    config = load_yaml(config_path)
    hebi_cfg = config.get("hebi", {})
    ik_check_state = _shadow_state_from_config(config, initial_arm_joints_rad)

    class HebiRvizShadowNode(Node):  # type: ignore[misc]
        def __init__(self) -> None:
            super().__init__("jaka_rh56_hebi_rviz_shadow")
            self.client = HebiMobileIOClient(
                family=str(hebi_cfg.get("family", "HEBI")),
                name=str(hebi_cfg.get("name", "mobileIO")),
                lookup_wait_sec=float(hebi_cfg.get("lookup_wait_sec", 2.0)),
                setup_ui=bool(hebi_cfg.get("setup_ui", True)),
                max_stale_feedback_sec=float(hebi_cfg.get("max_stale_feedback_sec", 0.25)),
            )
            self.client.connect()
            self.ik_state = ik_check_state
            self.follower = RelativePoseLagFollower(
                _relative_config_from_config(config),
                ik_checker=_make_shadow_ik_checker(self.ik_state),
            )
            topics = config.get("topics", {})
            self.shadow_pub = self.create_publisher(JointState, topics.get("shadow_joint_states", "/teleop/shadow_joint_states"), 10)
            self.status_pub = self.create_publisher(String, topics.get("shadow_status", "/teleop/shadow_status"), 10)
            self.pose_pub = self.create_publisher(String, topics.get("hebi_pose", "/teleop/hebi_mobile_io/pose"), 10)
            self.target_pub = self.create_publisher(Marker, topics.get("shadow_palm_target", "/teleop/shadow_palm_target"), 10)
            self.real_arm_joints: list[float] | None = None
            self.last_real_arm_time = 0.0
            self.create_subscription(JointState, real_arm_joint_topic, self._on_real_arm_joint_state, 10)
            self.create_timer(1.0 / max(float(hebi_cfg.get("publish_hz", 30.0)), 1e-6), self._publish)
            self.get_logger().info(
                f"HEBI Mobile I/O RViz shadow ready. Syncing idle pose from {real_arm_joint_topic}. "
                "B1 enables/resets pose. No hardware command topics are published."
            )

        def _on_real_arm_joint_state(self, message: Any) -> None:
            joints = extract_arm_joints_from_joint_state(message)
            if joints is not None:
                self.real_arm_joints = joints
                self.last_real_arm_time = time.time()

        def _publish_json(self, publisher: Any, payload: dict[str, Any]) -> None:
            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=False)
            publisher.publish(msg)

        def _publish(self) -> None:
            snapshot = self.client.read(timeout_ms=float(hebi_cfg.get("read_timeout_ms", 10.0)))
            q_current = self.real_arm_joints or self.ik_state.arm_joints_rad.tolist()
            actual = _actual_palm_pose_from_state(self.ik_state)
            output = self.follower.step(snapshot, actual, q_current, timestamp_sec=time.time())
            if output.command_deadman and output.palm_target_position_m is not None:
                q_cmd = output.log.get("q_cmd") or q_current
                self.ik_state.set_arm_joints_rad(q_cmd)
            joint_msg = JointState()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.name = list(ARM_JOINT_NAMES)
            joint_msg.position = self.ik_state.arm_joints_rad.tolist()
            self.shadow_pub.publish(joint_msg)
            real_arm_sync_active = time.time() - self.last_real_arm_time <= real_arm_sync_timeout_sec
            self._publish_json(
                self.pose_pub,
                {"schema_version": "hebi_mobile_io_pose_v0.1", "snapshot": snapshot.to_dict()},
            )
            status = dict(output.log)
            status.update(
                {
                    "mode": "hebi_mobile_io_rviz_shadow_palm_target_ik",
                    "hardware_commands_published": False,
                    "real_arm_joint_topic": real_arm_joint_topic,
                    "real_arm_sync_active": real_arm_sync_active,
                    "deadman": snapshot.enabled,
                }
            )
            self._publish_json(self.status_pub, status)
            target = output.palm_target_position_m
            if target is not None:
                marker = Marker()
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.header.frame_id = "jaka_Link_0"
                marker.ns = "hebi_rviz_shadow"
                marker.id = 0
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = float(target[0])
                marker.pose.position.y = float(target[1])
                marker.pose.position.z = float(target[2])
                marker.pose.orientation.w = 1.0
                marker.scale.x = marker.scale.y = marker.scale.z = 0.05
                marker.color.r = 0.1
                marker.color.g = 0.4
                marker.color.b = 1.0
                marker.color.a = 0.9
                self.target_pub.publish(marker)
            if bool(snapshot.raw_inputs.get("b8", False)):
                self.get_logger().info("B8 pressed; shutting down HEBI RViz shadow.")
                raise KeyboardInterrupt

    rclpy.init()
    node = HebiRvizShadowNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Preview HEBI Mobile I/O arm teleop in RViz without hardware commands.")
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--initial-joints", nargs=6, type=float, default=None)
    parser.add_argument("--real-arm-joint-topic", default="/jaka/joint_states")
    parser.add_argument("--real-arm-sync-timeout-sec", type=float, default=1.0)
    args = parser.parse_args(argv)
    run_hebi_rviz_shadow_node(
        config_path=args.config,
        initial_arm_joints_rad=args.initial_joints,
        real_arm_joint_topic=args.real_arm_joint_topic,
        real_arm_sync_timeout_sec=args.real_arm_sync_timeout_sec,
    )


if __name__ == "__main__":
    main()
