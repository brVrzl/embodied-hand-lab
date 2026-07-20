from __future__ import annotations

import argparse
import json
import signal
import time
from pathlib import Path
from typing import Any

from embodiment_core.config import load_yaml
from teleop_tools.hebi_mobile_io import HebiMobileIOSnapshot
from teleop_tools.pose_teleop_config import relative_pose_config_from_mapping
from teleop_tools.relative_pose_lag_follow import RelativePoseLagFollower, TcpPose
from teleop_tools.rviz_shadow_sync import extract_arm_joints_from_joint_state
from teleop_tools.teledex_calibration import calibration_from_project_config
from teleop_tools.teledex_phone import TeleDexPhoneClient
from teleop_tools.xbox_ros2 import PygameXboxController


DEFAULT_CONFIG = "configs/teleop/teledex_jaka_arm.yaml"


def _tcp_pose_from_pose_stamped(message: Any) -> TcpPose:
    position = message.pose.position
    orientation = message.pose.orientation
    return TcpPose(
        [float(position.x), float(position.y), float(position.z)],
        [float(orientation.w), float(orientation.x), float(orientation.y), float(orientation.z)],
    )


def run_node(
    *,
    config_path: str,
    jsonl_out: str | None = None,
    enable_motion: bool = False,
    deadman_source_override: str | None = None,
    allow_unconfirmed_calibration_for_this_run: bool = False,
    position_scale_override: float | None = None,
    max_target_velocity_override_m_s: float | None = None,
    max_target_acceleration_override_m_s2: float | None = None,
    workspace_min_override_m: list[float] | None = None,
    workspace_max_override_m: list[float] | None = None,
) -> None:
    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
    except Exception as exc:
        raise RuntimeError(
            "ROS2 Python packages are required. Source ROS2 first, e.g. scripts/source_ros2.sh."
        ) from exc

    config = load_yaml(config_path)
    relative_cfg = config.setdefault("relative_pose_lag_follow", {})
    if position_scale_override is not None:
        relative_cfg["position_scale"] = float(position_scale_override)
    if max_target_velocity_override_m_s is not None:
        relative_cfg["max_target_velocity_m_s"] = float(max_target_velocity_override_m_s)
    if max_target_acceleration_override_m_s2 is not None:
        relative_cfg["max_target_acceleration_m_s2"] = float(
            max_target_acceleration_override_m_s2
        )
    if workspace_min_override_m is not None:
        relative_cfg["workspace_min_m"] = [float(value) for value in workspace_min_override_m]
    if workspace_max_override_m is not None:
        relative_cfg["workspace_max_m"] = [float(value) for value in workspace_max_override_m]

    teledex_cfg = config.get("teledex", {})
    topics = config.get("topics", {})
    feedback_cfg = config.get("feedback", {})
    deadman_cfg = config.get("deadman", {})
    deadman_source = str(
        deadman_source_override or deadman_cfg.get("source", "teledex_button_a")
    )
    if deadman_source not in {
        "connected_phone",
        "xbox_rb",
        "teledex_button",
        "teledex_button_a",
        "teledex_button_b",
        "teledex_toggle",
    }:
        raise ValueError(
            "unsupported run gate; use teledex_button_a, teledex_button_b, "
            "teledex_toggle, connected_phone, or xbox_rb."
        )
    calibration_cfg = config.get("calibration", {})
    calibration, calibration_reason = calibration_from_project_config(config)
    calibration_required = bool(calibration_cfg.get("required_for_real_motion", True))
    calibration_confirmed = bool(
        calibration is not None and calibration.get("real_motion_confirmed", False)
    )
    unconfirmed_trial = bool(
        enable_motion
        and calibration_required
        and not calibration_confirmed
        and allow_unconfirmed_calibration_for_this_run
    )
    if enable_motion and calibration_required and not calibration_confirmed and not unconfirmed_trial:
        raise RuntimeError(
            "TeleDex real motion is blocked because frame calibration is not confirmed "
            f"({calibration_reason}). Run scripts/calibrate_teledex_jaka_frame.sh, verify all "
            "six directions in shadow, then explicitly confirm the calibration."
        )
    if unconfirmed_trial:
        if calibration is None:
            raise RuntimeError("An unconfirmed real trial still requires a valid calibration file.")
        matrix = calibration.get("phone_to_robot_rotation_matrix")
        flattened = [float(value) for row in matrix for value in row]
        if any(value not in {-1.0, 0.0, 1.0} for value in flattened):
            raise RuntimeError(
                "The unconfirmed real-trial override only accepts an exact 0/±1 signed-axis matrix."
            )
        if deadman_source not in {"teledex_button", "teledex_button_a"}:
            raise RuntimeError(
                "The unconfirmed real-trial override requires TeleDex Button A as the run gate."
            )
        if bool(relative_cfg.get("orientation_control_enabled", False)):
            raise RuntimeError(
                "The unconfirmed real-trial override requires orientation control to remain disabled."
            )
    calibration_matrix = (
        None if calibration is None else calibration["phone_to_robot_rotation_matrix"]
    )

    class TeleDexRealArmTeleopNode(Node):  # type: ignore[misc]
        def __init__(self) -> None:
            super().__init__("jaka_teledex_real_arm_teleop")
            self.motion_enabled = bool(enable_motion)
            self.deadman_source = deadman_source
            self.deadman_controller = None
            if self.motion_enabled and self.deadman_source == "xbox_rb":
                self.deadman_controller = PygameXboxController(
                    index=int(deadman_cfg.get("gamepad_index", 0)),
                    deadzone=float(deadman_cfg.get("deadzone", 0.12)),
                )
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
            self.follower = RelativePoseLagFollower(
                relative_pose_config_from_mapping(
                    config,
                    phone_to_robot_rotation_matrix=calibration_matrix,
                )
            )
            self.command_pub = self.create_publisher(
                String,
                topics.get("arm_palm_target_jog", "/jaka/teleop_palm_target_jog"),
                10,
            )
            self.pose_pub = self.create_publisher(
                String,
                topics.get("teledex_pose", "/teleop/teledex/pose"),
                10,
            )
            self.status_pub = self.create_publisher(
                String,
                topics.get("teledex_status", "/teleop/teledex/status"),
                10,
            )
            self.latest_tcp_pose: TcpPose | None = None
            self.latest_arm_joints: list[float] | None = None
            self.latest_tcp_feedback_sec = 0.0
            self.latest_joint_feedback_sec = 0.0
            self.feedback_timeout_sec = float(feedback_cfg.get("timeout_sec", 0.30))
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
            self.create_timer(
                1.0 / max(float(teledex_cfg.get("publish_hz", 30.0)), 1e-6),
                self._tick,
            )
            self.get_logger().warning(
                f"TeleDex real-arm publisher ready at {self.client.address}; "
                f"motion_enabled={self.motion_enabled}, calibration={calibration_reason}, "
                f"unconfirmed_trial={unconfirmed_trial}, "
                f"orientation_enabled={self.follower.config.orientation_control_enabled}. "
                f"run_gate={self.deadman_source}. Release Button A to stop when using the "
                "default gate; Freeze Pose and Reset Pose are not stop controls."
            )

        def _on_tcp_pose(self, message: Any) -> None:
            self.latest_tcp_pose = _tcp_pose_from_pose_stamped(message)
            self.latest_tcp_feedback_sec = time.time()

        def _on_joint_state(self, message: Any) -> None:
            joints = extract_arm_joints_from_joint_state(message)
            if joints is not None:
                self.latest_arm_joints = joints
                self.latest_joint_feedback_sec = time.time()

        @staticmethod
        def _json_message(payload: dict[str, Any]) -> Any:
            message = String()
            message.data = json.dumps(payload, ensure_ascii=False)
            return message

        def _publish_json(self, publisher: Any, payload: dict[str, Any]) -> None:
            try:
                publisher.publish(self._json_message(payload))
            except Exception:
                if rclpy.ok():
                    raise

        def _tick(self) -> None:
            phone_snapshot = self.client.read()
            if self.deadman_source == "xbox_rb":
                deadman_input = bool(
                    self.deadman_controller is not None
                    and self.deadman_controller.snapshot().buttons.get("rb", False)
                )
            elif self.deadman_source == "connected_phone":
                deadman_input = bool(phone_snapshot.valid)
            elif self.deadman_source == "teledex_button_b":
                deadman_input = bool(
                    phone_snapshot.raw_inputs.get("button_secondary", False)
                )
            elif self.deadman_source == "teledex_toggle":
                deadman_input = bool(phone_snapshot.raw_inputs.get("toggle", False))
            else:
                deadman_input = bool(phone_snapshot.raw_inputs.get("button", False))
            snapshot = HebiMobileIOSnapshot(
                timestamp_sec=phone_snapshot.timestamp_sec,
                position_m=phone_snapshot.position_m,
                quaternion_wxyz=phone_snapshot.quaternion_wxyz,
                raw_inputs={
                    **phone_snapshot.raw_inputs,
                    "b1": bool(phone_snapshot.valid and deadman_input),
                    "external_deadman_source": self.deadman_source,
                },
                valid=phone_snapshot.valid,
                reason=phone_snapshot.reason,
            )
            now = time.time()
            feedback_fresh = (
                self.latest_tcp_pose is not None
                and self.latest_arm_joints is not None
                and now - self.latest_tcp_feedback_sec <= self.feedback_timeout_sec
                and now - self.latest_joint_feedback_sec <= self.feedback_timeout_sec
            )
            if feedback_fresh:
                assert self.latest_tcp_pose is not None
                assert self.latest_arm_joints is not None
                output = self.follower.step(
                    snapshot,
                    self.latest_tcp_pose,
                    self.latest_arm_joints,
                    timestamp_sec=now,
                )
                follower_log = dict(output.log)
                deadman = bool(self.motion_enabled and output.command_deadman)
                hold_current = bool(deadman and follower_log.get("still_freeze", False))
                target_position = (
                    output.palm_target_position_m if deadman and not hold_current else None
                )
                target_quaternion = (
                    output.palm_target_quaternion_wxyz if deadman and not hold_current else None
                )
            else:
                self.follower.reset()
                follower_log = {
                    "command_deadman": False,
                    "reason": "waiting_for_fresh_jaka_feedback",
                    "has_tcp_pose": self.latest_tcp_pose is not None,
                    "has_arm_joints": self.latest_arm_joints is not None,
                }
                deadman = False
                hold_current = False
                target_position = None
                target_quaternion = None
            if not self.motion_enabled:
                follower_log["reason"] = "motion_not_enabled"

            command = {
                "schema_version": "teledex_palm_target_jog_v0.1",
                "timestamp_sec": now,
                "source": "teledex",
                "deadman": deadman,
                "hold_current": hold_current,
                # The source target is expressed in the JAKA controller TCP
                # frame while the bridge IK tracks the modeled RH56 palm body.
                # Align both origins on the deadman rising edge and preserve
                # only the operator-commanded relative displacement.
                "align_position_target_on_enable": True,
                "palm_velocity_m_s": [0.0, 0.0, 0.0],
                "wrist_roll_velocity_rad_s": 0.0,
                "palm_target_position_m": target_position,
                "palm_target_quaternion_wxyz": target_quaternion,
            }
            self._publish_json(self.command_pub, command)
            self._publish_json(
                self.pose_pub,
                {
                    "schema_version": "teledex_phone_pose_v0.1",
                    "snapshot": snapshot.to_dict(),
                    "action": command,
                    "follower": follower_log,
                },
            )
            self._publish_json(
                self.status_pub,
                {
                    "snapshot_valid": snapshot.valid,
                    "phone_connected": self.client.is_connected,
                    "motion_enabled": self.motion_enabled,
                    "deadman_source": self.deadman_source,
                    "deadman_input": deadman_input,
                    "deadman": deadman,
                    "feedback_fresh": feedback_fresh,
                    "calibration": calibration_reason,
                    "calibration_real_motion_confirmed": calibration_confirmed,
                    "unconfirmed_trial": unconfirmed_trial,
                    "orientation_control_enabled": self.follower.config.orientation_control_enabled,
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

        def destroy_node(self) -> bool:
            stop_command = {
                "schema_version": "teledex_palm_target_jog_v0.1",
                "timestamp_sec": time.time(),
                "source": "teledex_shutdown",
                "deadman": False,
                "hold_current": False,
                "palm_velocity_m_s": [0.0, 0.0, 0.0],
                "wrist_roll_velocity_rad_s": 0.0,
                "palm_target_position_m": None,
                "palm_target_quaternion_wxyz": None,
            }
            try:
                self._publish_json(self.command_pub, stop_command)
            finally:
                self.client.close()
                if self.deadman_controller is not None:
                    self.deadman_controller.pygame.quit()
                if self.log_handle is not None:
                    self.log_handle.close()
                    self.log_handle = None
            return super().destroy_node()

    signal.signal(
        signal.SIGTERM,
        lambda _signum, _frame: (_ for _ in ()).throw(KeyboardInterrupt()),
    )
    rclpy.init()
    node = TeleDexRealArmTeleopNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Publish calibrated TeleDex relative palm targets for the real JAKA bridge."
    )
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--jsonl-out", default=None)
    parser.add_argument(
        "--enable-motion",
        action="store_true",
        help="Allow run-gated commands; also requires a confirmed frame calibration.",
    )
    parser.add_argument(
        "--deadman-source",
        choices=(
            "teledex_button_a",
            "teledex_button_b",
            "teledex_toggle",
            "connected_phone",
            "xbox_rb",
        ),
        default=None,
        help=(
            "Override the configured run gate. Button A is the safe default; "
            "connected_phone removes the momentary hold requirement."
        ),
    )
    parser.add_argument(
        "--allow-unconfirmed-calibration-for-this-run",
        action="store_true",
        help=(
            "One-process real trial override. Requires an exact signed-axis calibration, "
            "Button A, and translation-only control; does not modify the calibration file."
        ),
    )
    parser.add_argument("--position-scale", type=float, default=None)
    parser.add_argument("--max-target-velocity-m-s", type=float, default=None)
    parser.add_argument("--max-target-acceleration-m-s2", type=float, default=None)
    parser.add_argument("--workspace-min-m", type=float, nargs=3, default=None)
    parser.add_argument("--workspace-max-m", type=float, nargs=3, default=None)
    args = parser.parse_args()
    run_node(
        config_path=args.config,
        jsonl_out=args.jsonl_out,
        enable_motion=args.enable_motion,
        deadman_source_override=args.deadman_source,
        allow_unconfirmed_calibration_for_this_run=(
            args.allow_unconfirmed_calibration_for_this_run
        ),
        position_scale_override=args.position_scale,
        max_target_velocity_override_m_s=args.max_target_velocity_m_s,
        max_target_acceleration_override_m_s2=args.max_target_acceleration_m_s2,
        workspace_min_override_m=args.workspace_min_m,
        workspace_max_override_m=args.workspace_max_m,
    )


if __name__ == "__main__":
    main()
