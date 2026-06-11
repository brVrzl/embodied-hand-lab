from __future__ import annotations

import argparse
import json
import math
import signal
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

from embodiment_core.config import load_yaml
from embodiment_core.types import HandState, JointState
from jaka_driver_adapter.adapter import JakaDriverAdapter
from jaka_driver_adapter.jaka_sdk_backend import JakaSDKBackend
from jaka_driver_adapter.servo_jog import (
    JakaPalmTargetJogController,
    parse_palm_target_jog_command,
)
from rh56_driver.hand_schema import CANONICAL_HAND_ORDER
from rh56_driver.node import RH56Driver
from rh56_driver.ros2_bridge import (
    apply_angle_command,
    apply_force_command,
    build_raw_feedback_payload,
    parse_angle_command,
    parse_code_command,
    parse_force_command,
)


SCHEMA_VERSION = "palmhand_lab.ros2_bridge.v1"

DEFAULT_JAKA_JOINT_STATES_TOPIC = "/jaka/joint_states"
DEFAULT_JAKA_TCP_POSE_TOPIC = "/jaka/tcp_pose"
DEFAULT_JAKA_STATE_FLAGS_TOPIC = "/jaka/state_flags"
DEFAULT_JAKA_TELEOP_PALM_TARGET_JOG_TOPIC = "/jaka/teleop_palm_target_jog"
DEFAULT_JAKA_TELEOP_STATUS_TOPIC = "/jaka/teleop_status"
DEFAULT_JAKA_TELEOP_PALM_TARGET_TOPIC = "/jaka/teleop_palm_target"
DEFAULT_RH56_STATE_TOPIC = "/rh56/state"
DEFAULT_RH56_RAW_FEEDBACK_TOPIC = "/rh56/raw_feedback"
DEFAULT_RH56_COMMAND_ANGLES_TOPIC = "/rh56/command_angles"
DEFAULT_RH56_COMMAND_FORCE_TOPIC = "/rh56/command_force"
DEFAULT_RH56_COMMAND_CODE_TOPIC = "/rh56/command_code"


@dataclass(frozen=True, slots=True)
class Ros2BridgeTopics:
    jaka_joint_states: str = DEFAULT_JAKA_JOINT_STATES_TOPIC
    jaka_tcp_pose: str = DEFAULT_JAKA_TCP_POSE_TOPIC
    jaka_state_flags: str = DEFAULT_JAKA_STATE_FLAGS_TOPIC
    jaka_teleop_palm_target_jog: str = DEFAULT_JAKA_TELEOP_PALM_TARGET_JOG_TOPIC
    jaka_teleop_status: str = DEFAULT_JAKA_TELEOP_STATUS_TOPIC
    jaka_teleop_palm_target: str = DEFAULT_JAKA_TELEOP_PALM_TARGET_TOPIC
    rh56_state: str = DEFAULT_RH56_STATE_TOPIC
    rh56_raw_feedback: str = DEFAULT_RH56_RAW_FEEDBACK_TOPIC
    rh56_command_angles: str = DEFAULT_RH56_COMMAND_ANGLES_TOPIC
    rh56_command_force: str = DEFAULT_RH56_COMMAND_FORCE_TOPIC
    rh56_command_code: str = DEFAULT_RH56_COMMAND_CODE_TOPIC


def rpy_to_quaternion_xyzw(roll: float, pitch: float, yaw: float) -> list[float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]


def build_hand_state_payload(
    state: HandState,
    *,
    backend_mode: str,
    timestamp_sec: float | None = None,
) -> dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION,
        "timestamp_sec": time.time() if timestamp_sec is None else float(timestamp_sec),
        "backend_mode": backend_mode,
        "frame_id": "rh56_palm",
        "canonical_order": list(CANONICAL_HAND_ORDER),
        "finger_state": {
            "angle_count_0_1000": list(state.finger_positions),
            "current_count": list(state.finger_currents),
            "force_count": list(state.force_estimate),
            "contact_flags_from_force_positive": [bool(value) for value in state.contact_flags],
        },
        "units": {
            "angle_count_0_1000": "vendor_count_open_1000_close_0",
            "current_count": "vendor_raw_count",
            "force_count": "signed_vendor_raw_count",
        },
        "mode": state.mode,
    }


def evaluate_hand_safety(
    state: HandState,
    safety_config: dict[str, Any],
) -> dict[str, Any]:
    current_values = [abs(float(value)) for value in state.finger_currents]
    force_values = [abs(float(value)) for value in state.force_estimate]
    max_current = max(current_values) if current_values else 0.0
    max_force = max(force_values) if force_values else 0.0
    current_warn = float(safety_config.get("current_warn_count", 0.0) or 0.0)
    current_stop = float(safety_config.get("current_stop_count", 0.0) or 0.0)
    force_stop = float(safety_config.get("force_stop_count", 0.0) or 0.0)
    current_warned = current_warn > 0.0 and max_current >= current_warn
    current_stopped = current_stop > 0.0 and max_current >= current_stop
    force_stopped = force_stop > 0.0 and max_force >= force_stop
    reasons: list[str] = []
    if current_stopped:
        reasons.append("hand_current_stop")
    if force_stopped:
        reasons.append("hand_force_stop")
    return {
        "enabled": bool(safety_config.get("estop_enabled", True)),
        "max_current_count": max_current,
        "max_force_count": max_force,
        "current_warn_count": current_warn,
        "current_stop_count": current_stop,
        "force_stop_count": force_stop,
        "current_warned": current_warned,
        "stop": bool(reasons),
        "reasons": reasons,
    }


def build_jaka_state_flags_payload(
    backend: JakaSDKBackend,
    *,
    timestamp_sec: float | None = None,
) -> dict[str, Any]:
    flags = collect_jaka_state_flags(backend)
    return {
        "schema_version": SCHEMA_VERSION,
        "timestamp_sec": time.time() if timestamp_sec is None else float(timestamp_sec),
        "controller_ip": backend.config.get("ip"),
        "state_flags": flags,
    }


def collect_jaka_state_flags(backend: JakaSDKBackend) -> dict[str, Any]:
    flags: dict[str, Any] = {}
    robot = getattr(backend, "_robot", None)
    if robot is not None:
        for method_name in (
            "is_in_estop",
            "is_in_collision",
            "is_on_limit",
            "is_in_drag_mode",
            "is_in_servomove",
            "is_in_pos",
        ):
            if hasattr(robot, method_name):
                flags[method_name] = _extract_scalar(backend.call_sdk_method(method_name))
        if hasattr(robot, "protective_stop_status"):
            flags["protective_stop_status"] = _extract_scalar(
                backend.call_sdk_method("protective_stop_status")
            )
        if hasattr(robot, "get_last_error"):
            flags["last_error_raw"] = _extract_scalar(backend.call_sdk_method("get_last_error"))
    return flags


def build_joint_state_dict(joint_state: JointState, *, frame_id: str = "jaka_base") -> dict[str, Any]:
    return {
        "frame_id": frame_id,
        "name": list(joint_state.names),
        "position_rad": list(joint_state.positions),
        "velocity_rad_s": list(joint_state.velocities),
        "effort": list(joint_state.efforts),
    }


def extract_jaka_tcp_pose(backend: JakaSDKBackend) -> dict[str, Any]:
    robot = getattr(backend, "_robot", None)
    if robot is None:
        raise RuntimeError("JAKA SDK backend is not connected.")
    if hasattr(robot, "get_actual_tcp_position"):
        return _extract_pose_payload(backend.call_sdk_method("get_actual_tcp_position"))
    if hasattr(robot, "get_tcp_position"):
        return _extract_pose_payload(backend.call_sdk_method("get_tcp_position"))
    raise RuntimeError("JAKA SDK object exposes no TCP pose getter.")


def _extract_payload(result: Any) -> Any:
    if isinstance(result, tuple) and len(result) >= 2:
        return result[1]
    return result


def _extract_scalar(result: Any) -> Any:
    payload = _extract_payload(result)
    if isinstance(payload, tuple) and len(payload) == 1:
        return payload[0]
    return payload


def _extract_pose_payload(result: Any) -> dict[str, Any]:
    payload = _extract_payload(result)
    if hasattr(payload, "tran") and hasattr(payload, "rpy"):
        rpy = [
            float(getattr(payload.rpy, "rx")),
            float(getattr(payload.rpy, "ry")),
            float(getattr(payload.rpy, "rz")),
        ]
        return {
            "frame_id": "jaka_base",
            "translation_m": [
                float(getattr(payload.tran, "x")) / 1000.0,
                float(getattr(payload.tran, "y")) / 1000.0,
                float(getattr(payload.tran, "z")) / 1000.0,
            ],
            "rpy_rad": rpy,
            "quaternion_xyzw": rpy_to_quaternion_xyzw(*rpy),
            "source_units": {"translation": "mm", "rotation": "rad"},
        }
    if isinstance(payload, (list, tuple)) and len(payload) >= 6:
        rpy = [float(v) for v in payload[3:6]]
        return {
            "frame_id": "jaka_base",
            "translation_m": [float(v) / 1000.0 for v in payload[:3]],
            "rpy_rad": rpy,
            "quaternion_xyzw": rpy_to_quaternion_xyzw(*rpy),
            "source_units": {"translation": "mm", "rotation": "rad"},
        }
    raise RuntimeError(f"Unrecognized JAKA TCP pose payload: {payload!r}")


def _load_real_arm_config(path: str | Path, ip: str | None) -> dict[str, Any]:
    config = load_yaml(Path(path))
    config["mode"] = "real"
    config["backend_type"] = "jaka_sdk"
    if ip:
        config["ip"] = ip
    return config


def _load_real_hand_config(
    path: str | Path,
    *,
    backend_type: str,
    port: str | None,
) -> dict[str, Any]:
    config = load_yaml(Path(path))
    config["mode"] = "real"
    config["backend_type"] = backend_type
    if backend_type == "serial_protocol":
        config["transport"] = "serial_rs485"
    if port:
        config.setdefault("serial", {})["port"] = port
    return config


def run_real_arm_hand_ros2_node(
    *,
    arm_config_path: str | Path = "configs/robot/jaka_mini2_real.yaml",
    hand_config_path: str | Path = "configs/hand/rh56_real.yaml",
    arm_ip: str | None = None,
    hand_backend_type: str = "serial_protocol",
    hand_port: str | None = None,
    arm_state_hz: float = 100.0,
    arm_flags_hz: float = 10.0,
    hand_state_hz: float = 20.0,
    enable_arm_teleop: bool = False,
    arm_teleop_hz: float = 50.0,
    arm_teleop_watchdog_sec: float = 0.25,
    arm_teleop_max_palm_velocity_m_s: float = 0.015,
    arm_teleop_max_wrist_roll_velocity_rad_s: float = 0.08,
    arm_teleop_max_joint_velocity_rad_s: float = 0.08,
    arm_teleop_max_joint_acceleration_rad_s2: float = 0.25,
    arm_teleop_max_session_excursion_rad: float = 0.0,
    arm_teleop_max_session_palm_excursion_m: float = 0.03,
    arm_teleop_tcp_velocity_horizon_sec: float = 0.12,
    arm_teleop_max_tcp_target_offset_m: float = 0.004,
    arm_teleop_max_raw_ik_error_rad: float = 0.04,
    arm_teleop_max_joint_tracking_error_rad: float = 0.012,
    arm_teleop_max_joint_tracking_error_fault_rad: float = 0.025,
    arm_teleop_saturation_hold_sec: float = 0.0,
    arm_teleop_joint_limit_margin_deg: float = 10.0,
    arm_teleop_prime_after_enable_ticks: int = 5,
    arm_teleop_step_num: int = 1,
    arm_teleop_sdk_servo_filter: str = "auto",
    arm_teleop_sdk_servo_filter_cutoff_hz: float = 0.5,
    arm_teleop_sdk_servo_filter_jerk_deg_s3: float = 50.0,
    arm_teleop_jsonl: str | Path | None = None,
    topics: Ros2BridgeTopics = Ros2BridgeTopics(),
) -> None:
    try:
        import rclpy  # type: ignore
        from geometry_msgs.msg import PoseStamped  # type: ignore
        from rclpy.node import Node  # type: ignore
        from sensor_msgs.msg import JointState as RosJointState  # type: ignore
        from std_msgs.msg import String  # type: ignore
        from visualization_msgs.msg import Marker  # type: ignore
    except Exception as exc:
        raise RuntimeError(
            "ROS2 Python packages are required. Source ROS2 Humble first, e.g. "
            "`source /opt/ros/humble/setup.bash`."
        ) from exc
    signal.signal(signal.SIGTERM, lambda _signum, _frame: (_ for _ in ()).throw(KeyboardInterrupt()))

    class RealArmHandNode(Node):  # type: ignore[misc]
        def __init__(self) -> None:
            super().__init__("palmhand_real_arm_hand_bridge")
            self.arm = JakaDriverAdapter(_load_real_arm_config(arm_config_path, arm_ip))
            self.arm.connect()
            self.arm_backend = self.arm.backend
            if not isinstance(self.arm_backend, JakaSDKBackend):
                raise RuntimeError("Real arm ROS2 bridge requires JakaSDKBackend.")
            self.hand = RH56Driver(
                _load_real_hand_config(
                    hand_config_path,
                    backend_type=hand_backend_type,
                    port=hand_port,
                )
            )
            self.hand.connect()
            self.hand_backend = self.hand.backend
            self.hand_backend_mode = self.hand.config.get(
                "backend_type", self.hand.config.get("mode", "unknown")
            )
            self.hand_safety_fault = False
            self.hand_safety_status: dict[str, Any] = {
                "enabled": bool(self.hand.config.get("safety", {}).get("estop_enabled", True)),
                "stop": False,
                "reasons": [],
            }
            self.arm_flags = collect_jaka_state_flags(self.arm_backend)
            self.arm_servo: JakaPalmTargetJogController | None = None
            self.arm_teleop_log_handle: Any | None = None
            if arm_teleop_jsonl:
                log_path = Path(arm_teleop_jsonl)
                log_path.parent.mkdir(parents=True, exist_ok=True)
                self.arm_teleop_log_handle = log_path.open("a", encoding="utf-8")

            self.joint_pub = self.create_publisher(
                RosJointState, topics.jaka_joint_states, 10
            )
            self.tcp_pub = self.create_publisher(PoseStamped, topics.jaka_tcp_pose, 10)
            self.flags_pub = self.create_publisher(String, topics.jaka_state_flags, 10)
            self.arm_teleop_status_pub = self.create_publisher(
                String, topics.jaka_teleop_status, 10
            )
            self.arm_teleop_target_pub = self.create_publisher(
                Marker, topics.jaka_teleop_palm_target, 10
            )
            self.hand_state_pub = self.create_publisher(String, topics.rh56_state, 10)
            self.hand_raw_pub = self.create_publisher(String, topics.rh56_raw_feedback, 10)
            self.create_subscription(String, topics.rh56_command_angles, self._on_hand_angles, 10)
            self.create_subscription(String, topics.rh56_command_force, self._on_hand_force, 10)
            self.create_subscription(String, topics.rh56_command_code, self._on_hand_code, 10)
            if enable_arm_teleop:
                self.arm_servo = JakaPalmTargetJogController(
                    self.arm_backend,
                    state_flags=lambda: collect_jaka_state_flags(self.arm_backend),
                    watchdog_sec=arm_teleop_watchdog_sec,
                    max_palm_velocity_m_s=arm_teleop_max_palm_velocity_m_s,
                    max_wrist_roll_velocity_rad_s=arm_teleop_max_wrist_roll_velocity_rad_s,
                    max_joint_velocity_rad_s=arm_teleop_max_joint_velocity_rad_s,
                    max_joint_acceleration_rad_s2=arm_teleop_max_joint_acceleration_rad_s2,
                    max_session_excursion_rad=arm_teleop_max_session_excursion_rad,
                    max_session_palm_excursion_m=arm_teleop_max_session_palm_excursion_m,
                    tcp_velocity_horizon_sec=arm_teleop_tcp_velocity_horizon_sec,
                    max_tcp_target_offset_m=arm_teleop_max_tcp_target_offset_m,
                    max_raw_ik_error_rad=arm_teleop_max_raw_ik_error_rad,
                    max_joint_tracking_error_rad=arm_teleop_max_joint_tracking_error_rad,
                    max_joint_tracking_error_fault_rad=arm_teleop_max_joint_tracking_error_fault_rad,
                    saturation_hold_sec=arm_teleop_saturation_hold_sec,
                    joint_limit_margin_rad=math.radians(arm_teleop_joint_limit_margin_deg),
                    prime_after_enable_ticks=arm_teleop_prime_after_enable_ticks,
                    step_num=arm_teleop_step_num,
                    sdk_servo_filter=arm_teleop_sdk_servo_filter,
                    sdk_servo_filter_cutoff_hz=arm_teleop_sdk_servo_filter_cutoff_hz,
                    sdk_servo_filter_jerk_deg_s3=arm_teleop_sdk_servo_filter_jerk_deg_s3,
                )
                self.create_subscription(
                    String,
                    topics.jaka_teleop_palm_target_jog,
                    self._on_arm_palm_target_jog,
                    10,
                )
                self.create_timer(
                    1.0 / max(float(arm_teleop_hz), 1e-6),
                    self._tick_arm_servo,
                )
                self.get_logger().warning(
                    "Arm Xbox palm-target IK subscription enabled on "
                    f"{topics.jaka_teleop_palm_target_jog}"
                )

            self.create_timer(1.0 / max(float(arm_state_hz), 1e-6), self._publish_arm_state)
            self.create_timer(1.0 / max(float(arm_flags_hz), 1e-6), self._publish_arm_flags)
            self.create_timer(1.0 / max(float(hand_state_hz), 1e-6), self._publish_hand_state)

        def _publish_json(self, publisher: Any, payload: dict[str, Any]) -> None:
            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=False)
            publisher.publish(msg)

        def _publish_arm_state(self) -> None:
            stamp = self.get_clock().now().to_msg()
            joint_state = self.arm.get_joint_state()
            joint_msg = RosJointState()
            joint_msg.header.stamp = stamp
            joint_msg.header.frame_id = "jaka_base"
            joint_msg.name = list(joint_state.names)
            joint_msg.position = list(joint_state.positions)
            joint_msg.velocity = list(joint_state.velocities)
            joint_msg.effort = list(joint_state.efforts)
            self.joint_pub.publish(joint_msg)

            tcp = extract_jaka_tcp_pose(self.arm_backend)
            pose_msg = PoseStamped()
            pose_msg.header.stamp = stamp
            pose_msg.header.frame_id = tcp.get("frame_id", "jaka_base")
            pose_msg.pose.position.x = float(tcp["translation_m"][0])
            pose_msg.pose.position.y = float(tcp["translation_m"][1])
            pose_msg.pose.position.z = float(tcp["translation_m"][2])
            qx, qy, qz, qw = tcp["quaternion_xyzw"]
            pose_msg.pose.orientation.x = float(qx)
            pose_msg.pose.orientation.y = float(qy)
            pose_msg.pose.orientation.z = float(qz)
            pose_msg.pose.orientation.w = float(qw)
            self.tcp_pub.publish(pose_msg)

        def _publish_arm_flags(self) -> None:
            self.arm_flags = collect_jaka_state_flags(self.arm_backend)
            self._publish_json(
                self.flags_pub,
                {
                    "schema_version": SCHEMA_VERSION,
                    "timestamp_sec": time.time(),
                    "controller_ip": self.arm_backend.config.get("ip"),
                    "state_flags": self.arm_flags,
                },
            )

        def _publish_hand_state(self) -> None:
            now = time.time()
            state = self.hand.read_state()
            self.hand_safety_status = evaluate_hand_safety(
                state,
                self.hand.config.get("safety", {}),
            )
            if self.hand_safety_status.get("stop") and not self.hand_safety_fault:
                self.hand_safety_fault = True
                self._apply_hand_safety_stop()
                self.get_logger().error(
                    "RH56 safety stop latched: %s",
                    self.hand_safety_status.get("reasons"),
                )
            hand_payload = build_hand_state_payload(
                state,
                backend_mode=str(self.hand_backend_mode),
                timestamp_sec=now,
            )
            hand_payload["safety"] = {
                **self.hand_safety_status,
                "fault_latched": self.hand_safety_fault,
            }
            self._publish_json(
                self.hand_state_pub,
                hand_payload,
            )
            self._publish_json(self.hand_raw_pub, build_raw_feedback_payload(self.hand_backend, timestamp=now))

        def _apply_hand_safety_stop(self) -> None:
            try:
                if hasattr(self.hand_backend, "set_canonical_forces"):
                    self.hand_backend.set_canonical_forces([0] * 6)
                if hasattr(self.hand_backend, "set_command_angles"):
                    open_pose = self.hand.config.get("gesture_presets", {}).get("open", [1000] * 6)
                    self.hand_backend.set_command_angles(open_pose)
                else:
                    self.hand.open()
            except Exception as exc:
                self.get_logger().error(f"RH56 safety stop command failed: {exc}")

        def _handle_command(self, callback: Callable[[Any], bool], message: Any, label: str) -> None:
            if self.hand_safety_fault:
                self.get_logger().warning(
                    "%s command ignored because RH56 safety fault is latched: %s",
                    label,
                    self.hand_safety_status.get("reasons"),
                )
                return
            try:
                callback(message)
            except Exception as exc:
                self.get_logger().error(f"{label} command failed: {exc}")

        def _on_hand_angles(self, message: Any) -> None:
            self._handle_command(
                lambda msg: apply_angle_command(self.hand_backend, parse_angle_command(msg)),
                message,
                "rh56 angle",
            )

        def _on_hand_force(self, message: Any) -> None:
            self._handle_command(
                lambda msg: apply_force_command(self.hand_backend, parse_force_command(msg)),
                message,
                "rh56 force",
            )

        def _on_hand_code(self, message: Any) -> None:
            self._handle_command(
                lambda msg: self.hand_backend.execute(parse_code_command(msg)),
                message,
                "rh56 code",
            )

        def _on_arm_palm_target_jog(self, message: Any) -> None:
            assert self.arm_servo is not None
            try:
                self.arm_servo.accept(parse_palm_target_jog_command(message))
            except Exception as exc:
                self.get_logger().error(f"JAKA palm-target jog command rejected: {exc}")

        def _tick_arm_servo(self) -> None:
            assert self.arm_servo is not None
            try:
                self.arm_servo.tick()
            except Exception as exc:
                self.arm_servo.command = None
                try:
                    self.arm_servo.latch_fault(f"servo_tick_error:{exc}")
                except Exception as disable_exc:
                    self.get_logger().error(f"JAKA servo disable failed: {disable_exc}")
                self.get_logger().error(f"JAKA palm-target servo stopped: {exc}")
            self.arm_flags = collect_jaka_state_flags(self.arm_backend)
            status = self.arm_servo.status()
            command = self.arm_servo.command
            if self.arm_teleop_log_handle is not None:
                self.arm_teleop_log_handle.write(
                    json.dumps(
                        {
                            "timestamp_sec": time.time(),
                            "schema_version": SCHEMA_VERSION,
                            "source": "xbox_ros2_bridge_arm_teleop",
                            "status": status,
                            "arm_command": (
                                None
                                if command is None
                                else {
                                    "deadman": command.deadman,
                                    "hold_current": command.hold_current,
                                    "palm_velocity_m_s": command.palm_velocity_m_s,
                                    "wrist_roll_velocity_rad_s": command.wrist_roll_velocity_rad_s,
                                    "palm_target_position_m": command.palm_target_position_m,
                                }
                            ),
                            "tick_dt": status.get("tick_dt_sec"),
                            "tcp_current": status.get("tcp_current"),
                            "tcp_target": status.get("tcp_target"),
                            "tcp_target_error": status.get("tcp_target_error_m"),
                            "q_current": status.get("q_current"),
                            "q_cmd": status.get("q_cmd"),
                            "raw_ik_q": status.get("raw_ik_q"),
                            "qdot_cmd": status.get("qdot_cmd"),
                            "joint_error": status.get("joint_error"),
                            "joint_tracking_error": {
                                "error_rad": status.get("joint_tracking_error_rad"),
                                "limit_rad": status.get("max_joint_tracking_error_rad"),
                                "fault_limit_rad": status.get(
                                    "max_joint_tracking_error_fault_rad"
                                ),
                                "limited": status.get("joint_tracking_error_limited"),
                                "faulted": status.get("joint_tracking_error_faulted"),
                                "indices_1_based": status.get(
                                    "joint_tracking_error_indices_1_based"
                                ),
                            },
                            "saturation": {
                                "is_saturated": status.get("is_saturated"),
                                "saturated_joint_indices_1_based": status.get(
                                    "saturated_joint_indices_1_based"
                                ),
                                "saturation_time_sec": status.get("saturation_time_sec"),
                            },
                            "watchdog": {
                                "active": status.get("watchdog_active"),
                                "reason": status.get("watchdog_reason"),
                            },
                            "servo_filter": {
                                "requested": status.get("sdk_servo_filter"),
                                "active": status.get("active_sdk_servo_filter"),
                                "cutoff_hz": status.get("sdk_servo_filter_cutoff_hz"),
                            },
                            "jaka_flags": self.arm_flags,
                            "jaka_collision": self.arm_flags.get("is_in_collision"),
                            "jaka_protective_stop": self.arm_flags.get("protective_stop_status"),
                        },
                        ensure_ascii=False,
                    )
                    + "\n"
                )
                self.arm_teleop_log_handle.flush()
            self._publish_json(self.arm_teleop_status_pub, status)
            target = status.get("palm_target_position_m")
            if target is not None:
                marker = Marker()
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.header.frame_id = "jaka_Link_0"
                marker.ns = "jaka_palm_target"
                marker.id = 0
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = float(target[0])
                marker.pose.position.y = float(target[1])
                marker.pose.position.z = float(target[2])
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.07
                marker.scale.y = 0.07
                marker.scale.z = 0.07
                marker.color.r = 1.0
                marker.color.g = 0.55
                marker.color.b = 0.0
                marker.color.a = 0.9
                self.arm_teleop_target_pub.publish(marker)

        def destroy_node(self) -> bool:
            if self.arm_servo is not None:
                try:
                    self.arm_servo.disable("bridge_shutdown")
                except Exception as exc:
                    self.get_logger().warning(f"JAKA servo cleanup failed: {exc}")
            try:
                close_port = getattr(self.hand_backend, "close_port", None)
                if callable(close_port):
                    close_port()
            except Exception as exc:
                self.get_logger().warning(f"RH56 cleanup failed: {exc}")
            if self.arm_teleop_log_handle is not None:
                self.arm_teleop_log_handle.close()
                self.arm_teleop_log_handle = None
            try:
                self.arm_backend.disconnect()
            except Exception as exc:
                self.get_logger().warning(f"JAKA cleanup failed: {exc}")
            return super().destroy_node()

    rclpy.init()
    node = RealArmHandNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Run the real JAKA+RH56 ROS2 state bridge.")
    parser.add_argument("--arm-config", default="configs/robot/jaka_mini2_real.yaml")
    parser.add_argument("--hand-config", default="configs/hand/rh56_real.yaml")
    parser.add_argument("--arm-ip", default=None)
    parser.add_argument("--hand-backend-type", default="serial_protocol")
    parser.add_argument("--hand-port", default=None)
    parser.add_argument("--arm-state-hz", type=float, default=100.0)
    parser.add_argument("--arm-flags-hz", type=float, default=10.0)
    parser.add_argument("--hand-state-hz", type=float, default=20.0)
    parser.add_argument(
        "--enable-arm-teleop",
        action="store_true",
        help="Accept bounded /jaka/teleop_palm_target_jog commands and own the IK+EDG servo lifecycle.",
    )
    parser.add_argument("--arm-teleop-hz", type=float, default=50.0)
    parser.add_argument("--arm-teleop-watchdog-sec", type=float, default=0.25)
    parser.add_argument("--arm-teleop-max-palm-velocity-m-s", type=float, default=0.015)
    parser.add_argument("--arm-teleop-max-wrist-roll-velocity-rad-s", type=float, default=0.08)
    parser.add_argument("--arm-teleop-max-joint-velocity-rad-s", type=float, default=0.08)
    parser.add_argument("--arm-teleop-max-joint-acceleration-rad-s2", type=float, default=0.25)
    parser.add_argument("--arm-teleop-max-session-excursion-rad", type=float, default=0.0)
    parser.add_argument("--arm-teleop-max-session-palm-excursion-m", type=float, default=0.03)
    parser.add_argument("--arm-teleop-tcp-velocity-horizon-sec", type=float, default=0.12)
    parser.add_argument("--arm-teleop-max-tcp-target-offset-m", type=float, default=0.004)
    parser.add_argument("--arm-teleop-max-raw-ik-error-rad", type=float, default=0.04)
    parser.add_argument("--arm-teleop-max-joint-tracking-error-rad", type=float, default=0.012)
    parser.add_argument("--arm-teleop-max-joint-tracking-error-fault-rad", type=float, default=0.025)
    parser.add_argument("--arm-teleop-saturation-hold-sec", type=float, default=0.0)
    parser.add_argument("--arm-teleop-joint-limit-margin-deg", type=float, default=10.0)
    parser.add_argument("--arm-teleop-prime-after-enable-ticks", type=int, default=5)
    parser.add_argument("--arm-teleop-step-num", type=int, default=1)
    parser.add_argument(
        "--arm-teleop-sdk-servo-filter",
        choices=("none", "auto", "joint_lpf", "joint_nlf"),
        default="auto",
    )
    parser.add_argument("--arm-teleop-sdk-servo-filter-cutoff-hz", type=float, default=0.5)
    parser.add_argument("--arm-teleop-sdk-servo-filter-jerk-deg-s3", type=float, default=50.0)
    parser.add_argument("--arm-teleop-jsonl", default=None)
    args = parser.parse_args(argv)
    run_real_arm_hand_ros2_node(
        arm_config_path=args.arm_config,
        hand_config_path=args.hand_config,
        arm_ip=args.arm_ip,
        hand_backend_type=args.hand_backend_type,
        hand_port=args.hand_port,
        arm_state_hz=args.arm_state_hz,
        arm_flags_hz=args.arm_flags_hz,
        hand_state_hz=args.hand_state_hz,
        enable_arm_teleop=args.enable_arm_teleop,
        arm_teleop_hz=args.arm_teleop_hz,
        arm_teleop_watchdog_sec=args.arm_teleop_watchdog_sec,
        arm_teleop_max_palm_velocity_m_s=args.arm_teleop_max_palm_velocity_m_s,
        arm_teleop_max_wrist_roll_velocity_rad_s=args.arm_teleop_max_wrist_roll_velocity_rad_s,
        arm_teleop_max_joint_velocity_rad_s=args.arm_teleop_max_joint_velocity_rad_s,
        arm_teleop_max_joint_acceleration_rad_s2=args.arm_teleop_max_joint_acceleration_rad_s2,
        arm_teleop_max_session_excursion_rad=args.arm_teleop_max_session_excursion_rad,
        arm_teleop_max_session_palm_excursion_m=args.arm_teleop_max_session_palm_excursion_m,
        arm_teleop_tcp_velocity_horizon_sec=args.arm_teleop_tcp_velocity_horizon_sec,
        arm_teleop_max_tcp_target_offset_m=args.arm_teleop_max_tcp_target_offset_m,
        arm_teleop_max_raw_ik_error_rad=args.arm_teleop_max_raw_ik_error_rad,
        arm_teleop_max_joint_tracking_error_rad=args.arm_teleop_max_joint_tracking_error_rad,
        arm_teleop_max_joint_tracking_error_fault_rad=args.arm_teleop_max_joint_tracking_error_fault_rad,
        arm_teleop_saturation_hold_sec=args.arm_teleop_saturation_hold_sec,
        arm_teleop_joint_limit_margin_deg=args.arm_teleop_joint_limit_margin_deg,
        arm_teleop_prime_after_enable_ticks=args.arm_teleop_prime_after_enable_ticks,
        arm_teleop_step_num=args.arm_teleop_step_num,
        arm_teleop_sdk_servo_filter=args.arm_teleop_sdk_servo_filter,
        arm_teleop_sdk_servo_filter_cutoff_hz=args.arm_teleop_sdk_servo_filter_cutoff_hz,
        arm_teleop_sdk_servo_filter_jerk_deg_s3=args.arm_teleop_sdk_servo_filter_jerk_deg_s3,
        arm_teleop_jsonl=args.arm_teleop_jsonl,
    )


if __name__ == "__main__":
    main()
