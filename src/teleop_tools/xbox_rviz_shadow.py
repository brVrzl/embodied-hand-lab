from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from embodiment_core.config import load_yaml
from jaka_driver_adapter.palm_target_ik import DEFAULT_MJCF, PalmTargetIkState
from jaka_driver_adapter.servo_jog import parse_palm_target_jog_command

from .xbox_ros2 import (
    DEFAULT_CONFIG,
    PygameXboxController,
    XboxPalmTargetAction,
    XboxPalmTargetMapper,
)
from .rviz_shadow_sync import ARM_JOINT_NAMES, extract_arm_joints_from_joint_state


HAND_PRESETS = {
    "open": [1000.0] * 6,
    "close": [0.0] * 6,
    "pinch": [400.0, 400.0, 500.0, 200.0, 200.0, 500.0],
}


class XboxRvizPalmTargetState(PalmTargetIkState):
    def __init__(
        self,
        initial_arm_joints_rad: list[float],
        *,
        mjcf_path: str | Path = DEFAULT_MJCF,
        ik_gain: float = 0.65,
        ik_damping: float = 0.05,
        ik_max_step_rad: float = 0.025,
        ik_iterations: int = 4,
        target_workspace_radius_m: float = 0.0,
    ) -> None:
        super().__init__(
            initial_arm_joints_rad,
            mjcf_path=mjcf_path,
            ik_gain=ik_gain,
            ik_damping=ik_damping,
            ik_max_step_rad=ik_max_step_rad,
            ik_iterations=ik_iterations,
            target_workspace_radius_m=target_workspace_radius_m,
        )
        self.hand_counts = list(HAND_PRESETS["open"])
        self._last_deadman = False

    def apply(self, *, action: XboxPalmTargetAction, dt: float) -> None:
        if action.deadman and not self._last_deadman:
            self.reset_session(self.arm_joints_rad.tolist())
        super().apply(
            palm_velocity_m_s=action.palm_velocity_m_s,
            wrist_roll_velocity_rad_s=action.wrist_roll_velocity_rad_s,
            dt=dt,
        )
        self._last_deadman = action.deadman
        if action.hand_command:
            self.hand_counts = list(HAND_PRESETS[action.hand_command])


class TopicActionMirror:
    def __init__(self, *, watchdog_sec: float = 0.25) -> None:
        self.watchdog_sec = max(0.0, float(watchdog_sec))
        self.last_action = XboxPalmTargetAction(
            palm_velocity_m_s=[0.0, 0.0, 0.0],
            wrist_roll_velocity_rad_s=0.0,
            deadman=False,
        )
        self.last_timestamp_sec = 0.0

    def accept(self, message: object, *, timestamp_sec: float) -> None:
        command = parse_palm_target_jog_command(message)
        self.last_action = XboxPalmTargetAction(
            palm_velocity_m_s=list(command.palm_velocity_m_s),
            wrist_roll_velocity_rad_s=command.wrist_roll_velocity_rad_s,
            deadman=command.deadman,
        )
        self.last_timestamp_sec = float(timestamp_sec)

    def action(self, *, timestamp_sec: float) -> XboxPalmTargetAction:
        if self.last_action.deadman and timestamp_sec - self.last_timestamp_sec <= self.watchdog_sec:
            return self.last_action
        return XboxPalmTargetAction(
            palm_velocity_m_s=[0.0, 0.0, 0.0],
            wrist_roll_velocity_rad_s=0.0,
            deadman=False,
        )


def run_xbox_rviz_shadow_node(
    *,
    config_path: str | Path = DEFAULT_CONFIG,
    initial_arm_joints_rad: list[float] | None = None,
    action_topic: str | None = None,
    real_arm_joint_topic: str = "/jaka/joint_states",
    action_watchdog_sec: float = 0.25,
) -> None:
    try:
        import rclpy  # type: ignore
        from rclpy.node import Node  # type: ignore
        from sensor_msgs.msg import JointState  # type: ignore
        from std_msgs.msg import String  # type: ignore
        from visualization_msgs.msg import Marker  # type: ignore
    except Exception as exc:
        raise RuntimeError("ROS2 Python packages are required. Source ROS2 Humble first.") from exc

    config = load_yaml(Path(config_path))
    input_cfg = config.get("input", {})
    shadow_cfg = config.get("shadow", {})
    direction_cfg = config.get("direction_calibration", {})
    mapper: XboxPalmTargetMapper | None = None
    controller: PygameXboxController | None = None
    topic_mirror: TopicActionMirror | None = None
    if action_topic:
        topic_mirror = TopicActionMirror(watchdog_sec=action_watchdog_sec)
    else:
        mapper = XboxPalmTargetMapper(
            max_translation_velocity_m_s=float(shadow_cfg.get("max_translation_velocity_m_s", 0.10)),
            max_wrist_roll_velocity_rad_s=float(shadow_cfg.get("max_wrist_roll_velocity_rad_s", 0.6)),
            precision_scale=float(config.get("precision_scale", 0.25)),
            velocity_filter_time_constant_sec=float(input_cfg.get("velocity_filter_time_constant_sec", 0.08)),
            translation_axis_map=direction_cfg.get("translation"),
            wrist_roll_axis_map=direction_cfg.get("wrist_roll"),
        )
        controller = PygameXboxController(
            index=int(input_cfg.get("gamepad_index", 0)),
            deadzone=float(input_cfg.get("deadzone", 0.12)),
        )
    state = XboxRvizPalmTargetState(
        initial_arm_joints_rad or shadow_cfg.get("initial_arm_joints_rad", [0.0] * 6),
        mjcf_path=shadow_cfg.get("mjcf_path", DEFAULT_MJCF),
        ik_gain=float(shadow_cfg.get("ik_gain", 0.65)),
        ik_damping=float(shadow_cfg.get("ik_damping", 0.05)),
        ik_max_step_rad=float(shadow_cfg.get("ik_max_step_rad", 0.025)),
        ik_iterations=int(shadow_cfg.get("ik_iterations", 4)),
        target_workspace_radius_m=float(shadow_cfg.get("target_workspace_radius_m", 0.0)),
    )

    class XboxRvizShadowNode(Node):  # type: ignore[misc]
        def __init__(self) -> None:
            super().__init__("jaka_rh56_xbox_rviz_shadow")
            self.arm_pub = self.create_publisher(JointState, "/teleop/shadow_joint_states", 10)
            self.hand_pub = self.create_publisher(String, "/teleop/shadow_rh56_state", 10)
            self.status_pub = self.create_publisher(String, "/teleop/shadow_status", 10)
            self.target_pub = self.create_publisher(Marker, "/teleop/shadow_palm_target", 10)
            self.latest_real_arm_joints_rad: list[float] | None = None
            self.latest_real_arm_timestamp_sec = 0.0
            self.last_tick = time.monotonic()
            if action_topic:
                self.create_subscription(String, action_topic, self._on_action_topic, 10)
            if real_arm_joint_topic:
                self.create_subscription(
                    JointState,
                    real_arm_joint_topic,
                    self._on_real_arm_joint_state,
                    10,
                )
            self.create_timer(1.0 / float(config.get("publish_hz", 50.0)), self._publish)
            if action_topic:
                self.get_logger().warning(
                    "RViz palm-target IK shadow mirroring "
                    f"{action_topic}; syncing idle pose from {real_arm_joint_topic}. "
                    "No hardware command topics are published."
                )
            else:
                assert controller is not None
                self.get_logger().warning(
                    f"RViz palm-target IK shadow ready: {controller.name}. "
                    f"Syncing idle pose from {real_arm_joint_topic}. "
                    "No hardware command topics are published."
                )

        def _on_action_topic(self, message: object) -> None:
            assert topic_mirror is not None
            try:
                topic_mirror.accept(message, timestamp_sec=time.monotonic())
            except Exception as exc:
                self.get_logger().error(f"Shadow action topic rejected: {exc}")

        def _on_real_arm_joint_state(self, message: object) -> None:
            joints = extract_arm_joints_from_joint_state(message)
            if joints is None:
                return
            self.latest_real_arm_joints_rad = joints
            self.latest_real_arm_timestamp_sec = time.monotonic()

        def _action(self, now: float) -> XboxPalmTargetAction:
            if topic_mirror is not None:
                return topic_mirror.action(timestamp_sec=now)
            assert mapper is not None
            assert controller is not None
            return mapper.map(controller.snapshot(), timestamp_sec=now)

        def _publish(self) -> None:
            now = time.monotonic()
            action = self._action(now)
            if (
                not action.deadman
                and self.latest_real_arm_joints_rad is not None
                and now - self.latest_real_arm_timestamp_sec <= action_watchdog_sec
            ):
                state.set_arm_joints_rad(self.latest_real_arm_joints_rad)
                state.hold_current_target()
            state.apply(action=action, dt=now - self.last_tick)
            self.last_tick = now
            stamp = self.get_clock().now().to_msg()

            arm_message = JointState()
            arm_message.header.stamp = stamp
            arm_message.name = list(ARM_JOINT_NAMES)
            arm_message.position = state.arm_joints_rad.tolist()
            self.arm_pub.publish(arm_message)

            hand_message = String()
            hand_message.data = json.dumps(
                {"finger_state": {"angle_count_0_1000": state.hand_counts}}
            )
            self.hand_pub.publish(hand_message)

            target_message = Marker()
            target_message.header.stamp = stamp
            target_message.header.frame_id = "jaka_Link_0"
            target_message.ns = "xbox_rviz_shadow"
            target_message.id = 0
            target_message.type = Marker.SPHERE
            target_message.action = Marker.ADD
            target_message.pose.position.x = float(state.target_palm_position_m[0])
            target_message.pose.position.y = float(state.target_palm_position_m[1])
            target_message.pose.position.z = float(state.target_palm_position_m[2])
            target_message.pose.orientation.w = 1.0
            target_message.scale.x = 0.07
            target_message.scale.y = 0.07
            target_message.scale.z = 0.07
            target_message.color.r = 0.0
            target_message.color.g = 0.65
            target_message.color.b = 1.0
            target_message.color.a = 0.85
            self.target_pub.publish(target_message)

            status_message = String()
            status_message.data = json.dumps(
                {
                    "mode": "rviz_shadow_palm_target_ik",
                    "hardware_commands_published": False,
                    "action_source": "topic" if action_topic else "controller",
                    "action_topic": action_topic,
                    "real_arm_joint_topic": real_arm_joint_topic,
                    "real_arm_sync_active": (
                        self.latest_real_arm_joints_rad is not None
                        and now - self.latest_real_arm_timestamp_sec <= action_watchdog_sec
                    ),
                    "deadman": action.deadman,
                    "precision": action.precision,
                    "palm_target_position_m": state.target_palm_position_m.tolist(),
                    "palm_preview_position_m": state.current_palm_position_m.tolist(),
                    "palm_target_error_m": state.target_error_m,
                    "palm_target_workspace_limited": state.target_workspace_limited,
                    "ik_joint_limit_limited": state.joint_limit_limited,
                    "ik_limited_joint_indices_1_based": state.limited_joint_indices_1_based,
                    "arm_joints_rad": state.arm_joints_rad.tolist(),
                    "hand_command": action.hand_command,
                }
            )
            self.status_pub.publish(status_message)

    rclpy.init()
    node = XboxRvizShadowNode()
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
        description="Preview Xbox palm-target IK teleop in RViz without publishing hardware commands."
    )
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--initial-joints", nargs=6, type=float, default=None)
    parser.add_argument(
        "--action-topic",
        default=None,
        help="Mirror an existing std_msgs/String palm-target jog topic instead of reading Xbox.",
    )
    parser.add_argument("--real-arm-joint-topic", default="/jaka/joint_states")
    parser.add_argument("--action-watchdog-sec", type=float, default=0.25)
    args = parser.parse_args(argv)
    run_xbox_rviz_shadow_node(
        config_path=args.config,
        initial_arm_joints_rad=args.initial_joints,
        action_topic=args.action_topic,
        real_arm_joint_topic=args.real_arm_joint_topic,
        action_watchdog_sec=args.action_watchdog_sec,
    )


if __name__ == "__main__":
    main()
