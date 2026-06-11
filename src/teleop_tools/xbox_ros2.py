from __future__ import annotations

import argparse
import json
import os
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from embodiment_core.config import load_yaml
from teleop_tools.direction_calibration import (
    AxisMapping,
    DEFAULT_XBOX_TRANSLATION_MAP,
    DEFAULT_XBOX_WRIST_ROLL_MAP,
    apply_vector_axis_map,
    parse_scalar_axis_map,
    parse_vector_axis_map,
)


DEFAULT_CONFIG = "configs/teleop/xbox_jaka_rh56.yaml"


def apply_deadzone(value: float, deadzone: float) -> float:
    value = max(-1.0, min(1.0, float(value)))
    deadzone = max(0.0, min(0.95, float(deadzone)))
    if abs(value) <= deadzone:
        return 0.0
    return (1.0 if value > 0.0 else -1.0) * (abs(value) - deadzone) / (1.0 - deadzone)


@dataclass(frozen=True, slots=True)
class XboxSnapshot:
    left_x: float = 0.0
    left_y: float = 0.0
    right_x: float = 0.0
    right_y: float = 0.0
    buttons: dict[str, bool] = field(default_factory=dict)
    dpad: tuple[int, int] = (0, 0)


@dataclass(frozen=True, slots=True)
class XboxPalmTargetAction:
    palm_velocity_m_s: list[float]
    wrist_roll_velocity_rad_s: float
    deadman: bool
    hand_command: str | None = None
    precision: bool = False


class XboxPalmTargetMapper:
    def __init__(
        self,
        *,
        max_translation_velocity_m_s: float,
        max_wrist_roll_velocity_rad_s: float,
        precision_scale: float,
        velocity_filter_time_constant_sec: float = 0.0,
        translation_axis_map: dict[str, Any] | None = None,
        wrist_roll_axis_map: dict[str, Any] | str | None = None,
    ) -> None:
        self.max_translation_velocity_m_s = abs(float(max_translation_velocity_m_s))
        self.max_wrist_roll_velocity_rad_s = abs(float(max_wrist_roll_velocity_rad_s))
        self.precision_scale = float(precision_scale)
        self.velocity_filter_time_constant_sec = max(0.0, float(velocity_filter_time_constant_sec))
        self.translation_axis_map = parse_vector_axis_map(
            translation_axis_map,
            default=DEFAULT_XBOX_TRANSLATION_MAP,
        )
        self.wrist_roll_axis_map: AxisMapping = parse_scalar_axis_map(
            wrist_roll_axis_map,
            default=DEFAULT_XBOX_WRIST_ROLL_MAP,
        )
        self._last_buttons: dict[str, bool] = {}
        self._filtered_palm_velocity = [0.0] * 3
        self._filtered_wrist_roll_velocity = 0.0
        self._last_filter_timestamp_sec: float | None = None

    def map(
        self,
        snapshot: XboxSnapshot,
        *,
        timestamp_sec: float | None = None,
    ) -> XboxPalmTargetAction:
        deadman = bool(snapshot.buttons.get("rb", False))
        precision = bool(snapshot.buttons.get("lb", False))
        scale = self.precision_scale if precision else 1.0
        axis_values = {
            "left_x": float(snapshot.left_x),
            "left_y": float(snapshot.left_y),
            "right_x": float(snapshot.right_x),
            "right_y": float(snapshot.right_y),
            "dpad_x": float(snapshot.dpad[0]),
            "dpad_y": float(snapshot.dpad[1]),
        }
        palm_velocity = (
            apply_vector_axis_map(axis_values, self.translation_axis_map)
            * self.max_translation_velocity_m_s
            * scale
        ).astype(float).tolist()
        wrist_roll_velocity = (
            self.wrist_roll_axis_map.apply(axis_values)
            * self.max_wrist_roll_velocity_rad_s
            * scale
        )
        if not deadman:
            palm_velocity = [0.0] * 3
            wrist_roll_velocity = 0.0
            self._reset_velocity_filter()
        else:
            palm_velocity, wrist_roll_velocity = self._filter_velocity(
                palm_velocity,
                wrist_roll_velocity,
                timestamp_sec=time.monotonic() if timestamp_sec is None else float(timestamp_sec),
            )

        a_pressed = self._pressed_once(snapshot, "a")
        b_pressed = self._pressed_once(snapshot, "b")
        x_pressed = self._pressed_once(snapshot, "x")
        hand_command = None
        if a_pressed:
            hand_command = "open"
        elif deadman and b_pressed:
            hand_command = "close"
        elif deadman and x_pressed:
            hand_command = "pinch"
        return XboxPalmTargetAction(
            palm_velocity_m_s=palm_velocity,
            wrist_roll_velocity_rad_s=wrist_roll_velocity,
            deadman=deadman,
            hand_command=hand_command,
            precision=precision,
        )

    def _pressed_once(self, snapshot: XboxSnapshot, name: str) -> bool:
        pressed = bool(snapshot.buttons.get(name, False))
        previous = self._last_buttons.get(name, False)
        self._last_buttons[name] = pressed
        return pressed and not previous

    def _filter_velocity(
        self,
        palm_velocity: list[float],
        wrist_roll_velocity: float,
        *,
        timestamp_sec: float,
    ) -> tuple[list[float], float]:
        if self.velocity_filter_time_constant_sec <= 0.0:
            self._filtered_palm_velocity = [float(value) for value in palm_velocity]
            self._filtered_wrist_roll_velocity = float(wrist_roll_velocity)
            self._last_filter_timestamp_sec = timestamp_sec
            return list(self._filtered_palm_velocity), self._filtered_wrist_roll_velocity
        if self._last_filter_timestamp_sec is None:
            alpha = 1.0
        else:
            dt = max(0.0, min(timestamp_sec - self._last_filter_timestamp_sec, 0.1))
            alpha = dt / (self.velocity_filter_time_constant_sec + dt) if dt > 0.0 else 0.0
        self._last_filter_timestamp_sec = timestamp_sec
        self._filtered_palm_velocity = [
            previous + alpha * (float(target) - previous)
            for previous, target in zip(self._filtered_palm_velocity, palm_velocity, strict=True)
        ]
        self._filtered_wrist_roll_velocity += alpha * (
            float(wrist_roll_velocity) - self._filtered_wrist_roll_velocity
        )
        return list(self._filtered_palm_velocity), self._filtered_wrist_roll_velocity

    def _reset_velocity_filter(self) -> None:
        self._filtered_palm_velocity = [0.0] * 3
        self._filtered_wrist_roll_velocity = 0.0
        self._last_filter_timestamp_sec = None


class PygameXboxController:
    """Read a Linux Xbox-style controller through pygame's SDL mapping."""

    def __init__(self, *, index: int = 0, deadzone: float = 0.12) -> None:
        os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
        os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
        try:
            import pygame
        except Exception as exc:
            raise RuntimeError("pygame is required. Install with `pip install -e '.[gamepad]'`.") from exc
        self.pygame = pygame
        self.deadzone = float(deadzone)
        pygame.init()
        pygame.joystick.init()
        count = pygame.joystick.get_count()
        if index >= count:
            raise RuntimeError(f"No gamepad at index {index}; pygame joystick count={count}.")
        self.joystick = pygame.joystick.Joystick(index)
        self.joystick.init()

    @property
    def name(self) -> str:
        return str(self.joystick.get_name())

    def snapshot(self) -> XboxSnapshot:
        self.pygame.event.pump()
        return XboxSnapshot(
            left_x=self._axis(0),
            left_y=self._axis(1),
            right_x=self._axis(3),
            right_y=self._axis(4),
            buttons={
                "a": self._button(0),
                "b": self._button(1),
                "x": self._button(2),
                "y": self._button(3),
                "lb": self._button(4),
                "rb": self._button(5),
                "back": self._button(6),
                "start": self._button(7),
            },
            dpad=self.joystick.get_hat(0) if self.joystick.get_numhats() else (0, 0),
        )

    def _axis(self, index: int) -> float:
        if index >= self.joystick.get_numaxes():
            return 0.0
        return apply_deadzone(float(self.joystick.get_axis(index)), self.deadzone)

    def _button(self, index: int) -> bool:
        return index < self.joystick.get_numbuttons() and bool(self.joystick.get_button(index))


def run_xbox_ros2_node(*, config_path: str | Path = DEFAULT_CONFIG) -> None:
    try:
        import rclpy  # type: ignore
        from rclpy.node import Node  # type: ignore
        from std_msgs.msg import String  # type: ignore
    except Exception as exc:
        raise RuntimeError("ROS2 Python packages are required. Source ROS2 Humble first.") from exc

    config = load_yaml(Path(config_path))
    topics = config.get("topics", {})
    arm_topic = topics.get("arm_palm_target_jog", "/jaka/teleop_palm_target_jog")
    hand_topic = topics.get("rh56_command_code", "/rh56/command_code")
    status_topic = topics.get("status", "/teleop/xbox_status")
    input_cfg = config.get("input", {})
    shadow_cfg = config.get("shadow", {})
    direction_cfg = config.get("direction_calibration", {})
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

    class XboxTeleopNode(Node):  # type: ignore[misc]
        def __init__(self) -> None:
            super().__init__("jaka_rh56_xbox_teleop")
            self.arm_pub = self.create_publisher(String, arm_topic, 10)
            self.hand_pub = self.create_publisher(String, hand_topic, 10)
            self.status_pub = self.create_publisher(String, status_topic, 10)
            self.create_timer(1.0 / float(config.get("publish_hz", 50.0)), self._publish_action)
            self.get_logger().info(f"Xbox controller ready: {controller.name}")

        def _publish_json(self, publisher: Any, payload: dict[str, Any]) -> None:
            message = String()
            message.data = json.dumps(payload)
            publisher.publish(message)

        def _publish_action(self) -> None:
            action = mapper.map(controller.snapshot(), timestamp_sec=time.monotonic())
            self._publish_json(
                self.arm_pub,
                {
                    "deadman": action.deadman,
                    "palm_velocity_m_s": action.palm_velocity_m_s,
                    "wrist_roll_velocity_rad_s": action.wrist_roll_velocity_rad_s,
                },
            )
            if action.hand_command:
                self._publish_json(self.hand_pub, {"command": action.hand_command})
            self._publish_json(
                self.status_pub,
                {
                    "controller": controller.name,
                    "deadman": action.deadman,
                    "precision": action.precision,
                    "palm_velocity_m_s": action.palm_velocity_m_s,
                    "wrist_roll_velocity_rad_s": action.wrist_roll_velocity_rad_s,
                    "hand_command": action.hand_command,
                },
            )

    rclpy.init()
    node = XboxTeleopNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Publish bounded JAKA+RH56 Xbox teleop intents over ROS2.")
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    args = parser.parse_args(argv)
    run_xbox_ros2_node(config_path=args.config)


if __name__ == "__main__":
    main()
