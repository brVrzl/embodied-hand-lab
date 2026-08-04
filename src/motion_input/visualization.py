"""Input-only visualization for live and replay providers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from .diagnostics import StreamingDiagnostics
from .model import MotionInputSample, Pose6D, Side


@dataclass(slots=True)
class HandView:
    wrist: Pose6D | None = None
    palm: Pose6D | None = None
    tracking_state: str = "not_tracking"
    confidence: float | None = None
    coordinate_frame: str = ""
    last_receive_ns: int | None = None


class MotionInputVisualizer:
    """Maintains a two-hand scene and renders text or optional matplotlib 3-D."""

    def __init__(self) -> None:
        self.hands = {Side.LEFT: HandView(), Side.RIGHT: HandView()}
        self.diagnostics = StreamingDiagnostics()
        self._interruptions: list[str] = []

    def observe(self, sample: MotionInputSample) -> None:
        self.diagnostics.observe(sample)
        if sample.side not in self.hands:
            return
        view = self.hands[sample.side]
        previous = view.tracking_state
        view.wrist = sample.wrist_pose
        view.palm = sample.palm_pose
        view.tracking_state = sample.tracking_state.value
        view.confidence = sample.tracking_confidence
        view.coordinate_frame = sample.coordinate_frame
        view.last_receive_ns = sample.receive_timestamp.nanoseconds
        if previous in ("tracking", "limited") and view.tracking_state not in (
            "tracking",
            "limited",
        ):
            self._interruptions.append(
                f"{sample.side.value}: {previous} -> {view.tracking_state} at seq {sample.sequence_number}"
            )
            self._interruptions = self._interruptions[-5:]

    def render_text(self) -> str:
        report = self.diagnostics.report()
        lines = ["UMIP motion input (tracking origin = coordinate-frame origin)"]
        for side in (Side.LEFT, Side.RIGHT):
            view = self.hands[side]
            confidence = "n/a" if view.confidence is None else f"{view.confidence:.2f}"
            lines.append(
                f"{side.value:>5} status={view.tracking_state:<13} confidence={confidence} "
                f"frame={view.coordinate_frame or 'n/a'}"
            )
            lines.append(f"      wrist={_pose_text(view.wrist)} palm={_pose_text(view.palm)}")
        frequencies = [
            value["tracking_frequency_hz"]
            for value in report["streams"].values()
            if value["tracking_frequency_hz"] is not None
        ]
        latencies = [
            value["latency_ms"]["mean"]
            for value in report["streams"].values()
            if value["latency_ms"] is not None
        ]
        lines.append(
            "rate="
            + (f"{sum(frequencies) / len(frequencies):.1f} Hz" if frequencies else "n/a")
            + " latency="
            + (f"{sum(latencies) / len(latencies):.2f} ms" if latencies else "n/a (clocks differ)")
        )
        if self._interruptions:
            lines.append("interruptions: " + " | ".join(self._interruptions))
        return "\n".join(lines)

    def render_matplotlib(self, *, block: bool = False, axis_limit_m: float = 1.0) -> None:
        """Render the origin and wrist/palm triads; matplotlib is optional."""

        try:
            import matplotlib.pyplot as plt
        except ImportError as exc:
            raise RuntimeError(
                "matplotlib is required for 3-D visualization; install .[motion-input-viz]"
            ) from exc
        figure = plt.figure("UMIP Motion Input")
        figure.clear()
        axis = figure.add_subplot(111, projection="3d")
        axis.set_xlabel("+X right [m]")
        axis.set_ylabel("+Y up [m]")
        axis.set_zlabel("+Z backward [m]")
        axis.set_xlim(-axis_limit_m, axis_limit_m)
        axis.set_ylim(-axis_limit_m, axis_limit_m)
        axis.set_zlim(-axis_limit_m, axis_limit_m)
        _draw_triad(axis, Pose6D((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)), "origin", 0.12)
        for side, view in self.hands.items():
            if view.wrist is not None:
                _draw_triad(axis, view.wrist, f"{side.value} wrist", 0.08)
            if view.palm is not None:
                _draw_triad(axis, view.palm, f"{side.value} palm", 0.05)
        axis.set_title(self.render_text())
        plt.draw()
        plt.pause(0.001)
        if block:
            plt.show()


def _pose_text(pose: Pose6D | None) -> str:
    if pose is None:
        return "n/a"
    return "(" + ", ".join(f"{value:+.3f}" for value in pose.position_m) + ")m"


def _draw_triad(axis: Any, pose: Pose6D, label: str, length: float) -> None:
    rotation = _quaternion_matrix(pose.orientation_xyzw)
    colors = ("r", "g", "b")
    origin = pose.position_m
    for index, color in enumerate(colors):
        direction = tuple(rotation[row][index] * length for row in range(3))
        axis.quiver(*origin, *direction, color=color)
    axis.text(*origin, label)


def _quaternion_matrix(q: tuple[float, float, float, float]) -> tuple[tuple[float, ...], ...]:
    x, y, z, w = q
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )
