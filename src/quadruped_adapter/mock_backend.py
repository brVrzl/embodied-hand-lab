from __future__ import annotations

from embodiment_core.logger import get_logger
from embodiment_core.types import QuadrupedState

from .interfaces import QuadrupedBackend


class MockQuadrupedBackend(QuadrupedBackend):
    def __init__(self) -> None:
        self._connected = False
        self._state = QuadrupedState(
            battery_percent=86.0,
            mode="idle",
            estopped=False,
            position_xyz=[0.0, 0.0, 0.0],
            orientation_xyzw=[0.0, 0.0, 0.0, 1.0],
            linear_velocity_xyz=[0.0, 0.0, 0.0],
            angular_velocity_xyz=[0.0, 0.0, 0.0],
        )
        self._logger = get_logger("MockQuadrupedBackend")

    def connect(self) -> bool:
        self._connected = True
        self._logger.info("Connected to mock quadruped backend.")
        return True

    def get_robot_state(self) -> QuadrupedState:
        if not self._connected:
            raise RuntimeError("Quadruped backend is not connected.")
        return QuadrupedState(**self._state.to_dict())

    def get_odom(self) -> dict:
        if not self._connected:
            raise RuntimeError("Quadruped backend is not connected.")
        return {
            "frame_id": "odom",
            "child_frame_id": "base_link",
            "position_xyz": list(self._state.position_xyz),
            "orientation_xyzw": list(self._state.orientation_xyzw),
        }

    def teleop(self, cmd_vel: dict[str, float]) -> bool:
        self._state.mode = "teleop"
        self._state.linear_velocity_xyz = [
            cmd_vel.get("linear_x", 0.0),
            cmd_vel.get("linear_y", 0.0),
            0.0,
        ]
        self._state.angular_velocity_xyz = [0.0, 0.0, cmd_vel.get("angular_z", 0.0)]
        self._state.position_xyz[0] += self._state.linear_velocity_xyz[0] * 0.1
        self._state.position_xyz[1] += self._state.linear_velocity_xyz[1] * 0.1
        self._logger.info("Mock quadruped teleop: %s", cmd_vel)
        return True

    def stand(self) -> bool:
        self._state.mode = "standing"
        self._logger.info("Mock quadruped stand.")
        return True

    def sit(self) -> bool:
        self._state.mode = "sitting"
        self._logger.info("Mock quadruped sit.")
        return True

    def estop(self) -> None:
        self._state.estopped = True
        self._state.mode = "estop"
        self._logger.warning("Mock quadruped estop.")

    def start_recording_hint(self) -> None:
        self._logger.info("Mock quadruped recording hint sent.")

