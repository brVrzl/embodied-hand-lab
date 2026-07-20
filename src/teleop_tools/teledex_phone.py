from __future__ import annotations

import math
import socket
import threading
import time
from collections.abc import Callable
from typing import Any

import numpy as np

from teleop_tools.hebi_mobile_io import HebiMobileIOSnapshot


def rotation_matrix_to_quaternion_wxyz(matrix: Any) -> list[float]:
    """Convert a proper 3x3 rotation matrix to a normalized wxyz quaternion."""
    rotation = np.asarray(matrix, dtype=np.float64)
    if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
        raise ValueError("TeleDex rotation must be a finite 3x3 matrix.")
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=2e-3):
        raise ValueError("TeleDex rotation matrix is not orthonormal.")
    if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=2e-3):
        raise ValueError("TeleDex rotation matrix must have determinant +1.")

    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = np.asarray(
            [
                0.25 * scale,
                (rotation[2, 1] - rotation[1, 2]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
            ],
            dtype=np.float64,
        )
    else:
        axis = int(np.argmax(np.diag(rotation)))
        if axis == 0:
            scale = math.sqrt(max(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2], 0.0)) * 2.0
            quaternion = np.asarray(
                [
                    (rotation[2, 1] - rotation[1, 2]) / scale,
                    0.25 * scale,
                    (rotation[0, 1] + rotation[1, 0]) / scale,
                    (rotation[0, 2] + rotation[2, 0]) / scale,
                ],
                dtype=np.float64,
            )
        elif axis == 1:
            scale = math.sqrt(max(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2], 0.0)) * 2.0
            quaternion = np.asarray(
                [
                    (rotation[0, 2] - rotation[2, 0]) / scale,
                    (rotation[0, 1] + rotation[1, 0]) / scale,
                    0.25 * scale,
                    (rotation[1, 2] + rotation[2, 1]) / scale,
                ],
                dtype=np.float64,
            )
        else:
            scale = math.sqrt(max(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1], 0.0)) * 2.0
            quaternion = np.asarray(
                [
                    (rotation[1, 0] - rotation[0, 1]) / scale,
                    (rotation[0, 2] + rotation[2, 0]) / scale,
                    (rotation[1, 2] + rotation[2, 1]) / scale,
                    0.25 * scale,
                ],
                dtype=np.float64,
            )
    norm = float(np.linalg.norm(quaternion))
    if norm <= 1e-9:
        raise ValueError("TeleDex rotation produced a zero quaternion.")
    quaternion /= norm
    if quaternion[0] < 0.0:
        quaternion *= -1.0
    return quaternion.astype(float).tolist()


def _assert_port_free(port: int) -> None:
    """Protect against TeleDex Session killing an unrelated port owner."""
    probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 0)
        probe.bind(("0.0.0.0", int(port)))
    except OSError as exc:
        raise RuntimeError(
            f"TeleDex port {port} is already in use. Stop that service or choose another port; "
            "this project will not let the upstream Session replace it."
        ) from exc
    finally:
        probe.close()


class TeleDexPhoneClient:
    """Safety-oriented adapter from the official TeleDex Session to phone snapshots.

    TeleDex callbacks run on its WebSocket thread. ``read`` returns an immutable
    snapshot and invalidates it when updates stop, so a held button cannot keep
    the robot deadman alive after Wi-Fi loss.
    """

    VALID_DEADMAN_FIELDS = frozenset({"button", "button_secondary", "toggle", "none"})

    def __init__(
        self,
        *,
        port: int = 8888,
        show_qr: bool = True,
        debug: bool = False,
        max_stale_feedback_sec: float = 0.20,
        server_start_timeout_sec: float = 3.0,
        deadman_field: str = "button",
        precision_scale: float = 1.0,
        session_factory: Callable[..., Any] | None = None,
        check_port_available: bool = True,
    ) -> None:
        if deadman_field not in self.VALID_DEADMAN_FIELDS:
            raise ValueError(
                f"deadman_field must be one of {sorted(self.VALID_DEADMAN_FIELDS)}, got {deadman_field!r}."
            )
        self.port = int(port)
        self.show_qr = bool(show_qr)
        self.debug = bool(debug)
        self.max_stale_feedback_sec = max(0.01, float(max_stale_feedback_sec))
        self.server_start_timeout_sec = max(0.1, float(server_start_timeout_sec))
        self.deadman_field = deadman_field
        self.precision_scale = max(0.0, min(1.0, float(precision_scale)))
        self._session_factory = session_factory
        self._check_port_available = bool(check_port_available)
        self._lock = threading.Lock()
        self._session: Any | None = None
        self._connected = False
        self._latest_data: dict[str, Any] | None = None
        self._latest_wall_sec = 0.0
        self._latest_monotonic_sec = 0.0
        self._sequence = 0

    @property
    def is_connected(self) -> bool:
        with self._lock:
            return self._connected

    @property
    def address(self) -> str:
        session = self._session
        host = getattr(session, "ip_address", "0.0.0.0") if session is not None else "0.0.0.0"
        port = getattr(session, "port", self.port) if session is not None else self.port
        return f"{host}:{port}"

    def connect(self) -> None:
        if self._session is not None:
            return
        if self._check_port_available:
            _assert_port_free(self.port)
        factory = self._session_factory
        if factory is None:
            try:
                from teledex import Session
            except Exception as exc:
                raise RuntimeError(
                    "TeleDex Python package is required. Install with: "
                    "pip install -e '.[teledex-teleop]'"
                ) from exc
            # TeleDex 0.0.7 documents ``button_secondary`` but its Session
            # currently drops that field while normalizing incoming frames.
            # Preserve it without replacing the upstream transport/session.
            class ExtendedControlSession(Session):  # type: ignore[misc, valid-type]
                def _process_data(self, data: dict[str, Any]) -> None:
                    super()._process_data(data)
                    self.latest_data["button_secondary"] = bool(
                        data.get("button_secondary", False)
                    )

            factory = ExtendedControlSession
        session = factory(port=self.port, debug=self.debug, show_qr=self.show_qr)
        session.on_update(self._on_update)
        session.on_connect(self._on_connect)
        session.on_disconnect(self._on_disconnect)
        self._session = session
        session.start()

        deadline = time.monotonic() + self.server_start_timeout_sec
        while getattr(session, "server", None) is None and time.monotonic() < deadline:
            thread = getattr(session, "_thread", None)
            if thread is not None and not thread.is_alive():
                break
            time.sleep(0.01)
        if getattr(session, "server", None) is None:
            self.close()
            raise RuntimeError(f"TeleDex WebSocket server did not start on port {self.port}.")
        actual_port = int(getattr(session, "port", self.port))
        if actual_port != self.port:
            self.close()
            raise RuntimeError(
                f"TeleDex unexpectedly changed port from {self.port} to {actual_port}; refusing an ambiguous endpoint."
            )

    def close(self) -> None:
        session, self._session = self._session, None
        with self._lock:
            self._connected = False
            self._latest_data = None
        if session is not None:
            try:
                session.stop()
            except Exception:
                if self.debug:
                    raise

    def _on_connect(self, _session: Any) -> None:
        with self._lock:
            self._connected = True

    def _on_disconnect(self, _session: Any) -> None:
        with self._lock:
            self._connected = False

    def _on_update(self, _session: Any, data: dict[str, Any]) -> None:
        copied: dict[str, Any] = {}
        for key, value in data.items():
            copied[key] = value.copy() if isinstance(value, np.ndarray) else value
        with self._lock:
            self._connected = True
            self._latest_data = copied
            self._latest_wall_sec = time.time()
            self._latest_monotonic_sec = time.monotonic()
            self._sequence += 1

    def read(self, *, timeout_ms: float | None = None) -> HebiMobileIOSnapshot:
        del timeout_ms
        now_wall = time.time()
        now_monotonic = time.monotonic()
        with self._lock:
            connected = self._connected
            data = None if self._latest_data is None else dict(self._latest_data)
            received_wall = self._latest_wall_sec
            received_monotonic = self._latest_monotonic_sec
            sequence = self._sequence
        if data is None:
            return self._invalid_snapshot(now_wall, "waiting_for_phone" if not connected else "waiting_for_pose")
        if not connected:
            return self._invalid_snapshot(now_wall, "phone_disconnected")
        age_sec = max(0.0, now_monotonic - received_monotonic)
        if age_sec > self.max_stale_feedback_sec:
            return self._invalid_snapshot(now_wall, "feedback_stale", age_sec=age_sec, sequence=sequence)

        try:
            position = np.asarray(data.get("position"), dtype=np.float64)
            if position.shape != (3,) or not np.all(np.isfinite(position)):
                raise ValueError("TeleDex position must contain 3 finite values.")
            quaternion = rotation_matrix_to_quaternion_wxyz(data.get("rotation"))
        except (TypeError, ValueError) as exc:
            return self._invalid_snapshot(
                now_wall,
                "invalid_pose",
                sequence=sequence,
                detail=str(exc),
            )

        button = bool(data.get("button", False))
        button_secondary = bool(data.get("button_secondary", False))
        toggle = bool(data.get("toggle", False))
        deadman = (
            button
            if self.deadman_field == "button"
            else button_secondary
            if self.deadman_field == "button_secondary"
            else toggle
            if self.deadman_field == "toggle"
            else False
        )
        return HebiMobileIOSnapshot(
            timestamp_sec=received_wall,
            position_m=position.astype(float).tolist(),
            quaternion_wxyz=quaternion,
            raw_inputs={
                "source": "teledex",
                "sequence": sequence,
                "age_sec": age_sec,
                "button": button,
                "button_secondary": button_secondary,
                "toggle": toggle,
                "deadman_field": self.deadman_field,
                "b1": deadman,
                "a3": self.precision_scale,
            },
            valid=True,
            reason="ok",
        )

    @staticmethod
    def _invalid_snapshot(
        timestamp_sec: float,
        reason: str,
        *,
        age_sec: float | None = None,
        sequence: int | None = None,
        detail: str | None = None,
    ) -> HebiMobileIOSnapshot:
        raw_inputs: dict[str, Any] = {"source": "teledex", "b1": False}
        if age_sec is not None:
            raw_inputs["age_sec"] = float(age_sec)
        if sequence is not None:
            raw_inputs["sequence"] = int(sequence)
        if detail is not None:
            raw_inputs["detail"] = detail
        return HebiMobileIOSnapshot(
            timestamp_sec=float(timestamp_sec),
            position_m=[0.0, 0.0, 0.0],
            quaternion_wxyz=[1.0, 0.0, 0.0, 0.0],
            raw_inputs=raw_inputs,
            valid=False,
            reason=reason,
        )

    def __enter__(self) -> "TeleDexPhoneClient":
        self.connect()
        return self

    def __exit__(self, _exc_type: Any, _exc: Any, _traceback: Any) -> None:
        self.close()
