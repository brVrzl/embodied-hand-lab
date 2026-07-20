from __future__ import annotations

import math
import time
from typing import Any

import numpy as np
import pytest

from teleop_tools.teledex_phone import TeleDexPhoneClient, rotation_matrix_to_quaternion_wxyz


class _AliveThread:
    def is_alive(self) -> bool:
        return True


class FakeSession:
    def __init__(self, *, port: int, debug: bool, show_qr: bool) -> None:
        self.port = port
        self.debug = debug
        self.show_qr = show_qr
        self.ip_address = "192.168.1.20"
        self.server: object | None = None
        self._thread = _AliveThread()
        self.update_callback = None
        self.connect_callback = None
        self.disconnect_callback = None
        self.stopped = False

    def on_update(self, callback: Any) -> None:
        self.update_callback = callback

    def on_connect(self, callback: Any) -> None:
        self.connect_callback = callback

    def on_disconnect(self, callback: Any) -> None:
        self.disconnect_callback = callback

    def start(self) -> None:
        self.server = object()

    def stop(self) -> None:
        self.stopped = True

    def push(self, data: dict[str, Any]) -> None:
        assert self.connect_callback is not None
        assert self.update_callback is not None
        self.connect_callback(self)
        self.update_callback(self, data)

    def disconnect(self) -> None:
        assert self.disconnect_callback is not None
        self.disconnect_callback(self)


def _make_client(**kwargs: Any) -> tuple[TeleDexPhoneClient, FakeSession]:
    sessions: list[FakeSession] = []

    def factory(**factory_kwargs: Any) -> FakeSession:
        session = FakeSession(**factory_kwargs)
        sessions.append(session)
        return session

    client = TeleDexPhoneClient(
        session_factory=factory,
        check_port_available=False,
        **kwargs,
    )
    client.connect()
    return client, sessions[0]


def test_rotation_matrix_to_quaternion_wxyz() -> None:
    angle = math.pi / 2.0
    matrix = np.asarray(
        [[math.cos(angle), -math.sin(angle), 0.0], [math.sin(angle), math.cos(angle), 0.0], [0.0, 0.0, 1.0]]
    )
    assert rotation_matrix_to_quaternion_wxyz(matrix) == pytest.approx(
        [math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)]
    )


def test_teledex_button_maps_to_deadman_and_pose_snapshot() -> None:
    client, session = _make_client(deadman_field="button", precision_scale=0.4)
    session.push(
        {
            "position": np.asarray([0.1, -0.2, 0.3]),
            "rotation": np.eye(3),
            "button": True,
            "toggle": False,
        }
    )
    snapshot = client.read()
    assert snapshot.valid
    assert snapshot.enabled
    assert snapshot.position_m == pytest.approx([0.1, -0.2, 0.3])
    assert snapshot.quaternion_wxyz == pytest.approx([1.0, 0.0, 0.0, 0.0])
    assert snapshot.raw_inputs["a3"] == pytest.approx(0.4)
    assert snapshot.raw_inputs["deadman_field"] == "button"
    client.close()
    assert session.stopped


def test_disconnect_and_stale_feedback_release_deadman() -> None:
    client, session = _make_client(max_stale_feedback_sec=0.01)
    session.push(
        {
            "position": np.zeros(3),
            "rotation": np.eye(3),
            "button": True,
            "toggle": False,
        }
    )
    assert client.read().enabled
    session.disconnect()
    disconnected = client.read()
    assert not disconnected.valid
    assert not disconnected.enabled
    assert disconnected.reason == "phone_disconnected"

    session.push(
        {
            "position": np.zeros(3),
            "rotation": np.eye(3),
            "button": True,
            "toggle": False,
        }
    )
    time.sleep(0.02)
    stale = client.read()
    assert not stale.valid
    assert not stale.enabled
    assert stale.reason == "feedback_stale"


def test_invalid_rotation_is_rejected_and_releases_deadman() -> None:
    client, session = _make_client()
    session.push(
        {
            "position": np.zeros(3),
            "rotation": np.zeros((3, 3)),
            "button": True,
            "toggle": False,
        }
    )
    snapshot = client.read()
    assert not snapshot.valid
    assert not snapshot.enabled
    assert snapshot.reason == "invalid_pose"


def test_toggle_deadman_requires_explicit_configuration() -> None:
    client, session = _make_client(deadman_field="toggle")
    session.push(
        {
            "position": np.zeros(3),
            "rotation": np.eye(3),
            "button": False,
            "toggle": True,
        }
    )
    assert client.read().enabled
    with pytest.raises(ValueError, match="deadman_field"):
        TeleDexPhoneClient(deadman_field="button_c")


def test_secondary_button_can_be_observed_and_selected_as_deadman() -> None:
    client, session = _make_client(deadman_field="button_secondary")
    session.push(
        {
            "position": np.zeros(3),
            "rotation": np.eye(3),
            "button": False,
            "button_secondary": True,
            "toggle": False,
        }
    )
    snapshot = client.read()
    assert snapshot.valid
    assert snapshot.enabled
    assert snapshot.raw_inputs["button"] is False
    assert snapshot.raw_inputs["button_secondary"] is True


def test_none_deadman_exposes_buttons_but_never_enables_motion() -> None:
    client, session = _make_client(deadman_field="none")
    session.push(
        {
            "position": np.zeros(3),
            "rotation": np.eye(3),
            "button": True,
            "toggle": True,
        }
    )
    snapshot = client.read()
    assert snapshot.valid
    assert snapshot.raw_inputs["button"] is True
    assert snapshot.raw_inputs["toggle"] is True
    assert not snapshot.enabled
