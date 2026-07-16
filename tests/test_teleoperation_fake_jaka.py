from __future__ import annotations

import threading

import pytest

from teleoperation.jaka.fake_backend import FakeJakaBackend


def test_fake_backend_repeated_lifecycle_and_deterministic_cleanup() -> None:
    for _ in range(2):
        backend = FakeJakaBackend()
        backend.connect()
        backend.enter_edg()
        backend.command_joints(1, (0.0,) * 6)
        backend.disconnect()
        assert backend.cleanup_events == ["leave_edg", "disconnect"]


def test_fake_backend_injected_failure_still_supports_explicit_cleanup() -> None:
    backend = FakeJakaBackend(fail_operations={"command_joints"})
    backend.connect()
    backend.enter_edg()
    with pytest.raises(RuntimeError, match="injected"):
        backend.command_joints(1, (0.0,) * 6)
    backend.disconnect()
    assert not backend.connected and not backend.edg_active


def test_fake_backend_enforces_single_thread_owner() -> None:
    backend = FakeJakaBackend()
    backend.connect()
    errors: list[Exception] = []

    def other_thread() -> None:
        try:
            backend.read_state()
        except Exception as error:  # test captures the owner violation
            errors.append(error)

    thread = threading.Thread(target=other_thread)
    thread.start()
    thread.join()
    assert "non-owner thread" in str(errors[0])
    backend.disconnect()
