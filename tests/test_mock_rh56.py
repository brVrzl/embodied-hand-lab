from __future__ import annotations

from rh56_driver.node import RH56Driver


def test_mock_rh56_basic_flow() -> None:
    driver = RH56Driver.from_yaml("configs/hand/rh56.yaml")
    assert driver.connect() is True
    assert driver.open() is True
    state = driver.read_state()
    assert state.mode == "open"
    assert driver.close() is True
    state = driver.read_state()
    assert all(value == 1.0 for value in state.finger_positions)
    assert driver.pinch() is True
    assert driver.preset_grasp("tripod") is True
    driver.stop()

