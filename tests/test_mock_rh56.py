from __future__ import annotations

from rh56_driver.node import RH56Driver


def test_mock_rh56_adapter_supports_the_offline_command_flow() -> None:
    driver = RH56Driver.from_yaml("configs/hand/rh56.yaml")
    assert driver.connect() is True

    assert driver.open() is True
    assert driver.read_state().mode == "open"
    assert driver.close() is True
    assert driver.read_state().finger_positions == [1.0] * 6
    assert driver.pinch() is True
    assert driver.preset_grasp("tripod") is True
    driver.stop()
