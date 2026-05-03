from __future__ import annotations

from quadruped_adapter.adapter import QuadrupedAdapter


def test_mock_quadruped_basic_flow() -> None:
    dog = QuadrupedAdapter.from_yaml("configs/quadruped/default.yaml")
    assert dog.connect() is True
    assert dog.stand() is True
    assert dog.teleop({"linear_x": 0.2, "linear_y": 0.0, "angular_z": 0.1}) is True
    state = dog.get_robot_state()
    assert state.mode == "teleop"
    odom = dog.get_odom()
    assert odom["frame_id"] == "odom"
    dog.start_recording_hint()
    dog.estop()
    assert dog.get_robot_state().estopped is True

