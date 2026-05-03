from __future__ import annotations

from jaka_driver_adapter.adapter import JakaDriverAdapter
from embodiment_core.types import Pose


def test_mock_jaka_basic_flow() -> None:
    adapter = JakaDriverAdapter.from_yaml("configs/robot/jaka_mini2.yaml")
    assert adapter.connect() is True
    state = adapter.get_joint_state()
    assert len(state.positions) == 6
    assert adapter.move_joints([0.1] * 6) is True
    assert adapter.get_joint_state().positions == [0.1] * 6
    assert (
        adapter.move_pose(
            Pose(position=[0.3, 0.1, 0.2], orientation_xyzw=[0.0, 0.0, 0.0, 1.0])
        )
        is True
    )
    adapter.set_speed_scale(0.5)
    adapter.stop()

