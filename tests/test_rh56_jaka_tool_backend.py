from __future__ import annotations

from rh56_driver.jaka_tool_backend import RH56JakaToolBackend


def test_rh56_jaka_tool_backend_builds_known_open_close_frames() -> None:
    config = {
        "gesture_presets": {
            "open": [1000, 1000, 1000, 1000, 1000, 1000],
            "close": [0, 0, 0, 0, 0, 0],
        },
        "jaka_tool_rs485": {
            "robot_config_path": "configs/robot/jaka_mini2.yaml",
            "channel_id": 1,
            "hand_id": 1,
        },
        "serial": {
            "hand_id": 1,
        },
    }

    backend = RH56JakaToolBackend(config)

    assert backend.build_open_frame().hex(" ").upper() == (
        "EB 90 01 0F 12 CE 05 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 77"
    )
    assert backend.build_close_frame().hex(" ").upper() == (
        "EB 90 01 0F 12 CE 05 00 00 00 00 00 00 00 00 00 00 00 00 F5"
    )


def test_rh56_jaka_tool_backend_reads_tcp_signal_feedback() -> None:
    class FakeSignalClient:
        def get_signal_values(self) -> dict[str, float]:
            return {
                "rh56_angle_0": 1.0,
                "rh56_angle_1": 2.0,
                "rh56_angle_2": 3.0,
                "rh56_angle_3": 4.0,
                "rh56_angle_4": 5.0,
                "rh56_angle_5": 6.0,
            }

    backend = RH56JakaToolBackend(
        {
            "gesture_presets": {"open": [1000, 1000, 1000, 1000, 1000, 1000]},
            "jaka_tool_rs485": {
                "robot_config_path": "configs/robot/jaka_mini2.yaml",
                "channel_id": 1,
                "hand_id": 1,
                "state_feedback": {"enabled": True},
            },
            "serial": {"hand_id": 1},
        }
    )
    backend.signal_client = FakeSignalClient()  # type: ignore[assignment]

    assert backend.read_state().finger_positions == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
