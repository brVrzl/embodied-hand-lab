from __future__ import annotations

from rh56_driver.jaka_tio_signal_client import extract_tio_signal_values


def test_extract_tio_signal_values_from_rs485_signal_list_response() -> None:
    response = {
        "rx": [
            {
                "errorCode": "0",
                "errorMsg": "",
                "rh56_angle_0": {"sigType": 3, "sigAddr": 1546, "chnId": 1, "frequency": 0.0, "value": 1000},
                "rh56_angle_1": {"sigType": 3, "sigAddr": 1548, "chnId": 1, "frequency": 0.0, "value": 0},
                "num": 2,
                "cmdName": "get_rs485_signal_list",
            }
        ]
    }

    assert extract_tio_signal_values(response) == {
        "rh56_angle_0": 1000.0,
        "rh56_angle_1": 0.0,
    }


def test_extract_tio_signal_values_from_get_tio_signals_response() -> None:
    response = {
        "rx": [
            {
                "signals": [
                    ["rh56_angle_0", 1, 3, 1546, 1000, 0.0],
                    ["rh56_angle_1", 1, 3, 1548, 0, 0.0],
                ],
                "errorCode": "0",
                "cmdName": "get_tio_signals",
                "errorMsg": "",
            }
        ]
    }

    assert extract_tio_signal_values(response) == {
        "rh56_angle_0": 1000.0,
        "rh56_angle_1": 0.0,
    }
