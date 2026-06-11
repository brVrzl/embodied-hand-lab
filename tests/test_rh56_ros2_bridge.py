from __future__ import annotations

import pytest

from embodiment_core.types import HandState
from rh56_driver.ros2_bridge import (
    STATE_SCHEMA_VERSION,
    apply_angle_command,
    apply_force_command,
    build_raw_feedback_payload,
    build_state_payload,
    parse_angle_command,
    parse_code_command,
    parse_force_command,
)


class FakeCanonicalBackend:
    REG = {"STATUS": 1612, "ERROR": 1606, "TEMP": 1618}

    def __init__(self) -> None:
        self.protocol_order = ["pinky", "ring", "middle", "index", "thumb_close", "thumb_lateral"]
        self.angles: list[int] | None = None
        self.forces: list[int] | None = None

    def set_canonical_angles(self, values: list[int]) -> bool:
        self.angles = values
        return True

    def set_canonical_forces(self, values: list[int]) -> bool:
        self.forces = values
        return True

    def get_angles(self) -> list[float]:
        return [4.0, 3.0, 2.0, 1.0, 5.0, 6.0]

    def get_forces(self) -> list[float]:
        return [40.0, 30.0, 20.0, 10.0, 50.0, 60.0]

    def get_currents(self) -> list[float]:
        return [400.0, 300.0, 200.0, 100.0, 500.0, 600.0]

    def read_register(self, address: int, length: int) -> list[int]:
        assert length == 6
        return [address] * 6


def test_state_payload_uses_explicit_units_and_canonical_order() -> None:
    state = HandState(
        mode="open",
        finger_positions=[0.0, 0.1, 0.2, 0.3, 0.4, 0.5],
        finger_currents=[1.0] * 6,
        contact_flags=[False, False, True, True, False, True],
        force_estimate=[2.0] * 6,
    )

    payload = build_state_payload(
        state,
        backend_mode="mock",
        timestamp=123.0,
        position_unit="normalized_0_1",
    )

    assert payload["schema_version"] == STATE_SCHEMA_VERSION
    assert payload["timestamp"] == 123.0
    assert payload["hand"]["position_unit"] == "normalized_0_1"
    assert payload["hand"]["order"] == ["index", "middle", "ring", "pinky", "thumb_close", "thumb_lateral"]
    assert payload["hand"]["contact_binary"] == [False, False, True, True, False, True]


def test_command_parsers_reject_ambiguous_units_or_orders() -> None:
    assert parse_angle_command({"values": [1000, 900, 800, 700, 600, 500]}).unit == "rh56_angle_raw_0_1000"
    assert parse_force_command({"values": [500] * 6}).unit == "rh56_force_raw_0_1000"
    assert parse_code_command({"preset": "tripod"}).preset_name == "tripod"

    with pytest.raises(ValueError, match="canonical"):
        parse_angle_command({"values": [1000] * 6, "order": "protocol"})
    with pytest.raises(ValueError, match="calibrated"):
        apply_angle_command(FakeCanonicalBackend(), parse_angle_command({"values": [0.0] * 6, "unit": "normalized_0_1"}))


def test_apply_commands_and_raw_feedback_payload() -> None:
    backend = FakeCanonicalBackend()

    assert apply_angle_command(backend, parse_angle_command({"values": [1000, 900, 800, 700, 600, 500]})) is True
    assert apply_force_command(backend, parse_force_command({"values": [100, 200, 300, 400, 500, 600]})) is True

    assert backend.angles == [1000, 900, 800, 700, 600, 500]
    assert backend.forces == [100, 200, 300, 400, 500, 600]

    payload = build_raw_feedback_payload(backend, timestamp=456.0)  # type: ignore[arg-type]
    assert payload["timestamp"] == 456.0
    assert payload["available"] is True
    assert payload["protocol_order"] == ["pinky", "ring", "middle", "index", "thumb_close", "thumb_lateral"]
    assert payload["angles_raw"] == [4.0, 3.0, 2.0, 1.0, 5.0, 6.0]


def test_apply_commands_clamp_to_configured_hand_safety_limits() -> None:
    backend = FakeCanonicalBackend()
    backend.config = {
        "safety": {"max_close_strength": 0.8, "max_force_limit": 260},
        "hand_schema": {
            "dof_calibration": {
                name: {
                    "raw_open": 1000,
                    "raw_close": 0,
                    "safe_min": 0,
                    "safe_max": 1000,
                    "default_force_limit": 260,
                }
                for name in ["index", "middle", "ring", "pinky", "thumb_close", "thumb_lateral"]
            }
        },
    }

    assert apply_angle_command(backend, parse_angle_command({"values": [0, 100, 200, 300, 400, 500]})) is True
    assert apply_force_command(backend, parse_force_command({"values": [1000] * 6})) is True

    assert backend.angles == [200, 200, 200, 300, 400, 500]
    assert backend.forces == [260] * 6
