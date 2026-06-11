from __future__ import annotations

from rh56_driver.serial_backend import RH56SerialBackend


def _u16_bytes(values: list[int]) -> list[int]:
    data: list[int] = []
    for value in values:
        data.extend([value & 0xFF, (value >> 8) & 0xFF])
    return data


def test_serial_backend_converts_canonical_commands_to_official_protocol_order() -> None:
    backend = RH56SerialBackend({"hand_schema": {"protocol_order": ["pinky", "ring", "middle", "index", "thumb_close", "thumb_lateral"]}})
    writes: list[tuple[int, list[int]]] = []

    def fake_write_register(address: int, data_bytes: list[int]) -> bool:
        writes.append((address, data_bytes))
        return True

    backend.write_register = fake_write_register  # type: ignore[method-assign]

    assert backend.set_canonical_angles([10, 20, 30, 40, 50, 60]) is True

    assert writes == [(backend.REG["ANGLE_SET"], _u16_bytes([40, 30, 20, 10, 50, 60]))]


def test_serial_backend_returns_canonical_state_from_official_protocol_order() -> None:
    backend = RH56SerialBackend({"hand_schema": {"protocol_order": ["pinky", "ring", "middle", "index", "thumb_close", "thumb_lateral"]}})
    reads = {
        backend.REG["ANGLE_ACT"]: _u16_bytes([4, 3, 2, 1, 5, 6]),
        backend.REG["FORCE_ACT"]: _u16_bytes([40, 30, 20, 10, 50, 60]),
        backend.REG["CURRENT"]: _u16_bytes([400, 300, 200, 100, 500, 600]),
    }

    def fake_read_register(address: int, length: int) -> list[int]:
        assert length == 12
        return reads[address]

    backend.read_register = fake_read_register  # type: ignore[method-assign]

    state = backend.read_state()

    assert state.finger_positions == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    assert state.force_estimate == [10.0, 20.0, 30.0, 40.0, 50.0, 60.0]
    assert state.finger_currents == [100.0, 200.0, 300.0, 400.0, 500.0, 600.0]
    assert state.contact_flags == [True, True, True, True, True, True]


def test_serial_backend_decodes_force_feedback_as_signed_16_bit() -> None:
    backend = RH56SerialBackend({"hand_schema": {"protocol_order": ["pinky", "ring", "middle", "index", "thumb_close", "thumb_lateral"]}})

    def fake_read_register(address: int, length: int) -> list[int]:
        assert address == backend.REG["FORCE_ACT"]
        assert length == 12
        return _u16_bytes([65535, 65505, 10, 0, 65349, 32769])

    backend.read_register = fake_read_register  # type: ignore[method-assign]

    assert backend.get_forces() == [-1.0, -31.0, 10.0, 0.0, -187.0, -32767.0]
    assert backend.get_canonical_forces() == [0.0, 10.0, -31.0, -1.0, -187.0, -32767.0]
