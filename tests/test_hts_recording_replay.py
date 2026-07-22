from __future__ import annotations

import ast
from pathlib import Path

from motion_input import (
    HtsCanonicalAssembler,
    HtsRawRecordingWriter,
    ReceivedHtsDatagram,
    inspect_datagram,
    parse_hts_datagram,
    replay_datagrams,
)


def _payload(side: str, sequence: int) -> bytes:
    points = ",".join("0" for _ in range(63))
    return (
        f"{side} wrist | f = {sequence} | t = {sequence * 100}:, 1,2,3,0,0,0,1\n"
        f"{side} landmarks | f = {sequence} | t = {sequence * 100}:, {points}"
    ).encode()


def _state_from(datagrams: list[ReceivedHtsDatagram]):
    assembler = HtsCanonicalAssembler()
    state = assembler.state(now_monotonic_ns=0)
    for datagram in datagrams:
        state = assembler.ingest(
            parse_hts_datagram(datagram.payload),
            receive_monotonic_ns=datagram.receive_monotonic_ns,
            source_endpoint=datagram.source_endpoint,
            datagram_size=len(datagram.payload),
        )
    return state


def test_raw_capture_replay_uses_same_parser_and_canonical_contract(tmp_path: Path) -> None:
    datagrams = [
        ReceivedHtsDatagram(_payload("Left", 1), "10.24.2.3", 50001, 1000, 2000),
        ReceivedHtsDatagram(_payload("Right", 1), "10.24.2.3", 50002, 1100, 2100),
    ]
    path = tmp_path / "capture.hts.jsonl"
    with HtsRawRecordingWriter(path, metadata={"test": True}) as writer:
        for datagram in datagrams:
            writer.write(datagram)

    replayed = list(replay_datagrams(path))
    assert replayed == datagrams
    live_state = _state_from(datagrams)
    replay_state = _state_from(replayed)
    assert replay_state.left == live_state.left
    assert replay_state.right == live_state.right
    assert replay_state.host_monotonic_ns == live_state.host_monotonic_ns


def test_raw_inspector_reports_required_fields() -> None:
    datagram = ReceivedHtsDatagram(b"Right wrist:, 1,2,3,0,0,0,1", "10.0.0.2", 7, 11, 12)
    report = inspect_datagram(datagram)
    assert report["source_endpoint"] == "10.0.0.2:7"
    assert report["datagram_bytes"] == len(datagram.payload)
    assert report["receive_monotonic_ns"] == 11
    assert report["utf8_preview"] == datagram.payload.decode()
    assert report["apparent_delimiters"]["comma"] == 7


def test_live_tool_imports_no_robot_or_hand_driver_modules() -> None:
    path = Path(__file__).parents[1] / "tools" / "quest_hand_tracking_streamer.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    imported: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module)
    forbidden = ("jaka", "rh56", "inspire", "teleoperation")
    assert not any(name.startswith(forbidden) for name in imported)
