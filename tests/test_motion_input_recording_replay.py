from __future__ import annotations

import json

from motion_input.provider import ProviderState
from motion_input.recording import MotionRecordingReader, MotionRecordingWriter
from motion_input.replay import ReplayMode, ReplayProvider

from test_motion_input_protocol import make_device, make_sample


def test_recording_and_replay_use_the_same_sample_contract(tmp_path) -> None:
    path = tmp_path / "session.umip.jsonl"
    expected = [make_sample(index) for index in range(5)]
    with MotionRecordingWriter(
        path,
        recording_id="recording-test",
        device=make_device(),
        metadata={"purpose": "unit-test"},
    ) as writer:
        for sample in expected:
            writer.write(sample)
    assert writer.sample_count == 5

    reader = MotionRecordingReader(path)
    assert list(reader.samples()) == expected
    assert reader.header is not None
    assert reader.header.format_version == "1.0"
    assert reader.footer_sample_count == 5

    provider = ReplayProvider(str(path), mode=ReplayMode.IMMEDIATE)
    with provider:
        actual = list(provider.iter_samples())
        assert provider.state is ProviderState.EXHAUSTED
    assert actual == expected


def test_recording_is_recoverable_without_footer(tmp_path) -> None:
    path = tmp_path / "interrupted.umip.jsonl"
    with MotionRecordingWriter(path, recording_id="interrupted", device=make_device()) as writer:
        writer.write(make_sample(0))
        writer.write(make_sample(1))
    lines = path.read_text(encoding="utf-8").splitlines()
    path.write_text("\n".join(lines[:-1]) + "\n", encoding="utf-8")
    reader = MotionRecordingReader(path)
    assert [sample.sequence_number for sample in reader.samples()] == [0, 1]
    assert reader.footer_sample_count is None


def test_same_major_unknown_record_type_is_skipped(tmp_path) -> None:
    path = tmp_path / "future.umip.jsonl"
    with MotionRecordingWriter(path, recording_id="future", device=make_device()) as writer:
        writer.write(make_sample(0))
    lines = path.read_text(encoding="utf-8").splitlines()
    lines.insert(1, json.dumps({"record_type": "future_index", "payload": [1, 2]}))
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    assert [sample.sequence_number for sample in MotionRecordingReader(path).samples()] == [0]


def test_fixed_rate_replay_uses_injected_clock_without_changing_samples(tmp_path) -> None:
    path = tmp_path / "timed.umip.jsonl"
    with MotionRecordingWriter(path, recording_id="timed", device=make_device()) as writer:
        writer.write(make_sample(0))
        writer.write(make_sample(1))
    now = [0]
    sleeps: list[float] = []

    def monotonic_ns() -> int:
        return now[0]

    def sleep(seconds: float) -> None:
        sleeps.append(seconds)
        now[0] += int(seconds * 1e9)

    provider = ReplayProvider(
        str(path),
        mode=ReplayMode.FIXED_RATE,
        fixed_rate_hz=100.0,
        monotonic_ns=monotonic_ns,
        sleep=sleep,
    )
    with provider:
        assert provider.read() == make_sample(0)
        assert provider.read() == make_sample(1)
    assert sleeps == [0.01]
