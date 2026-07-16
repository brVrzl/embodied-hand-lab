#!/usr/bin/env python3
"""Record, replay, visualize, and diagnose UMIP motion data only."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time
import uuid

from motion_input import (
    DeviceDescriptor,
    MotionRecordingWriter,
    QuestMotionProvider,
    ReplayMode,
    ReplayProvider,
    StreamingDiagnostics,
    UdpQuestSource,
)
from motion_input.serialization import dumps_sample
from motion_input.visualization import MotionInputVisualizer


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    record = subparsers.add_parser("record", help="record Quest UDP input as UMIP")
    record.add_argument("output", type=Path)
    record.add_argument("--host", default="0.0.0.0")
    record.add_argument("--port", type=int, default=7060)
    record.add_argument("--allowed-sender")
    record.add_argument("--device-id", default="quest-3")
    record.add_argument("--model", default="Meta Quest 3")
    record.add_argument("--serial-number")
    record.add_argument("--duration-sec", type=float)
    record.add_argument("--max-samples", type=int)
    record.add_argument("--flush-every-sample", action="store_true")

    replay = subparsers.add_parser("replay", help="replay a UMIP recording to stdout")
    _add_replay_arguments(replay)

    visualize = subparsers.add_parser("visualize", help="visualize a UMIP recording")
    _add_replay_arguments(visualize)
    visualize.add_argument("--matplotlib", action="store_true")
    visualize.add_argument("--refresh-hz", type=float, default=10.0)

    diagnose = subparsers.add_parser("diagnose", help="produce a quantitative JSON report")
    diagnose.add_argument("recording", type=Path)
    diagnose.add_argument("--output", type=Path)
    return parser


def _add_replay_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("recording", type=Path)
    parser.add_argument(
        "--mode", choices=[mode.value for mode in ReplayMode], default=ReplayMode.AS_RECORDED.value
    )
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--fixed-rate-hz", type=float)


def _replay_provider(args: argparse.Namespace) -> ReplayProvider:
    return ReplayProvider(
        str(args.recording),
        mode=ReplayMode(args.mode),
        speed=args.speed,
        fixed_rate_hz=args.fixed_rate_hz,
    )


def _record(args: argparse.Namespace) -> int:
    if args.duration_sec is not None and args.duration_sec <= 0:
        raise SystemExit("--duration-sec must be positive")
    if args.max_samples is not None and args.max_samples <= 0:
        raise SystemExit("--max-samples must be positive")
    device = DeviceDescriptor(
        device_id=args.device_id,
        device_type="xr_headset",
        manufacturer="Meta",
        model=args.model,
        serial_number=args.serial_number,
        metadata={"transport": "udp", "input_only": True},
    )
    provider = QuestMotionProvider(
        UdpQuestSource(args.host, args.port, allowed_sender=args.allowed_sender),
        device=device,
    )
    diagnostics = StreamingDiagnostics()
    started = time.monotonic()
    with provider, MotionRecordingWriter(
        args.output,
        recording_id=str(uuid.uuid4()),
        device=device,
        metadata={"source": "quest_udp"},
        flush_every_sample=args.flush_every_sample,
    ) as writer:
        try:
            while True:
                if args.duration_sec is not None and time.monotonic() - started >= args.duration_sec:
                    break
                if args.max_samples is not None and writer.sample_count >= args.max_samples:
                    break
                sample = provider.read(timeout_s=0.25)
                if sample is None:
                    continue
                writer.write(sample)
                diagnostics.observe(sample)
        except KeyboardInterrupt:
            pass
    print(json.dumps({"samples": writer.sample_count, "diagnostics": diagnostics.report()}, indent=2))
    return 0


def _replay(args: argparse.Namespace) -> int:
    provider = _replay_provider(args)
    with provider:
        for sample in provider.iter_samples(timeout_s=None):
            print(dumps_sample(sample))
    return 0


def _visualize(args: argparse.Namespace) -> int:
    if args.refresh_hz <= 0:
        raise SystemExit("--refresh-hz must be positive")
    provider = _replay_provider(args)
    visualizer = MotionInputVisualizer()
    next_render = 0.0
    with provider:
        for sample in provider.iter_samples(timeout_s=None):
            visualizer.observe(sample)
            now = time.monotonic()
            if now < next_render:
                continue
            if args.matplotlib:
                visualizer.render_matplotlib()
            else:
                print("\x1b[2J\x1b[H" + visualizer.render_text(), flush=True)
            next_render = now + 1.0 / args.refresh_hz
    if not args.matplotlib:
        print(visualizer.render_text())
    return 0


def _diagnose(args: argparse.Namespace) -> int:
    diagnostics = StreamingDiagnostics()
    provider = ReplayProvider(str(args.recording), mode=ReplayMode.IMMEDIATE)
    with provider:
        for sample in provider.iter_samples():
            diagnostics.observe(sample)
    payload = json.dumps(diagnostics.report(), indent=2, sort_keys=True) + "\n"
    if args.output is None:
        sys.stdout.write(payload)
    else:
        # CLI output is intentionally explicit; core recording never writes reports implicitly.
        args.output.write_text(payload, encoding="utf-8")
    return 0


def main() -> int:
    args = _parser().parse_args()
    if args.command == "record":
        return _record(args)
    if args.command == "replay":
        return _replay(args)
    if args.command == "visualize":
        return _visualize(args)
    if args.command == "diagnose":
        return _diagnose(args)
    raise AssertionError(args.command)


if __name__ == "__main__":
    raise SystemExit(main())
