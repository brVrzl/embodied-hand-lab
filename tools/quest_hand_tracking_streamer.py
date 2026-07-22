#!/usr/bin/env python3
"""Inspect, validate, record, and replay Meta Quest Hand Tracking Streamer input.

This executable imports only ``motion_input`` and Python's standard library.
It has no JAKA SDK, Inspire/RH56, ROS control, IK, or command output path.
"""

from __future__ import annotations

import argparse
from datetime import datetime
import ipaddress
import json
from pathlib import Path
import socket
import sys
import time

from motion_input import (
    HTS_DEFAULT_UDP_PORT,
    HtsCanonicalAssembler,
    HtsRawRecordingWriter,
    HtsTelemetry,
    HtsUdpReceiver,
    SerializationError,
    evaluate_required_right_hand_recording,
    inspect_datagram,
    parse_hts_datagram,
    replay_datagrams,
)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    inspect = subparsers.add_parser("inspect", help="bounded raw UDP packet inspection")
    _add_network_arguments(inspect)
    inspect.add_argument("--duration-sec", type=float, default=15.0)
    inspect.add_argument("--preview-count", type=int, default=5)
    inspect.add_argument("--output", type=Path)

    live = subparsers.add_parser("live", help="bounded validated Quest-only gate")
    _add_network_arguments(live)
    live.add_argument("--duration-sec", type=float, default=90.0)
    live.add_argument("--stale-ms", type=float, default=250.0)
    live.add_argument("--frozen-sec", type=float, default=2.0)
    live.add_argument("--output", type=Path)
    live.add_argument("--report", type=Path)
    live.add_argument("--telemetry-hz", type=float, default=2.0)

    replay = subparsers.add_parser("replay", help="replay raw capture through live parser")
    replay.add_argument("recording", type=Path)
    replay.add_argument("--as-recorded", action="store_true")
    replay.add_argument("--speed", type=float, default=1.0)
    replay.add_argument("--stale-ms", type=float, default=250.0)
    replay.add_argument("--frozen-sec", type=float, default=2.0)
    replay.add_argument(
        "--pose-table-hz",
        type=float,
        default=0.0,
        help="optional recorded-time terminal pose table rate; zero disables it",
    )
    replay.add_argument("--report", type=Path)
    return parser


def _add_network_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=HTS_DEFAULT_UDP_PORT)
    parser.add_argument("--project-ip", help="LAN address shown for entry in the Quest app")
    parser.add_argument("--allowed-sender", help="optional Quest IPv4 allow-list")


def _project_ip(explicit: str | None) -> str:
    if explicit:
        candidate = explicit
    else:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            # UDP connect selects a route but sends no packet.
            probe.connect(("1.1.1.1", 9))
            candidate = str(probe.getsockname()[0])
        finally:
            probe.close()
    try:
        address = ipaddress.ip_address(candidate)
    except ValueError as exc:
        raise SystemExit(f"invalid --project-ip {candidate!r}") from exc
    if not isinstance(address, ipaddress.IPv4Address):
        raise SystemExit("PROJECT_IP must be IPv4 for HTS v1.1")
    if address.is_loopback or address.is_unspecified or address.is_multicast:
        raise SystemExit(f"refusing unsuitable PROJECT_IP={address}")
    return str(address)


def _default_capture_path(prefix: str) -> Path:
    stamp = datetime.now().astimezone().strftime("%Y%m%dT%H%M%S%z")
    return Path("logs/quest_input") / f"{prefix}_{stamp}.hts.jsonl"


def _announce(project_ip: str, port: int, output: Path) -> None:
    print(f"PROJECT_IP={project_ip}", flush=True)
    print(f"PORT={port}", flush=True)
    print("TRANSPORT=UDP", flush=True)
    print(f"RAW_LOG={output.resolve()}", flush=True)
    print("SAFETY=Quest input only; JAKA and Inspire are not imported or connected", flush=True)


def _inspect(args: argparse.Namespace) -> int:
    _validate_positive(args.duration_sec, "--duration-sec")
    if args.preview_count < 0:
        raise SystemExit("--preview-count must be non-negative")
    project_ip = _project_ip(args.project_ip)
    output = args.output or _default_capture_path("hts_raw_inspect")
    output.parent.mkdir(parents=True, exist_ok=True)
    _announce(project_ip, args.port, output)
    started = time.monotonic()
    count = 0
    byte_count = 0
    sources: set[str] = set()
    with HtsUdpReceiver(
        args.bind, args.port, allowed_sender=args.allowed_sender
    ) as receiver, HtsRawRecordingWriter(
        output,
        metadata={"mode": "raw_inspect", "project_ip": project_ip, "port": args.port},
    ) as writer:
        try:
            while time.monotonic() - started < args.duration_sec:
                datagram = receiver.receive(timeout_s=0.2)
                if datagram is None:
                    continue
                writer.write(datagram)
                count += 1
                byte_count += len(datagram.payload)
                sources.add(datagram.source_endpoint)
                if count <= args.preview_count:
                    print(json.dumps(inspect_datagram(datagram), sort_keys=True), flush=True)
        except KeyboardInterrupt:
            print("STOP=operator interrupt", flush=True)
    elapsed = max(time.monotonic() - started, 1e-9)
    print(
        json.dumps(
            {
                "datagrams": count,
                "bytes": byte_count,
                "duration_s": elapsed,
                "datagram_rate_hz": count / elapsed,
                "sources": sorted(sources),
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if count else 2


def _live(args: argparse.Namespace) -> int:
    _validate_positive(args.duration_sec, "--duration-sec")
    _validate_positive(args.stale_ms, "--stale-ms")
    _validate_positive(args.frozen_sec, "--frozen-sec")
    _validate_positive(args.telemetry_hz, "--telemetry-hz")
    project_ip = _project_ip(args.project_ip)
    output = args.output or _default_capture_path("hts_live")
    report_path = args.report or output.with_suffix(".report.json")
    output.parent.mkdir(parents=True, exist_ok=True)
    report_path.parent.mkdir(parents=True, exist_ok=True)
    _announce(project_ip, args.port, output)
    print(f"REPORT={report_path.resolve()}", flush=True)
    print(
        "SOURCE_CLOCK=Quest monotonic epoch is independent; absolute one-way latency is unavailable",
        flush=True,
    )

    assembler = HtsCanonicalAssembler(stale_after_s=args.stale_ms / 1000.0)
    telemetry = HtsTelemetry(frozen_after_s=args.frozen_sec)
    started = time.monotonic()
    next_table = started
    rejected_messages = 0
    last_state = assembler.state(now_monotonic_ns=time.monotonic_ns())
    with HtsUdpReceiver(
        args.bind, args.port, allowed_sender=args.allowed_sender
    ) as receiver, HtsRawRecordingWriter(
        output,
        metadata={
            "mode": "validated_live_gate",
            "project_ip": project_ip,
            "port": args.port,
            "coordinate_validation": "verified_live_motion_sequence_2026-07-17",
        },
    ) as writer:
        try:
            while time.monotonic() - started < args.duration_sec:
                datagram = receiver.receive(timeout_s=0.05)
                now_ns = time.monotonic_ns()
                if datagram is not None:
                    writer.write(datagram)
                    try:
                        packets = parse_hts_datagram(datagram.payload)
                        last_state = assembler.ingest(
                            packets,
                            receive_monotonic_ns=datagram.receive_monotonic_ns,
                            source_endpoint=datagram.source_endpoint,
                            datagram_size=len(datagram.payload),
                        )
                        telemetry.observe(datagram, packets, last_state)
                    except SerializationError as exc:
                        telemetry.observe_malformed()
                        if rejected_messages < 10:
                            print(f"REJECTED={exc}", file=sys.stderr, flush=True)
                            rejected_messages += 1
                else:
                    last_state = assembler.state(now_monotonic_ns=now_ns)
                    telemetry.observe_tracking_state(last_state)
                if time.monotonic() >= next_table:
                    _print_pose_table(last_state)
                    next_table = time.monotonic() + 1.0 / args.telemetry_hz
        except KeyboardInterrupt:
            print("STOP=operator interrupt", flush=True)

    now_ns = time.monotonic_ns()
    final_report = telemetry.report(now_monotonic_ns=now_ns, assembler=assembler)
    final_report.update(
        {
            "raw_log": str(output.resolve()),
            "report": str(report_path.resolve()),
            "project_ip": project_ip,
            "port": args.port,
            "bounded_duration_s": args.duration_sec,
            "safety": "Quest input only; no robot command path imported",
        }
    )
    report_path.write_text(
        json.dumps(final_report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(final_report, indent=2, sort_keys=True))
    return 0 if telemetry.datagrams and telemetry.malformed_datagrams == 0 else 2


def _replay(args: argparse.Namespace) -> int:
    _validate_positive(args.speed, "--speed")
    _validate_positive(args.stale_ms, "--stale-ms")
    _validate_positive(args.frozen_sec, "--frozen-sec")
    if args.pose_table_hz < 0:
        raise SystemExit("--pose-table-hz must be non-negative")
    assembler = HtsCanonicalAssembler(stale_after_s=args.stale_ms / 1000.0)
    telemetry = HtsTelemetry(frozen_after_s=args.frozen_sec)
    last_receive_ns = 0
    next_table_ns: int | None = None
    for datagram in replay_datagrams(
        args.recording, as_recorded=args.as_recorded, speed=args.speed
    ):
        last_receive_ns = datagram.receive_monotonic_ns
        try:
            packets = parse_hts_datagram(datagram.payload)
            state = assembler.ingest(
                packets,
                receive_monotonic_ns=datagram.receive_monotonic_ns,
                source_endpoint=datagram.source_endpoint,
                datagram_size=len(datagram.payload),
            )
            telemetry.observe(datagram, packets, state)
            if args.pose_table_hz > 0 and (
                next_table_ns is None or datagram.receive_monotonic_ns >= next_table_ns
            ):
                _print_pose_table(state)
                next_table_ns = datagram.receive_monotonic_ns + int(
                    1_000_000_000 / args.pose_table_hz
                )
        except SerializationError as exc:
            telemetry.observe_malformed()
            print(f"REJECTED={exc}", file=sys.stderr)
    report = telemetry.report(now_monotonic_ns=last_receive_ns, assembler=assembler)
    report["replay_source"] = str(args.recording.resolve())
    report["required_right_hand_gate"] = evaluate_required_right_hand_recording(
        args.recording, stale_after_s=args.stale_ms / 1000.0
    )
    payload = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.report is None:
        sys.stdout.write(payload)
    else:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(payload, encoding="utf-8")
        print(f"REPORT={args.report.resolve()}")
    return 0 if telemetry.datagrams and telemetry.malformed_datagrams == 0 else 2


def _print_pose_table(state: object) -> None:
    # Kept dependency-free and deliberately compact for headset motion checks.
    assert hasattr(state, "left") and hasattr(state, "right")
    parts = []
    for label, hand in (("L", state.left), ("R", state.right)):
        if hand.tracking_valid and hand.wrist_pose is not None:
            x, y, z = hand.wrist_pose.position_m
            parts.append(
                f"{label}=TRACK xyz_m=({x:+.3f},{y:+.3f},{z:+.3f}) "
                f"qnorm={hand.raw_quaternion_norm:.4f} joints={len(hand.joints)} "
                f"age_ms={hand.stream_age_s * 1000.0:.1f}"
            )
        else:
            age = "none" if hand.stream_age_s is None else f"{hand.stream_age_s * 1000.0:.1f}"
            parts.append(f"{label}=NOT_TRACKING age_ms={age}")
    head = "HEAD=TRACK" if state.head is not None else "HEAD=UNAVAILABLE"
    print(" | ".join([*parts, head]), flush=True)


def _validate_positive(value: float, option: str) -> None:
    if value <= 0:
        raise SystemExit(f"{option} must be positive")


def main() -> int:
    args = _parser().parse_args()
    if args.command == "inspect":
        return _inspect(args)
    if args.command == "live":
        return _live(args)
    if args.command == "replay":
        return _replay(args)
    raise AssertionError(args.command)


if __name__ == "__main__":
    raise SystemExit(main())
