#!/usr/bin/env python3
"""Bounded, input-only Quest hand/head plus CTRL UDP transport gate.

This executable imports no MuJoCo, viewer, IK, JAKA SDK, Inspire/RH56 hardware,
or robot-target module.  It has one live UDP source and no keyboard fallback.
"""

from __future__ import annotations

import argparse
from datetime import datetime
import json
import logging
from pathlib import Path
import socket
import sys
import time

from motion_input.errors import SerializationError
from motion_input.hts_transport import HtsRawRecordingWriter, HtsUdpReceiver
from motion_input.transport_gate import QuestTransportGate


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument("--project-ip", default="10.24.1.68")
    parser.add_argument("--allowed-sender")
    parser.add_argument("--duration-sec", type=float, default=180.0)
    parser.add_argument("--stale-ms", type=float, default=250.0)
    parser.add_argument("--print-hz", type=float, default=5.0)
    parser.add_argument("--required-data-timeout-sec", type=float, default=20.0)
    parser.add_argument("--log", type=Path)
    parser.add_argument("--raw-log", type=Path)
    parser.add_argument("--report", type=Path)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    for name, value in (
        ("--duration-sec", args.duration_sec),
        ("--stale-ms", args.stale_ms),
        ("--print-hz", args.print_hz),
        ("--required-data-timeout-sec", args.required_data_timeout_sec),
    ):
        if value <= 0:
            raise SystemExit(f"{name} must be positive")

    stamp = datetime.now().astimezone().strftime("%Y%m%dT%H%M%S%z")
    directory = Path("logs/quest_transport_gate")
    log_path = args.log or directory / f"transport_gate_{stamp}.log"
    raw_path = args.raw_log or directory / f"transport_gate_{stamp}.hts.jsonl"
    report_path = args.report or directory / f"transport_gate_{stamp}.report.json"
    for path in (log_path, raw_path, report_path):
        path.parent.mkdir(parents=True, exist_ok=True)
    logger = _logger(log_path)

    forbidden = _forbidden_imports()
    if forbidden:
        logger.error("forbidden modules already imported: %s", forbidden)
        return 4

    started_ns = time.monotonic_ns()
    gate = QuestTransportGate(
        stale_after_s=args.stale_ms / 1000.0,
        started_monotonic_ns=started_ns,
    )
    logger.info(
        "start transport-only gate host=%s bind=%s port=%d project_ip=%s "
        "source=live_udp_only no_keyboard_fallback=1",
        socket.gethostname(),
        args.bind,
        args.port,
        args.project_ip,
    )
    print(f"HOST={socket.gethostname()}", flush=True)
    print(f"PROJECT_IP={args.project_ip}", flush=True)
    print(f"LISTEN={args.bind}:{args.port}", flush=True)
    print("SOURCE=live_udp_only", flush=True)
    print("SAFETY=input-only; no MuJoCo, viewer, JAKA, Inspire, RH56, IK, or robot targets", flush=True)
    print(f"LOG={log_path.resolve()}", flush=True)
    print(f"RAW_LOG={raw_path.resolve()}", flush=True)
    print(f"REPORT={report_path.resolve()}", flush=True)

    next_summary_ns = started_ns
    deadline_ns = started_ns + int(args.duration_sec * 1e9)
    missing: tuple[str, ...] = ()
    interrupted = False
    with HtsUdpReceiver(
        args.bind,
        args.port,
        allowed_sender=args.allowed_sender,
    ) as receiver, HtsRawRecordingWriter(
        raw_path,
        metadata={
            "mode": "quest_controller_transport_gate",
            "source": "live_udp_only",
            "project_ip": args.project_ip,
            "port": args.port,
            "safety": "input_only_no_robot_targets",
        },
    ) as raw_writer:
        try:
            while time.monotonic_ns() < deadline_ns:
                try:
                    datagram = receiver.receive(timeout_s=0.02)
                except SerializationError as exc:
                    logger.exception("UDP transport rejection: %s", exc)
                    continue
                now_ns = time.monotonic_ns()
                if datagram is not None:
                    raw_writer.write(datagram)
                    result = gate.ingest(datagram)
                    if not result.accepted:
                        logger.error(
                            "packet rejected kind=%s source=%s error=%s",
                            result.kind,
                            datagram.source_endpoint,
                            result.error,
                        )
                else:
                    gate.poll(now_ns)

                if now_ns >= next_summary_ns:
                    summary = gate.summary(now_ns)
                    line = json.dumps(summary, separators=(",", ":"), sort_keys=True)
                    print(line, flush=True)
                    logger.info("summary %s", line)
                    next_summary_ns = now_ns + int(1e9 / args.print_hz)

                missing = gate.missing_required_streams_after(
                    args.required_data_timeout_sec,
                    now_monotonic_ns=now_ns,
                )
                if missing:
                    logger.error(
                        "required data absent after %.3fs: %s",
                        args.required_data_timeout_sec,
                        ",".join(missing),
                    )
                    break
        except KeyboardInterrupt:
            interrupted = True
            logger.info("operator interrupt")

    final = gate.final_report(time.monotonic_ns())
    final.update(
        {
            "hostname": socket.gethostname(),
            "project_ip": args.project_ip,
            "bind": args.bind,
            "port": args.port,
            "source": "live_udp_only",
            "keyboard_fallback": False,
            "required_streams_missing": list(missing),
            "operator_interrupted": interrupted,
            "log": str(log_path.resolve()),
            "raw_log": str(raw_path.resolve()),
            "report": str(report_path.resolve()),
            "forbidden_imports": _forbidden_imports(),
        }
    )
    passed = (
        not missing
        and final["ctrl_packets"] > 0
        and final["hand_packets"] > 0
        and final["malformed_ctrl"] == 0
        and final["malformed_hand_head"] == 0
        and not final["forbidden_imports"]
    )
    final["status"] = "PASS" if passed else "FAIL"
    report_path.write_text(
        json.dumps(final, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    logger.info("final %s", json.dumps(final, separators=(",", ":"), sort_keys=True))
    print(json.dumps(final, indent=2, sort_keys=True), flush=True)
    if missing:
        return 3
    return 0 if passed else 2


def _logger(path: Path) -> logging.Logger:
    logger = logging.getLogger("quest_controller_transport_gate")
    logger.setLevel(logging.INFO)
    logger.propagate = False
    logger.handlers.clear()
    handler = logging.FileHandler(path, mode="x", encoding="utf-8")
    handler.setFormatter(
        logging.Formatter("%(asctime)s %(levelname)s %(message)s")
    )
    logger.addHandler(handler)
    return logger


def _forbidden_imports() -> list[str]:
    prefixes = (
        "mujoco",
        "jaka_driver_adapter",
        "rh56_driver",
        "robot_bringup",
        "quest_jaka_sim",
    )
    return sorted(
        name
        for name in sys.modules
        if any(name == prefix or name.startswith(prefix + ".") for prefix in prefixes)
    )


if __name__ == "__main__":
    raise SystemExit(main())
