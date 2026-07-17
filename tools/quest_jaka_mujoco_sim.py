#!/usr/bin/env python3
"""Quest HTS to JAKA MuJoCo simulation; no physical hardware backend exists."""

from __future__ import annotations

import argparse
from dataclasses import replace
from datetime import datetime
import importlib
import ipaddress
import json
from pathlib import Path
import socket
import time

import mujoco

from motion_input import HtsRawRecordingReader, HtsRawRecordingWriter, HtsUdpReceiver
from quest_jaka_sim import JakaMujocoSimulation, QuestJakaReplaySession, ReplayConfig
from quest_jaka_sim.simulation import build_viewer_mjcf


DEFAULT_CONFIG = Path("configs/sim/quest_hts_jaka_mini2_offline.yaml")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    replay = commands.add_parser("replay", help="deterministic recorded-input simulation")
    replay.add_argument("recording", type=Path)
    replay.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    replay.add_argument("--report", type=Path)
    replay.add_argument("--events", type=Path)
    replay.add_argument("--viewer", action="store_true")
    replay.add_argument("--realtime", action="store_true")
    replay.add_argument("--speed", type=float, default=1.0)
    replay.add_argument(
        "--realtime-from-sec",
        type=float,
        default=0.0,
        help="fast-forward before this recorded time, then display in real time",
    )

    live = commands.add_parser("live", help="Quest-only live input to MuJoCo viewer")
    live.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    live.add_argument("--bind", default="0.0.0.0")
    live.add_argument("--port", type=int, default=9000)
    live.add_argument("--project-ip")
    live.add_argument("--allowed-sender")
    live.add_argument("--duration-sec", type=float, default=120.0)
    live.add_argument("--report", type=Path)
    live.add_argument("--output", type=Path)
    live.add_argument("--events", type=Path)
    return parser


def _paths(prefix: str) -> tuple[Path, Path]:
    stamp = datetime.now().astimezone().strftime("%Y%m%dT%H%M%S%z")
    root = Path("logs/quest_jaka_sim")
    return root / f"{prefix}_{stamp}.json", root / f"{prefix}_{stamp}.hts.jsonl"


def _make_session(config: ReplayConfig) -> tuple[JakaMujocoSimulation, QuestJakaReplaySession]:
    augmented = build_viewer_mjcf(
        config.mjcf_path, Path("logs/quest_jaka_sim/quest_jaka_viewer_model.xml")
    )
    simulation = JakaMujocoSimulation(config, mjcf_path=augmented)
    return simulation, QuestJakaReplaySession(config, simulation)


def _viewer(simulation: JakaMujocoSimulation, session: QuestJakaReplaySession):
    module = importlib.import_module("mujoco.viewer")

    def key_callback(keycode: int) -> None:
        if keycode == 32:
            session.request_toggle()

    handle = module.launch_passive(
        simulation.model,
        simulation.data,
        key_callback=key_callback,
        show_left_ui=False,
        show_right_ui=False,
    )
    handle.opt.geomgroup[5] = 1
    handle.cam.azimuth = -130
    handle.cam.elevation = -25
    handle.cam.distance = 1.25
    handle.cam.lookat[:] = [-0.05, -0.30, 0.24]
    return handle


def _sync_viewer(handle: object, simulation: JakaMujocoSimulation, session: QuestJakaReplaySession) -> None:
    actual = simulation.current_tcp_pose
    desired = simulation.last_safe_target
    error = float(
        sum((a - b) ** 2 for a, b in zip(actual.position_m, desired.position_m)) ** 0.5
    )
    handle.set_texts(
        (
            None,
            mujoco.mjtGridPos.mjGRID_TOPLEFT,
            "Quest -> JAKA SIM ONLY",
            (
                f"state={session.operator.state.value} right_valid={session.right_hand_valid}\n"
                f"target={session.last_reason} accepted={session.accepted_targets}\n"
                f"desired={tuple(round(v, 4) for v in desired.position_m)}\n"
                f"sim_tcp={tuple(round(v, 4) for v in actual.position_m)} err={error:.4f} m\n"
                "BLUE=desired TCP  GREEN=simulated TCP  SPACE=engage/capture/disengage"
            ),
        )
    )
    handle.sync()


def _write_report(report: dict[str, object], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"REPORT={path.resolve()}")
    print(
        "SUMMARY "
        f"frames={report['frame_count']} valid={report['valid_input_frames']} "
        f"accepted={report['accepted_target_count']} "
        f"rejections={report['rejection_counts_by_reason']} "
        f"final_state={report['final_state']} "
        f"max_tcp_error_m={report['maximum_desired_to_simulated_tcp_error_m']:.6f}"
    )
    print("SAFETY=simulation only; no JAKA or Inspire connection/import/command path")


def _write_events(records: list[dict[str, object]], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "".join(json.dumps(record, sort_keys=True) + "\n" for record in records),
        encoding="utf-8",
    )
    print(f"EVENTS={path.resolve()}")


def _replay(args: argparse.Namespace) -> int:
    if args.speed <= 0 or args.realtime_from_sec < 0:
        raise SystemExit("speed must be positive and realtime-from must be non-negative")
    config = ReplayConfig.load(args.config)
    simulation, session = _make_session(config)
    datagrams = list(HtsRawRecordingReader(args.recording).datagrams())
    if not datagrams:
        raise SystemExit("recording contains no datagrams")
    base_ns = datagrams[0].receive_monotonic_ns
    previous_ns = base_ns
    handle = _viewer(simulation, session) if args.viewer else None
    realtime = bool(args.realtime or args.viewer)
    wall_start: float | None = None
    last_viewer_sync = 0.0
    try:
        for datagram in datagrams:
            elapsed = (datagram.receive_monotonic_ns - base_ns) / 1e9
            dt = max(0.0, (datagram.receive_monotonic_ns - previous_ns) / 1e9)
            previous_ns = datagram.receive_monotonic_ns
            if realtime and elapsed >= args.realtime_from_sec:
                if wall_start is None:
                    wall_start = time.monotonic() - (
                        elapsed - args.realtime_from_sec
                    ) / args.speed
                deadline = wall_start + (elapsed - args.realtime_from_sec) / args.speed
                while time.monotonic() < deadline:
                    chunk = min(0.01, max(0.0, deadline - time.monotonic()))
                    simulation.step(chunk * args.speed)
                    if handle is not None:
                        if not handle.is_running():
                            break
                        if time.monotonic() - last_viewer_sync >= 1.0 / 60.0:
                            _sync_viewer(handle, simulation, session)
                            last_viewer_sync = time.monotonic()
                    time.sleep(min(chunk, 0.01))
            else:
                simulation.step(dt)
            session.process(datagram)
            if (
                handle is not None
                and elapsed >= args.realtime_from_sec
                and time.monotonic() - last_viewer_sync >= 1.0 / 60.0
            ):
                _sync_viewer(handle, simulation, session)
                last_viewer_sync = time.monotonic()
                if not handle.is_running():
                    break
        simulation.step(0.25)
    finally:
        if handle is not None:
            handle.close()
    report = session.report(replay_source=str(args.recording))
    report["mode"] = "recorded_replay"
    report["realtime"] = realtime
    report_path = args.report or _paths("quest_jaka_replay")[0]
    events_path = args.events or report_path.with_suffix(".events.jsonl")
    report["event_log"] = str(events_path.resolve())
    _write_events(session.event_records, events_path)
    _write_report(report, report_path)
    return 0 if report["accepted_target_count"] > 0 else 2


def _project_ip(explicit: str | None) -> str:
    if explicit:
        candidate = explicit
    else:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe.connect(("1.1.1.1", 9))
            candidate = str(probe.getsockname()[0])
        finally:
            probe.close()
    address = ipaddress.ip_address(candidate)
    if not isinstance(address, ipaddress.IPv4Address) or address.is_loopback:
        raise SystemExit("PROJECT_IP must be a non-loopback IPv4 address")
    return str(address)


def _live(args: argparse.Namespace) -> int:
    if args.duration_sec <= 0:
        raise SystemExit("duration must be positive")
    config = replace(ReplayConfig.load(args.config), engagement_schedule_s=())
    simulation, session = _make_session(config)
    project_ip = _project_ip(args.project_ip)
    default_report, default_capture = _paths("quest_jaka_live_sim")
    report_path = args.report or default_report
    capture_path = args.output or default_capture
    capture_path.parent.mkdir(parents=True, exist_ok=True)
    print(f"PROJECT_IP={project_ip}")
    print(f"PORT={args.port}")
    print("TRANSPORT=UDP")
    print("CONTROL=SPACE once to arm, SPACE again on a fresh sample to capture reference")
    print("SAFETY=Quest to MuJoCo only; JAKA and Inspire hardware paths are absent")
    handle = _viewer(simulation, session)
    started = time.monotonic()
    previous = started
    with HtsUdpReceiver(
        args.bind, args.port, allowed_sender=args.allowed_sender
    ) as receiver, HtsRawRecordingWriter(
        capture_path, metadata={"mode": "quest_jaka_live_sim_only"}
    ) as writer:
        try:
            while handle.is_running() and time.monotonic() - started < args.duration_sec:
                now = time.monotonic()
                simulation.step(now - previous)
                previous = now
                datagram = receiver.receive(timeout_s=0.001)
                if datagram is not None:
                    writer.write(datagram)
                    session.process(datagram)
                else:
                    session.tick(time.monotonic_ns())
                _sync_viewer(handle, simulation, session)
        except KeyboardInterrupt:
            pass
        finally:
            handle.close()
    report = session.report(replay_source=str(capture_path))
    report["mode"] = "live_quest_to_simulation_only"
    events_path = args.events or report_path.with_suffix(".events.jsonl")
    report["event_log"] = str(events_path.resolve())
    _write_events(session.event_records, events_path)
    _write_report(report, report_path)
    return 0


def main() -> int:
    args = _parser().parse_args()
    if args.command == "replay":
        return _replay(args)
    if args.command == "live":
        return _live(args)
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
