#!/usr/bin/env python3
"""Quest HTS to JAKA MuJoCo simulation; no physical hardware backend exists."""

from __future__ import annotations

import argparse
import copy
import ctypes
import ctypes.util
from dataclasses import replace
from datetime import datetime
import hashlib
import importlib
import ipaddress
import json
import math
import os
from pathlib import Path
import socket
import subprocess
import time

import mujoco
import numpy as np

from episode_dataset.camera import AsyncRGBDCamera
from episode_dataset.collector import CaptureState, SingleEpisodeCollector
from episode_dataset.episode import ControlSample
from episode_dataset.preview import (
    AsyncDualCameraPreview,
    PreviewStatus,
)
from episode_dataset.runtime import EpisodeDataRuntime
from motion_input import HtsRawRecordingReader, HtsRawRecordingWriter
from quest_jaka_sim import (
    AnalogClutchSample,
    ArmOutputMode,
    JakaEquivalent125HzMujocoAdapter,
    JakaMujocoSimulation,
    MujocoArmTargetAdapter,
    QuestJakaReplaySession,
    RecordingArmTargetAdapter,
    ReplayConfig,
    SharedJakaTargetGenerator,
    SmoothQuestJakaSession,
)
from quest_jaka_sim.simulation import build_viewer_mjcf
from quest_jaka_sim.se3 import quaternion_angle_rad
from quest_jaka_sim.live_controller import LiveQuestControllerRouter
from quest_jaka_sim.live_input import QuestDatagramReceiverWorker


DEFAULT_CONFIG = Path("configs/sim/quest_hts_jaka_mini2_offline.yaml")


class _XClientMessageData(ctypes.Union):
    _fields_ = [
        ("b", ctypes.c_char * 20),
        ("s", ctypes.c_short * 10),
        ("l", ctypes.c_long * 5),
    ]


class _XClientMessageEvent(ctypes.Structure):
    _fields_ = [
        ("type", ctypes.c_int),
        ("serial", ctypes.c_ulong),
        ("send_event", ctypes.c_int),
        ("display", ctypes.c_void_p),
        ("window", ctypes.c_ulong),
        ("message_type", ctypes.c_ulong),
        ("format", ctypes.c_int),
        ("data", _XClientMessageData),
    ]


class _XEvent(ctypes.Union):
    _fields_ = [
        ("xclient", _XClientMessageEvent),
        ("pad", ctypes.c_long * 24),
    ]


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

    smooth_live = commands.add_parser(
        "live-6dof", help="fixed-rate filtered Quest 6-DoF input to MuJoCo"
    )
    smooth_live.add_argument(
        "--config",
        type=Path,
        default=Path("configs/sim/quest_hts_jaka_mini2_live_demo.yaml"),
    )
    smooth_live.add_argument("--bind", default="0.0.0.0")
    smooth_live.add_argument("--port", type=int, default=9000)
    smooth_live.add_argument("--project-ip")
    smooth_live.add_argument("--allowed-sender")
    smooth_live.add_argument("--duration-sec", type=float, default=180.0)
    smooth_live.add_argument("--report", type=Path)
    smooth_live.add_argument("--output", type=Path)
    smooth_live.add_argument("--events", type=Path)
    smooth_live.add_argument("--arm-emitted-events", type=Path)
    smooth_live.add_argument(
        "--arm-output-mode",
        choices=tuple(mode.value for mode in ArmOutputMode),
        default=ArmOutputMode.SHAPED_500HZ.value,
    )
    smooth_live.add_argument(
        "--viewer",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="open the MuJoCo passive viewer (default: enabled)",
    )
    smooth_live.add_argument(
        "--telemetry-hz",
        type=float,
        default=2.0,
        help="terminal tracking/clutch status rate; 0 disables (default: 2)",
    )
    smooth_live.add_argument(
        "--ik-debug",
        action="store_true",
        help="show optional joint/IK/singularity/continuation diagnostics",
    )
    smooth_live.add_argument(
        "--episode-data-config",
        type=Path,
        help=(
            "simulation-only single-episode dual-D435 recording config; starts in IDLE, "
            "records while arm index is held, finalizes on release, then exits"
        ),
    )
    smooth_live.add_argument("--episode-root", type=Path)
    smooth_live.add_argument("--task-name", default="unlabeled_task")
    smooth_live.add_argument("--operator", default="unknown")
    smooth_live.add_argument(
        "--episode-preview",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    smooth_replay = commands.add_parser(
        "replay-6dof", help="deterministic fixed-rate filtered 6-DoF replay"
    )
    smooth_replay.add_argument("recording", type=Path)
    smooth_replay.add_argument(
        "--config",
        type=Path,
        default=Path("configs/sim/quest_hts_jaka_mini2_live_demo.yaml"),
    )
    smooth_replay.add_argument("--report", type=Path)
    smooth_replay.add_argument("--events", type=Path)
    smooth_replay.add_argument("--arm-emitted-events", type=Path)
    smooth_replay.add_argument(
        "--arm-output-mode",
        choices=tuple(mode.value for mode in ArmOutputMode),
        default=ArmOutputMode.SHAPED_500HZ.value,
    )
    smooth_replay.add_argument("--viewer", action="store_true")
    smooth_replay.add_argument("--realtime", action="store_true")
    smooth_replay.add_argument("--speed", type=float, default=1.0)
    smooth_replay.add_argument("--engage-at-sec", type=float, default=1.0)
    smooth_replay.add_argument(
        "--arm-cycle-period-sec",
        type=float,
        help="explicit deterministic offline index press/release cycle period",
    )
    smooth_replay.add_argument("--arm-cycle-count", type=int, default=10)
    smooth_replay.add_argument(
        "--hand-engage-at-sec",
        type=float,
        help="explicit deterministic offline grip press for hand validation",
    )
    smooth_replay.add_argument(
        "--hand-cycle-period-sec",
        type=float,
        help="explicit deterministic offline grip press/release cycle period",
    )
    smooth_replay.add_argument("--hand-cycle-count", type=int, default=4)
    smooth_replay.add_argument(
        "--hand-reacquisition-ms",
        type=float,
        choices=(150.0, 200.0, 250.0, 300.0),
        help="offline-only hand reference-capture state duration",
    )
    smooth_replay.add_argument("--duration-sec", type=float)
    return parser


def _paths(prefix: str) -> tuple[Path, Path]:
    stamp = datetime.now().astimezone().strftime("%Y%m%dT%H%M%S%z")
    root = Path("logs/quest_jaka_sim")
    return root / f"{prefix}_{stamp}.json", root / f"{prefix}_{stamp}.hts.jsonl"


def _make_session(config: ReplayConfig) -> tuple[JakaMujocoSimulation, QuestJakaReplaySession]:
    augmented = build_viewer_mjcf(
        config.mjcf_path,
        Path("logs/quest_jaka_sim/quest_jaka_viewer_model.xml"),
    )
    simulation = JakaMujocoSimulation(config, mjcf_path=augmented)
    return simulation, QuestJakaReplaySession(config, simulation)


def _make_smooth_session(
    config: ReplayConfig,
    *,
    arm_output_mode: str = ArmOutputMode.SHAPED_500HZ.value,
) -> tuple[JakaMujocoSimulation, SmoothQuestJakaSession]:
    output = Path("logs/quest_jaka_sim/quest_jaka_viewer_model.xml")
    hand_enabled = bool(config.raw.get("hand_retargeting", {}).get("enabled", False))
    augmented = build_viewer_mjcf(
        config.mjcf_path,
        output,
        arm_only=not hand_enabled,
    )
    simulation = JakaMujocoSimulation(config, mjcf_path=augmented)
    target_generator = SharedJakaTargetGenerator(config, mjcf_path=config.mjcf_path)
    arm_output = (
        MujocoArmTargetAdapter(simulation)
        if arm_output_mode == ArmOutputMode.SHAPED_500HZ.value
        else JakaEquivalent125HzMujocoAdapter(simulation)
    )
    return simulation, SmoothQuestJakaSession(
        config,
        target_generator,
        arm_output=arm_output,
        mujoco_plant=simulation,
    )


def _step_smooth_simulation(
    simulation: JakaMujocoSimulation,
    session: SmoothQuestJakaSession,
    dt_s: float,
) -> None:
    steps = max(0, int(round(max(0.0, dt_s) / simulation.model.opt.timestep)))
    advance = getattr(session.arm_output, "advance_to", None)
    if advance is None:
        simulation.step(dt_s)
        return
    for _ in range(steps):
        advance(float(simulation.data.time))
        simulation.step(float(simulation.model.opt.timestep))


def _own_mujoco_x11_window() -> int | None:
    tree = subprocess.run(
        ["xwininfo", "-root", "-tree"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    for line in tree.splitlines():
        if '("MuJoCo" "MuJoCo")' not in line:
            continue
        window_text = line.strip().split(maxsplit=1)[0]
        properties = subprocess.run(
            ["xprop", "-id", window_text, "_NET_WM_PID"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout
        if f"= {os.getpid()}" in properties:
            return int(window_text, 16)
    return None


def _request_x11_fullscreen() -> tuple[bool, str | None]:
    if not os.environ.get("DISPLAY"):
        return False, "DISPLAY is not set"
    try:
        window = _own_mujoco_x11_window()
        if window is None:
            return False, "MuJoCo X11 window for this process was not found"
        library_name = ctypes.util.find_library("X11")
        if library_name is None:
            return False, "libX11 was not found"
        x11 = ctypes.CDLL(library_name)
        x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
        x11.XOpenDisplay.restype = ctypes.c_void_p
        x11.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
        x11.XDefaultRootWindow.restype = ctypes.c_ulong
        x11.XInternAtom.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
        x11.XInternAtom.restype = ctypes.c_ulong
        x11.XSendEvent.argtypes = [
            ctypes.c_void_p,
            ctypes.c_ulong,
            ctypes.c_int,
            ctypes.c_long,
            ctypes.POINTER(_XEvent),
        ]
        x11.XSendEvent.restype = ctypes.c_int
        x11.XFlush.argtypes = [ctypes.c_void_p]
        x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
        display = x11.XOpenDisplay(None)
        if not display:
            return False, f"cannot open X11 display {os.environ['DISPLAY']}"
        try:
            event = _XEvent()
            event.xclient.type = 33  # ClientMessage
            event.xclient.send_event = 1
            event.xclient.display = display
            event.xclient.window = window
            event.xclient.message_type = x11.XInternAtom(
                display, b"_NET_WM_STATE", 0
            )
            event.xclient.format = 32
            event.xclient.data.l[0] = 1  # _NET_WM_STATE_ADD
            event.xclient.data.l[1] = x11.XInternAtom(
                display, b"_NET_WM_STATE_FULLSCREEN", 0
            )
            event.xclient.data.l[3] = 1  # normal application request
            sent = x11.XSendEvent(
                display,
                x11.XDefaultRootWindow(display),
                0,
                0x180000,  # SubstructureNotifyMask | SubstructureRedirectMask
                ctypes.byref(event),
            )
            x11.XFlush(display)
        finally:
            x11.XCloseDisplay(display)
        if not sent:
            return False, "X11 window manager rejected the fullscreen request"
        time.sleep(0.1)
        state = subprocess.run(
            ["xprop", "-id", hex(window), "_NET_WM_STATE"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout
        if "_NET_WM_STATE_FULLSCREEN" not in state:
            return False, "window manager did not enter fullscreen"
        return True, None
    except (FileNotFoundError, OSError, subprocess.CalledProcessError) as exc:
        return False, str(exc).replace("\n", " ")


def _viewer(simulation: JakaMujocoSimulation):
    module = importlib.import_module("mujoco.viewer")

    handle = module.launch_passive(
        simulation.model,
        simulation.data,
        show_left_ui=False,
        show_right_ui=False,
    )
    handle.opt.geomgroup[5] = 1
    handle.cam.azimuth = -130
    handle.cam.elevation = -25
    handle.cam.distance = 1.25
    handle.cam.lookat[:] = [-0.05, -0.30, 0.24]
    print(f"VIEWER_CAMERA_LOOKAT={handle.cam.lookat.tolist()}")
    print(f"VIEWER_CAMERA_DISTANCE={handle.cam.distance:g}")
    print(f"VIEWER_CAMERA_AZIMUTH={handle.cam.azimuth:g}")
    print(f"VIEWER_CAMERA_ELEVATION={handle.cam.elevation:g}")
    fullscreen, reason = _request_x11_fullscreen()
    print(f"VIEWER_FULLSCREEN={str(fullscreen).lower()}")
    if reason is not None:
        print(f"VIEWER_FULLSCREEN_REASON={reason}")
    return handle

def _sync_viewer(
    handle: object,
    simulation: JakaMujocoSimulation,
    session: object,
) -> None:
    actual = simulation.current_tcp_pose
    desired = simulation.last_safe_target
    error = float(
        sum((a - b) ** 2 for a, b in zip(actual.position_m, desired.position_m)) ** 0.5
    )
    orientation_error_deg = math.degrees(
        quaternion_angle_rad(actual.orientation_xyzw, desired.orientation_xyzw)
    )
    hand_result = getattr(session, "last_hand_result", None)
    hand_status = (
        "disabled"
        if hand_result is None
        else f"valid={hand_result.valid} cost={hand_result.optimizer_cost}"
    )
    arm = getattr(session, "arm_clutch", None)
    hand = getattr(session, "hand_clutch", None)
    latest = session.event_records[-1] if getattr(session, "event_records", None) else {}
    metrics = latest.get("metrics") or {}
    debug_text = ""
    if getattr(session, "ik_debug", False) and metrics:
        q = latest.get("accepted_joint_target_rad") or metrics.get("ik_candidate_rad") or ()
        dq = metrics.get("joint_delta_rad") or ()
        q_deg = tuple(round(math.degrees(v), 2) for v in q)
        dq_deg = tuple(round(math.degrees(v), 3) for v in dq)
        roll_deg = math.degrees(metrics.get("target_tool_axial_roll_rad", 0.0))
        swing_deg = math.degrees(metrics.get("target_tool_swing_rad", 0.0))
        ratio = metrics.get("j6_axial_contribution_ratio")
        margin_deg = math.degrees(metrics.get("nearest_safe_joint_limit_margin_rad", 0.0))
        debug_text = (
            f"\nq_deg={q_deg}\ndq_deg={dq_deg}\n"
            f"tool swing/roll={swing_deg:.3f}/{roll_deg:.3f} deg "
            f"J6_ratio(diag)={ratio}\n"
            f"cond={metrics.get('jacobian_condition')} "
            f"sigma_min={metrics.get('minimum_jacobian_singular_value')} "
            f"safe_limit_margin={margin_deg:.2f} deg "
            f"branch_switch={metrics.get('branch_switch')}\n"
            f"continuation={latest.get('continuation_fraction', 1.0):.4f} "
            f"backtracks={latest.get('continuation_backtracks', 0)} "
            f"backlog={latest.get('requested_backlog_m', 0.0) * 1000.0:.2f} mm/"
            f"{latest.get('requested_backlog_deg', 0.0):.2f} deg "
            f"singularity_warning={latest.get('singularity_warning', False)}"
        )
    instruction = (
        "LEFT INDEX=arm hold-to-run; LEFT GRIP=hand hold-to-run"
        if getattr(session, "clutch_provider", "unavailable") == "quest_ctrl_udp_v1"
        else "Clutches require an explicit timestamped provider"
    )
    texts = [
        (
            None,
            mujoco.mjtGridPos.mjGRID_TOPLEFT,
            "Quest -> JAKA SIM ONLY",
            (
                f"arm={getattr(getattr(arm, 'state', None), 'value', 'legacy')} "
                f"hand={getattr(getattr(hand, 'state', None), 'value', 'legacy')}\n"
                f"index={latest.get('index_trigger_value', 0.0):.2f} age={latest.get('index_trigger_age_s', 0.0):.3f}s "
                f"grip={latest.get('grip_trigger_value', 0.0):.2f} age={latest.get('grip_trigger_age_s', 0.0):.3f}s\n"
                f"wrist_valid={latest.get('right_wrist_valid', session.right_hand_valid)} "
                f"age={latest.get('right_wrist_age_s')} skeleton_valid={latest.get('hand_skeleton_valid')}\n"
                f"target={session.last_reason} accepted={session.accepted_targets}\n"
                f"arm_ref={latest.get('arm_reference_pose')}\n"
                f"desired={tuple(round(v, 4) for v in desired.position_m)}\n"
                f"delta={latest.get('operator_delta')} ik={latest.get('ik_status')}\n"
                f"sim_tcp={tuple(round(v, 4) for v in actual.position_m)} pos_err={error*1000:.1f} mm\n"
                f"orientation_err={orientation_error_deg:.1f} deg orientation=ENABLED\n"
                f"filter={getattr(getattr(session, 'profile', None), 'name', 'legacy')}\n"
                f"arm_output={getattr(simulation, 'arm_output_mode', 'shaped-500hz')}\n"
                f"head_yaw={latest.get('captured_head_yaw_rad')} hand_retarget={hand_status}\n"
                f"retarget_status={latest.get('hand_retarget_status')}\n"
                f"hand_reacquire={latest.get('hand_reacquisition_fraction')} "
                f"arm_fault={latest.get('active_arm_fault')} hand_fault={latest.get('active_hand_fault')}\n"
                f"cycles arm={getattr(arm, 'cycle_count', 0)} hand={getattr(hand, 'cycle_count', 0)}\n"
                f"{instruction}\n"
                f"BLUE=desired TCP frame GREEN=simulated TCP frame{debug_text}"
            ),
        )
    ]
    handle.set_texts(texts)
    handle.sync()


def _write_report(report: dict[str, object], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"REPORT={path.resolve()}")
    frames = report.get("frame_count", report.get("input_frame_count", 0))
    rejections = report.get("rejection_counts_by_reason", report.get("rejections", {}))
    print(
        "SUMMARY "
        f"frames={frames} accepted={report['accepted_target_count']} "
        f"rejections={rejections} final_state={report['final_state']} "
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


def _run_manifest(
    *,
    config_path: Path,
    duration_s: float,
    raw_input_path: Path,
    report_path: Path,
    events_path: Path,
    target_hz: float,
    simulation: JakaMujocoSimulation,
    arm_output_mode: str,
    emitted_path: Path | None,
) -> dict[str, object]:
    commit = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    resolved_config = config_path.resolve()
    return {
        "schema_version": "quest_jaka_joint_recording.v1",
        "git_commit": commit,
        "config_path": str(resolved_config),
        "config_sha256": hashlib.sha256(resolved_config.read_bytes()).hexdigest(),
        "recording_duration_s": float(duration_s),
        "arm_joint_order": [f"jaka_joint_{index}" for index in range(1, 7)],
        "hand_channel_order": [
            "thumb_lateral",
            "thumb_close",
            "index",
            "middle",
            "ring",
            "pinky",
        ],
        "rates_hz": {
            "target_generation": target_hz,
            "selected_arm_output": (
                125.0
                if arm_output_mode == ArmOutputMode.JAKA_EQUIVALENT_125HZ.value
                else 500.0
            ),
            "mujoco_physics": 1.0 / float(simulation.model.opt.timestep),
            "simulation_event_log": target_hz,
        },
        "file_paths": {
            "raw_input": str(raw_input_path.resolve()),
            "control_events": str(events_path.resolve()),
            "report": str(report_path.resolve()),
            "arm_emitted_125hz": (
                None if emitted_path is None else str(emitted_path.resolve())
            ),
        },
        "simulation_only": True,
    }


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
    handle = _viewer(simulation) if args.viewer else None
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


def _start_episode_data_runtime(
    args: argparse.Namespace,
    config: ReplayConfig,
) -> tuple[
    SingleEpisodeCollector,
    AsyncRGBDCamera,
    AsyncRGBDCamera,
    AsyncDualCameraPreview | None,
]:
    try:
        hardware_config = config.raw.get("hardware_adapter", {})
        runtime = EpisodeDataRuntime.start(
            args.episode_data_config,
            episode_root=args.episode_root,
            task_name=args.task_name,
            operator=args.operator,
            control_config_path=args.config,
            maximum_start_delta_rad=float(
                hardware_config.get("startup_alignment_tolerance_rad", 0.001)
            ),
            preview_enabled=args.episode_preview,
            metadata={
                "raw_streams": {
                    "quest_raw_datagram": "unavailable",
                    "quest_decoded_input": "unavailable",
                    "accepted_arm_target_60hz": "commanded",
                    "emitted_arm_command_125hz": (
                        "commanded"
                        if args.arm_output_mode
                        == ArmOutputMode.JAKA_EQUIVALENT_125HZ.value
                        else "unavailable"
                    ),
                    "jaka_arm_q": "unavailable_simulated_arm_q_available",
                    "jaka_arm_dq": "unavailable_simulated_arm_dq_available",
                    "native_telemetry": "unavailable",
                    "rh56_target": "commanded_simulation",
                    "rh56_feedback": "measured_simulation",
                    "workspace_rgbd": "measured",
                    "wrist_rgbd": "measured",
                    "fault_events": "measured_simulation",
                },
                "simulation_only": True,
                "physically_validated": False,
            },
        )
        return (
            runtime.collector,
            runtime.cameras["workspace"],
            runtime.cameras["wrist"],
            runtime.preview,
        )
    except (OSError, RuntimeError, ValueError) as exc:
        raise SystemExit(str(exc)) from exc


def _simulation_control_sample(
    session: SmoothQuestJakaSession,
    simulation: JakaMujocoSimulation,
    timestamp_ns: int,
) -> tuple[ControlSample, bool, dict[str, object]]:
    event = dict(session.event_records[-1])
    tcp = simulation.current_tcp_pose
    hand_observation = event.get("actual_hand_actuator_position_rad")
    hand_target = event.get("commanded_hand_target_rad")
    accepted_target = event.get("accepted_joint_target_rad")
    action_status = "accepted"
    if accepted_target is None and event.get("control_state") == "HOLD_REJECTED":
        held = session.last_accepted_target
        if held is not None:
            accepted_target = held.joint_position_rad
            action_status = "held_rejected"
    sample = ControlSample(
        host_monotonic_ns=timestamp_ns,
        accepted_arm_q=accepted_target,
        arm_q_measured=tuple(float(value) for value in simulation.arm_joints_rad),
        arm_dq_measured=tuple(float(value) for value in simulation.data.qvel[simulation.arm_dof_ids]),
        tcp_pose_xyzw=(*tcp.position_m, *tcp.orientation_xyzw),
        arm_action_status=action_status,
        hand_observation=hand_observation,
        hand_source="measured" if hand_observation is not None else "unavailable",
        hand_target=hand_target,
        arm_trigger=session.arm_clutch.state.value == "engaged",
        hand_grip=session.hand_clutch.state.value in {"reacquire", "engaged"},
        accepted_target_sequence=event.get("accepted_target_sequence"),
        reference_generation=event.get("reference_generation"),
        source_timestamps_ns={
            "quest": event.get("source_timestamp_ns"),
            "quest_host_receive": event.get("raw_quest_wrist_timestamp_ns"),
            "accepted_action_host": (
                event.get("control_monotonic_ns")
                if action_status == "accepted"
                else session.last_accepted_target.generated_monotonic_ns
            )
            if accepted_target is not None
            else None,
            "accepted_action_source": (
                event.get("accepted_source_timestamp_ns")
                if action_status == "accepted"
                else session.last_accepted_target.source_timestamp_ns
            )
            if accepted_target is not None
            else None,
        },
        source_timestamp_domains={
            "quest": "quest_source_clock_ns",
            "quest_host_receive": "host_monotonic_ns",
            "accepted_action_host": "host_monotonic_ns",
            "accepted_action_source": "quest_source_clock_ns",
        },
        control_heartbeat_valid=event.get("control_state") != "HARD_STOP",
        tracking_hard_fault=bool(event.get("active_arm_fault")),
        controller_fault=False,
    )
    reference_established = event.get("arm_reference_pose") is not None
    return sample, reference_established, event


def _live_6dof(args: argparse.Namespace) -> int:
    if args.duration_sec <= 0:
        raise SystemExit("duration must be positive")
    if args.telemetry_hz < 0:
        raise SystemExit("telemetry-hz must be non-negative")
    if args.episode_root is not None and args.episode_data_config is None:
        raise SystemExit("--episode-root requires --episode-data-config")
    config = replace(
        ReplayConfig.load(args.config),
        engagement_schedule_s=(),
    )
    simulation, session = _make_smooth_session(
        config,
        arm_output_mode=args.arm_output_mode,
    )
    clutch_config = config.raw.get("clutches", {})
    controller_router = LiveQuestControllerRouter(
        stale_after_s=float(clutch_config.get("stale_after_ms", 150.0)) / 1000.0,
        released_at=float(clutch_config.get("released_at", 0.55)),
    )
    rates = config.raw.get("rates", {})
    target_hz = float(rates.get("target_generation_hz", 60.0))
    viewer_hz = float(rates.get("viewer_hz", 60.0))
    if target_hz <= 0 or viewer_hz <= 0:
        raise SystemExit("fixed rates must be positive")
    project_ip = _project_ip(args.project_ip)
    default_report, default_capture = _paths("quest_jaka_live_6dof")
    report_path = args.report or default_report
    capture_path = args.output or default_capture
    events_path = args.events or report_path.with_suffix(".events.jsonl")
    emitted_path = args.arm_emitted_events or report_path.with_suffix(
        ".arm_emitted_125hz.jsonl"
    )
    capture_path.parent.mkdir(parents=True, exist_ok=True)
    print(f"EVENT_LOG={events_path.resolve()}")
    print(f"PROJECT_IP={project_ip}")
    print(f"PORT={args.port}")
    print("TRANSPORT=UDP")
    print(
        f"RATES=input~30Hz target={target_hz:g}Hz "
        f"mujoco={1.0/simulation.model.opt.timestep:g}Hz viewer={viewer_hz:g}Hz"
    )
    print(f"ARM_OUTPUT={args.arm_output_mode}")
    _print_effective_speed_config(config, simulation)
    print("JAKA hardware control disabled")
    print("RH56 hardware control disabled")
    print("CONTROL=LEFT INDEX arm clutch; LEFT GRIP RH56 hand clutch")
    print("MODE=filtered relative 6-DoF; BLUE desired frame, GREEN simulated frame")
    print("SAFETY=Quest to MuJoCo only; JAKA and Inspire hardware paths are absent")
    if args.ik_debug:
        print(
            "IK_DEBUG=enabled; viewer/STATUS show q, dq, tool swing/roll, "
            "IK/singularity/continuation metrics (J6 contribution is diagnostic only)"
        )
    print("操作提示：")
    print("1. 打开带 CTRL sidecar 的 Quest Hand Tracking Streamer。")
    print(f"2. 确认 Quest 与主机同网，并将目标设为 {project_ip}:{args.port}/UDP unicast。")
    print("3. 开启右手追踪、Head Pose、Debug Info，确认扩展 CTRL sender 已启用，再点击 Start Streaming。")
    print("4. 右手进入视野；等待终端显示 right_valid=True、controller_valid=True。")
    print("5. 左手 index 先完全释放，再按住以捕获 arm reference；grip 独立捕获 hand reference。")
    print("6. 按住 index 移动/旋转右手；按住 grip 控制四指、thumb_close 和 thumb_lateral。")
    print("7. 本入口仅控制 MuJoCo；退出请关闭 viewer 或按 Ctrl-C。")
    handle = _viewer(simulation) if args.viewer else None
    session.ik_debug = bool(args.ik_debug)
    episode_collector: SingleEpisodeCollector | None = None
    workspace_camera: AsyncRGBDCamera | None = None
    wrist_camera: AsyncRGBDCamera | None = None
    episode_preview: AsyncDualCameraPreview | None = None
    last_camera_timestamp = {"workspace": -1, "wrist": -1}
    emitted_record_index = 0
    if args.episode_data_config is not None:
        (
            episode_collector,
            workspace_camera,
            wrist_camera,
            episode_preview,
        ) = _start_episode_data_runtime(args, config)
        print(
            f"EPISODE_CAPTURE=IDLE id={episode_collector.writer.temporary_id} "
            f"root={episode_collector.writer.root}"
        )
    started = time.monotonic()
    sim_period = float(simulation.model.opt.timestep)
    target_period = 1.0 / target_hz
    viewer_period = 1.0 / viewer_hz
    next_sim = next_target = next_viewer = started
    sim_overrun_steps = 0
    target_skipped_ticks = 0
    viewer_skipped_frames = 0
    viewer_updates = 0
    next_telemetry = started
    with HtsRawRecordingWriter(
        capture_path,
        metadata={
            "mode": "quest_jaka_live_smooth_6dof_simulation_only",
            "controller_provider": "quest_ctrl_udp_v1",
        },
    ) as writer:
        worker = QuestDatagramReceiverWorker(
            bind=args.bind,
            port=args.port,
            allowed_sender=args.allowed_sender,
            record=writer.write,
        )
        worker.start()
        try:
            while (handle is None or handle.is_running()) and time.monotonic() - started < args.duration_sec:
                worker.raise_if_failed()
                for datagram in worker.drain():
                    if episode_collector is not None and episode_collector.state is CaptureState.REC:
                        episode_collector.writer.append_raw(
                            "quest_raw_datagram",
                            {
                                "host_monotonic_ns": datagram.receive_monotonic_ns,
                                "source_endpoint": datagram.source_endpoint,
                                "payload_hex": datagram.payload.hex(),
                            },
                        )
                    controller_router.ingest(datagram, session)
                now = time.monotonic()
                if episode_collector is not None:
                    assert workspace_camera is not None and wrist_camera is not None
                    for camera in (workspace_camera, wrist_camera):
                        if camera.error is not None:
                            episode_collector.camera_fault(camera.role, str(camera.error))
                            continue
                        frame, skipped = camera.latest_after(
                            last_camera_timestamp[camera.role]
                        )
                        if frame is not None:
                            episode_collector.ingest_camera(
                                frame, skipped_frames=skipped
                            )
                            last_camera_timestamp[camera.role] = frame.host_monotonic_ns
                steps = 0
                while now >= next_sim and steps < 20:
                    _step_smooth_simulation(simulation, session, sim_period)
                    next_sim += sim_period
                    steps += 1
                if now >= next_sim:
                    sim_overrun_steps += int((now - next_sim) / sim_period) + 1
                    next_sim = now + sim_period
                if now >= next_target:
                    skipped = max(0, int((now - next_target) / target_period))
                    target_skipped_ticks += skipped
                    control_now_ns = time.monotonic_ns()
                    controller_router.poll(control_now_ns, session)
                    session.control_tick(control_now_ns)
                    if episode_collector is not None:
                        control_sample, reference_established, event = _simulation_control_sample(
                            session, simulation, control_now_ns
                        )
                        episode_collector.ingest_control(
                            control_sample,
                            reference_established=reference_established,
                            raw_records={
                                "accepted_arm_target_60hz": {
                                    "host_monotonic_ns": control_now_ns,
                                    "accepted_target_sequence": event.get("accepted_target_sequence"),
                                    "accepted_joint_target_rad": event.get("accepted_joint_target_rad"),
                                    "reference_generation": event.get("reference_generation"),
                                },
                                "rh56_target": {
                                    "host_monotonic_ns": control_now_ns,
                                    "status": "commanded_simulation",
                                    "target_rad": event.get("commanded_hand_target_rad"),
                                },
                                "rh56_observation": {
                                    "host_monotonic_ns": control_now_ns,
                                    "status": "measured_simulation",
                                    "position_rad": event.get("actual_hand_actuator_position_rad"),
                                },
                            },
                        )
                        if isinstance(session.arm_output, JakaEquivalent125HzMujocoAdapter):
                            records = session.arm_output.records
                            if episode_collector.state is CaptureState.REC:
                                episode_collector.writer.append_raw_batch(
                                    [
                                        ("emitted_arm_command_125hz", record)
                                        for record in records[emitted_record_index:]
                                    ]
                                )
                            emitted_record_index = len(records)
                    # Run at most one target/IK update after a stall.  Replaying
                    # expired deadlines back-to-back turns one host pause into a
                    # visible burst of joint targets.
                    next_target += (skipped + 1) * target_period
                if now >= next_viewer:
                    skipped = max(0, int((now - next_viewer) / viewer_period))
                    viewer_skipped_frames += skipped
                    if handle is not None:
                        _sync_viewer(
                            handle,
                            simulation,
                            session,
                        )
                        viewer_updates += 1
                    if episode_collector is not None and episode_preview is not None:
                        episode_preview.update(
                            PreviewStatus(
                                state=episode_collector.state,
                                temporary_id=episode_collector.writer.temporary_id,
                                episode_start_ns=episode_collector.writer.start_monotonic_ns,
                                arm_trigger=session.arm_clutch.state.value == "engaged",
                                hand_grip=session.hand_clutch.state.value in {"reacquire", "engaged"},
                                recording_frame_count=episode_collector.writer.sample_count,
                            ),
                        )
                        if episode_preview.closed:
                            reason = (
                                "preview_error"
                                if episode_preview.error is not None
                                else "preview_closed"
                            )
                            episode_collector.shutdown(reason)
                            break
                    next_viewer += (skipped + 1) * viewer_period
                if episode_collector is not None and episode_collector.state is CaptureState.DONE:
                    break
                if args.telemetry_hz > 0 and now >= next_telemetry:
                    controller = controller_router.last_state
                    latest = session.event_records[-1] if session.event_records else {}
                    print(
                        "STATUS "
                        f"right_valid={latest.get('right_wrist_valid', False)} "
                        f"controller_valid={controller.controller_valid} "
                        f"arm={session.arm_clutch.state.value} "
                        f"hand={session.hand_clutch.state.value} "
                        f"target={session.last_reason} "
                        f"accepted={session.accepted_targets}",
                        flush=True,
                    )
                    thumb = latest.get("thumb_close_debug") or {}
                    lateral = latest.get("thumb_lateral_debug") or {}
                    if thumb:
                        print(
                            "THUMB "
                            f"bend={thumb.get('normalized_thumb_bend')} "
                            f"pinch={thumb.get('normalized_pinch')} "
                            f"base={thumb.get('base_bend_contribution')} "
                            f"assist={thumb.get('pinch_assist_contribution')} "
                            f"feature={thumb.get('combined_feature_normalized')} "
                            f"feature_ref_rad={thumb.get('captured_feature_reference_rad')} "
                            f"delta_rad={thumb.get('feature_delta_rad')} "
                            f"requested_rad={thumb.get('requested_target_rad')} "
                            f"clipped_rad={thumb.get('clipped_target_rad')} "
                            f"actual_rad={thumb.get('actual_mujoco_joint_rad')} "
                            f"saturated={thumb.get('saturation')}",
                            flush=True,
                        )
                    if lateral:
                        print(
                            "LATERAL "
                            f"raw={lateral.get('raw_across_palm')} "
                            f"feature={lateral.get('feature_normalized')} "
                            f"feature_ref={lateral.get('captured_feature_reference')} "
                            f"delta={lateral.get('feature_delta')} "
                            f"requested_rad={lateral.get('requested_target_rad')} "
                            f"clipped_rad={lateral.get('clipped_target_rad')} "
                            f"actual_rad={lateral.get('actual_mujoco_joint_rad')} "
                            f"saturated={lateral.get('saturation')}",
                            flush=True,
                        )
                    if args.ik_debug and latest.get("metrics"):
                        metrics = latest["metrics"]
                        q = latest.get("accepted_joint_target_rad") or metrics.get("ik_candidate_rad") or ()
                        dq = metrics.get("joint_delta_rad") or ()
                        print(
                            "IKDBG "
                            f"q_deg={tuple(round(math.degrees(v), 2) for v in q)} "
                            f"dq_deg={tuple(round(math.degrees(v), 3) for v in dq)} "
                            f"tool_swing_deg={math.degrees(metrics.get('target_tool_swing_rad', 0.0)):.3f} "
                            f"tool_roll_deg={math.degrees(metrics.get('target_tool_axial_roll_rad', 0.0)):.3f} "
                            f"j6_expected_deg={math.degrees(metrics.get('j6_expected_delta_rad', 0.0)):.3f} "
                            f"j6_actual_axial_deg={math.degrees(metrics.get('j6_axial_contribution_rad', 0.0)):.3f} "
                            f"j6_ratio={metrics.get('j6_axial_contribution_ratio')} "
                            f"pos_err_mm={1000.0 * metrics.get('ik_error_m', 0.0):.3f} "
                            f"ori_err_deg={math.degrees(metrics.get('ik_orientation_error_rad', 0.0)):.3f} "
                            f"cond={metrics.get('jacobian_condition'):.3f} "
                            f"sigma_min={metrics.get('minimum_jacobian_singular_value'):.6f} "
                            f"safe_limit_margin_deg={math.degrees(metrics.get('nearest_safe_joint_limit_margin_rad', 0.0)):.2f} "
                            f"accepted={latest.get('accepted')} reason={latest.get('reason')} "
                            f"branch_switch={metrics.get('branch_switch')} hold_last={latest.get('hold_last')} "
                            f"continuation={latest.get('continuation_fraction', 1.0):.4f} "
                            f"backtracks={latest.get('continuation_backtracks', 0)} "
                            f"backlog_mm={1000.0 * latest.get('requested_backlog_m', 0.0):.2f} "
                            f"backlog_deg={latest.get('requested_backlog_deg', 0.0):.2f} "
                            f"singularity_warning={latest.get('singularity_warning', False)}",
                            flush=True,
                        )
                    next_telemetry = now + 1.0 / args.telemetry_hz
                deadline = min(next_sim, next_target, next_viewer)
                time.sleep(max(0.0, min(0.001, deadline - time.monotonic())))
        except KeyboardInterrupt:
            if episode_collector is not None:
                episode_collector.shutdown("operator_interrupt")
        finally:
            if episode_collector is not None:
                episode_collector.shutdown("capture_loop_ended")
            worker.close()
            if handle is not None:
                handle.close()
            if episode_preview is not None:
                episode_preview.stop()
            if workspace_camera is not None:
                workspace_camera.stop()
            if wrist_camera is not None:
                wrist_camera.stop()
    report = session.report(str(capture_path.resolve()))
    report.update(
        mode="live_quest_to_smooth_6dof_simulation_only",
        policy_source="yaml",
        event_log=str(events_path.resolve()),
        raw_receive_queue_drops=worker.dropped,
        simulation_overrun_steps=sim_overrun_steps,
        target_skipped_ticks=target_skipped_ticks,
        viewer_skipped_frames=viewer_skipped_frames,
        viewer_update_count=viewer_updates,
        viewer_rate_hz=(viewer_updates / max(time.monotonic() - started, 1e-9)),
        arm_output_mode=args.arm_output_mode,
        **controller_router.telemetry(),
    )
    if isinstance(session.arm_output, JakaEquivalent125HzMujocoAdapter):
        _write_events(session.arm_output.records, emitted_path)
        report.update(session.arm_output.report())
        report["arm_emitted_event_log"] = str(emitted_path.resolve())
    report["recording_manifest"] = _run_manifest(
        config_path=args.config,
        duration_s=max(time.monotonic() - started, 0.0),
        raw_input_path=capture_path,
        report_path=report_path,
        events_path=events_path,
        target_hz=target_hz,
        simulation=simulation,
        arm_output_mode=args.arm_output_mode,
        emitted_path=(
            emitted_path
            if isinstance(session.arm_output, JakaEquivalent125HzMujocoAdapter)
            else None
        ),
    )
    _write_events(session.event_records, events_path)
    _write_report(report, report_path)
    if episode_collector is not None and episode_collector.result is not None:
        print(f"EPISODE_RESULT={episode_collector.result.resolve()}")
    if isinstance(session.arm_output, JakaEquivalent125HzMujocoAdapter):
        session.arm_output.close()
    return 0


def _replay_6dof(args: argparse.Namespace) -> int:
    if args.speed <= 0 or args.engage_at_sec < 0:
        raise SystemExit("speed must be positive and engage time non-negative")
    if args.arm_cycle_period_sec is not None and (
        args.arm_cycle_period_sec <= 0 or args.arm_cycle_count <= 0
    ):
        raise SystemExit("arm cycle period and count must be positive")
    if args.hand_cycle_period_sec is not None and (
        args.hand_engage_at_sec is None
        or args.hand_cycle_period_sec <= 0
        or args.hand_cycle_count <= 0
    ):
        raise SystemExit("hand cycles require positive period/count and --hand-engage-at-sec")
    config = replace(
        ReplayConfig.load(args.config),
        engagement_schedule_s=(),
    )
    if args.hand_reacquisition_ms is not None:
        raw = copy.deepcopy(dict(config.raw))
        raw.setdefault("clutches", {})["hand_reacquisition_ms"] = args.hand_reacquisition_ms
        config = replace(config, raw=raw)
    simulation, session = _make_smooth_session(
        config,
        arm_output_mode=args.arm_output_mode,
    )
    datagrams = list(HtsRawRecordingReader(args.recording).datagrams())
    if not datagrams:
        raise SystemExit("recording contains no datagrams")
    base_ns = datagrams[0].receive_monotonic_ns
    recorded_controller = any(
        datagram.payload.startswith(b"CTRL,") for datagram in datagrams
    )
    clutch_config = config.raw.get("clutches", {})
    controller_router = LiveQuestControllerRouter(
        stale_after_s=float(clutch_config.get("stale_after_ms", 150.0)) / 1000.0,
        released_at=float(clutch_config.get("released_at", 0.55)),
    )
    recorded_end_ns = datagrams[-1].receive_monotonic_ns
    end_ns = (
        recorded_end_ns
        if args.duration_sec is None
        else min(recorded_end_ns, base_ns + int(args.duration_sec * 1e9))
    )
    rates = config.raw.get("rates", {})
    control_period_ns = int(round(1e9 / float(rates.get("target_generation_hz", 60.0))))
    sim_period_ns = int(round(simulation.model.opt.timestep * 1e9))
    viewer_period_ns = int(round(1e9 / float(rates.get("viewer_hz", 60.0))))
    handle = _viewer(simulation) if args.viewer else None
    realtime = bool(args.realtime or args.viewer)
    wall_start = time.monotonic()
    index = 0
    now_ns = base_ns
    next_control_ns = base_ns
    next_viewer_ns = base_ns
    clutch_sequence = 0
    viewer_updates = 0
    try:
        while now_ns <= end_ns and (handle is None or handle.is_running()):
            while index < len(datagrams) and datagrams[index].receive_monotonic_ns <= now_ns:
                controller_router.ingest(datagrams[index], session)
                index += 1
            elapsed_s = (now_ns - base_ns) / 1e9
            if now_ns >= next_control_ns:
                clutch_sequence += 1
                if recorded_controller:
                    controller_router.poll(now_ns, session)
                elif args.arm_cycle_period_sec is None:
                    index_pressed = elapsed_s >= args.engage_at_sec
                else:
                    cycle_elapsed = elapsed_s - args.engage_at_sec
                    cycle_index = int(cycle_elapsed / args.arm_cycle_period_sec)
                    index_pressed = (
                        cycle_elapsed >= 0.0
                        and cycle_index < args.arm_cycle_count
                        and cycle_elapsed % args.arm_cycle_period_sec
                        < args.arm_cycle_period_sec / 2.0
                    )
                if not recorded_controller and args.hand_engage_at_sec is None:
                    grip_pressed = False
                elif not recorded_controller and args.hand_cycle_period_sec is None:
                    grip_pressed = elapsed_s >= args.hand_engage_at_sec
                elif not recorded_controller:
                    hand_elapsed = elapsed_s - args.hand_engage_at_sec
                    hand_cycle_index = int(hand_elapsed / args.hand_cycle_period_sec)
                    grip_pressed = (
                        hand_elapsed >= 0.0
                        and hand_cycle_index < args.hand_cycle_count
                        and hand_elapsed % args.hand_cycle_period_sec
                        < args.hand_cycle_period_sec / 2.0
                    )
                if not recorded_controller:
                    # Known legacy recordings contain HTS only; retain their
                    # explicit deterministic CLI clutch source.
                    session.set_clutch_samples(
                        index=AnalogClutchSample(
                            1.0 if index_pressed else 0.0,
                            now_ns,
                            clutch_sequence,
                        ),
                        grip=AnalogClutchSample(
                            1.0 if grip_pressed else 0.0,
                            now_ns,
                            clutch_sequence,
                        ),
                        left_controller_valid=True,
                        provider="deterministic_replay_cli",
                    )
                session.control_tick(now_ns)
                next_control_ns += control_period_ns
            _step_smooth_simulation(
                simulation,
                session,
                simulation.model.opt.timestep,
            )
            if handle is not None and now_ns >= next_viewer_ns:
                _sync_viewer(handle, simulation, session)
                viewer_updates += 1
                next_viewer_ns += viewer_period_ns
            now_ns += sim_period_ns
            if realtime:
                deadline = wall_start + ((now_ns - base_ns) / 1e9) / args.speed
                time.sleep(max(0.0, min(0.005, deadline - time.monotonic())))
    finally:
        if handle is not None:
            handle.close()
    report = session.report(str(args.recording.resolve()))
    report.update(
        mode="recorded_smooth_6dof_simulation_only",
        policy_source="yaml",
        deterministic=True,
        replay_speed=args.speed,
        viewer_update_count=viewer_updates,
        recorded_controller_replayed=recorded_controller,
        arm_output_mode=args.arm_output_mode,
        **controller_router.telemetry(),
    )
    report_path = args.report or _paths("quest_jaka_replay_6dof")[0]
    events_path = args.events or report_path.with_suffix(".events.jsonl")
    emitted_path = args.arm_emitted_events or report_path.with_suffix(
        ".arm_emitted_125hz.jsonl"
    )
    report["event_log"] = str(events_path.resolve())
    if isinstance(session.arm_output, JakaEquivalent125HzMujocoAdapter):
        _write_events(session.arm_output.records, emitted_path)
        report.update(session.arm_output.report())
        report["arm_emitted_event_log"] = str(emitted_path.resolve())
    report["recording_manifest"] = _run_manifest(
        config_path=args.config,
        duration_s=(end_ns - base_ns) / 1e9,
        raw_input_path=args.recording,
        report_path=report_path,
        events_path=events_path,
        target_hz=float(rates.get("target_generation_hz", 60.0)),
        simulation=simulation,
        arm_output_mode=args.arm_output_mode,
        emitted_path=(
            emitted_path
            if isinstance(session.arm_output, JakaEquivalent125HzMujocoAdapter)
            else None
        ),
    )
    _write_events(session.event_records, events_path)
    _write_report(report, report_path)
    if isinstance(session.arm_output, JakaEquivalent125HzMujocoAdapter):
        session.arm_output.close()
    return 0 if report["accepted_target_count"] > 0 else 2


def _print_effective_speed_config(
    config: ReplayConfig,
    simulation: JakaMujocoSimulation,
) -> None:
    """Print all speed/latency values that affect the simulation run."""

    raw = config.raw
    mapping = raw.get("provisional_calibration", {})
    filter_values = raw.get("filter", {}).get("profiles", {}).get(
        raw.get("filter", {}).get("selected_profile", ""), {}
    )
    rates = raw.get("rates", {})
    limits = config.feasibility
    command = config.command_limits
    contract = config.output_contract
    print("SIM_POLICY=yaml")
    print(f"JOINT_SPEED_LIMITS_RAD_S={list(contract.velocity_boundaries_rad_s)}")
    print(f"JOINT_ACCELERATION_LIMITS_RAD_S2={[command.maximum_acceleration_rad_s2] * 6}")
    print(f"JOINT_JERK_LIMITS_RAD_S3={[command.maximum_jerk_rad_s3] * 6}")
    print(f"TCP_LINEAR_VELOCITY_LIMIT_M_S={limits.maximum_tcp_velocity_m_s:g}")
    print(f"TCP_ANGULAR_VELOCITY_LIMIT_RAD_S={limits.maximum_tcp_angular_velocity_rad_s:g}")
    print(f"QUEST_TRANSLATION_SCALE={mapping.get('translation_scale_per_axis')}")
    print(f"QUEST_ROTATION_SCALE={mapping.get('orientation_scale_per_axis', mapping.get('orientation_scale'))}")
    print(
        "FILTER="
        f"profile={raw.get('filter', {}).get('selected_profile')} "
        f"translation_min_cutoff={filter_values.get('translation_min_cutoff')} "
        f"translation_beta={filter_values.get('translation_beta')} "
        f"translation_derivative_cutoff={filter_values.get('translation_derivative_cutoff')} "
        f"rotation_min_cutoff={filter_values.get('rotation_min_cutoff')} "
        f"rotation_beta={filter_values.get('rotation_beta')} "
        f"rotation_derivative_cutoff={filter_values.get('rotation_derivative_cutoff')} "
        f"maximum_dt={filter_values.get('maximum_filter_dt')}"
    )
    print(f"TRANSLATION_DEADBAND_M={mapping.get('translation_deadband_m')}")
    print(f"ORIENTATION_DEADBAND_DEG={mapping.get('orientation_deadband_deg')}")
    print(f"INPUT_INTERPOLATION_DELAY_MS={float(rates.get('interpolation_delay_ms', 20.0)):g}")
    print(f"TARGET_IK_HZ={float(rates.get('target_generation_hz', 60.0)):g}")
    print(f"MUJOCO_CONTROL_HZ={1.0 / float(simulation.model.opt.timestep):g}")
    print(f"SERVO_CONTRACT_PERIOD_MS={contract.servo_period_ns / 1e6:g}")
    print(
        "OUTPUT_FEASIBILITY_ACCELERATION_PERIOD_MS="
        f"{(contract.feasibility_acceleration_period_ns or contract.servo_period_ns) / 1e6:g}"
    )
    print(f"IK_MAX_STEP_RAD={config.ik_max_step_rad:g}")
    print(f"MAXIMUM_JOINT_TARGET_JUMP_RAD={limits.maximum_joint_target_jump_rad:g}")
    print(f"POSITION_TRACKING_FREQUENCY_RAD_S={command.position_tracking_frequency_rad_s:g}")
    print("JAKA hardware control disabled")
    print("RH56 hardware control disabled")
    print(f"RH56_SIMULATION_CONTROL_ENABLED={str(bool(getattr(simulation, 'hand_available', False))).lower()}")


def main() -> int:
    args = _parser().parse_args()
    if args.command == "replay":
        return _replay(args)
    if args.command == "live-6dof":
        return _live_6dof(args)
    if args.command == "replay-6dof":
        return _replay_6dof(args)
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
