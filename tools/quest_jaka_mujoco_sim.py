#!/usr/bin/env python3
"""Quest HTS to JAKA MuJoCo simulation; no physical hardware backend exists."""

from __future__ import annotations

import argparse
import copy
from dataclasses import replace
from datetime import datetime
import importlib
import ipaddress
import json
import math
from pathlib import Path
import socket
import time

import mujoco

from motion_input import HtsRawRecordingReader, HtsRawRecordingWriter
from quest_jaka_sim import (
    AnalogClutchSample,
    JakaMujocoSimulation,
    QuestJakaReplaySession,
    ReplayConfig,
    SmoothQuestJakaSession,
)
from quest_jaka_sim.simulation import build_viewer_mjcf
from quest_jaka_sim.se3 import quaternion_angle_rad
from quest_jaka_sim.live_controller import LiveQuestControllerRouter
from quest_jaka_sim.live_input import QuestDatagramReceiverWorker


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
    smooth_replay.add_argument("--viewer", action="store_true")
    smooth_replay.add_argument("--realtime", action="store_true")
    smooth_replay.add_argument("--speed", type=float, default=1.0)
    smooth_replay.add_argument("--engage-at-sec", type=float, default=1.0)
    smooth_replay.add_argument(
        "--arm-cycle-period-sec",
        type=float,
        help="explicit deterministic offline arm press/release cycle period",
    )
    smooth_replay.add_argument("--arm-cycle-count", type=int, default=10)
    smooth_replay.add_argument(
        "--hand-engage-at-sec",
        type=float,
        help="explicit deterministic fake grip press for offline validation",
    )
    smooth_replay.add_argument(
        "--hand-cycle-period-sec",
        type=float,
        help="explicit deterministic offline hand press/release cycle period",
    )
    smooth_replay.add_argument("--hand-cycle-count", type=int, default=4)
    smooth_replay.add_argument("--duration-sec", type=float)
    smooth_replay.add_argument(
        "--hand-reacquisition-ms",
        type=float,
        choices=(150.0, 200.0, 250.0, 300.0),
        help="offline-only fixed reacquisition-duration comparison",
    )
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


def _make_smooth_session(
    config: ReplayConfig,
) -> tuple[JakaMujocoSimulation, SmoothQuestJakaSession]:
    augmented = build_viewer_mjcf(
        config.mjcf_path, Path("logs/quest_jaka_sim/quest_jaka_viewer_model.xml")
    )
    simulation = JakaMujocoSimulation(config, mjcf_path=augmented)
    return simulation, SmoothQuestJakaSession(config, simulation)


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
    return handle


def _sync_viewer(handle: object, simulation: JakaMujocoSimulation, session: object) -> None:
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
        else f"{hand_result.backend} valid={hand_result.valid} cost={hand_result.optimizer_cost}"
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
    handle.set_texts(
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
                f"head_yaw={latest.get('captured_head_yaw_rad')} hand_retarget={hand_status}\n"
                f"retarget_status={latest.get('hand_retarget_status')}\n"
                f"hand_reacquire={latest.get('hand_reacquisition_fraction')} "
                f"arm_fault={latest.get('active_arm_fault')} hand_fault={latest.get('active_hand_fault')}\n"
                f"cycles arm={getattr(arm, 'cycle_count', 0)} hand={getattr(hand, 'cycle_count', 0)}\n"
                f"{instruction}\n"
                f"BLUE=desired TCP frame GREEN=simulated TCP frame{debug_text}"
            ),
        )
    )
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


def _live_6dof(args: argparse.Namespace) -> int:
    if args.duration_sec <= 0:
        raise SystemExit("duration must be positive")
    if args.telemetry_hz < 0:
        raise SystemExit("telemetry-hz must be non-negative")
    config = replace(ReplayConfig.load(args.config), engagement_schedule_s=())
    simulation, session = _make_smooth_session(config)
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
    capture_path.parent.mkdir(parents=True, exist_ok=True)
    print(f"PROJECT_IP={project_ip}")
    print(f"PORT={args.port}")
    print("TRANSPORT=UDP")
    print(
        f"RATES=input~30Hz target={target_hz:g}Hz "
        f"mujoco={1.0/simulation.model.opt.timestep:g}Hz viewer={viewer_hz:g}Hz"
    )
    print("CONTROL=LEFT INDEX arm clutch; LEFT GRIP hand clutch; both are independent hold-to-run")
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
    print("5. 左手食指扳机先完全释放，再按住以捕获 reference 并进入 engaged。")
    print("6. 按住食指扳机移动/旋转右手；释放即 disengage，下一次按下会重新捕获。")
    print("7. 左手 grip 仅控制仿真 RH56；退出请关闭 viewer 或按 Ctrl-C。")
    handle = _viewer(simulation) if args.viewer else None
    session.ik_debug = bool(args.ik_debug)
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
                    controller_router.ingest(datagram, session)
                now = time.monotonic()
                steps = 0
                while now >= next_sim and steps < 20:
                    simulation.step(sim_period)
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
                    # Run at most one target/IK update after a stall.  Replaying
                    # expired deadlines back-to-back turns one host pause into a
                    # visible burst of joint targets.
                    next_target += (skipped + 1) * target_period
                if now >= next_viewer:
                    skipped = max(0, int((now - next_viewer) / viewer_period))
                    viewer_skipped_frames += skipped
                    if handle is not None:
                        _sync_viewer(handle, simulation, session)
                        viewer_updates += 1
                    next_viewer += (skipped + 1) * viewer_period
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
            pass
        finally:
            worker.close()
            if handle is not None:
                handle.close()
    report = session.report(str(capture_path.resolve()))
    report.update(
        mode="live_quest_to_smooth_6dof_simulation_only",
        event_log=str(events_path.resolve()),
        raw_receive_queue_drops=worker.dropped,
        simulation_overrun_steps=sim_overrun_steps,
        target_skipped_ticks=target_skipped_ticks,
        viewer_skipped_frames=viewer_skipped_frames,
        viewer_update_count=viewer_updates,
        viewer_rate_hz=(viewer_updates / max(time.monotonic() - started, 1e-9)),
        **controller_router.telemetry(),
    )
    _write_events(session.event_records, events_path)
    _write_report(report, report_path)
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
    config = replace(ReplayConfig.load(args.config), engagement_schedule_s=())
    if args.hand_reacquisition_ms is not None:
        raw = copy.deepcopy(dict(config.raw))
        raw.setdefault("clutches", {})["hand_reacquisition_ms"] = args.hand_reacquisition_ms
        config = replace(config, raw=raw)
    simulation, session = _make_smooth_session(config)
    datagrams = list(HtsRawRecordingReader(args.recording).datagrams())
    if not datagrams:
        raise SystemExit("recording contains no datagrams")
    base_ns = datagrams[0].receive_monotonic_ns
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
                session.ingest(datagrams[index])
                index += 1
            elapsed_s = (now_ns - base_ns) / 1e9
            if now_ns >= next_control_ns:
                clutch_sequence += 1
                if args.arm_cycle_period_sec is None:
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
                if args.hand_engage_at_sec is None:
                    grip_pressed = False
                elif args.hand_cycle_period_sec is None:
                    grip_pressed = elapsed_s >= args.hand_engage_at_sec
                else:
                    hand_elapsed = elapsed_s - args.hand_engage_at_sec
                    hand_cycle_index = int(hand_elapsed / args.hand_cycle_period_sec)
                    grip_pressed = (
                        hand_elapsed >= 0.0
                        and hand_cycle_index < args.hand_cycle_count
                        and hand_elapsed % args.hand_cycle_period_sec
                        < args.hand_cycle_period_sec / 2.0
                    )
                # This is an explicitly labelled deterministic offline source,
                # never a claim of live Quest-controller support.
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
            simulation.step(simulation.model.opt.timestep)
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
        deterministic=True,
        replay_speed=args.speed,
        viewer_update_count=viewer_updates,
    )
    report_path = args.report or _paths("quest_jaka_replay_6dof")[0]
    events_path = args.events or report_path.with_suffix(".events.jsonl")
    report["event_log"] = str(events_path.resolve())
    _write_events(session.event_records, events_path)
    _write_report(report, report_path)
    return 0 if report["accepted_target_count"] > 0 else 2


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
