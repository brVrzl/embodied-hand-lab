from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from teleoperation.dagger_arbiter import (
    DaggerAction,
    DaggerActionArbiter,
    DaggerActionSource,
    DaggerState,
    ExpertArmCandidateSink,
    ExpertHandCandidateSink,
)
from tools.act_dagger_rollout import (
    DaggerCoordinator,
    DaggerProvenanceWriter,
    EXPERT_CONTROL_RATE_HZ,
    _Rh56OwnershipDispatch,
    _dagger_control_period_ns,
    _physical_episode_release_ready,
    _wait_for_native_target_acceptance,
)


def _action(source: DaggerActionSource, value: float) -> DaggerAction:
    return DaggerAction.from_vector([value] * 12, source=source)


def test_startup_alignment_waits_for_native_acknowledgement() -> None:
    statuses = iter(
        [
            SimpleNamespace(last_sequence=0),
            SimpleNamespace(last_sequence=7),
        ]
    )
    runtime = SimpleNamespace(latest_status=lambda: next(statuses))
    native = SimpleNamespace(
        process=SimpleNamespace(poll=lambda: None),
    )

    status = _wait_for_native_target_acceptance(runtime, native, 7)

    assert status.last_sequence == 7


def test_dagger_uses_physical_episode_release_gate() -> None:
    event = {
        "right_wrist_valid": True,
        "hand_skeleton_valid": True,
        "input_recovery_active": False,
    }
    assert _physical_episode_release_ready(
        arm_clutch_state="disengaged",
        hand_clutch_state="disengaged",
        latest_event=event,
        dispatch_failed=False,
    )
    assert not _physical_episode_release_ready(
        arm_clutch_state="disengaged",
        hand_clutch_state="engaged",
        latest_event=event,
        dispatch_failed=False,
    )
    assert not _physical_episode_release_ready(
        arm_clutch_state="disengaged",
        hand_clutch_state="disengaged",
        latest_event=event,
        dispatch_failed=True,
    )


def test_policy_takeover_hold_and_expert_handshake() -> None:
    arbiter = DaggerActionArbiter()
    policy = _action(DaggerActionSource.POLICY, 0.1)
    expert = _action(DaggerActionSource.EXPERT, 0.2)
    hold = _action(DaggerActionSource.HOLD, 0.0)

    decision = arbiter.select(policy=policy, expert=None, hold=hold)
    assert decision.state is DaggerState.POLICY_RUN
    assert decision.action is policy

    intervention_id = arbiter.request_takeover(10)
    assert intervention_id == "takeover_0001"
    decision = arbiter.select(policy=policy, expert=expert, hold=hold)
    assert decision.state is DaggerState.TAKEOVER_HOLD
    assert decision.action is hold

    assert not arbiter.observe_clutch(
        20, released=False, pressed=True, valid=True
    )
    assert arbiter.state is DaggerState.TAKEOVER_HOLD
    assert not arbiter.observe_clutch(
        30, released=True, pressed=False, valid=True
    )
    assert arbiter.state is DaggerState.EXPERT_ARMING
    assert arbiter.observe_clutch(40, released=False, pressed=True, valid=True)
    arbiter.mark_expert_ready(50)

    decision = arbiter.select(policy=policy, expert=expert, hold=hold)
    assert decision.state is DaggerState.EXPERT_RUN
    assert decision.action is expert

    arbiter.resume_policy(60)
    assert arbiter.state is DaggerState.POLICY_RUN
    assert arbiter.intervention_id is None
    assert arbiter.select(policy=policy, expert=expert, hold=hold).action is policy
    assert arbiter.request_takeover(70) == "takeover_0002"


def test_invalid_expert_candidate_never_falls_back_to_policy() -> None:
    arbiter = DaggerActionArbiter()
    policy = _action(DaggerActionSource.POLICY, 0.1)
    hold = _action(DaggerActionSource.HOLD, 0.0)
    arbiter.request_takeover(1)
    arbiter.observe_clutch(2, released=True, pressed=False, valid=True)
    arbiter.observe_clutch(3, released=False, pressed=True, valid=True)
    arbiter.mark_expert_ready(4)

    decision = arbiter.select(policy=policy, expert=None, hold=hold)
    assert decision.state is DaggerState.EXPERT_RUN
    assert decision.action is hold
    assert decision.reason == "expert_candidate_unavailable"


def test_policy_resume_requires_active_expert_episode() -> None:
    arbiter = DaggerActionArbiter()

    with pytest.raises(RuntimeError, match="policy resume is unavailable"):
        arbiter.resume_policy(1)


def test_dagger_uses_30_hz_policy_and_60_hz_expert_periods() -> None:
    assert _dagger_control_period_ns(
        DaggerState.POLICY_RUN,
        policy_rate_hz=30.0,
    ) == round(1e9 / 30.0)
    assert _dagger_control_period_ns(
        DaggerState.TAKEOVER_HOLD,
        policy_rate_hz=30.0,
    ) == round(1e9 / EXPERT_CONTROL_RATE_HZ)
    assert _dagger_control_period_ns(
        DaggerState.EXPERT_ARMING,
        policy_rate_hz=30.0,
    ) == round(1e9 / EXPERT_CONTROL_RATE_HZ)
    assert _dagger_control_period_ns(
        DaggerState.EXPERT_RUN,
        policy_rate_hz=30.0,
    ) == round(1e9 / EXPERT_CONTROL_RATE_HZ)


def test_safe_stop_has_no_selected_action() -> None:
    arbiter = DaggerActionArbiter()
    arbiter.safe_stop(10, reason="controller_stale")
    hold = _action(DaggerActionSource.HOLD, 0.0)
    decision = arbiter.select(policy=None, expert=None, hold=hold)
    assert decision.state is DaggerState.SAFE_STOP
    assert decision.action is None


def test_candidate_sinks_do_not_dispatch() -> None:
    arm = ExpertArmCandidateSink()
    assert arm.heartbeat_value is None

    hand = ExpertHandCandidateSink(lambda _now: (0.1,) * 6)
    assert hand.activate_from_measured(1) == (0.1,) * 6
    hand.submit_target((0.2,) * 6, 2)
    assert hand.target == (0.2,) * 6
    hand.hold("takeover")
    assert hand.hold_reason == "takeover"


def test_rh56_dispatch_reactivates_after_takeover_hold() -> None:
    events: list[tuple[object, ...]] = []

    class Worker:
        def activate_from_measured(self, monotonic_ns: int) -> tuple[float, ...]:
            events.append(("activate", monotonic_ns))
            return (0.1,) * 6

        def submit_target(self, target: tuple[float, ...], monotonic_ns: int) -> None:
            events.append(("submit", target, monotonic_ns))

        def hold(self, reason: str) -> None:
            events.append(("hold", reason))

    dispatch = _Rh56OwnershipDispatch(Worker())
    dispatch.submit_target((0.2,) * 6, 1)
    dispatch.submit_target((0.3,) * 6, 2)
    dispatch.hold("takeover_hold")
    dispatch.submit_target((0.4,) * 6, 3)

    assert events == [
        ("activate", 1),
        ("submit", (0.2,) * 6, 1),
        ("submit", (0.3,) * 6, 2),
        ("hold", "takeover_hold"),
        ("activate", 3),
        ("submit", (0.4,) * 6, 3),
    ]


def test_provenance_records_selected_source_and_label(tmp_path: Path) -> None:
    path = tmp_path / "dagger_control.jsonl"
    writer = DaggerProvenanceWriter(path)
    writer.start()
    coordinator = DaggerCoordinator(provenance=writer)
    hold = _action(DaggerActionSource.HOLD, 0.0)
    policy = _action(DaggerActionSource.POLICY, 0.1)
    expert = _action(DaggerActionSource.EXPERT, 0.2)
    try:
        coordinator.tick(1, policy=policy, expert=None, hold=hold)
        coordinator.request_takeover(2)
        coordinator.tick(3, policy=policy, expert=None, hold=hold)
        coordinator.observe_clutch(4, released=True, pressed=False, valid=True)
        coordinator.observe_clutch(5, released=False, pressed=True, valid=True)
        coordinator.mark_expert_ready(6)
        coordinator.tick(7, policy=policy, expert=expert, hold=hold)
    finally:
        writer.finish()

    rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
    assert [row["action_source"] for row in rows] == ["policy", "hold", "expert"]
    assert [row["action_label_valid"] for row in rows] == [False, False, True]
    assert rows[-1]["intervention_id"] == "takeover_0001"


def test_provenance_writer_rejects_duplicate_start(tmp_path: Path) -> None:
    writer = DaggerProvenanceWriter(tmp_path / "rows.jsonl")
    writer.start()
    try:
        with pytest.raises(RuntimeError):
            writer.start()
    finally:
        writer.finish()
