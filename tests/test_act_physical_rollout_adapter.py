from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import socket
import sys
import threading
import time
from types import SimpleNamespace

import pytest
import numpy as np

from teleoperation.jaka.quest_adapter import JakaAcceptedJointTargetAdapter
from teleoperation.wire import TargetKind


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "tools") not in sys.path:
    sys.path.insert(0, str(ROOT / "tools"))


def _load_rollout_tool():
    shadow_spec = importlib.util.spec_from_file_location(
        "act_live_shadow", ROOT / "tools" / "act_live_shadow.py"
    )
    assert shadow_spec is not None and shadow_spec.loader is not None
    shadow = importlib.util.module_from_spec(shadow_spec)
    sys.modules[shadow_spec.name] = shadow
    shadow_spec.loader.exec_module(shadow)
    rollout_spec = importlib.util.spec_from_file_location(
        "act_physical_rollout", ROOT / "tools" / "act_physical_rollout.py"
    )
    assert rollout_spec is not None and rollout_spec.loader is not None
    rollout = importlib.util.module_from_spec(rollout_spec)
    sys.modules[rollout_spec.name] = rollout
    rollout_spec.loader.exec_module(rollout)
    return rollout


def _load_model_worker():
    spec = importlib.util.spec_from_file_location(
        "act_shadow_model_worker", ROOT / "tools" / "act_shadow_model_worker.py"
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class _Runtime:
    def __init__(self) -> None:
        self.packets = []

    def dispatch_packet(self, packet: object) -> bool:
        self.packets.append(packet)
        return True

    def dispatch_stop(self, *, sequence: int) -> bool:
        return True


def test_policy_absolute_joint_target_uses_existing_adapter_contract() -> None:
    runtime = _Runtime()
    adapter = JakaAcceptedJointTargetAdapter(runtime, allow_motion=True)
    now = time.monotonic_ns()
    assert adapter.apply_joint_position(
        (0.1, -0.2, 0.3, -0.4, 0.5, -0.6),
        source_capture_ns=now - 1,
        local_receive_ns=now,
        processing_ns=now,
    )
    packet = runtime.packets[0]
    assert packet.kind is TargetKind.JOINT_POSITION
    assert packet.payload[:6] == (0.1, -0.2, 0.3, -0.4, 0.5, -0.6)
    assert packet.source_capture_ns == now - 1


def test_policy_absolute_joint_target_rejects_bad_shape_or_nonfinite() -> None:
    adapter = JakaAcceptedJointTargetAdapter(_Runtime(), allow_motion=True)
    with pytest.raises(ValueError):
        adapter.apply_joint_position((0.0,) * 5)
    with pytest.raises(ValueError):
        adapter.apply_joint_position((0.0, 0.0, 0.0, float("nan"), 0.0, 0.0))


def test_physical_rollout_duration_is_bounded_at_180_seconds() -> None:
    rollout = _load_rollout_tool()
    rollout._validate_rollout_duration(180.0)
    with pytest.raises(ValueError, match=r"within \(0,180\]"):
        rollout._validate_rollout_duration(180.001)
    with pytest.raises(ValueError, match=r"within \(0,180\]"):
        rollout._validate_rollout_duration(0.0)


def test_rh56_projection_is_boundary_only_and_reports_each_changed_channel() -> None:
    project_rh56_command = _load_rollout_tool()._project_rh56_command
    raw = np.asarray(
        [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, -0.00054, 0.2, 1.004, 0.4, -0.01, 0.8],
        dtype=np.float64,
    )
    projected, events = project_rh56_command(raw, legal_min=0.0, legal_max=1.0)
    assert np.array_equal(raw[:6], projected[:6])
    assert projected[6:].tolist() == [0.0, 0.2, 1.0, 0.4, 0.0, 0.8]
    assert [(event["channel"], event["raw_value"], event["projected_value"]) for event in events] == [
        ("index", -0.00054, 0.0),
        ("ring", 1.004, 1.0),
        ("thumb_close", -0.01, 0.0),
    ]
    assert [event["correction_magnitude"] for event in events] == pytest.approx(
        [0.00054, 0.004, 0.01]
    )
    assert raw[6] == pytest.approx(-0.00054)


def test_rh56_projection_preserves_exact_legal_boundaries() -> None:
    project = _load_rollout_tool()._project_rh56_command
    raw = np.asarray([0.0] * 6 + [0.0, 1.0, 0.0, 1.0, 0.0, 1.0])
    projected, events = project(raw, legal_min=0.0, legal_max=1.0)
    assert np.array_equal(projected, raw)
    assert events == []


@pytest.mark.parametrize("value", [-0.02, 1.02])
def test_rh56_projection_accepts_exact_correction_tolerance(value: float) -> None:
    project = _load_rollout_tool()._project_rh56_command
    raw = np.asarray([0.0] * 6 + [value, 0.2, 0.3, 0.4, 0.5, 0.6])
    projected, events = project(raw, legal_min=0.0, legal_max=1.0)
    assert projected[6] == (0.0 if value < 0.0 else 1.0)
    assert events[0]["correction_magnitude"] == pytest.approx(0.02)


@pytest.mark.parametrize("value", [-0.020001, 1.020001])
def test_rh56_projection_fails_closed_on_excessive_extrapolation(value: float) -> None:
    project = _load_rollout_tool()._project_rh56_command
    raw = np.asarray([0.0] * 6 + [value, 0.2, 0.3, 0.4, 0.5, 0.6])
    with pytest.raises(ValueError, match="bounded projection tolerance"):
        project(raw, legal_min=0.0, legal_max=1.0)


@pytest.mark.parametrize("value", [float("nan"), float("inf"), float("-inf")])
def test_rh56_projection_rejects_nonfinite_policy_output(value: float) -> None:
    project = _load_rollout_tool()._project_rh56_command
    raw = np.asarray([0.0] * 6 + [value, 0.2, 0.3, 0.4, 0.5, 0.6])
    with pytest.raises(ValueError, match="finite"):
        project(raw, legal_min=0.0, legal_max=1.0)


def test_canonical_strong_act_options_derive_one_query_per_control_tick() -> None:
    rollout = _load_rollout_tool()
    options = rollout._resolve_execution_options(
        requested_mode=None,
        command_rate_hz=30.0,
        query_rate_hz=None,
        temporal_ensemble_coeff=None,
        max_source_horizon=None,
        max_prediction_age_ticks=None,
        temporal_buffer_capacity=None,
    )
    assert options == {
        "execution_mode": "canonical_temporal_ensemble",
        "query_rate_hz": 30.0,
        "temporal_ensemble_coeff": 0.01,
        "max_source_horizon": None,
        "max_prediction_age_ticks": None,
        "temporal_buffer_capacity": 4096,
    }


def test_canonical_strong_act_rejects_legacy_executor_overrides() -> None:
    rollout = _load_rollout_tool()
    common = dict(
        requested_mode="canonical_temporal_ensemble",
        command_rate_hz=30.0,
        query_rate_hz=None,
        temporal_ensemble_coeff=None,
        max_source_horizon=None,
        max_prediction_age_ticks=None,
        temporal_buffer_capacity=None,
    )
    with pytest.raises(ValueError, match="query rate"):
        rollout._resolve_execution_options(**{**common, "query_rate_hz": 15.0})


def test_consume_k_mode_is_removed_from_rollout_options() -> None:
    rollout = _load_rollout_tool()
    with pytest.raises(ValueError, match="unsupported ACT execution mode"):
        rollout._resolve_execution_options(
            requested_mode="consume_k",
            command_rate_hz=30.0,
            query_rate_hz=15.0,
            temporal_ensemble_coeff=None,
            max_source_horizon=None,
            max_prediction_age_ticks=None,
            temporal_buffer_capacity=None,
        )


def test_generic_temporal_ensemble_mode_alias_is_removed() -> None:
    rollout = _load_rollout_tool()
    with pytest.raises(ValueError, match="unsupported ACT execution mode"):
        rollout._resolve_execution_options(
            requested_mode="temporal_ensemble",
            command_rate_hz=30.0,
            query_rate_hz=30.0,
            temporal_ensemble_coeff=None,
            max_source_horizon=None,
            max_prediction_age_ticks=None,
            temporal_buffer_capacity=None,
        )


def test_control_tick_timing_keeps_bounded_percentile_statistics() -> None:
    timing_type = _load_rollout_tool().BoundedStageTiming
    timing = timing_type(capacity=2)
    timing.add({"critical_path": 10.0})
    timing.add({"critical_path": 20.0})
    timing.add({"critical_path": 30.0})

    summary = timing.summary()["critical_path"]
    assert summary["count"] == 2
    assert summary["p50"] == pytest.approx(25.0)
    assert summary["max"] == pytest.approx(30.0)
    assert timing.summary()["rh56_command_call"]["count"] == 0


def test_temporal_freshness_uses_newest_contributor_not_oldest_history() -> None:
    rollout = _load_rollout_tool()
    contributor = SimpleNamespace
    selection = SimpleNamespace(
        contributors=(
            contributor(query_timestamp_ns=1_000_000_000),
            contributor(query_timestamp_ns=1_240_000_000),
        )
    )
    oldest, newest = rollout._temporal_selection_ages_ms(
        selection, 1_250_000_000
    )
    assert oldest == pytest.approx(250.0)
    assert newest == pytest.approx(10.0)


def test_model_ipc_timing_sideband_is_consumed_before_next_request() -> None:
    rollout = _load_rollout_tool()
    worker = _load_model_worker()
    left, right = socket.socketpair()
    try:
        sender = threading.Thread(
            target=worker._send_timed_prediction,
            args=(right, {"prediction": np.zeros((2, 12), dtype=np.float32), "timing_ms": {}}),
        )
        sender.start()
        response, timing = rollout._timed_model_request(left, {"request": 1})
        sender.join(timeout=1.0)
        assert not sender.is_alive()
        assert response["prediction"].shape == (2, 12)
        assert timing["policy_socket_receive"] >= 0.0
        assert response["timing_ms"]["worker_socket_send"] >= 0.0
        assert response["timing_ms"]["worker_response_serialization"] >= 0.0
    finally:
        left.close()
        right.close()


def test_rollout_model_worker_detects_force_checkpoint_input(tmp_path: Path) -> None:
    rollout = _load_rollout_tool()
    standard = tmp_path / "standard"
    standard.mkdir()
    (standard / "config.json").write_text(
        json.dumps(
            {
                "type": "act",
                "chunk_size": 60,
                "input_features": {
                    "observation.images.workspace": {"type": "VISUAL", "shape": [3, 240, 320]},
                    "observation.images.wrist": {"type": "VISUAL", "shape": [3, 240, 320]},
                    "observation.state": {"type": "STATE", "shape": [12]},
                },
                "output_features": {"action": {"type": "ACTION", "shape": [12]}},
            }
        ),
        encoding="utf-8",
    )
    force = tmp_path / "force"
    force.mkdir()
    (force / "config.json").write_text(
        json.dumps(
            {
                "type": "act",
                "chunk_size": 60,
                "input_features": {
                    "observation.images.workspace": {"type": "VISUAL", "shape": [3, 240, 320]},
                    "observation.images.wrist": {"type": "VISUAL", "shape": [3, 240, 320]},
                    "observation.state": {"type": "STATE", "shape": [12]},
                    "observation.environment_state": {"shape": [6]},
                },
                "output_features": {"action": {"type": "ACTION", "shape": [12]}},
            }
        ),
        encoding="utf-8",
    )
    assert not rollout.ModelWorker(standard, tmp_path / "standard-out").requires_environment_state
    assert rollout.ModelWorker(force, tmp_path / "force-out").requires_environment_state


def test_model_worker_observation_contract_requires_force_only_for_force_policy() -> None:
    worker = _load_model_worker()
    base = {
        worker.WORKSPACE_KEY: np.zeros((3, 240, 320), dtype=np.float32),
        worker.WRIST_KEY: np.zeros((3, 240, 320), dtype=np.float32),
        worker.STATE_KEY: np.zeros((12,), dtype=np.float32),
    }
    worker._validate_observation(base, requires_environment_state=False)
    with pytest.raises(ValueError, match="environment_state"):
        worker._validate_observation(base, requires_environment_state=True)
    force = dict(base)
    force[worker.ENVIRONMENT_STATE_KEY] = np.zeros((6,), dtype=np.float32)
    worker._validate_observation(force, requires_environment_state=True)
    with pytest.raises(ValueError, match="must not receive"):
        worker._validate_observation(force, requires_environment_state=False)


def test_rollout_command_log_reuses_inference_status_snapshot() -> None:
    rollout = _load_rollout_tool()
    startup = SimpleNamespace(observation_monotonic_ns=10)
    queried = SimpleNamespace(observation_monotonic_ns=20)
    prediction = SimpleNamespace(observation=SimpleNamespace(status=queried))

    assert rollout._command_status_sample(None, startup) is startup
    assert rollout._command_status_sample(prediction, startup) is queried
