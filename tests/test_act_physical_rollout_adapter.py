from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys
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


def test_rh56_projection_is_boundary_only_and_reports_each_changed_channel() -> None:
    project_rh56_command = _load_rollout_tool()._project_rh56_command
    raw = np.asarray(
        [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, -0.00054, 0.2, 1.2, 0.4, -0.01, 0.8],
        dtype=np.float64,
    )
    projected, events = project_rh56_command(raw, legal_min=0.0, legal_max=1.0)
    assert np.array_equal(raw[:6], projected[:6])
    assert projected[6:].tolist() == [0.0, 0.2, 1.0, 0.4, 0.0, 0.8]
    assert [(event["channel"], event["raw_value"], event["projected_value"]) for event in events] == [
        ("index", -0.00054, 0.0),
        ("ring", 1.2, 1.0),
        ("thumb_close", -0.01, 0.0),
    ]
    assert raw[6] == pytest.approx(-0.00054)


def test_rollout_model_worker_detects_force_checkpoint_input(tmp_path: Path) -> None:
    rollout = _load_rollout_tool()
    standard = tmp_path / "standard"
    standard.mkdir()
    (standard / "config.json").write_text(
        json.dumps({"input_features": {"observation.state": {"shape": [12]}}}),
        encoding="utf-8",
    )
    force = tmp_path / "force"
    force.mkdir()
    (force / "config.json").write_text(
        json.dumps(
            {
                "input_features": {
                    "observation.state": {"shape": [12]},
                    "observation.environment_state": {"shape": [6]},
                }
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
