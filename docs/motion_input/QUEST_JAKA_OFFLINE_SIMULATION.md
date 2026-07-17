# Quest right hand to JAKA MuJoCo offline integration

Status: **offline recorded-input gate PASS**. The mapping is explicitly
uncalibrated and simulation-only. No physical JAKA or Inspire connection,
login, enable, SDK session, or command occurred.

## Integration checkpoint and ownership

- Quest input checkpoint: `7f4036eaffffde74c5ccb2698734e7c68094673d`
- committed JAKA/RH56 MuJoCo foundation in its ancestry:
  `6faa64b3776aa536ba699fe4967956f34e0865b5`
- branch: `feature/quest-jaka-offline-simulation`
- worktree: `/home/thor/projects/embodied_lab_quest_jaka_sim`
- base MJCF: `data/sim_assets/jaka_rh56_visual_coacd.xml`
- existing IK: `jaka_driver_adapter.palm_target_ik.PalmTargetIkState`

The hardware-control commits `bc15d55` and `52b67fa` were inspected but not
merged. They add native/live JAKA and TeleDex paths that are unnecessary for
this offline gate. The selected Quest commit already contains the current
JAKA/RH56 MuJoCo model, position actuators, joint limits, and damped-least-
squares IK through its mainline ancestry.

Task-owned files:

- `configs/sim/quest_hts_jaka_mini2_offline.yaml`
- `src/quest_jaka_sim/__init__.py`
- `src/quest_jaka_sim/mapping.py`
- `src/quest_jaka_sim/simulation.py`
- `tools/quest_jaka_mujoco_sim.py`
- `tests/test_quest_jaka_sim.py`
- this document

Runtime JSON/JSONL evidence and the generated marker MJCF are ignored under
`logs/quest_jaka_sim/`.

## Provisional frame chain

```text
raw HTS Unity world
  -> existing Unity/OpenXR-style conversion
quest_world
  -> explicit right-wrist reference capture
canonical_operator relative delta
  -> quest_jaka_sim_uncalibrated_v1
future_robot_base
  -> captured simulated JAKA TCP reference
desired_robot_tcp
```

`quest_jaka_sim_uncalibrated_v1` is a versioned simulation hypothesis, not a
physical Quest-to-JAKA calibration. Its configurable basis rows are robot-base
X/Y/Z and columns are canonical-operator X/Y/Z:

```text
[ 1  0  0 ]   operator X -> robot X
[ 0  0  1 ]   operator Z -> robot Y
[ 0  1  0 ]   operator Y -> robot Z
```

The per-operator-axis translation scales are `[0.04, 0.03, 0.04]`. Translation
deadband is 1 mm in operator delta, maximum operator displacement is 300 mm,
and maximum desired TCP displacement from the captured simulated reference is
15 mm. Orientation following is disabled. Configuration exposes orientation
scale and deadband for later offline work, and quaternion/basis tests cover
identity and 90-degree rotations, but no orientation target is sent in this
gate.

At each explicit engagement, the Quest pipeline first enters
`ARMED_REFERENCE_CAPTURE`. A subsequent fresh 21-joint right-hand sample
captures both the operator reference and current simulated JAKA TCP, clears
derivative history, and emits zero displacement.

## Offline feasibility and simulation behavior

Each desired target is evaluated using the existing MuJoCo JAKA model and IK:

- 15 mm target envelope and 4 mm per-sample target-jump limit;
- DLS IK with 2.5 mm position tolerance;
- existing Mini2 joint limits with a 5-degree margin;
- translational Jacobian condition number <= 40 and minimum singular value >=
  0.02;
- TCP velocity <= 25 mm/s;
- joint velocity <= 1.2 rad/s and joint acceleration <= 20 rad/s²;
- new self/environment contacts relative to the captured model baseline.

The current model exposes no validated signed-distance scene query, so minimum
collision distance is `null` when no new contact exists. Only the model floor
is an environment; no validated task scene is claimed.

The simulated plant uses the model's existing position actuators and normal
`mj_step` calls at the 2 ms model timestep. It does not teleport joints for the
dynamic tracking result. Zero gravity is enabled because the existing low-gain
shadow actuators otherwise sag until the RH56 touches the floor; this makes the
viewer a bounded actuator-response visualization, **not** validated physical
dynamics. IK uses a separate scratch `MjData`, so rejected candidates never
alter the plant.

Any mapping or feasibility rejection causes `DISENGAGED`, invalidates both
references, and holds the last safe simulated actuator target. Recovery alone
never resumes output.

Structured reasons are:

```text
INPUT_INVALID, DISENGAGED, OUTSIDE_OPERATOR_ENVELOPE, TARGET_JUMP,
OUTSIDE_ROBOT_WORKSPACE, IK_FAILED, JOINT_LIMIT, NEAR_SINGULARITY,
VELOCITY_LIMIT, ACCELERATION_LIMIT, SELF_COLLISION, ENVIRONMENT_COLLISION
```

## Reproducible recorded gate

Headless deterministic replay:

```bash
cd /home/thor/projects/embodied_lab_quest_jaka_sim
PYTHONPATH=src /home/thor/projects/embodied_lab/.venv/bin/python \
  tools/quest_jaka_mujoco_sim.py replay \
  /home/thor/projects/embodied_lab_quest_input/logs/quest_input/quest_live_retry_20260717T1704+0800.hts.jsonl \
  --report logs/quest_jaka_sim/main_motion.report.json
```

Full recorded-time viewer replay on the external display:

```bash
DISPLAY=:1 PYTHONPATH=src /home/thor/projects/embodied_lab/.venv/bin/python \
  tools/quest_jaka_mujoco_sim.py replay \
  /home/thor/projects/embodied_lab_quest_input/logs/quest_input/quest_live_retry_20260717T1704+0800.hts.jsonl \
  --viewer --realtime \
  --report logs/quest_jaka_sim/viewer_realtime.report.json
```

For the bounded validation run, `--realtime-from-sec 69` fast-forwarded setup
and displayed the complete recorded motion segment in real time. The passive
MuJoCo viewer showed the existing JAKA+RH56 visual model, a blue desired-TCP
marker, a green simulated-TCP marker, state, right-hand validity, rejection
reason, and desired-to-simulated tracking error.

Machine-readable results from the headless gate:

| Metric | Observed |
|---|---:|
| right-hand frames / valid 21-joint frames | 6,064 / 6,064 |
| invalid-input events | 1 |
| accepted desired targets | 367 |
| IK attempts / IK successes | 368 / 368 |
| IK success rate | 100% |
| maximum Jacobian condition number | 4.771 |
| minimum translational singular value | 0.1260 |
| maximum desired TCP displacement | 4.765 mm |
| maximum desired TCP velocity | 23.203 mm/s |
| maximum joint velocity | 0.1465 rad/s |
| maximum accepted joint acceleration | 19.704 rad/s² |
| maximum desired-to-simulated TCP error | 3.470 mm |
| minimum new-contact distance | unavailable (`null`; no new contacts accepted) |

The startup hand-stream interruption occurred after the first explicit
reference capture. It produced `ENGAGED -> DISENGAGED` with reason
`right_hand_stale`. The recovered stream remained disengaged for about 68
seconds until the separately configured engagement at 69 seconds. Near the end
of the forward/back segment, one candidate exceeded the conservative joint-
acceleration threshold, was rejected as `ACCELERATION_LIMIT`, and disengaged
the pipeline. The simulated plant held its last safe actuator target.

## Empirical axis response

Axis findings use accepted desired targets in the operator-commanded recording
windows, not coordinate assumptions:

- right-hand left/right (71–73 s): dominant robot-base **X**, positive sign,
  4.887 mm X range versus 0.604/0.751 mm Y/Z; correlation 0.9999996. Hand
  right (+canonical X) produces +robot-base X;
- right-hand forward/back (73–75 s): dominant robot-base **Y**, positive sign,
  3.943 mm Y range versus 1.257/0.520 mm X/Z; correlation approximately 1.0.
  Because hand forward is -canonical Z, it produces -robot-base Y;
- up/down: not independently present/identified in the capture, so robot-base Z
  remains unvalidated.

These findings validate the configured simulation response only. They do not
establish a physical direction calibration.

## Prepared live Quest-to-simulation gate

The same executable supports a live Quest-only UDP receiver and the same
MuJoCo mapper, feasibility checks, actuator stepping, markers, overlay, raw
capture, event log, and report. This command is prepared but is not part of the
recorded-input checkpoint:

```bash
DISPLAY=:1 PYTHONPATH=src /home/thor/projects/embodied_lab/.venv/bin/python \
  tools/quest_jaka_mujoco_sim.py live \
  --project-ip 10.24.1.68 --port 9000 --allowed-sender 10.24.0.78 \
  --duration-sec 120
```

Press SPACE once with a fresh tracked right hand to arm, then SPACE again on a
fresh sample to capture both references. A third press disengages. Tracking
loss or target rejection also disengages. The live mode contains no JAKA SDK,
ROS control, RH56 driver, or physical backend import.

## Remaining unknowns and next gate

- physical operator-to-robot calibration is absent;
- robot-base Z response needs an isolated recorded/live up-down motion;
- orientation following is disabled and unvalidated;
- zero-gravity actuator response is not physical dynamics validation;
- signed collision clearance and a validated environment scene are absent;
- live Quest-to-simulation latency and interactive direction response remain
  for the next Quest-only gate.

No JAKA or Inspire hardware was connected or commanded during audit,
implementation, testing, headless replay, or viewer replay.
