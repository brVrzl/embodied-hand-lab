# Quest-to-JAKA singularity and rejected-target liveness follow-up (2026-07-23)

This is an offline-only follow-up to checkpoint `44424bf9e5dcec8a9415fa0d18a85e1390b6a2cd`.
No JAKA connection, EDG entry, servo enable, Quest-controlled physical motion, RH56 command,
payload write, tool-frame write, or user-frame write was performed.

## Exact cause and simulation/hardware audit

The successful simulation command is:

```bash
./scripts/run_quest_jaka_sim_demo.sh --viewer
# -> .venv/bin/python tools/quest_jaka_mujoco_sim.py live-6dof \
#      --config configs/sim/quest_hts_jaka_mini2_live_demo.yaml --viewer
```

The hardware commands are `tools/quest_jaka_hardware.py e2-isolated` and
`tools/quest_jaka_hardware.py p4-live`. All three construct the same `ReplayConfig`,
`SmoothQuestJakaSession`, `SharedJakaTargetGenerator` policy and continuation logic.
There was no simulation-specific override of `minimum_wrist_bend_deg`, condition 60,
sigma 0.0125, or the five-step continuation policy. No environment variable or second
hardware config supplied a different singularity gate.

Simulation avoided the observed stop for two independent, concrete reasons:

1. Its configured initial J5 is +65 degrees. The manual_02 physical start was J5
   -23.83 degrees and the commanded branch approached -15 degrees.
2. A rejected simulation candidate simply held the last target; simulation has no native
   100 ms producer watchdog. Hardware emitted no packet on rejection, so seven fixed-J5
   rejections (41 backtracks) made valid target silence look like producer death and the
   native worker returned `command_stream_timeout`.

The direct failure was therefore not an IK solver failure, branch switch, JAKA alarm,
tracking loss, clutch release, speed violation, or EDG timing failure.

## Model scan and threshold justification

The committed Mini2 MJCF was scanned with the manual_02 starting joint vector while J5
was swept around zero. The spatial Jacobian uses the existing 0.25 m rotation characteristic
length. Representative values are:

| J5 (deg) | condition | sigma min | wrist-axis abs cosine | max dq for 1 deg roll/yaw (rad) |
|---:|---:|---:|---:|---:|
| -20 | 23.97 | 0.02942 | 0.9397 | 0.0537 / 0.0688 |
| -15 | 32.13 | 0.02219 | 0.9659 | 0.0707 / 0.0926 |
| -14.968 | 32.20 | 0.02214 | 0.9661 | 0.0708 / 0.0928 |
| -12 | 40.29 | 0.01780 | 0.9781 | 0.0880 / 0.1162 |
| -10 | 48.43 | 0.01486 | 0.9848 | 0.1052 / 0.1396 |
| -8 | 60.65 | 0.01191 | 0.9903 | 0.1308 / 0.1744 |
| -5 | 97.28 | 0.00746 | 0.9962 | 0.2070 / 0.2782 |
| -1 | 487.81 | 0.00150 | 0.9998 | 1.0152 / 1.3789 |
| 0 | 5.44e7 | ~0 | 1.0000 | unbounded/model singular |

This scan does not establish physical safety. It does show that J5=-14.968 degrees is
geometrically near the wrist alignment but remains comfortably inside both already-approved
Jacobian limits. The existing actual hard limits become active around |J5|=8 degrees for
this posture, and the loss is direction-specific: roll/yaw joint demand grows much faster
than pitch or some translations. Full singular values, manipulability, +/-XYZ and
+/-roll/pitch/yaw pseudoinverse demand, per-direction IK convergence and branch results are
in `docs/measurements/quest_jaka_manual02_singularity_audit_20260723.json`.

## New shared policy

- `wrist_proximity_warning_deg: 15` is warning-only diagnostic metadata. The compatibility
  reader maps the old name to this warning semantic; it can never independently reject.
- Slowdown/backtracking starts at condition 48 or sigma 0.015625.
- Hard candidate rejection remains condition >60 or sigma <0.0125.
- Recovery is condition <=45 and sigma >=0.016875. This hysteresis prevents chatter.
- Risk is `max(condition/60, 0.0125/sigma)`. Candidate risk relative to the last accepted
  state classifies motion as `TOWARD`, `TANGENT`, or `AWAY` with a 0.01 dead band.
- `TOWARD` motion in the slowdown region is backtracked/held. `TANGENT` and `AWAY` motion
  remain eligible for all ordinary IK, joint, collision and continuity checks.
- A hard candidate is not accepted. If the authoritative current state is itself already
  beyond the hard boundary, an improving `AWAY` candidate is permitted to retreat; tangent
  or worsening motion is a shared `HARD_STOP` condition.
- IK continues to seed exclusively from the previous accepted J1-J6. No rejected candidate
  changes that seed, and no JAKA-side IK was added.
- DLS damping was previously fixed at 0.05. It now rises with a smoothstep from 0.05 to 0.10
  as the solver-weighted sigma moves from 0.025 to 0.0125. This is solver robustness, not a
  post-target trajectory shaper.

## HOLD_REJECTED and timeout contract

An immutable `AcceptedArmTarget` remains the only object allowed to change the joint target.
A recoverable rejection does not fabricate one. Instead the shared session publishes an
immutable `ArmControlHeartbeat(state=HOLD_REJECTED, reason=...)` while retaining the last
accepted target and reference generation. The JAKA representation adapter serializes that
heartbeat without q values; the native worker refreshes producer liveness but does not call
IK or `resampler.accept`. It continues the last safe point on the 8 ms grid.

Timeout meanings are now distinct:

| Signal | Contract |
|---|---|
| producer heartbeat / IPC age | 100 ms; accepted target or explicit heartbeat refreshes it; expiry is a safe `command_stream_timeout` |
| accepted-target age | diagnostic only while a healthy heartbeat reports `HOLD_REJECTED`; stationary safe targets do not expire |
| Quest wrist age | 250 ms shared input policy; stale tracking disengages/stops and cannot refresh heartbeat with recorded data |
| clutch/controller age | 150 ms; release/stale faults stop normally |
| target socket/IPC error | immediate native transport fault |
| controller/state/SDK error | existing immediate fault/cleanup paths |
| controlled/fatal legacy transport bounds | 500/2000 ms remain for their existing non-joint modes; P4 joint mode uses the explicit 100 ms heartbeat contract |

Clutch release, Quest tracking loss, malformed input, IPC failure, operator stop, SDK fault,
controller fault and current-state hard singularity still stop. A healthy rejected candidate
does not. Recovery uses the same reference and last accepted seed, so it introduces no
reference recapture or branch jump.

## manual_02 replay

The replay consumes every recorded mapped desired TCP and original control timestamp from
`logs/quest_jaka_p4_manual_02.jsonl`, which was produced from
`logs/quest_jaka_p4_manual_02_capture.jsonl`. For the simulation-start comparison the same
relative TCP stream is re-anchored to the configured simulation TCP, matching fresh-reference
semantics.

| Case | Accepted | Rejected | Backtracks | Branch switches | max IK target velocity |
|---|---:|---:|---:|---:|---:|
| recorded old physical run | 228 | 7 fixed-J5 `NEAR_SINGULARITY` | 41 | 0 | no violation recorded |
| corrected physical start | 235/235 | 0 | 0 | 0 | 1.683 rad/s |
| corrected simulation start | 235/235 | 0 | 0 | 0 | 0.520 rad/s |

The corrected physical replay has no hard rejection at J5=-14.968 degrees, no producer gap
near 100 ms (maximum tick/publication gap 43.404 ms), no pi-rad/s diagnostic boundary
crossing, no endpoint corruption and no branch switch. A synthetic one-degree J5 retreat
is accepted from the held state without restart, speed-boundary violation, or branch switch. With the same
physical starting J1-J6, MuJoCo and plant-free generators have exact tick-by-tick trace
equality before the adapter boundary.

The native fake-worker regression separately sends a fixed accepted target followed by six
`HOLD_REJECTED` heartbeats. It emits unchanged 8 ms points, performs zero IK calls, does not
timeout, and stops only on explicit STOP. Existing heartbeat-loss tests still end in
`command_stream_timeout`.

## Offline verification result

- Focused singularity, shared-pipeline parity, hardware-adapter, native worker and EDG
  resampler set: `101 passed`.
- Native worker Release build: passed.
- `compileall`: passed.
- `git diff --check`: passed.
- Full repository invocation was run three times: each completed with `611 passed, 1 skipped,
  1 failed`. The first two failures were different cases in the unmodified historical
  `tests/test_jaka_zero_motion_probe.py`; the final post-checkpoint run failed the equally
  unmodified `tests/test_jaka_minimal_joint_probe.py` fake lifecycle. Every failure was a
  strict host-scheduling deadline check and each failing case passed immediately in isolation.
  A complete isolated zero-motion file run produced `25 passed, 1 failed` from the same
  nondeterministic hard-period check. No task path failed. The unrelated probe thresholds/code
  were not weakened or changed.

## Remaining physical uncertainty and next gate

Offline replay cannot validate the physical Mini2 Jacobian/model calibration, joint-zero
offsets, installed payload/CoM, mounting orientation, active TCP/user frame, encoder behavior,
true EDG lag, or controller-version alarm thresholds. Those remain separate from target and
liveness parity.

The next recommended gate is one bounded live recovery check only: start from a known healthy
posture, engage jump-free, approach the warning region slowly with one wrist-related axis,
observe `PROXIMITY` then (if reached) `HOLD_REJECTED`, deliberately retreat, verify recovery
without restart, release clutch, and stop. Do not combine axes or cross the hard Jacobian
boundary. This document does not authorize or execute that gate.
