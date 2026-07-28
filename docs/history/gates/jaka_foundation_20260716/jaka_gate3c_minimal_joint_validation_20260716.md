# JAKA Gate 3C — minimal predefined joint motion

> **Status: historical snapshot, 2026-07-16.** This result applies only to the
> exact small J6 procedures below and does not authorize current operation.
> See [`docs/operation/hardware_prerequisites.md`](../../../operation/hardware_prerequisites.md).

Date: 2026-07-16  
Current status: 0.25-degree Stage 3C-2 and the separately approved +5-degree
joint-6 validation are successfully completed and physically accepted

## Boundary

Gate 3C is a single predefined joint-space outward-and-return test. It contains
no TeleDex, Cartesian target, Quest input, RH56 access, historical HEBI logic,
filtering, or Ruckig. The first motion is capped at 0.25 degrees and may not
repeat automatically.

The Stage 3C-1 executable is `jaka_gate3c_plan_probe`. Link inspection verifies
that it contains no EDG, servo-mode, power/enable, joint-motion, or Cartesian
motion symbol. Its physical run only logged in, read state, logged out, and
terminated.

## Joint selection

Selected joint: **joint 6 / `jaka_joint_6`**, positive direction.

This is selected because joint 6 rotates the wrist/tool assembly about its own
axis without sweeping the upstream arm links. Joints 1–3 would create much
larger link and TCP translation; joints 4–5 move the wrist/tool axis through
space. Joint 6 minimizes arm-link swept volume, but its cable-twist consequence
means Stage 3C-2 still requires direct operator confirmation that the mounted
assembly and cables have clearance and that positive rotation is understood.

For an axial tool frame, tool-origin translation should be approximately zero
and orientation should change by 0.25 degrees. A point 100 mm radially from the
joint axis has a conservative small-angle sweep bound of 0.436 mm. This is a
geometric estimate, not a measured collision guarantee.

## Stage 3C-1 physical plan

Observed controller state passed fault, power/enable, E-stop, collision,
finiteness, six-joint radians, and tool/user 0/0 checks.

| Item | Plan |
|---|---:|
| Start joint 6 | 0.474237234950 rad / 27.171792 degrees |
| Outward target joint 6 | 0.478600558080 rad / 27.421792 degrees |
| Direction/displacement | positive / 0.004363323130 rad / 0.25 degrees |
| Other target deltas | exactly zero |
| Return target | fresh start vector |
| Start lower/upper safe-limit margin | 382.171792 / 327.828208 degrees |
| Target lower/upper safe-limit margin | 382.421792 / 327.578208 degrees |
| Requested period | 8 ms |
| Outward / hold / return | 2.0 / 0.4 / 2.0 s |
| Expected motion duration | 4.4 s |
| Configured velocity limit | 0.005 rad/s / 0.286479 deg/s |
| Configured acceleration limit | 0.010 rad/s² / 0.572958 deg/s² |
| Configured jerk limit | 0.040 rad/s³ / 2.291831 deg/s³ |
| Tracking-error abort | 0.0005 rad / 0.028648 degrees |

Complete observed start vector:

`[1.56975867764, -0.02833518629, -0.770853166864, 0.505283204717, 0.134604423998, 0.47423723495]` rad.

Complete outward target vector:

`[1.56975867764, -0.02833518629, -0.770853166864, 0.505283204717, 0.134604423998, 0.47860055808]` rad.

The trajectory is an auditable seventh-order state-to-state polynomial
`35s^4 - 84s^5 + 70s^6 - 20s^7`. Position, velocity, acceleration, and jerk are
stationary/zero at both endpoints. Offline numerical maxima are:

- velocity 0.004772385 rad/s;
- acceleration 0.008195615 rad/s²;
- jerk 0.028634308 rad/s³.

All remain below the configured caps. The return uses the same duration and
limits with the displacement sign reversed; it is not a snap-back.

## Validation

- Four Gate 3C plan/trajectory tests passed.
- Default invocation is non-connecting.
- Link inspection found no command-writing symbol in the plan executable.
- Offline trajectory endpoints and velocity/acceleration/jerk bounds passed.
- Physical Stage 3C-1 outcome: completed; EDG entries 0, servo changes 0,
  commands 0, logout code 0.

Machine-readable plan:
`docs/gate3c_measurements/jaka_gate3c_stage1_plan_20260716.json`.

## Required Stage 3C-2 decision

No motion is authorized by this report. Before Stage 3C-2, the operator must
explicitly confirm all of the following at the current robot pose:

- emergency stop accessible;
- workspace clear and no person inside it;
- RH56 assembly and all cables clear for positive joint-6 rotation;
- joint 6 and the expected positive direction are understood;
- ready to interrupt immediately;
- approval phrase `I_APPROVE_GATE3C_STAGE_2_0_25_DEGREE_MOTION`.

After approval, the motion-capable native executable must repeat the countdown,
capture a fresh start vector, rebuild the target from that vector, and recheck
the plan. The historical numbers above are review evidence only and must not be
used as the command target.

## Stage 3C-2 physical result

The approved single outward/hold/return sequence completed all 551 planned
commands without automatic repetition. Fresh start, outward, and return vectors
were respectively:

- `[1.56975867764, -0.02833518629, -0.770853166864, 0.505283204717, 0.134604423998, 0.47423723495]` rad;
- `[1.56975867764, -0.02833518629, -0.770853166864, 0.505283204717, 0.134604423998, 0.47860055808]` rad;
- the exact fresh start vector.

Final EDG observation was
`[1.56974910247, -0.0283092399844, -0.770824650994, 0.505272809833, 0.134599789618, 0.474467749061]` rad.

| Measurement | Result |
|---|---:|
| Commands | 551 / 551 |
| Observed outward joint-6 displacement | 0.004227318 rad / 0.242207 degrees |
| Final maximum return error | 0.000230514 rad / 0.013207 degrees |
| Peak tracking error | 0.000492149 rad / 0.028198 degrees |
| Tracking abort threshold | 0.000500000 rad / 0.028648 degrees |
| Timing warnings / hard misses | 0 / 0 |
| Completion / period misses | 0 / 0 |
| Period mean / p95 / p99 / max | 8.000102 / 8.006001 / 8.010866 / 8.106149 ms |
| Wake mean / p99 / max | 0.057996 / 0.068193 / 0.161546 ms |
| EDG read mean / p99 / max | 0.014312 / 0.026570 / 0.063547 ms |
| Command mean / p99 / max | 0.033819 / 0.056792 / 0.490502 ms |
| Servo disable / EDG exit / logout | code 0 / 0 / 0 |

Observed per-joint spans were
`[0.0000200, 0.0000873, 0.0000520, 0, 0, 0.0042420]` rad. Non-selected joint
targets remained exactly invariant; their small observed spans are encoder and
normal holding observations.

The peak tracking error passed by only 7.851 microradians. This is not an abort,
but the small margin argues against increasing displacement or moving to
Cartesian validation before reviewing following lag and tracking-error policy.

The trajectory CSV contains every commanded position, velocity, acceleration,
preceding EDG observation, tracking error, period, wake lateness, state-read
duration, and command duration. Its numeric stream used approximately six
significant digits, a recording limitation that does not affect the internal
full-precision safety calculations. No rerun is authorized for this limitation.

Machine-readable artifacts:

- `docs/gate3c_measurements/jaka_gate3c_stage2_motion_20260716.json`;
- `docs/gate3c_measurements/jaka_gate3c_stage2_trajectory_20260716.csv`.

The operator subsequently accepted Stage 3C-2 as a successful minimal-motion
validation and authorized a separate +5-degree joint-6 test under a revised
velocity-aware tracking policy. Its plan, physical software telemetry, and
accepted operator observation are recorded in
`docs/jaka_gate3c_5degree_joint6_plan_20260716.md`.
