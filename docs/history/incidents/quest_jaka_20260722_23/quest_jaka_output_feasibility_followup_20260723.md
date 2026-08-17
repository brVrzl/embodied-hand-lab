# Quest-to-JAKA AcceptedArmTarget output-feasibility follow-up — 2026-07-23

> **Status: historical snapshot, 2026-07-23.** This document preserves the
> incident diagnosis before the later `root_cause_fix` production baseline. It
> is not the current operating guide. See
> [`docs/status/current_status.md`](../../../status/current_status.md).

## Scope and confirmed root cause

This checkpoint is offline-only. No JAKA connection, servo enable, EDG entry,
Quest-controlled physical motion, RH56 command, payload change, or frame change
was performed.

The stopped P4 run accepted sequence 214 even though its J6 transition was not
representable by the approved JAKA ServoJ contract:

| evidence | value |
|---|---:|
| accepted interval | 0.016667076 s |
| J6 transition | 0.467537878 to 0.395653276 rad |
| direct accepted-target velocity | -4.312970188 rad/s |
| native observed/predicted preemption-aware velocity | about -4.37 rad/s |
| approved output boundary | pi rad/s |

`maximum_ik_target_velocity_rad_s: 14.0` was and remains a numerical IK
pathology/branch-continuity guard. It is deliberately loose enough to identify
large unsmoothed IK jumps and is not an actuator or ServoJ permission. Before
this change, `command_maximum_joint_velocity_rad_s` was a MuJoCo post-adapter
plant-command shaping parameter, while the native worker independently received
the same pi value as a final defensive check. No pre-adapter acceptance rule
consumed that physical output contract. That semantic gap was the defect.

The new sole pre-adapter authority is:

```yaml
shared_target_generation:
  maximum_output_joint_velocity_rad_s: 3.141592653589793
```

Both shared simulation and plant-free hardware generation consume it. The
hardware launcher passes the same loaded value to the native defensive validator.
The MuJoCo command-trajectory limits remain simulation plant behavior after the
immutable adapter boundary and do not decide whether an AcceptedArmTarget exists.

## Timestamp and 8 ms prediction contract

The feasibility time domain is `AcceptedArmTarget.generated_monotonic_ns`, a
local host `CLOCK_MONOTONIC` timestamp. Raw Quest timestamps, nominal 1/60 s,
viewer timing, and a fixed 8 ms candidate interval are not used for adjacent
accepted targets.

The dependency-neutral `JointOutputFeasibilityTracker` mirrors the native
latest-segment policy with bounded state:

1. Advance the preceding active segment only to the most recent possible 8 ms
   deadline before the new generated timestamp.
2. Use that virtual last-emitted J1-J6 point as the replacement segment start.
3. Use the actual adjacent accepted-generation timestamp interval as the new
   segment duration (8 ms only for the first aligned target).
4. Compute each segment slope from that start to the candidate endpoint.
5. Treat the candidate as feasible at or below pi plus the shared 1e-12 rad/s
   floating-point comparison tolerance.

This includes the small residual left by active-segment preemption. For the
recorded sequence 213 to 214, the direct J6 calculation is -4.31297 rad/s and
the preemption-aware contract prediction is about -4.37 rad/s. The native
worker still computes diagnostics from actual command timestamps. If its final
check fires for a shared-valid stream, it is now labelled an internal
output-feasibility contract violation rather than an ordinary operator rejection.

The failed run's concentrated J6 change followed continuous full-orientation IK:
the filtered TCP orientation changed about three degrees in that 60 Hz tick and
coupled into J4/J6. There was no IK branch switch, singularity, timing pause, or
resampler timestamp error.

## Acceptance, continuation, hold, and timeout semantics

The output check runs inside `SharedJakaTargetGenerator.evaluate()` before it
commits the candidate joint/TCP continuation state. A failed full candidate has
reason `OUTPUT_VELOCITY_INFEASIBLE` and cannot construct an
`AcceptedArmTarget`. The existing coupled SE(3) continuation loop then retries
smaller fractions; it is still the only target-step reduction authority.

If a configured fraction is feasible, that trial becomes the next immutable
target. Nothing is clamped or rescaled after acceptance. If no fraction is
feasible, the existing shared rejection contract publishes
`ArmControlHeartbeat(state=HOLD_REJECTED, reason=OUTPUT_VELOCITY_INFEASIBLE)`.
The accepted payload remains unchanged, the native resampler emits the last safe
point, and fresh input continues to be evaluated. Recovery uses the same
reference, last accepted IK seed, and last emitted point; it does not recapture
or restart.

`command_stream_timeout_ms: 100` is producer/IPC liveness, not target-motion
age. Either a new immutable target or a fresh explicit HOLD_REJECTED heartbeat
updates native liveness. Stopping the producer heartbeat still produces
`command_stream_timeout`; stale Quest input, tracking loss, clutch release,
explicit STOP, SDK/controller faults, and IPC failure retain their existing stop
paths. A held stationary target does not refresh itself from stale Quest data.

## Exact failed-P4 replay

Evidence files:

- `docs/measurements/quest_jaka_p4_output_feasibility_replay_20260723.json`
- `docs/measurements/quest_jaka_p4_output_feasibility_native_fake_20260723.json`
- `docs/measurements/quest_jaka_p4_output_feasibility_native_fake_emitted_20260723.jsonl`

Physical-start replay results:

| metric | result |
|---|---:|
| active shared ticks | 219 |
| accepted ticks | 219 |
| full candidates backtracked for output feasibility | 6 |
| total continuation backtracks | 11 |
| final HOLD_REJECTED ticks | 0 |
| accepted output-contract violations | 0 |
| branch switches | 0 |
| maximum accepted predicted output velocity | 2.200326 rad/s |
| maximum producer publication gap | 17.396 ms |

At recorded sequence 214, fraction 1.0 was rejected as
`OUTPUT_VELOCITY_INFEASIBLE`; fraction 0.5 was accepted. Its predicted J6 output
velocity was -2.200326 rad/s. Sequence 215 was accepted continuously at fraction
0.25, so the replay proves continued progress/recovery without process restart.

The corrected 219-point stream was then sent, with its exact recorded intervals,
through the fake native 8 ms worker. It accepted all 219 targets, made zero IK
calls, produced zero native speed-boundary rejections, exited on explicit STOP,
and reported error/cleanup code 0. Maximum native emitted J6 velocity was
2.202230 rad/s. Thus the native defensive assertion no longer terminates this
recoverable stream.

## Simulation parity impact

The same recorded TCP requests replayed from the configured successful MuJoCo
initial posture produced 219/219 accepted ticks, zero output-feasibility
backtracks, zero branch switches, and a maximum predicted output velocity of
1.837843 rad/s. The new rule therefore changed none of that successful case's
AcceptedArmTargets. It did change the physical-start replay at the six ticks
whose full IK candidates exceeded the shared output contract. This is the
intended pre-adapter parity: simulation and hardware given the same initial
J1-J6 and input make the same acceptance/backtracking decision.

## Remaining physical uncertainty and next gate

Offline replay cannot prove physical tracking, controller-version behavior,
payload/gravity configuration, or the effect of real scheduling jitter on
measured joint error. A bounded live validation should therefore repeat only the
previously successful startup and one gentle forward/return plus modest wrist
motion for 30 seconds. It should verify that shared continuation records
`OUTPUT_VELOCITY_INFEASIBLE` attempts when needed, the native defensive count
stays zero, tracking remains bounded, clutch release stops, and cleanup succeeds.
It must require a new explicit operator authorization and is not executed by
this checkpoint.

## Offline verification

Commands run:

```text
.venv/bin/python -m pytest tests/test_quest_jaka_output_feasibility.py tests/test_quest_jaka_shared_pipeline.py tests/test_quest_jaka_singularity_liveness.py tests/test_jaka_edg_resampler.py tests/test_native_jaka_servo_worker.py -q
.venv/bin/python -m pytest -q
.venv/bin/python -m compileall -q src tools tests
cmake -S native/jaka_servo_worker -B build/jaka_servo_worker -DCMAKE_BUILD_TYPE=Release
cmake --build build/jaka_servo_worker -j2
git diff --check
```

The focused contract/continuation/parity/singularity/resampler/native set passed
85 tests. The repository contains 619 tests plus one skip: every test passed
across the verification run and isolated reruns. The full combined invocation
reported 618 passed, one skipped, and one unrelated native real-time scheduling
probe failure; that single failed probe passed immediately in isolation. Earlier
combined attempts showed the same host-scheduling class in different historical
zero/minimal-motion probes, each passing immediately alone. No task-owned test
failed after its correction. Compileall, the native Release build, and
`git diff --check` passed.
