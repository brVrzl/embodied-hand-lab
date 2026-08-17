# JAKA Gate 3B — zero-motion EDG validation

> **Status: historical snapshot, 2026-07-16.** The PASS is limited to this
> zero-motion gate and does not authorize current hardware operation. See
> [`docs/status/validation_matrix.md`](../../../status/validation_matrix.md).

Date: 2026-07-16  
Current status: Gate 3B accepted complete after clean timing-corrected Stage 5
and clean operator physical observation

## Scope boundary

This gate validates one disposable native process, one SDK client, joint-space
EDG lifecycle, and an invariant joint target captured from current state. It
does not contain TeleDex, Cartesian targets, IK, filtering, trajectory
generation, motion scaling, clutching, recentering, minimal motion, or hand
control.

The probe is isolated in `native/jaka_zero_motion_probe`. Default invocation is
a non-connecting dry-run and reports zero EDG entries and zero commands.

## Gate 3A findings carried forward

- The deprecated combined status getter is absent from the executable and from
  the cyclic path. It cannot add its observed 20–66 ms latency to the loop.
- The probe constructs one SDK client in one disposable process. It never
  reconnects or reuses the process.
- The SDK heartbeat-thread CPU cost is included in process metrics.
- Command-capable cleanup disables process-owned servo-move mode, exits EDG,
  and logs out, then the executable terminates;
  it does not assume logout ends SDK-owned threads.
- Python is absent from the control loop.

## Restricted production API

The entry/exit-only binary is link-time restricted to the following JAKA
symbols beyond constructor and destructor:

- login/logout;
- SDK version, simple status, robot state, E-stop, collision, servo state,
  tool/user IDs, and actual joint position;
- `edg_init` and `edg_get_stat`.

There is no servo-enable/disable call, joint/Cartesian target, combined status
call, TCP getter,
Cartesian servo call, general movement, program control, frame write, collision
configuration, or IO API.

The separately compiled command-capable binary additionally exposes only
`servo_move_enable` and `edg_servo_j`. It owns a paired servo-mode
enable/disable lifecycle and still excludes Cartesian/general motion and all
unrelated write APIs.

## Predeclared fail-fast thresholds

These constants are compiled and cannot be relaxed through the physical CLI:

| Condition | Threshold and action |
|---|---|
| Requested period | 8,000,000 ns |
| Hard wake lateness | >2,000,000 ns: stop before another command |
| Start-period overrun | >8,800,000 ns: record |
| Consecutive period overruns | 2: stop before another command |
| Completion miss | First completion beyond next release: stop |
| SDK read/command failure | First failure: stop, no retry |
| Invalid EDG state/target | First occurrence: stop |
| Signal/operator interrupt | Stop request, cease commands, EDG exit, logout |
| Initial standard-state-to-EDG-state delta | Default ≤0.000001 rad; configurable only within (0, 0.0001] rad |

The issued command is a value copy of the captured invariant target on every
cycle. Maximum intentional delta is calculated for every issued command.
Observed encoder delta is measured separately and is not called intentional
motion.

## Staged physical gates

| Stage | Capability | Status |
|---|---|---|
| 1 | Release compile, static inspection, fake lifecycle/failures | Completed |
| 2 | Physical login, preflight, capture/print invariant target, logout | Completed; no EDG and no command |
| 3 | EDG enter, one EDG state read, immediate EDG exit, logout; no command | Accepted: SDK lifecycle clean and operator observed no motion, abnormal sound/vibration, collision indication, or alarm |
| 4 | One-second invariant command, maximum 125 commands | Accepted: 125 commands, clean cleanup/timing, and no operator-observed physical anomaly |
| 5 | Five-second invariant command, maximum 625 commands | Timing-corrected retry completed 625 commands and clean cleanup; operator observation pending |

Stages 3–5 require the previous machine-readable physical result. A later stage
cannot be selected without that receipt. Every physical stage also requires:

- explicit vendor backend and physical-hardware flag;
- exact zero-motion acknowledgement and stage-specific approval phrase;
- E-stop-access and clear-workspace flags;
- explicit controller and local EDG-state IPv4 addresses;
- explicit radians unit and six-joint configuration;
- explicit expected tool and user frame IDs;
- explicit result file; cyclic stages additionally require raw timing output.

The Stage 3 executable never calls `servo_move_enable` and requires the mode to
remain inactive. The command-capable Stage 4/5 executable requires the mode to
be initially inactive, then owns `servo_move_enable(true/false)` between EDG
entry and exit. This boundary was fake-tested but has not been physically run.

## Stage 1 validation

After the state-preparation correction, the Gate 3B suite passed 24 tests. Nine
additional native-worker and import-isolation tests also passed.

Coverage includes:

- invariant target construction and exact zero intentional delta;
- finite values, joint count, radians, frame IDs, and initial delta;
- EDG entry/read/command/exit and logout failures;
- first completion miss and repeated start-period overrun;
- signal stop and cleanup order;
- fixed 700-sample stores and a complete 625-cycle fake run;
- physical flags and prior-stage receipt enforcement;
- absence of slow status, Cartesian, TeleDex, and hand dependencies.

Fake tests validate software lifecycle only. No physical connection, EDG entry,
or physical command occurred in Stage 1. The machine-readable Stage 1 result is
`docs/jaka_gate3b_stage1_result_20260716.json`.

## Pending physical records after the first blocked attempt

### Stage 2 — physical read-only preflight

Authorized setup:

- controller: `192.168.71.50`;
- local EDG-state address reserved for later stages: `192.168.71.19`;
- expected tool/user IDs: `0/0`;
- joint count/unit: six/radians;
- E-stop accessible and workspace clear, explicitly confirmed by operator.

The disposable process exited 0 with lifecycle trace
`login,preflight,logout`. Cycle count, EDG reads, EDG commands, EDG entry, and
EDG exit were all zero. No motion observation is claimed by software; no command
was issued.

Directly observed state:

| Field | Value |
|---|---|
| SDK version | `libadd jakaAPI_version: V2.2.7stable_linux` |
| Fault code | 0 |
| Powered / enabled | true / true |
| Emergency stop | false |
| Collision | false |
| Servo-move active | false |
| Tool / user ID | 0 / 0 |
| Captured invariant target | `[1.56975867764, -0.028407091638, -0.770925072212, 0.505283204717, 0.134604423998, 0.47423723495]` rad |

Timing:

| Measurement | Result |
|---|---:|
| Combined SDK login/connection | 166.316952 ms |
| Read-only preflight batch | 17.676769 ms |
| Logout | 50.210761 ms |
| Internal process measurement through cleanup | 234.443999 ms |
| External launch-to-exit wall time | 258.848883 ms |
| Process CPU | 55.814 ms / 23.81% over the short process |
| Threads before client / after client cleanup | 1 / 2 |

As in Gate 3A, the SDK heartbeat thread remained present after logout/client
destruction until process termination. A post-run process check found no
remaining probe process.

The machine-readable receipt is
`docs/gate3b_measurements/jaka_gate3b_stage2_preflight_20260716.json`.

Stage 2 met its read-only objective, but `servo_move_active=false` is a deliberate
Stage 3 blocker. The probe will not enable it. Stage 3 may only be attempted
after separate approval and safe operator preparation of the required state;
it will rerun preflight and stop before EDG if the state is still inactive.

### Stage 3 — blocked physical entry/exit attempt

The operator separately approved Stage 3 and reported that the required state
had been prepared manually. For additional protection, the physical attempt used
the separately compiled `jaka_edg_entry_exit_probe`. Link inspection confirmed
that this binary has no `edg_servo_j`, `edg_servo_p`, servo-enable, or general
movement symbol. It can only perform preflight reads, `edg_init`, one
`edg_get_stat`, EDG exit, and logout.

The new preflight captured:

| Field | Value |
|---|---|
| Fault code | 0 |
| Powered / enabled | true / true |
| Emergency stop | false |
| Collision | false |
| Servo-move active | **false** |
| Tool / user ID | 0 / 0 |
| Captured joint vector | `[1.56975867764, -0.028407091638, -0.770925072212, 0.505283204717, 0.134604423998, 0.47423723495]` rad |
| Maximum difference from Stage 2 vector | 0 rad |

Because the SDK servo-move flag was false, the probe returned outcome
`servo_move_mode_not_prepared_by_operator` and exit code 2 before calling
`edg_init`. Exact lifecycle was `login,preflight,logout`.

| Measurement | Result |
|---|---:|
| Login/connection | 163.189100 ms |
| Preflight reads | 11.969599 ms |
| EDG entry calls / duration | 0 / 0 ms |
| EDG state reads / duration | 0 / 0 ms |
| Joint/Cartesian commands | 0 / 0 |
| EDG exit calls / duration | 0 / 0 ms |
| Logout | 50.178486 ms |
| Internal lifecycle | 225.507518 ms |
| External launch-to-exit wall time | 248.841081 ms |
| Threads before client / after client cleanup | 1 / 2 |

No software motion observation is available. Since neither EDG nor a command
was reached, this attempt intentionally introduced no robot target. A post-run
process check found no remaining probe process.

The machine-readable failed-stage receipt is
`docs/gate3b_measurements/jaka_gate3b_stage3_entry_exit_20260716.json`.

This finding distinguishes controller servo enable (`enabled=true`) from the
SDK servo-move mode (`is_in_servomove=false`). The installed API documents
`servo_move_enable` as the method that changes the latter, but that method is
explicitly prohibited by Gate 3B and is absent from the entry-only binary.
No assumption or automatic state change was made.

## Pending physical records

EDG entry/exit, EDG state timing, cyclic command timing, deadline misses,
encoder delta, and operator-observed robot behavior remain unmeasured because
Stage 3 was blocked before EDG and Stages 4–5 are consequently unavailable.

### Revised Stage 3 — EDG lifecycle completed; safety comparison aborted stage

After the state-preparation review, the operator authorized a revised Stage 3
that required `is_in_servomove=false` and excluded servo-mode enable and all
target APIs. Static link inspection again confirmed that
`jaka_edg_entry_exit_probe` had no `servo_move_enable`, `edg_servo_j`,
`edg_servo_p`, or general motion symbol.

The physical lifecycle was:

`login,preflight,enter_edg,exit_edg,logout`

All five SDK lifecycle/read calls returned success. The one EDG observation was
finite, but its maximum difference from the immediately preceding standard
joint observation was `0.000013831979541 rad` (13.832 microradians), exceeding
the configured `0.000001 rad` observation-comparison threshold. The probe
therefore recorded `initial_edg_delta_exceeded`, performed immediate EDG exit
and logout, and returned a nonzero status. No command stage was reached.

| Field | Result |
|---|---:|
| Fault / E-stop / collision | 0 / false / false |
| Powered / normally enabled | true / true |
| Servo-move mode | false |
| Tool / user ID | 0 / 0 |
| Captured joint vector | `[1.56975867764, -0.028407091638, -0.770913087987, 0.505283204717, 0.134604423998, 0.47423723495]` rad |
| Difference from Stage 2 vector | 0.000011984224821 rad |
| Initial EDG-observation difference | **0.000013831979541 rad** |
| EDG entry | code 0, 18.690377 ms |
| Single EDG read | code 0, 8.697736 ms |
| EDG exit | code 0, 7.129996 ms |
| Logout | code 0, 50.164581 ms |
| Total internal lifecycle | 268.249118 ms |
| Process CPU | 115.402 ms / 43.020% of wall time |
| Threads before SDK / after SDK cleanup | 1 / 2 |
| Servo-mode enable/disable calls | 0 / 0 |
| Joint/Cartesian target calls | 0 / 0 |

The exact EDG observation vector was not serialized by this version of the
probe; only its validated finiteness and maximum difference were retained. This
is an instrumentation gap to correct before another attempt. The measured
difference is between two robot-state observations acquired through different
SDK paths and is not an intentional target delta. Any future invariant command
would still be constructed by exact copying, for an intentional delta of zero.

The operator subsequently reported no visible motion, abnormal sound,
vibration, collision indication, or controller alarm during EDG entry/read/exit
and cleanup. The operator accepted this revised Stage 3 as successful lifecycle
validation. The 13.832-microradian value is classified as a non-simultaneous,
cross-API encoder observation difference, not an intentional command delta.

The disposable process terminated after logout. As previously observed, the
SDK-owned heartbeat thread remained present after logout until process exit; no
probe process remained afterward.

The machine-readable receipt is
`docs/gate3b_measurements/jaka_gate3b_stage3_entry_exit_revised_20260716.json`.

The accepted receipt is
`docs/gate3b_measurements/jaka_gate3b_stage3_accepted_receipt_20260716.json`.

### Stage 4 — one-second invariant-current-joint command loop

Before Stage 4, the instrumentation was corrected to serialize the complete
standard, target, initial EDG, and final EDG vectors and signed per-joint
differences. Intentional command identity remains exact. Cross-API/encoder
observation thresholds are now distinct:

- observation warning: 0.000050 rad;
- observation abort: 0.000500 rad;
- first-command maximum difference: 0.000100 rad.

After a three-second operator countdown, the probe repeated every preflight
read and captured this invariant target immediately before EDG entry:

`[1.56975867764, -0.028407091638, -0.770925072212, 0.505283204717, 0.134604423998, 0.47423723495]` rad.

The target was an exact copy of that vector. Every component of the intentional
delta was zero. The command-capable disposable process then performed:

`login,preflight,precommand_check,enter_edg,enable_servo_move,125 invariant commands,final state read,disable_servo_move,exit_edg,logout`

Exact SDK functions were `login_in`; the bounded preflight getters;
`edg_init(true)`; `edg_get_stat`; `servo_move_enable(true)` followed by
`is_in_servomove`; 125 calls to `edg_servo_j`; one final `edg_get_stat`;
`servo_move_enable(false)`; `edg_init(false)`; and `login_out`. No state getter
ran inside the cyclic command loop.

| Measurement | Result |
|---|---:|
| Outcome | completed |
| Command cycles | 125 |
| Command interval duration | 0.992135425 s |
| Start-period mean / median | 8.000471 / 7.999905 ms |
| Start-period std. dev. | 0.010315 ms |
| Start-period min / max | 7.945827 / 8.071085 ms |
| Start-period p95 / p99 | 8.005006 / 8.048659 ms |
| Wake lateness mean / max | 0.060260 / 0.117195 ms |
| Command-call mean / p95 / p99 / max | 0.080481 / 0.189921 / 0.274405 / 0.519703 ms |
| Completion misses / period overruns | 0 / 0 |
| SDK failures | 0 |
| EDG entry / first read | 13.462327 / 12.398284 ms |
| Servo enable / disable | 11.938962 / 7.044024 ms |
| EDG exit / logout | 6.647303 / 50.175208 ms |
| Process CPU | 2.184839 s; 50.897% of 4.292641 s wall time |
| Intentional command delta | `[0, 0, 0, 0, 0, 0]` rad |

Initial EDG observation:

`[1.56974910247, -0.0283965064456, -0.770911917456, 0.505272809833, 0.134599789618, 0.47422340297]` rad.

Signed cross-API difference (initial EDG minus standard observation):

`[-9.57517687161e-06, 1.05851924129e-05, 1.31547562878e-05, -1.03948832225e-05, -4.63438056164e-06, -1.3831979541e-05]` rad.

Final EDG observation:

`[1.56974910247, -0.0283965064456, -0.770842104287, 0.505255356541, 0.134599789618, 0.47422340297]` rad.

Signed encoder change during the command interval:

`[0, 0, 6.98131688891e-05, -1.74532922222e-05, 0, 0]` rad, corresponding to
`[0, 0, +0.004, -0.001, 0, 0]` degrees at the SDK's apparent reported
resolution. The 69.813-microradian maximum raised an observation warning but
did not approach the 500-microradian abort threshold.

All SDK cleanup return codes were zero. The process-owned servo-move mode was
disabled before EDG exit. As in prior gates, the SDK heartbeat thread remained
after logout until disposable process exit; no probe process remained.

The operator reported no visible motion, abnormal sound, vibration, collision
indication, or controller alarm during EDG entry, the first command, the full
loop, servo-mode disable, EDG exit, logout, or cleanup. The robot appeared to
remain in its original pose. Stage 4 is therefore accepted as physically clean.

Machine-readable artifacts:

- `docs/gate3b_measurements/jaka_gate3b_stage4_one_second_20260716.json`;
- `docs/gate3b_measurements/jaka_gate3b_stage4_one_second_timing_20260716.csv`.

### Stage 5 — pre-command target-continuity abort

The operator authorized Stage 5 after accepting Stage 4 as physically clean.
To satisfy both “same invariant target” and the established requirement that an
invariant command be an exact copy of current state, the probe added a strict
post-countdown equality check against the Stage 4 target.

The preliminary and final post-countdown Stage 5 capture was:

`[1.56975867764, -0.0283831231886, -0.770901103763, 0.505283204717, 0.134604423998, 0.47423723495]` rad.

Relative to the Stage 4 target, its signed difference was:

`[0, +2.39684494e-05, +2.39684490e-05, 0, 0, 0]` rad.

The maximum difference was 23.968 microradians. This is small encoder-state
evolution, but exact equality was required: commanding the old Stage 4 vector
would no longer have zero intentional delta, while using the new current vector
would not be the exact same target. The probe therefore recorded
`stage5_target_not_identical_to_stage4` and stopped.

Exact lifecycle:

`login,preflight,precommand_check,logout`

No `edg_init`, `edg_get_stat`, `servo_move_enable`, `edg_servo_j`, or other
target call was reached. EDG entry count, servo-mode changes, and command count
were all zero. Login, all read-only checks, and logout returned success; logout
took 50.251276 ms. The disposable process terminated and no probe process
remained.

This is a specification boundary rather than evidence of robot instability.
Stage 5 must not be retried until one of these mutually exclusive definitions
is explicitly selected:

1. zero-motion means an invariant exact copy of the fresh post-countdown
   current state, permitting the Stage 5 vector to differ slightly from the
   earlier Stage 4 vector; or
2. target continuity means reusing the exact Stage 4 vector, accepting a small,
   explicitly bounded intentional return delta from current state.

Option 1 preserves the established zero-intentional-motion semantics and is the
recommended interpretation. Option 2 would constitute a small intentional
joint displacement and does not belong in Gate 3B.

Machine-readable artifacts:

- `docs/gate3b_measurements/jaka_gate3b_stage5_five_second_20260716.json`;
- `docs/gate3b_measurements/jaka_gate3b_stage5_five_second_timing_20260716.csv`
  (header only; zero cycles).

### Stage 5 corrected retry — hard wake-lateness abort

The operator clarified that “same invariant target” means the same procedure,
not reuse of the historical Stage 4 numbers. The probe was corrected to retain
the Stage 4 vector only as historical measurement data, serialize the signed
inter-run change, and construct the Stage 5 target as an exact copy of a fresh
post-countdown state.

Historical Stage 4 target:

`[1.56975867764, -0.028407091638, -0.770925072212, 0.505283204717, 0.134604423998, 0.47423723495]` rad.

Fresh Stage 5 capture and invariant command target:

`[1.56977066187, -0.0283831231886, -0.770901103763, 0.505283204717, 0.134604423998, 0.47423723495]` rad.

Signed inter-run observation change:

`[1.19842282023e-05, 2.39684493602e-05, 2.39684494854e-05, -4.4408920985e-13, 3.39422934204e-13, -4.5896619838e-13]` rad.

The Stage 5 intentional command delta was exactly
`[0, 0, 0, 0, 0, 0]` rad. The target was never updated.

Initial EDG observation:

`[1.56974910247, -0.0283790531533, -0.770894464163, 0.505272809833, 0.134599789618, 0.47422340297]` rad.

Its maximum cross-API difference from the fresh standard observation was
21.559 microradians, below both observation warning and abort thresholds.

The loop completed 78 successful commands over 0.627677 seconds. Before cycle
79 could issue a command, its wake-up was 3.668462 ms late, exceeding the fixed
2 ms hard-abort threshold. The loop stopped immediately and did not send the
79th command.

| Measurement | Result |
|---|---:|
| Outcome | `hard_wake_lateness` |
| Commands completed | 78 of 625 |
| SDK/command failures | 0 |
| Intentional target delta | 0 rad |
| Start-period mean / median | 8.000753 / 8.000259 ms |
| Start-period std. dev. | 0.008264 ms |
| Start-period min / max | 7.987481 / 8.066992 ms |
| Start-period p95 / p99 | 8.004758 / 8.026395 ms |
| Wake lateness mean / p95 / p99 / max | 0.104873 / 0.065320 / 0.863375 / **3.668462 ms** |
| Command-call mean / p95 / p99 / max | 0.052333 / 0.050443 / 0.179633 / 0.542574 ms |
| Completion misses / recorded period overruns | 0 / 0 |
| EDG entry / initial read | 12.384463 / 13.147701 ms |
| Servo enable / disable | 11.779066 / 11.337420 ms |
| EDG exit / logout | 6.246957 / 50.189330 ms |
| Process CPU | 2.013214 s; 51.233% of 3.929521 s wall time |

The final EDG encoder observation was intentionally skipped after the timing
abort so cleanup could begin immediately; encoder drift during this partial run
is therefore unavailable and must not be inferred. Cleanup order was
`disable_servo_move,exit_edg,logout`; every cleanup SDK return code was zero.
The SDK heartbeat thread again persisted after logout until disposable process
exit, and no probe process remained.

The operator reported no visible motion, abnormal sound, vibration, collision
indication, or controller alarm during the partial 78-command exposure or
cleanup. Stage 5 did not establish five-second timing stability, and Gate 3C is
not recommended until the scheduling-latency semantics are corrected or
validated and Stage 5 completes.

Machine-readable artifacts:

- `docs/gate3b_measurements/jaka_gate3b_stage5_five_second_retry_20260716.json`;
- `docs/gate3b_measurements/jaka_gate3b_stage5_five_second_retry_timing_20260716.csv`.

## Stage 5 timing-semantics review and corrected abort policy

The reported 3.668 ms wake lateness and 8.067 ms maximum period came from an
instrumentation-order defect: the loop calculated wake lateness and aborted
before appending that cycle's start-to-start period. The omitted late period
would have been approximately 11.668 ms. There was no clock discontinuity.

Exact reviewed semantics:

1. Wake lateness is `cycle_start - scheduled_release`, clamped to zero.
2. Both values use C++ `std::chrono::steady_clock`; on this Linux/libstdc++
   system it is the monotonic, non-wall-clock clock.
3. Releases are absolute `sleep_until` deadlines. The deadline advances from a
   stored time point, not by sleeping for a relative remainder each cycle.
4. Lateness is sampled immediately after wake and before the SDK command.
5. The old loop retained its original deadline grid after a late start, which
   compressed the next interval when it caught up. It never queued target
   objects, but could issue the following invariant command sooner than 8 ms.
6. The first command has wake-lateness data but no start-to-start period because
   no previous command start exists.
7. Durations are converted to unsigned nanoseconds only after ordering the time
   points. `steady_clock` cannot step with wall-clock changes, and the runtime is
   far below integer overflow range.
8. Gate 3A/3B makes SDK heartbeat load observable through process CPU and thread
   counts. The corrected loop additionally samples `sched_getcpu()` from fixed
   storage to count CPU migrations and uses `getrusage` outside the loop to
   report minor/major page faults. These measurements cannot attribute every
   scheduler delay to a specific kernel task, but distinguish migration and
   page-fault evidence from unexplained host scheduling latency.

Corrected timing policy:

- start period above 8.8 ms or wake lateness above 2 ms: warning;
- isolated warning with start period at or below 12 ms: command may proceed if
  the target is current/invariant and the SDK call succeeds;
- after such a warning, re-anchor the next absolute deadline to eight
  milliseconds after the actual late start, preventing compressed catch-up or
  backlog;
- start period above 12 ms: fatal before another command;
- wake debt of one full 8 ms cycle: fatal before another command;
- two consecutive start-period warnings: fatal;
- completion after the nominal 8 ms release: warning and schedule realignment;
- completion later than 12 ms from its scheduled release, or two consecutive
  completion misses: fatal;
- SDK error, invalid target, signal/operator stop, or cleanup error remains
  immediately fatal.

The late cycle is now appended to period statistics before classification, so
period and wake reports cannot disagree by omission. The loop remains bounded
and allocation-free. A 25-cycle absolute-deadline scheduler warm-up now runs
before the final post-countdown state capture and before EDG entry. Fake tests
cover an isolated 3.6 ms wake warning with schedule realignment and no backlog,
as well as repeated and hard misses. No root privileges, real-time policy, or
system-wide configuration are used.

### Stage 5 timing-corrected five-second result

The timing-corrected Stage 5 used this fresh post-countdown invariant target:

`[1.56975867764, -0.0283591547393, -0.770877135313, 0.505283204717, 0.134604423998, 0.47423723495]` rad.

The target was copied exactly from current state and remained unchanged;
intentional command delta was `[0, 0, 0, 0, 0, 0]` rad. All 625 commands
completed.

| Measurement | Result |
|---|---:|
| Outcome | completed |
| Commands / duration | 625 / 4.992088099 s |
| Timing warnings / hard misses | 0 / 0 |
| Completion misses / period warnings | 0 / 0 |
| Schedule realignments | 0 |
| Start period mean / median | 8.000077 / 8.000003 ms |
| Start period std. dev. | 0.030793 ms |
| Start period min / max | 7.524754 / 8.473845 ms |
| Start period p95 / p99 | 8.006401 / 8.018465 ms |
| Wake lateness mean / p95 / p99 / max | 0.059122 / 0.063212 / 0.074167 / 0.535689 ms |
| Command mean / p95 / p99 / max | 0.035445 / 0.050181 / 0.084362 / 0.550490 ms |
| CPU migrations / CPUs | 0 / start 4, finish 4 |
| Minor / major page faults in loop | 1 / 0 |
| SDK failures | 0 |
| Process CPU | 4.362867 s; 51.434% of 8.482483 s wall time |
| Servo disable / EDG exit / logout | codes 0 / 0 / 0 |

Initial EDG observation:

`[1.56974910247, -0.0283441465689, -0.770877010871, 0.505272809833, 0.134599789618, 0.47422340297]` rad.

Final EDG observation:

`[1.56969674259, -0.0282568801078, -0.770807197702, 0.505272809833, 0.134599789618, 0.47422340297]` rad.

Signed encoder observation change:

`[-5.23598766666e-05, 8.72664611111e-05, 6.98131688889e-05, 0, 0, 0]` rad,
equivalent to `[-0.003, +0.005, +0.004, 0, 0, 0]` degrees. The 87.267
microradian maximum generated an observation warning but remained below the
500-microradian abort threshold.

Lifecycle and cleanup were
`login,preflight,precommand_check,enter_edg,enable_servo_move,625 commands,final state read,disable_servo_move,exit_edg,logout`.
Every SDK and cleanup return code was zero. No probe process remained after the
disposable process exited.

The operator reported no visible motion, abnormal sound, vibration, collision
indication, or controller alarm during EDG entry, the first command, all 625
commands, servo-mode disable, EDG exit, logout, or cleanup. The robot appeared
to remain in its original pose. Gate 3B is accepted complete.

Machine-readable artifacts:

- `docs/gate3b_measurements/jaka_gate3b_stage5_timing_policy_retry_20260716.json`;
- `docs/gate3b_measurements/jaka_gate3b_stage5_timing_policy_retry_20260716.csv`.
