# Current status

## Executive state

The primary current stack is Meta Quest 3 HTS/CTRL input, a left Touch
controller, the shared Quest/JAKA target pipeline, MuJoCo, a separately gated
JAKA Mini2 ServoJ/EDG adapter, and a PC-direct Inspire RH56DFX hand path.
Simulation and default tests do not connect to hardware.

The shared arm path is implemented and covered offline:

```text
validated Quest input
  -> release-before-press clutch/reference capture
  -> frame mapping, filters, and bounded continuation
  -> shared IK, collision, singularity, limit, and output checks
  -> immutable AcceptedArmTarget
  -> MuJoCo adapter OR JAKA accepted-joint adapter
```

Both adapters receive the same accepted J1--J6 radians. The physical adapter
does not follow MuJoCo `qpos`, remap, filter, or recompute IK. Current native
`joint-teleop` makes zero JAKA `kine_inverse` calls and sends the accepted
absolute target through the 8 ms latest-destination/PWL worker.

## Validation summary

| Capability | Current status |
|---|---|
| Quest HTS/CTRL parsing, freshness, bounded queue | Implemented and offline tested |
| Release-before-press arm/hand clutching | Implemented and simulation validated; partially physically exercised |
| Mapping, filters, continuation IK, branch/singularity/limit checks | Implemented and offline/simulation tested |
| Shared output velocity and acceleration feasibility | Implemented and offline tested; latest acceleration correction lacks a bounded post-fix physical validation |
| `HOLD_REJECTED` recovery | Implemented and offline/simulation tested |
| MuJoCo accepted-target output | Simulation validated |
| Native 8 ms PWL/latest destination and zero-IK joint mode | Fake-worker/offline tested; partially physically exercised |
| Sole-session JAKA controller polling | Implemented; bounded physical timing evidence exists |
| RH56 PC-direct read/command scheduler | Offline tested and physically exercised |
| Integrated arm + RH56 simulation | Validated as a six-arm-actuator plus six-hand-actuator approximation |
| Physical combined operation | One bounded 60.105 s PASS; no 300 s PASS |
| TCP calibration | Not complete |
| Dual-camera physical dataset capture and policy training | Not validated end to end |

Offline, replay, and simulation results are not physical PASS evidence.

## Physical evidence

The operator-confirmed translation basis maps Quest right/up/forward to
robot-base `-Y/+Z/+X`. Selected bounded physical runs confirmed those three
directions, but the full translation/orientation envelope is not validated.

An earlier larger run triggered a J4 servo collision alarm after approximately
128 mm of Quest/TCP displacement with substantial wrist motion. Its cause is
unresolved. The operator later recorded payload 0.8 kg and center of mass
`[9.289, 12.427, 36.961]` mm, but this has not proven payload mismatch was the
sole collision cause.

The current controller monitor uses the sole JAKA SDK session. A rejected
second-session monitor prevented the primary worker from reaching `CONNECTED`
and produced no motion; it is not part of the current design.

After command-loop scheduling corrections, combined session
`quest_jaka_rh56_combined_20260730_194001_3792423` completed 60.105 seconds
with zero hard timing miss, controller alarm, arm/RH56 worker fault,
serial/protocol fault, or transport symptom. This is a PASS only for that
bounded 60-second configuration and envelope.

A later post-instrumentation run reached 200.943 seconds with zero hard timing
faults. Fresh CTRL packets then reported `active=0`, and the retained liveness
policy correctly stopped with `producer_liveness_loss`. That is correct safety
behavior, not a completed duration gate. No 300-second combined run has a PASS.

Four unisolated combined runs on 2026-07-31 stopped on native cycle-start hard
timing faults. All recorded `configured_control_cpu=-1`; trigger correlation
showed no clutch transition near three faults, and grip was released at all
four terminal events. The direct cause was OS scheduler wake delay of the
unisolated `SCHED_OTHER` native thread, not an RH56 fault or a demonstrated
trigger-transition defect. The maintained combined gate now requires an
explicit verified `--native-control-cpu`; on the recorded 14-CPU host, CPU6 is
the previously measured low-load choice. This correction is offline tested but
does not add a new physical PASS.

A later requested 300-second episode used verified CPU6 affinity but stopped
fail-closed at 42.281 seconds. The native thread did not migrate, yet two
consecutive starts were delayed by 3.998 and 4.025 ms. The kernel tick is 4 ms
(`CONFIG_HZ=250`), CPU6 is not kernel-isolated, and the control thread was
still `SCHED_OTHER` priority 0. Grip had been stable-released for 0.972 s and
index stable-pressed for 0.853 s, so no trigger transition coincided with the
fault. Process affinity is necessary but not sufficient.

The maintained combined gate now also requires the fixed native
`SCHED_FIFO` priority 10. Only the 8 ms native control thread is promoted,
after SDK helper-thread setup, and it returns to `SCHED_OTHER` before cleanup.
The entry checks inherited `RLIMIT_RTPRIO >= 10` before hardware I/O and the
native worker verifies actual policy/priority. A one-shot, explicitly
authorized `prlimit` scope was used without running the control stack as root.
The latest episode ran 174.915 seconds on CPU6/SCHED_FIFO 10 with zero hard
timing miss, controller alarm, collision, E-stop, RH56 worker fault, or cleanup
error. The operator then removed Quest, so the retained native watchdog stopped
with `producer_liveness_loss`; this is useful partial evidence but not a
300-second PASS. Timing and all other safety thresholds remained unchanged.

The shared session now has an offline-tested distinction between a transient
Quest CTRL/wrist outage and actual producer death. The transient path
immediately pauses/holds, emits no-motion heartbeats for at most 10 seconds,
and requires release-before-press reference recapture after data returns.
Timeout is terminal. The native producer watchdog remains 100 ms, so Python or
IPC death is not masked. This new recovery behavior is not yet physically
validated.

The latest shared output-acceleration correction is offline tested but has not
received its required bounded post-fix physical validation. Do not infer a
physical PASS from accepted-target replay or fake-worker results.

## RH56DFX status

The maintained physical hand route is PC-direct USB/RS485, not JAKA tool-RS485.
Opening the serial path performs zero register writes. The controller uses
fresh measured `ANGLE_ACT` for activation, bounded target range/rate/delta, raw
feedback, stale/protocol/error gates, and deterministic cleanup. The physically
selected scheduler profile is `fast40`.

Current feedback meanings are deliberately limited:

- `ANGLE_ACT`: measured feedback for the six commanded actuator axes;
- `CURRENT`: raw current-register telemetry;
- `FORCE_ACT`: raw load/force-register telemetry;
- `ERROR`: raw error values; nonzero faults;
- `STATUS`: raw status values; nonzero code meanings are not guessed.

These are not complete passive-joint state, tactile, slip, or calibrated
contact-force signals.

Quest hand-only PC-direct operation and short combined sessions have physical
evidence, including the 60.105-second combined PASS above. Complete
Quest-driven physical hand teleoperation, target/feedback characterization,
and a long-duration all-gates validation remain incomplete.

The maintained live configuration and both physical RH56 entry assemblers now
load `configs/hand/quest_rh56_real_retarget.yaml`, align from measured
`ANGLE_ACT` toward the current Quest pose on each grip press, and enable the
physically derived index-pinch three-channel relationship. The previous
combined path accidentally inherited the simulation-uncalibrated calibration;
the correction is offline tested but has not yet been revalidated on hardware.

A later bottle-grasp combined run stopped fail-closed on middle-channel
`ERROR=4` after contact had already been detected. The log showed that a grip
release followed by loaded reacquisition rebased `FORCE_ACT` to the loaded
values and cleared provisional/latched holds; the command shaper also retained
closing momentum after the contact target moved toward relief. The controller
now preserves the no-load baseline and contact state across loaded
reacquisition, restores a provisional hold when release races detection, and
discards residual closing velocity at the contact clamp. This correction is
offline regression tested and received partial post-fix object-contact evidence
in the 2026-07-31 combined run: contact detection reached 7, loaded activation
state was preserved 10 times, and RH56 `ERROR` remained zero. The run was not a
PASS because it stopped fail-closed after 148.9 seconds on a separate grip
reactivation race. A measured-activation write inside the previous 40 Hz
command window was deferred as rate-limited after its one-shot force flag had
already been consumed. Forced measured activation now bypasses only that
ordinary command window; normal commands remain on the 40 Hz scheduler. This
latest race correction is offline regression tested but not yet physically
revalidated.

An unloaded bounded hand-only endpoint test then commanded canonical
`[index, middle, ring, pinky, thumb_close, thumb_lateral] =
[0, 0, 0, 0, 0, 0.9]`. Final measured normalized feedback was approximately
`[0.004, 0.004, 0.002, 0.002, 0.005, 0.914]`, with zero current, zero ERROR,
all STATUS values 2, and no serial/protocol fault. This physically confirms
that the thumb-lateral actuator can reach the 0.9 region while unloaded; it
does not by itself validate Quest retarget coverage or loaded motion there.

## Simulation limits

The integrated live MuJoCo model has six JAKA and six RH56 position actuators.
Six equality constraints approximate RH56 coupling: thumb PIP/DIP follow the
thumb-close actuator through fitted polynomial relationships, and each finger
DIP follows its MCP. The model does not reproduce tendon compliance, backlash,
current/force control, calibrated actuator force limits, or complete physical
underactuation.

The live viewer/plant injects a provisional table and mounting members.
`SharedJakaTargetGenerator` uses the base MJCF, so that injected table is not
part of shared pre-acceptance collision authority. The scene must not be used
as proof of physical workspace clearance or collision safety.

## Current physical entry

The maintained combined wrapper is:

```bash
./scripts/run_quest_jaka_rh56_teleop.sh --help
```

It requires exact arm and hand approvals, completed hand prerequisites,
accessible E-stop, a clear workspace, bounded duration, stable/verified device
identity, and `--no-auto-retry`. The wrapper permits at most 300 seconds, but
that upper bound is not a validated operating duration. It also requires an
explicit verified `--native-control-cpu`; unisolated combined operation is
rejected before hardware I/O. The fixed native control priority is
`SCHED_FIFO` 10, and inherited `RLIMIT_RTPRIO >= 10` is also required before
hardware I/O.

The arm-only isolation and RH56 staged inspection entries are:

```bash
./scripts/run_quest_jaka_bounded_teleop.sh --help
./scripts/run_quest_rh56_hand_test.sh --help
```

Inspecting help or running plant-free tests does not authorize hardware.

## Blocking work

1. Resolve the prior J4 collision cause without weakening collision, limit,
   timing, tracking, or liveness boundaries.
2. Perform the separately authorized bounded physical validation of the latest
   output-acceleration correction.
3. Complete and verify TCP calibration at the controller; current recorded
   TCP1--TCP10 values are zero.
4. Physically validate the new bounded Quest input-recovery/re-clutch behavior
   and complete a full duration gate; operator-ended runs must not be
   relabelled as a 300-second PASS.
5. Complete physical RH56 target/feedback characterization and the staged
   Quest-driven hand validation.
6. Calibrate camera/robot time and geometry before physical dual-D435 dataset
   collection.
7. Validate dataset quality and framework adapters before claiming ACT,
   Diffusion Policy, or OpenPI training support.

See [system architecture](../architecture/SYSTEM_ARCHITECTURE.md) and
[real hardware safety](../safety/REAL_HARDWARE_SAFETY.md) for the current
contracts. Dated evidence under `docs/history/` remains evidence only and does
not override these pages or the active source.
