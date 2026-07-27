# Teleoperation command-health ABI v1 and C++ shaping reference

Status: **isolated offline research prototype**. This design is not an operator
procedure, a JAKA adapter, or evidence of physical readiness. It does not load
or link the JAKA SDK and has no Quest, ROS, network, MuJoCo, or RH56 dependency.

## Boundary and compatibility contract

The `teleop_command_abi` namespace defines a robot-independent, fixed-layout v1
contract between an accepted-target producer, a joint shaper, and a command
consumer. Vendor state is normalized before it reaches the boundary. JAKA,
Quest, MuJoCo, ROS, and Ruckig types are forbidden from the public ABI.

All wire structures are standard-layout, trivially copyable C++17 PODs. They
use fixed-width integers, SI units, monotonic nanosecond timestamps, fixed
arrays with `MAX_DOF=8`, an explicit runtime `dof`, no pointers, no heap-owned
strings, and no variable-sized containers. Mini2 uses `dof=6`; the ABI logic
does not permanently encode six axes. Unused axes and every explicit reserved
byte must be zero.

V1 is a same-host shared-memory/in-process layout contract with an explicit
little-endian host marker; it is **not** a portable network serialization.
Cross-host transport would require a separately versioned stable serializer.
The validator rejects a non-little-endian marker, future/unknown versions,
wrong structure sizes, and nonzero reserved bytes. The current GCC 13.3 ARM64
layout is fixed and tested as follows:

| Structure | bytes | alignment | selected fixed offset |
| --- | ---: | ---: | ---: |
| `AbiHeader` | 16 | 4 | schema at 4 |
| `AcceptedJointTargetV1` | 128 | 8 | joints at 64 |
| `MeasuredJointStateV1` | 240 | 8 | position at 48 |
| `JointDynamicLimitsV1` | 344 | 8 | min position at 24 |
| `ShapedJointCommandV1` | 256 | 8 | position at 64 |
| `TransportHealthV1` | 64 | 8 | normalized vendor category at 60 |

Compile-time `sizeof`, `alignof`, `offsetof`, standard-layout, and trivial-copy
assertions are paired with Python `ctypes` runtime checks against the compiled
library. A new field or changed layout requires a new schema version; v1 is not
silently extended.

## Orthogonal state and message meanings

Engagement, target validity, output mode, and stop class are separate enums:

- engagement: disengaged or engaged;
- target validity: no target, accepted, or rejected while keeping the previous
  accepted target;
- output mode: inactive, active tracking, controlled braking, stopped, or hard
  stopped;
- stop class: none, controlled, or immediate.

Stop reasons distinguish clutch release, stale input, timing fault, controller
alarm, SDK failure, estop, collision, producer failure, epoch mismatch, and
invalid command. Transport and controller states are also orthogonal. The
integer vendor category is a small normalized enum, never a vendor handle,
vendor structure, or raw SDK status object.

`AcceptedJointTargetV1` carries a strictly increasing publication sequence, a
safety epoch, source/acceptance/expiry timestamps, engagement/validity, reason
code, and joint positions. Only `accepted + engaged` may carry a new vector.
`rejected_keep_previous` is a fresh liveness message but all of its joint slots
must be zero, preventing replacement-target smuggling.

`MeasuredJointStateV1` is the only initialization/restart source. It contains
measured q/dq/ddq, its own sequence/timestamp/validity, and the safety epoch.
The shaper never reconstructs measured state from the last target.

`JointDynamicLimitsV1` supplies finite per-axis position min/max and positive
velocity, acceleration, and jerk maxima. No global all-axis limit is assumed.
The checked 50 rad/s³ value remains a research policy bound, not a Mini2 vendor
limit.

`ShapedJointCommandV1` identifies both its strictly increasing output sequence
and the accepted target source sequence. Controlled braking retains the last
accepted target as `source_sequence` and separately records the release event
sequence in the shaper snapshot. Active/braking values must be finite and
bounded. Stopped output may expose the final held position with zero dq/ddq.
The core emits no command after a terminal hard stop; the ABI validator also
permits only all-zero observation fields in an explicitly hard-stopped record.
Every command has a `valid_until_monotonic_ns` consumer freshness deadline.

`TransportHealthV1` carries health sequence, last consumed output, epoch,
sample time, normalized transport/controller states, stale/deadline/alarm/
estop/collision/enable flags, and a normalized vendor-status category. It
defines evidence exchange only. The SDK-free fake lifecycle below consumes it;
there is still no real transport or vendor implementation.

## Sequence, epoch, freshness, and validation

Publication and output sequences are strictly increasing within an epoch.
Skipped values are allowed so a one-slot latest-wins mailbox can supersede old
commands; duplicates and older values are rejected. A safety-epoch change
invalidates all shaper state and requires explicit initialization from a new
valid measured state. It is never an implicit resume signal.

Source time must not exceed accepted time, which must not exceed expiry. A
causal consumer additionally requires accepted/generated time at or before its
caller-supplied `now_ns` and expiry at or after `now_ns`. Candidate C
feed-forward uses consecutive **source** timestamps. Equal or decreasing
source timestamps fail closed.

All five structures have pure `noexcept` validators returning `{ok, error,
field, index}`. They do not throw or format strings. Tests cover unknown enum
bytes/schema versions/DOF, NaN and infinities, timestamp and sequence order,
epoch mismatch, invalid limits, stale validity, target smuggling, invalid mode
combinations, booleans outside 0/1, and reserved bytes.

## Shaper modes and algorithms

The replaceable `IJointShaper` interface and concrete
`ReferenceJointShaperV1` implement the shaping portion of this state machine:

```text
Uninitialized --Initialize(measured, limits, epoch)--> ActiveTracking
ActiveTracking --clutch release--> ControlledBraking --> Stopped
Uninitialized/ActiveTracking/ControlledBraking/Stopped --hard fault--> HardStopped
```

Active tracking is an independent C++ conformance port of the current Python
Candidate C reference law: one-replacement source-timestamp velocity
feed-forward, `36 * position_error + 10 * velocity_error`, then per-axis jerk,
acceleration, velocity, and position-limit enforcement on an exact 8 ms grid.
It is an architecture/reference backend, **not a production shaper**.

Controlled braking does not call the active tracking law. It starts from the
current q/dq/ddq, emits a distinct output mode, and preserves q/dq/ddq
continuity. If residual acceleration prevents the original synchronized
analytic stop, the planner first ramps acceleration to zero at the per-axis
jerk limit, bounds the resulting velocity/position excursion, then brakes any
residual velocity to zero. Independent-axis neutralization is used only when
the common-duration equation has no valid solution. A planning failure records
`position_limit`, `velocity_limit`, `numerical`, or `invalid_dynamic_state`
and enters hard stop; it never silently loops. Release while already braking
or stopped and repeated hard stop are idempotent. Repeated `Stopped` records
carry final observed q with zero dq/ddq for fixed-window offline evaluation;
they are not sendable movement commands. The lifecycle consumer accepts the
first marker, enters `Stopped`, and rejects subsequent sends.

The tick path has fixed-size state, bounded loops, caller-provided time, and no
file I/O, JSON, logging formatter, mutex wait, sleep, or dynamic allocation.
The C++ test overrides global allocation and observed zero allocations across
100,000 active ticks. `noexcept` prevents exceptions escaping the timing path.
This is design/test evidence, not scheduler or realtime certification.

## Recoverable clutch pause and reference recapture

`teleop_rearchitecture.engagement.EngagementCoordinator` is a pure,
robot-independent reference state machine:

```text
Uninitialized -> Disengaged -> Engaging -> ActiveTracking <-> HoldRejected
                                  |                |
                                  +------ ControlledBraking -> StoppedReady
any state ----------------------------------------------------> HardStopped
HardStopped --explicit valid measured-state reset------------> Disengaged
```

Clutch release freezes the last accepted source sequence and starts controlled
braking. Input observations continue in a depth-one latest slot, but no target
is emitted in `ControlledBraking` or `StoppedReady`. Re-engagement is refused
until `StoppedReady`; it then increments the safety epoch and atomically
captures the current measured q/dq/ddq and current input pose. Old target,
feed-forward/filter/relative accumulator, rejected-target, and braking history
are represented as cleared. Therefore the first relative pose is exactly the
identity even if the operator moved or rotated the controller while paused.
An old-epoch target is counted and rejected. A hard stop cannot be cleared by
pressing clutch; explicit reset with valid measured state is required.

The coordinator deliberately has no Quest receiver or mapping code. Its
capture object is the future boundary at which the existing release-before-
press input logic can initialize kinematics and the shaper.

## Python conformance, fake consumer, and fake lifecycle

`teleop_rearchitecture.cpp_shaping` is a test/evaluator-only `ctypes` bridge.
It checks compiled sizes/alignments and drives the C++ core without defining a
transport. Active output conforms to the Python Candidate C reference over a
240-tick changing-target trace to 2e-15 rad position/velocity and 2e-14
rad/s² acceleration tolerance (floating-point rounding only).

The in-memory fake consumer validates DOF, epoch, sequence, finite data, mode,
and freshness; accepts only the newest one-slot command; counts superseded
commands; latches terminal state on stale/invalid/epoch/failure conditions;
and records a fixed 256-entry telemetry ring. Tests inject duplicate, skipped,
stale, epoch-mismatched, non-finite, producer-disappearance, and hard-stop
events. It has no SDK return code, EDG behavior, network IPC, or file output.

`FakeJakaLifecycleAdapter` is a separately named, SDK-free test double for the
future thin hardware boundary. It models sole-session ownership and
`Disconnected -> Connecting -> Connected -> ServoReady -> Streaming ->
ControlledStopping -> Stopped`, with any active state able to latch `Faulted`
and explicit `CleaningUp -> Disconnected`. Re-engagement from stopped requires
a newer epoch and a valid `MeasuredJointStateV1`; output sequence restarts only
inside that new epoch. It validates shaped commands and normalized health,
classifies stale/deadline/epoch/sequence/transport/controller failures, and
never performs IK, mapping, filtering, interpolation, shaping, or braking.
Cleanup now leaves both the hard-fault latch and an explicit reset requirement;
reconnection is impossible until `ResetAfterCleanup` is called. The audited
future transport boundary and unresolved controller pause semantics are in
[`jaka_clutch_recovery_transport_contract.md`](jaka_clutch_recovery_transport_contract.md).

The send path writes only a fixed 256-record ring. Records contain output and
source sequence, epoch, mode, command age, deadline slack, validation reason,
and lifecycle result. Ring wrap increments an overflow counter. A separate
terminal-fault slot survives ring wrap and cleanup. No send-path JSON, file
I/O, allocation, blocking, or logging formatter is present.

## Offline conformance result and limitations

The checked unified benchmark adds `candidate_c_cpp_reference`. Its active,
interpolated, settling, and dynamics results match the Python Candidate C
reference within floating-point rounding. Mean Python-to-C++ `ctypes` call
time was 4.67/4.72 µs on the two fixtures; p99 was 8.44/7.02 µs and maxima
29.32/46.12 µs. This excludes scheduling, serialization, process IPC, SDK,
controller, network, and plant response, so it is not realtime proof.

The 60-state controlled-stop sweep compares Ruckig 0.19.4 Python explicit
braking with the independent C++ analytic profile. Both completed 60/60, had
zero direction-inconsistent cases, zero q/dq/ddq limit violations, identical
strict stop-time mean/p95/max of 139.47/288/312 ms, and zero post-completion
drift. Maximum per-case envelope differences were 0 ms stop time, 0.930 mrad
joint displacement, 0.268 mm palm-model displacement, 0.000658 rad/s velocity,
1.242 rad/s² acceleration, and 14.229 rad/s³ jerk. The checked tolerances are
stored with every comparison. This is bounded envelope conformance, not a
claim that the analytic profile implements Ruckig or produces identical ticks.

The residual-acceleration sweep adds 115 deterministic states. All 113 states
inside position/dynamic limits complete; two states only 0.1 mrad from a joint
limit with outward acceleration fail closed as `POSITION_LIMIT`, as expected.
All completed cases are direction-consistent under the declared expected
single crossing. The largest legal synthetic boundary reaches 584 ms stop
time, 1.4513 rad/s velocity excursion, 0.4854 rad joint displacement, and
0.2830 m palm-model displacement. That large envelope is important: the
neutralization phase fixes an erroneous hard stop but does not make high
residual acceleration benign. The checked data is
[`residual_acceleration_stop_sweep.json`](teleop_rearchitecture/results/residual_acceleration_stop_sweep.json).

An explicit no-SDK manifest drives 36 Python files plus CTest in one process,
then checks `/proc/self/maps`. `readelf` and `nm` also gate the C API library
and test executable. The current run passed 340 Python tests and found no JAKA
library image, dependency, or symbol. The historical SDK-linked native-worker
test is named in `forbidden_test_paths` and is not part of this manifest.

All results are offline command/FK evidence. There is no scheduler-load proof,
plant model, physical stop guarantee, real controller alarm lifecycle, JAKA
SDK transport, process/network IPC, Quest input, RH56 command, or physical
validation in this phase. The C++ active tracker remains reference-only and
the fake lifecycle is not a JAKA adapter.
