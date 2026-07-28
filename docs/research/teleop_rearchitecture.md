# Quest/JAKA teleoperation rearchitecture

Status: **offline research, merged for review; not production or physical
validation**. No JAKA, EDG, ServoJ, Quest, or RH56 connection is authorized by
this page.

## Purpose and retained decision

The production path remains:

```text
HTS + CTRL -> validation and release-before-press clutch
-> mapping/filtering -> shared continuation IK and safety
-> immutable AcceptedArmTarget
-> MuJoCo adapter OR production JAKA PWL/EDG adapter
```

The research isolates a possible future boundary after accepted full IK:

```text
Accepted joint target -> independent bounded shaper
-> thin transport owning the sole SDK session
```

The transport may validate sequence, epoch, freshness, health, and first-command
continuity. It must not repeat IK, mapping, filtering, interpolation, collision
planning, singularity handling, reference capture, or clutch semantics.
`HOLD_REJECTED` remains a fresh producer heartbeat, not a liveness stop.

The retained implementation is intentionally narrow:

- `native/teleop_shaping/`: versioned POD ABI, bounded reference shaper,
  controlled braking, fake consumer/lifecycle, and thin SDK-free transport;
- `src/teleop_rearchitecture/cpp_shaping.py`: offline ctypes binding used by
  focused ABI/shaper tests;
- `src/teleop_rearchitecture/engagement.py`: robot-independent reference
  capture and safety-epoch coordination;
- `native/research_thin_jaka_worker/`: separately gated research translation
  unit;
- `teleop_command_abi.md`: the stable ABI and shaping contract;
- `jaka_clutch_recovery_transport_contract.md`: unresolved controller
  pause/restart semantics and required future gates.

## Safety and lifecycle contract

Normal clutch release and hard faults are distinct:

```text
Active/HoldRejected -> ControlledBraking -> StoppedReady
StoppedReady + fresh measured state + new reference/epoch -> Active

alarm/estop/collision/SDK/timing/session fault -> HardStopped
HardStopped -> cleanup -> explicit reset only
```

While released, input is latest-only and no motion target is accumulated.
Re-engagement captures current measured q/dq/ddq and the current input pose,
increments the safety epoch, clears old target/feed-forward/rejection history,
and requires the first new command to be continuous with measured state. An
old-epoch command is rejected.

The reference shaper uses an exact 8 ms grid with explicit position, velocity,
acceleration, and jerk limits. Its controlled-braking path preserves q/dq/ddq
continuity and fails closed when position or dynamic limits make a stop
unplannable. Those values are research policy, not JAKA Mini2 vendor limits.

## Offline evidence retained

Focused tests retain the durable behavior:

- ABI size/alignment and fail-closed validation;
- low, middle, and high representative braking speeds without reversal;
- hard-stop preemption with no further output;
- release-before-press reference recapture after controller movement;
- latest-only input, old-epoch rejection, long stopped dwell, explicit
  hard-stop reset, and residual measured-velocity preservation;
- fake lifecycle/transport fault classification, cleanup, telemetry bounds,
  pause policy defaults, and repeated release/re-engagement cycles.

The earlier development phase also ran broad candidate benchmarks, 60-state
stop comparisons, a 115-state residual-acceleration sweep, and generated JSON
reports. Those one-off evaluators, result files, dependency, and byte-for-byte
artifact tests were removed after the design decision because they were not
production interfaces or long-term regressions. Their exact content remains
recoverable from Git history before the repository-maintenance commit.

## Known limitations

- The C++ shaper and thin adapter remain reference/research implementations.
- There is no scheduler-load, process-IPC, network, SDK, controller, or plant
  proof.
- Controller behavior during a stopped EDG interval is unknown: no-command,
  repeated stopped position, and EDG/servo restart semantics require vendor
  evidence and a separately authorized no-motion gate.
- EDG feedback exposes q/dq but not ddq; freshness and initialization rules
  remain unvalidated on hardware.
- The production PWL/output-acceleration correction has not received its
  bounded post-fix physical validation, and the historical J4 collision cause
  remains unresolved.

Do not replace the production adapter, enable recovery, or infer a physical
PASS from this offline research.
