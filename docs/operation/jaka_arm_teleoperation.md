# JAKA arm teleoperation

The current physical entry point is `tools/quest_jaka_hardware.py`. It is not a
normal quick-start command. Inspecting help is safe:

```bash
.venv/bin/python tools/quest_jaka_hardware.py --help
```

Its stages are deliberately separated (`p2-shadow`, `e2-isolated`, `p4-live`,
and `post-payload-diagnostic`) and require stage-specific acknowledgements.
Never copy an old historical invocation without reconciling it with current
`--help`, current config, and the approved gate.

## Current runtime contract

- Target generation is 60 Hz; native output is approximately 125 Hz (8 ms).
- Physical output consumes the shared immutable accepted six-joint target.
- Joint-teleop mode performs zero native JAKA IK calls.
- Output is absolute `edg_servo_j(..., ABS, 1)`.
- Resampling is piecewise-linear toward the latest destination with no stale
  queue replay.
- Post-EDG `q_hold` is authoritative and first engagement must be continuous.
- `HOLD_REJECTED` holds the last safe target with a fresh heartbeat.
- Actual liveness loss, tracking fault, controller alarm, collision, SDK error,
  or hard timing failure stops and cleans up.
- The sole SDK session performs lightweight health polling every two command
  cycles; extended collision/estop queries occur only after unhealthy status.

See [current status](../status/current_status.md) before proposing a physical
stage. The next recommended gate is not yet authorized and must occur in a new
session.
