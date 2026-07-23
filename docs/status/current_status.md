# Current status

## What works

The Quest HTS/CTRL input boundary, release-before-press controller clutch,
fresh wrist/head/TCP reference capture, coordinate mapping, filters, bounded
continuation IK, Jacobian-based singularity handling, collision/limit/branch
checks, output velocity/acceleration feasibility, `HOLD_REJECTED`, immutable
accepted target, MuJoCo adapter, JAKA joint adapter, and native
latest-destination resampler are implemented and covered offline.

The live Quest/MuJoCo arm path and simulated RH56 grip retargeting are validated
in simulation. The default test suite and fake native worker require no
hardware.

## Latest physical evidence

A historical larger Quest/JAKA run produced a J4 servo collision alarm after
approximately 128 mm Quest/TCP displacement with substantial wrist motion. The
cause is unresolved. The operator subsequently reported applying payload
0.8 kg and COM `[9.289, 12.427, 36.961]` mm.

A synchronous/second-session health instrumentation attempt produced no motion:
the second SDK login prevented the primary worker from reaching `CONNECTED`.
The design was replaced with lightweight health polling on the sole SDK session.
A later bounded physical run completed 27.09 s and 3377 command ticks with no
timing warnings, hard misses, or controller alarm, validating that polling
timing path in that envelope. It stopped before a J4 point because the replayed
accepted targets contained controller-visible acceleration of
14.199679 rad/s².

Current HEAD adds a shared 4π rad/s² output-acceleration gate before
`AcceptedArmTarget` construction. Offline replay now produces a safe
`HOLD_REJECTED` and recovers on the next feasible tick. That correction has not
yet been physically validated. TCP remains recorded as zero.

## Exact next bounded physical gate

Open a new Codex session and obtain explicit authorization for a bounded repeat
of the post-payload diagnostic after the acceleration fix:

- maximum about 30 seconds in a known healthy posture;
- verify controller payload/COM, installation, zero TCP record, unchanged
  safety limits, alarms, workspace, and stop access without writing settings;
- release before press and confirm a still, jump-free first engagement;
- one gentle forward-and-return translation followed, separately, by one
  modest single-axis wrist motion;
- confirm any acceleration-infeasible candidate stays `HOLD_REJECTED`, recovery
  is immediate on a feasible retreat, native defensive acceleration rejection
  remains zero, tracking/timing/health stay bounded, and release stops/cleans
  up;
- preserve accepted/emitted targets, metrics, controller state, and stop reason.

Do not repeat the approximately 128 mm multi-axis/large-wrist run, combine axes,
approach a hard singularity, change payload/TCP/controller settings, or expand
the envelope in the same gate. This maintenance session does not authorize the
test.
