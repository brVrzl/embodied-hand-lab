# Shared target pipeline

## Acceptance is the adapter boundary

`SmoothQuestJakaSession` consumes validated input, owns clutch/reference state,
advances a bounded continuation, and asks `SharedJakaTargetGenerator` for a
candidate. A candidate becomes an immutable
`teleoperation.accepted_target.AcceptedArmTarget` only after all shared checks
pass.

Checks cover:

- finite and fresh input;
- pose residual and continuation progress;
- joint limits, self/environment collision, and branch continuity;
- Jacobian singularity quality and directional recovery;
- accepted-output joint velocity;
- accepted-output joint acceleration, including the first 8 ms emitted step
  and replacement of an active native interpolation segment.

The configured continuation allows at most five backtracks and a minimum
fraction of 1/32. The output boundaries are currently π rad/s velocity and
4π rad/s² acceleration. These are shared policy values, not native
post-processing conveniences.

## Rejection and liveness

Candidate feasibility and liveness are deliberately separate:

- A recoverable candidate rejection produces no `AcceptedArmTarget`.
- The session emits a fresh `ArmControlHeartbeat` in `HOLD_REJECTED`.
- The native worker holds the last safe emitted destination; a rejected target
  is not queued for later replay.
- Actual producer/input/IPC timeout or another hard fault safely stops.

The next feasible input can recover without restarting the control session.
This behavior is protected by shared-pipeline, singularity, output-feasibility,
and native-worker tests.

## Singularity policy

Actual full Jacobian quality is authoritative. Slowdown and hard-rejection use
condition number and minimum singular value, with hysteresis and directional
classification (toward, tangent, or away). J5 proximity to 15° remains warning
metadata; it is not a fixed hard gate. A safe retreat can therefore remain
possible without weakening the hard Jacobian boundary.

## Adapters

`MujocoArmTargetAdapter` writes the accepted six joints into the simulation
plant. `JakaAcceptedJointTargetAdapter` serializes the same J1–J6 radians in
absolute mode. The physical adapter contains no coordinate mapping, filter, IK,
branch selector, or feasibility policy. Native checks remain defensive
assertions against transport or implementation defects.
