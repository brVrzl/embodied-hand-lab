# Simulation and hardware parity

## Contract

Simulation and physical execution share validated Quest facts, clutch state,
reference capture, mapping, filters, continuation, IK, collision and
singularity checks, output feasibility, and the immutable accepted target.
They diverge only at the output adapter.

```text
                         +-> MuJoCo accepted-joint adapter
AcceptedArmTarget -------+
                         +-> JAKA accepted-joint adapter -> EDG worker
```

The hardware path must not:

- read or follow MuJoCo `qpos`;
- independently recompute IK;
- repeat the latest target as a staircase;
- replay a queue of stale destinations;
- write controller configuration.

In current joint-teleop mode, native JAKA inverse-kinematics call count must
remain zero. The worker emits `edg_servo_j(..., ABS, 1)` at a target period of
8 ms. Its piecewise-linear resampler continuously moves from the last emitted
point toward the latest accepted destination. A newer destination replaces the
active segment without building a backlog.

## Startup and stop

The measured post-EDG joint state (`q_hold`) is authoritative. The first
accepted target must be continuous with it; clutch reference capture alone
cannot legalize a joint jump. Release, timeout, tracking error, controller
fault, SDK error, or hard loop-timing fault terminates output and runs cleanup.

The current worker uses the sole JAKA SDK session. Every two 8 ms cycles it
performs the lightweight status read. Only an unhealthy status triggers the
additional emergency-stop/collision queries. A prior two-session monitoring
design was tried physically and failed safely because the second login kept the
primary worker from reaching `CONNECTED`; it is historical, not current.

## Evidence boundary

Offline fake-worker tests verify serialization, resampling, startup,
latest-destination behavior, zero native IK, timing accounting, controller
health policy, and cleanup. Historical physical evidence validates selected
foundation and timing behavior only. The latest output-acceleration correction
has not yet received a post-fix physical gate.
