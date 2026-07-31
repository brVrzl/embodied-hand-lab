# Offline benchmark harness

## Supported scope

The repository currently provides one engineering smoke benchmark:
`mujoco_joint_reach_preshape_smoke`.

It creates a fresh `JakaMujocoSimulation`, samples a small seeded joint-target
jitter, checks the target against the committed MuJoCo model limits, and calls
the existing public output methods:

- `set_accepted_arm_joint_target`
- `set_hand_actuator_target`
- `step`

The benchmark uses the six simulated JAKA actuated joint positions and the six
RH56 actuator-driven joint positions as its observations. A run passes only
when both final errors are within the configured tolerances.

This is an offline plant smoke test. Targets enter the MuJoCo adapter directly,
so it does not validate Quest input, mapping, filtering, continuation IK,
shared acceptance, the physical adapter, or a robot controller.

## Run

From the repository root:

```bash
.venv/bin/python tools/run_benchmark.py \
  configs/benchmark/smoke.yaml \
  --output artifacts/benchmarks/mujoco-smoke.json
```

The default config argument is `configs/benchmark/smoke.yaml`, so the shorter
equivalent is:

```bash
.venv/bin/python tools/run_benchmark.py \
  --output artifacts/benchmarks/mujoco-smoke.json
```

Override only the seeded target jitter with:

```bash
.venv/bin/python tools/run_benchmark.py \
  --seed 43 \
  --output artifacts/benchmarks/mujoco-smoke-seed43.json
```

The result is written through a temporary file in the output directory and an
atomic same-filesystem replacement. Exit status is:

- `0`: configured tracking criteria passed
- `1`: simulation completed but a tolerance was not met
- `2`: invalid configuration, action boundary, or unavailable model

A benchmark `passed` status applies only to these configured offline metrics.
It is not a physical validation PASS.

## Configuration

The YAML schema is
`embodied_lab.mujoco_joint_reach_preshape.v1`. Unknown fields are rejected.
The current fields define:

- benchmark ID and uint64 seed
- the existing Quest/JAKA replay configuration used to construct MuJoCo
- fixed duration and control period
- six arm target offsets and small symmetric seeded jitter bounds, in radians
- arm final-position tolerance, in radians
- six named simulated RH56 actuator targets, in radians
- hand actuator-driven-joint tolerance, in radians

Duration must be an integer multiple of the control period. Every sampled arm
target must remain inside the model joint ranges, and every hand target must
remain inside the six actuator control ranges. These are MuJoCo model bounds,
not a replacement for the shared physical safety pipeline.

## Result contract

`embodied_lab.benchmark_result.v1` records:

- status and explicit failure reason
- seed
- complete effective benchmark snapshot and source SHA-256
- complete referenced replay-config snapshot, path, and SHA-256
- model path and SHA-256
- MuJoCo version, timestep, gravity mode, and reset method
- sampled target and jitter
- model action bounds
- steps and simulated duration
- final arm and hand errors
- completion step/time
- simulated joint and command speed summaries
- TCP displacement and maximum MuJoCo contact count
- explicit limitations

Contact count is diagnostic only. It is not used as evidence of grasp,
retention, or task success.

## What is not implemented

The current model has no maintained benchmark object/task layer. This harness
therefore does not claim or score:

- grasp acquisition
- object lift, hold, transport, placement, or release
- disturbance resistance
- contact-force accuracy
- tactile contact or slip
- camera-based object pose
- physical robot behavior
- sim-to-real agreement

The RH56 equality-coupled model is an approximation of the underactuated hand.
The six driven-joint pre-shape metric must not be interpreted as complete
passive-joint state or real-hand feedback.

Add object tasks only after a maintained resettable object scene, success
predicate, collision/contact calibration, and representative regression
evidence exist. Real evaluations must use a separate physical gate and retain
the repository safety requirements.

## Offline validation

The focused test is:

```bash
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m pytest -q \
  -p no:cacheprovider tests/test_benchmarking.py
```

It checks strict configuration, same-seed fresh-instance determinism, a model
action-bound rejection, atomic result replacement, CLI help, and one complete
headless MuJoCo smoke run. No hardware is opened or commanded.
