# Execution roadmap

## Starting point

The repository currently has a simulation-validated Quest/JAKA target path,
offline-tested safety and data contracts, a PC-direct RH56 implementation,
atomic canonical episodes, an offline MuJoCo smoke benchmark, and optional
distributed-runtime checks. It does not yet have a physically validated
multimodal collection system or a maintained policy trainer.

The order below is deliberate. A later phase must not be used to bypass an
earlier safety, calibration, or data-quality dependency.

| Phase | Work location | Primary result |
| --- | --- | --- |
| 0. Transfer and reproduce | Offline, next workstation | Known-good software baseline |
| 1. Resolve arm validation blockers | Offline, then separately gated physical | Bounded post-fix arm evidence |
| 2. Characterize RH56 | Offline, then separately gated physical | Trusted six-axis command/feedback envelope |
| 3. Calibrate cameras and time | Offline tools plus authorized cameras/robot | Versioned geometry and clock model |
| 4. Pilot collection | Separately gated physical | Small accepted multimodal corpus |
| 5. Dataset qualification | Offline | Frozen split, statistics, and quality report |
| 6. ACT baseline | Training server | First closed-loop learning baseline |
| 7. Diffusion Policy baseline | Training server | Second independently evaluated baseline |
| 8. OpenPI integration decision | Training server research | Evidence-backed go/no-go and adapter |
| 9. Task benchmark | Offline MuJoCo first | Resettable object-task harness |
| 10. Sim-to-real and deployment | Replay, Jetson, then gated physical | Measured deployment package |

## Phase 0 — transfer and reproduce

**Inputs**

- This repository copy, including preserved models, calibration material, and
  evidence.
- A Linux workstation or training host with Python 3.10 or newer.

**Actions**

1. Preserve the transferred directory as a source bundle before editing it.
2. Run the installation and validation commands from
   `docs/maintenance/FINAL_REPOSITORY_REVIEW.md`.
3. Save `embodied-lab doctor --json` output.
4. On Linux, build the native JAKA resampler/worker targets without connecting
   to a controller.
5. On a GPU host, run the PyTorch capability check and CPU/Gloo smoke before
   attempting NCCL.

**Outputs**

- Host inventory, package versions, test report, native build report, and
  benchmark JSON tied to the transferred source bundle.

**Acceptance**

- YAML parses; CLI help and headless simulation pass; the default offline
  suite has no unexpected failure; platform-specific skips are explained.

**Dependencies and risks**

- Optional robotics/learning wheels may lag the host Python release.
- The current copy has no project-local Git metadata, so record a source-bundle
  hash rather than inventing a revision.

## Phase 1 — resolve arm validation blockers

**Inputs**

- Offline-passing shared pipeline and native fake-worker tests.
- Controller-side verification of payload, installation, TCP, safety limits,
  alarms, and frames.
- A new authorization for one exact bounded physical gate.

**Actions**

1. Reproduce and analyze the earlier J4 collision envelope offline; do not
   weaken collision, joint, timing, tracking, or liveness checks.
2. Complete TCP calibration through an approved operator procedure outside
   automatic repository configuration writes.
3. Validate the latest shared output-acceleration correction using the smallest
   useful displacement, duration, and joint envelope.
4. Preserve the first terminal reason and native/controller evidence.
5. Expand one variable at a time only after review.

**Outputs**

- A dated gate report, raw metrics, exact configuration snapshot, and explicit
  PASS/FAIL for only that envelope.

**Acceptance**

- Startup is continuous; native joint mode reports zero IK calls; no controller
  alarm, hard timing miss, tracking crossing, or liveness loss occurs; output
  velocity and acceleration remain within the retained limits.

**Dependencies and risks**

- Requires real JAKA access, E-stop access, a clear workspace, and explicit
  authorization.
- The earlier J4 collision cause is unresolved and is a blocker to broader
  motion, not an invitation to raise limits.

## Phase 2 — characterize RH56DFX

**Inputs**

- Verified PC-direct serial identity and the current `fast40` profile.
- A separately authorized hand-only gate.

**Actions**

1. Confirm that opening the transport produces zero register writes.
2. Record command, `ANGLE_ACT`, `CURRENT`, `FORCE_ACT`, `ERROR`, and `STATUS`
   under small single-axis and representative multi-axis trajectories.
3. Measure command-to-feedback delay, steady discrepancy, repeatability,
   saturation, and disconnect/stale-feedback behavior.
4. Validate release/hold and deterministic cleanup.
5. Perform Quest-driven hand trials only after the measured-first clutch and
   target bounds pass the hand-only gate.

**Outputs**

- Versioned actuator envelope, latency/quality report, and accepted feedback
  semantics.

**Acceptance**

- No protocol/checksum/stale/error fault; commands stay inside the configured
  range/rate/delta boundaries; the first command is continuous with fresh
  `ANGLE_ACT`; faults suppress new writes.

**Dependencies and risks**

- `CURRENT` and `FORCE_ACT` remain raw proxies. They must not be relabelled as
  tactile state, slip, or calibrated contact force.
- Passive finger-joint state is not available from the six actuator fields.

## Phase 3 — camera geometry and time

**Inputs**

- Two identified D435 devices, stable USB topology, target image modes, robot
  state timestamps, and a calibration target.

**Actions**

1. Record serial numbers, firmware, stream modes, intrinsics, depth scale, and
   USB bus assignment.
2. Calibrate external-camera-to-robot and wrist-camera-to-tool transforms with
   versioned inputs and residuals.
3. Measure device time to host monotonic receive time. Preserve device,
   source, receive, and canonical sample times as distinct fields.
4. Measure jitter, dropped/duplicate/out-of-order frames, camera freeze
   detection, and arm/hand/camera alignment under motion.
5. Choose a training sample rate only after measuring source rates and
   alignment error.

**Outputs**

- Calibration bundle, clock model, quality report, and updated collection
  configuration.

**Acceptance**

- Transform residual and clock-alignment thresholds are selected before
  collection, then met on held-out calibration motions; device identity and
  calibration hashes are recorded in each episode.

**Dependencies and risks**

- Requires authorized camera access and, for hand-eye calibration, an
  separately safe robot procedure.
- Software receive time cannot be silently substituted for missing device
  time.

## Phase 4 — first physical dataset

**Inputs**

- Accepted arm and hand gates, accepted camera/time calibration, canonical
  schema, a written task protocol, and operator labels.

**Actions**

1. First run aborted/failed/successful lifecycle drills without policy
   training.
2. Collect a small pilot across the intended object and initial-state
   variation. Do not scale episode count until quality review passes.
3. Finalize each episode atomically; preserve interrupted work as explicitly
   incomplete rather than renaming it complete.
4. Record task/object/initial state/operator/hardware/calibration/environment
   metadata and an explicit success or failure category.
5. Review RGB/depth playback, state/action plots, synchronization, and terminal
   events after every short batch.

**Outputs**

- Canonical episode directories, dataset manifest, quality reports, and a
  frozen pilot split.

**Acceptance**

- No corrupt final episode; monotonic canonical timestamps; all arrays have
  schema-consistent length and shape; action bounds hold; missing/repeated
  frames are marked; calibration and source provenance exist; labels are
  complete.

**Dependencies and risks**

- Physical collection requires a separate authorization and does not begin
  merely because the data writer passes offline tests.
- A technically readable episode with bad synchronization is not
  training-eligible.

## Phase 5 — dataset qualification

**Inputs**

- Pilot episodes and their deep-validation reports.

**Actions**

1. Freeze episode-level train/validation/test membership before windowing.
2. Group by task, object, scene, or capture session where needed to prevent
   leakage.
3. Compute normalization only from the train split.
4. Report missing frames, cadence, latency proxies, command-feedback
   discrepancy, all-zero channels, saturation, duration, and label balance.
5. Export a small ACT HDF5 subset and test round-trip shapes and camera order.
6. Keep canonical episodes as the source of truth; derived formats receive
   hashes and conversion metadata.

**Outputs**

- Versioned manifest, split rationale, normalization statistics, export
  reports, and rejected-episode list.

**Acceptance**

- Re-running with the same seed and corpus yields the same split; no episode
  crosses splits; statistics exclude validation/test; every accepted episode
  passes deep validation.

**Dependencies and risks**

- Do not repair bad synchronization by silently interpolating across long
  gaps.
- Shared filesystems may require sharding or local cache only after profiling
  demonstrates a bottleneck.

## Phase 6 — ACT minimum closed loop

**Inputs**

- Qualified canonical dataset, ACT adapter/export, frozen observation/action
  schema, normalization, and a GPU training host.

**Actions**

1. Pin an upstream ACT revision outside this repository.
2. Overfit a tiny training-only subset in FP32 on one GPU.
3. Adapt the pinned reference's hard-coded 14-D ALOHA paths to the 12-D
   project embodiment, then train a small baseline with explicit camera order,
   action dimension, chunk size, padding mask, KL settings, batch semantics,
   and seed.
4. Validate open-loop action error and chunk behavior on the frozen validation
   split.
5. Export inference weights plus config/schema/normalization hashes.
6. Replay inference on recorded episodes before considering a robot.

**Outputs**

- Reproducible run record, checkpoints, validation metrics, inference export,
  and replay latency report.

**Acceptance**

- Tiny-set overfit succeeds; resume reproduces step/optimizer/scheduler state;
  camera/action ordering is asserted; replay output is finite and bounded;
  inference meets a stated latency budget on the deployment candidate.

**Dependencies and risks**

- Single GPU is the correctness baseline. Use DDP only after the same trainer
  passes single-process and CPU/Gloo communication checks.
- ACT success is not inferred from training loss alone.

## Phase 7 — Diffusion Policy minimum closed loop

**Inputs**

- The same frozen data split and schema used for ACT.

**Actions**

1. Pin an upstream Diffusion Policy revision outside this repository.
2. Define observation, prediction, and action horizons; diffusion steps;
   camera order; depth policy; and action normalization.
3. Overfit a tiny subset, then train a baseline with EMA state included in
   checkpoints.
4. Evaluate sampling cost, stochastic variance, action smoothness, and replay
   latency separately from training throughput.
5. Compare with ACT using the same split and task metrics.

**Outputs**

- Resumable model/optimizer/scheduler/EMA checkpoint, inference export, and
  comparable evaluation report.

**Acceptance**

- EMA is consistent after resume and across ranks if DDP is used; validation
  seeds are recorded; generated actions remain finite and bounded; replay
  latency fits the intended control interface.

**Dependencies and risks**

- Validation sampling can dominate runtime. Do not duplicate the full
  validation set on every rank without accounting for it.
- Horizon or chunk changes alter deployment semantics and require a new export.

## Phase 8 — OpenPI/π0 decision

**Inputs**

- Qualified dataset, language/task metadata, substantial accelerator capacity,
  and an identified upstream OpenPI release.

**Actions**

1. Follow the selected upstream framework's native JAX or PyTorch distribution
   and checkpoint system; do not wrap it in a second local DDP/FSDP layer.
2. Build an external adapter from canonical episodes to required image,
   language, state, and action inputs.
3. Document tokenizer, camera/image preprocessing, action representation,
   normalization, and checkpoint conversion.
4. Run a small data-loader and forward/inference test before allocating a
   large training job.
5. Decide whether fine-tuning benefit justifies infrastructure and deployment
   complexity.

**Outputs**

- Adapter contract, compatibility matrix, resource measurement, and explicit
  go/no-go decision.

**Acceptance**

- No source schema is mutated to mimic the framework; converted samples pass
  shape/semantic checks; inference exports can be reproduced from a pinned
  upstream revision.

**Dependencies and risks**

- Large-model training may require upstream FSDP/mesh sharding and more memory
  than ACT or Diffusion Policy.
- OpenPI support is currently planned, not implemented.

## Phase 9 — MuJoCo task benchmark

**Inputs**

- Current smoke benchmark, a maintained object scene, calibrated contacts, and
  explicit task semantics.

**Actions**

1. Add one resettable reach-and-pre-shape or grasp-acquisition task before a
   task suite.
2. Define reset, seed, observation, action, termination, success, failure
   reason, recording, and result export through one interface.
3. Add object-pose variation only after deterministic baseline behavior.
4. Introduce lift/hold/transport/place/release metrics only when object state
   makes them observable.
5. Separate simulator-only metrics from those needing external perception or
   physical sensing.

**Outputs**

- Versioned task configuration, deterministic smoke result, failure taxonomy,
  and comparable aggregate summary.

**Acceptance**

- Same seed/config/source produces the same initial conditions and stable
  outcome; reset leaves no hidden state; each metric has an observable
  definition; no pre-shape result is called a grasp.

**Dependencies and risks**

- RH56 coupling and contact parameters are approximate and need calibration.
- Simulation success cannot be promoted to physical success.

## Phase 10 — sim-to-real and Jetson Thor

**Inputs**

- Selected checkpoint, frozen schema and normalization, replay corpus,
  benchmark evidence, and a Jetson-compatible runtime.

**Actions**

1. Export model or EMA weights, resolved config, input/output schemas, camera
   order, image size, action bounds, frequency, and hashes.
2. Establish x86 reference outputs on fixed replay samples.
3. Move to Jetson, verify operators, and compare FP32/FP16/BF16 where actually
   supported.
4. Consider ONNX/TensorRT only after native inference works; quantify output
   error after conversion.
5. Measure preprocessing, transfer, forward, postprocessing, and end-to-end
   latency under load.
6. Run replay and no-actuator shadow mode before requesting a separately
   authorized physical policy gate.
7. Progress through observation-only, bounded one-axis/short-envelope, then
   task gates; stop on the first hard fault.

**Outputs**

- Deployment bundle, compatibility and numerical-difference report, latency
  profile, replay result, and gated physical evidence when authorized.

**Acceptance**

- Deployment consumes exactly the frozen schema; outputs agree within a
  predeclared tolerance; latency meets the intended control deadline; stale
  observations and non-finite/out-of-range actions stop safely.

**Dependencies and risks**

- Jetson is a collection/inference target, not the default large-scale
  training host.
- TensorRT support depends on the selected model's actual operators and shapes.

## Priority and experiment location

| Priority | Work | Offline possible | Real hardware required |
| --- | --- | --- | --- |
| P0 | Transfer reproduction, dataset validation, benchmark, trainer adapters | Yes | No |
| P0 | J4 cause analysis, TCP plan, post-fix gate preparation | Mostly | Final evidence |
| P1 | RH56 characterization and camera/time calibration | Tooling yes | Final evidence |
| P1 | Pilot data quality and ACT baseline | QA/training yes | Collection |
| P2 | Diffusion Policy baseline and object benchmark | Yes after data/assets | Final task validation |
| P3 | OpenPI study, Jetson optimization, broader sim-to-real | Mostly | Final deployment gates |

At every phase, keep offline, simulation, replay, physical PASS, physical
FAIL, and unverified claims distinct.
