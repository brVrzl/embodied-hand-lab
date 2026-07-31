# Known limitations

This page records current boundaries, not a backlog of hypothetical features.
Offline tests, replay, and MuJoCo results are not physical PASS evidence.

## Physical arm and combined operation

- The combined arm/RH56 path has one bounded 60.105-second physical PASS for
  its recorded configuration and motion envelope. A later run reached 200.943
  seconds with no hard timing fault, then fresh CTRL packets reported
  `active=0`; the retained liveness policy correctly stopped with
  `producer_liveness_loss`. There is no 300-second PASS.
- The latest shared output-acceleration feasibility correction is offline
  tested but has not received a bounded post-fix physical validation. The
  earlier 60.105-second result must not be used as evidence for that change.
- The cause of the earlier J4 collision alarm remains unresolved. The operator
  corrected the recorded payload mismatch, but that does not prove payload was
  the sole cause.
- Translation and orientation have only bounded, partial physical coverage.
  Historical small-motion results do not establish the full workspace or
  orientation envelope.
- TCP1 through TCP10 are recorded as zero. A completed TCP calibration is not
  claimed.
- The controller-health path uses lightweight polling through the sole JAKA
  SDK session. Its timing path has bounded physical evidence, but collision or
  E-stop was not deliberately induced as a validation method.
- Quest tracking/controller invalidity remains a hard liveness stop. It is not
  an accepted-target rejection and must not be converted into
  `HOLD_REJECTED`.

## RH56DFX hand

- PC-direct identity checking, read-only feedback, bounded commands, and short
  Quest hand-only operation have physical evidence. Complete long-duration
  Quest-driven hand teleoperation and target-to-feedback characterization are
  not finished.
- `ANGLE_ACT` is feedback for the six commanded actuator axes. `CURRENT`,
  `FORCE_ACT`, `ERROR`, and `STATUS` are raw device-register fields. They are
  not complete passive-joint state, calibrated contact force, tactile, or slip
  sensing. Meanings of nonzero `STATUS` values remain unvalidated.
- The combined summary field `rh56_commands` counts backend register-write
  attempts. Worker diagnostics separately report successful serial writes;
  neither count proves that the hand reached a commanded pose.
- The selected `fast40` scheduler has bounded evidence, but physical target
  continuity, feedback latency, and behavior across the full safe command
  range remain only partially characterized.

## Simulation and digital twin

- The integrated MuJoCo hand is a six-position-actuator approximation. Six
  equality constraints approximate coupled RH56 joints; tendon compliance,
  backlash, current/force control, calibrated force limits, and complete
  physical underactuation are not modeled.
- The live viewer can inject a provisional table and mounting geometry.
  `SharedJakaTargetGenerator` uses the base MJCF, so the table is not part of
  shared pre-acceptance collision authority. The scene is not proof of
  physical workspace clearance.
- The digital twin remains an **Integrated Workspace**, not **Simulation
  Ready**. Calibration tasks and documented failed trajectories remain open.
- Contact count and joint pre-shape in the current smoke benchmark do not
  establish grasp, lift, retention, placement, or sim-to-real performance.

## Cameras, data, and learning

- RealSense profile fallback is offline tested but has not been validated on
  the target dual-D435 hardware/profile combinations.
- Camera-to-robot extrinsics, cross-device time synchronization, and a complete
  dual-camera physical episode capture have not been validated end to end.
- Canonical episode validation, manifest/statistics tooling, selected export
  paths, and an offline MuJoCo smoke benchmark exist. They do not constitute a
  validated production dataset or policy-training run.
- No current ACT, Diffusion Policy, or OpenPI/pi0 trainer consumes the
  repository training example. Distributed utilities are infrastructure
  scaffolding, not model-training support.
- Jetson Thor collection/inference and model export have documentation
  contracts but no validated deployment in this repository.

## External runtime facts

- Quest Unity/APK/runtime version and the build installed on the headset remain
  external facts. A source audit cannot prove which build is deployed.
- The iPhone/MediaPipe route is experimental and is not a current
  Quest/JAKA production entry.
- Vendor reference sources are retained as supplied and are not necessarily
  importable project modules.

Current evidence is summarized in the [validation matrix](validation_matrix.md)
and [current status](current_status.md). Dated reports in the
[history index](../history/README.md), including the
[Quest/JAKA output-feasibility follow-up](../history/incidents/quest_jaka_20260722_23/quest_jaka_output_feasibility_followup_20260723.md),
remain evidence only and do not override current source or safety policy.
