# Repository consolidation audit

Date: 2026-07-24. Worktree: `/home/thor/projects/embodied_lab`. Branch:
`feature/jaka-teledex-control-foundation`.

## Recovery checkpoint

Before deletion, local HEAD and
`origin/feature/jaka-teledex-control-foundation` were both
`e1afa45dc5e1c8ea58cf7c2fe82a044f26253f7c` with ahead/behind `0/0`.
Every tracked deletion in this cleanup is therefore recoverable from GitHub.
The concurrent modification to `tools/teleop_mujoco_jaka_rh56.py`, untracked
`learned_policy/`, linked worktrees, captures and local artifacts were excluded.

## Removed areas

The operator explicitly retired these independent research paths:

- geometry/tactile pregrasp prediction and dataset generation;
- ManiSkill/SAPIEN tasks, agent, viewer, recorder and dependencies;
- Xbox and TeleDex input/control paths;
- episode/data recorder and its provisional collection protocol;
- tennis-ball grasp flows and grasp benchmark;
- legacy RH56 collision-mode comparisons, proxy diagnostics and generated-stage
  research tools;
- pure research plans and current-document descriptions of removed features.

Tests dedicated only to those deleted features were removed with them. Safety,
shared Quest/JAKA, simulation parity, native EDG, RH56 driver, RealSense,
digital-twin and retained HEBI/iPhone tests remain.

## Retained authority and assets

- Quest HTS/CTRL to immutable `AcceptedArmTarget`, MuJoCo adapter and JAKA
  representation-only adapter;
- MuJoCo JAKA+RH56 runtime and committed `visual_coacd` collision asset, builder,
  manifest and safety regression;
- ROS2/RViz, HEBI phone experiments, iPhone RH56 experiments, physical RH56
  driver and bounded JAKA diagnostics;
- RealSense calibration/point-cloud work and the integrated digital twin;
- physical gate and incident evidence;
- Correll RH56DFX source assets plus upstream MIT license as a reference only.

The Correll reference is not a mounted runtime collision mode. The sole
supported RH56 runtime collision representation is the committed CoACD asset.

## HEBI smoothness review

HEBI's perceived smoothness comes from a separate lag-follow architecture:
target low-pass filtering, Cartesian velocity/acceleration shaping, lead and
workspace limits, tracking-error pauses, and optional SDK Servo filtering.
Copying those after `AcceptedArmTarget` would violate Quest/JAKA parity and
reintroduce hardware-only shaping.

Two ideas are reusable and already represented in the current stack: explicit
deadman/reference state, and tracking error staged as warning/hold/fault. HEBI
does not provide evidence that the unresolved Quest/JAKA collision issue should
be hidden by a second filter. The shared output-feasibility and physical
controller contracts remain the correct authority.

## Documentation policy

Current instructions remain indexed by `docs/README.md`. Digital-twin current
status is consolidated in `docs/digital_twin/README.md`; calibration, capture,
registration and measurement evidence remain beside it. Unique physical gate
and incident evidence is preserved under `docs/history/`. Deleted research
plans do not remain as competing current guidance.
