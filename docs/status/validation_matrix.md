# Validation matrix

Statuses apply to the current source and preserved evidence. In this table:

- **offline tested** means deterministic tests, plant-free replay, or static
  validation without a device;
- **simulation validated** means the maintained MuJoCo path was loaded and
  exercised;
- **physical partial** means only the stated bounded configuration and
  envelope were exercised;
- **not validated** means implementation or documentation is not a PASS.

| Capability | Implementation / offline | Simulation | Physical | Current evidence and boundary |
| --- | --- | --- | --- | --- |
| Quest HTS/CTRL parsing, freshness, bounded queue | Offline tested | Validated | Input observed in bounded runs | Deployed headset build remains external |
| Release-before-press arm/hand clutch and reference capture | Offline tested | Validated | Partial | Post-EDG measured `q_hold` and no-jump startup contract |
| Frame mapping, translation/quaternion filtering | Offline tested | Validated | Partial | Right/up/forward directions observed; full envelope unproven |
| Shared continuation IK, branch, joint-limit, singularity, and collision checks | Offline tested | Validated | Partial | Maximum five backtracks, minimum fraction 1/32; J5 15 degrees is warning only |
| Immutable `AcceptedArmTarget` shared before adapters | Offline tested | Validated | Partial | MuJoCo and JAKA receive the same accepted J1--J6 radians |
| Output velocity feasibility | Offline tested | Validated | Partial | Checked before acceptance |
| Latest output-acceleration feasibility correction | Offline tested and replayed | Validated offline | **Not validated post-fix** | Current project-selected limit is 4π rad/s² |
| Candidate rejection and `HOLD_REJECTED` heartbeat | Offline tested | Validated | Latest acceleration recovery not validated | Holds the last safe target while producer liveness remains fresh |
| Native 125 Hz / 8 ms latest-destination PWL resampler | Offline and fake-worker tested | Replay validated | Partial | No MuJoCo `qpos` following |
| Native accepted-joint worker | Offline and fake-worker tested | Replay validated | Partial | Joint mode makes zero JAKA `kine_inverse` calls |
| Sole-session controller-health polling | Offline and fake-worker tested | n/a | Bounded timing path exercised | Collision/E-stop was not deliberately induced |
| MuJoCo JAKA arm adapter | Offline tested | Validated | n/a | Consumes shared accepted targets |
| Integrated MuJoCo RH56 | Offline tested | Validated approximation | n/a | Six position actuators plus six equality constraints; not complete physical underactuation |
| JAKA-only arm model | Offline tested | Validated | n/a | Exactly six JAKA actuators; RH56 command path absent |
| Physical JAKA translation/orientation | Offline path tested | Validated | Partial | Earlier larger motion ended in unresolved J4 collision |
| Physical RH56 PC-direct route | Scheduler/protocol tested offline | Hand model validated separately | Partial | Identity/read-only/bounded command and hand-only 15/30/40/50 Hz evidence; `fast40` selected |
| Combined Quest/JAKA/RH56 operation | Fake/gate tested offline | Integrated simulation validated | **One bounded 60.105 s PASS; no 300 s PASS** | Later 200.943 s run correctly hard-stopped after fresh CTRL reported `active=0`; latest acceleration fix remains physically unvalidated |
| Controller collision and other hard-fault propagation | Offline/fake-worker tested | Fake-worker validated | Not intentionally induced | Collision, alarm, E-stop, SDK, hard timing, and true liveness faults remain hard stops |
| TCP calibration | Interfaces/model frames tested | Frame tests pass | **Not validated** | TCP1--TCP10 recorded zero |
| Dual-D435 capture and synchronization | Adapter/profile fallback tested offline | n/a | **Not validated end to end** | Serial roles, extrinsics, and device-time alignment still require target-hardware work |
| Canonical episode lifecycle, validation, split, statistics, and export | Offline tested | Synthetic/offline fixtures | Not physically validated | Does not yet prove a production multimodal dataset |
| MuJoCo joint-reach/RH56 pre-shape benchmark | Offline tested | Smoke benchmark validated | n/a | No object task, grasp/lift metric, or sim-to-real claim |
| Distributed communication scaffolding | PyTorch 2.11 Linux/aarch64 Gloo one- and two-process collectives passed; `torchrun` parent exited cleanly | n/a | n/a | One-GPU NCCL collective passed on NVIDIA Thor; no multi-GPU, multi-node, model trainer, or Slurm execution |
| ACT / Diffusion Policy / OpenPI training | Integration boundary documented | n/a | n/a | No current trainer or trained checkpoint |
| Digital-twin workspace | Tools/config tests present | Integrated Workspace | n/a | Provisional geometry and unresolved calibration; not Simulation Ready |
| Foundation J6 gates | Historical implementation | n/a | Exact +0.25° and +5° gates passed | Historical bounded evidence, not full teleoperation |

The combined physical durations and limitations are detailed in
[current status](current_status.md#physical-evidence). The J6 statement is
limited to the dated
[minimal-joint validation](../history/gates/jaka_foundation_20260716/jaka_gate3c_minimal_joint_validation_20260716.md).
Earlier limitations are preserved in the
[physical parity audit](../history/incidents/quest_jaka_20260722_23/quest_jaka_physical_parity_audit_20260722.md)
and
[output-feasibility follow-up](../history/incidents/quest_jaka_20260722_23/quest_jaka_output_feasibility_followup_20260723.md).
Historical results never authorize a new physical gate.
