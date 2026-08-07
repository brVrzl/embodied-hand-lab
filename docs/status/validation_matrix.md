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
| Physical RH56 PC-direct route | Scheduler/protocol tested offline | Hand model validated separately | Partial | Identity/read-only/bounded command and fixed 40 Hz hand-only evidence |
| Combined Quest/JAKA/RH56 operation | Fake/gate tested offline | Integrated simulation validated | **One bounded 60.105 s PASS; no 300 s PASS** | Later 200.943 s run correctly hard-stopped after fresh CTRL reported `active=0`; latest acceleration fix remains physically unvalidated |
| Controller collision and other hard-fault propagation | Offline/fake-worker tested | Fake-worker validated | Not intentionally induced | Collision, alarm, E-stop, SDK, hard timing, and true liveness faults remain hard stops |
| TCP calibration | Interfaces/model frames tested | Frame tests pass | **Not validated** | TCP1--TCP10 recorded zero |
| Dual-D435 capture and synchronization | Adapter/profile fallback tested offline | n/a | **Not validated end to end** | Serial roles, extrinsics, and device-time alignment still require target-hardware work |
| Canonical episode lifecycle, validation, split, statistics, and export | Offline tested | Synthetic/offline fixtures | Not physically validated | Does not yet prove a production multimodal dataset |
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

---

# 中文版：验证矩阵

本表适用于当前 source 和保留证据：

- **offline tested**：确定性测试、无 plant replay 或静态检查，不使用设备；
- **simulation validated**：加载并运行维护的 MuJoCo 路径；
- **physical partial**：只验证了表中明确写出的有界配置和范围；
- **not validated**：实现或文档存在，但不能称为 PASS。

| 能力 | 离线/实现 | 仿真 | 真机 | 当前边界 |
| --- | --- | --- | --- | --- |
| Quest HTS/CTRL parsing、freshness、bounded queue | 已测试 | 已验证 | 有界运行中观察过输入 | headset 实际部署版本仍是外部事实 |
| release-before-press clutch/reference | 已测试 | 已验证 | 部分验证 | post-EDG measured hold 和 no-jump startup 合约 |
| frame mapping、filter、continuation IK、branch、limit、singularity、collision | 已测试 | 已验证 | 部分验证 | 最多五次 backtrack；J5 15° 是 warning |
| `AcceptedArmTarget` 共享边界 | 已测试 | 已验证 | 部分验证 | MuJoCo/JAKA 收到相同六关节 radians |
| output velocity/acceleration feasibility | 已测试 | 已验证 | 部分验证 | native 最终动态 hard check 仍保留 |
| 最新 acceleration correction | 已测试/replay | 离线已验证 | 修正后未验证 | 当前项目值为 4π rad/s² |
| candidate rejection 与 `HOLD_REJECTED` heartbeat | 已测试 | 已验证 | 最新修正未验证 | 保持最后 safe target，活性仍需新鲜 |
| native 125 Hz / 8 ms PWL worker | offline/fake-worker | replay | 部分验证 | 不跟随 MuJoCo `qpos` |
| sole-session JAKA health polling | offline/fake-worker | 不适用 | 有界 timing path | 未主动制造 collision/E-stop |
| RH56 PC-direct | scheduler/protocol 已测 | hand model 已测 | 部分验证 | identity/read-only/bounded hand evidence |
| combined Quest/JAKA/RH56 | fake/gate 已测 | integrated simulation | 60.105 s bounded PASS；无 300 s PASS | producer liveness 和最新 acceleration fix 仍有限制 |
| hard-fault propagation | offline/fake-worker | fake-worker | 未主动诱发 | alarm、collision、E-stop、SDK、timing、liveness 仍 hard stop |
| TCP calibration | frame/model tests | frame tests | 未验证 | TCP1--TCP10 记录为零 |
| dual-D435 capture/sync | adapter/profile offline | 不适用 | end-to-end 未验证 | serial、extrinsic、time alignment 待验证 |
| episode lifecycle、validation、split、export | offline tested | synthetic/offline | 未验证 | 不等于 production multimodal dataset |
| policy training | 只有边界 | 不适用 | 不适用 | 没有维护中的 trainer |

联合真机时长和限制详见[当前状态](current_status.md#physical-evidence)。历史 gate 和 incident
只能证明当时的有界结果，不能授权新的真机运行。
