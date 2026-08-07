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
- The fixed 40 Hz scheduler has bounded evidence, but physical target
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
- Camera-to-robot extrinsics, cross-device time synchronization, and the
  integrated dual-camera physical v2 episode path have not been physically
  validated end to end.
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

---

# 中文版：已知限制

本页记录当前边界，不是对假想功能的 backlog。离线测试、回放和 MuJoCo 结果都不是真机 PASS
证据。

## 机械臂与联合运行

- 联合 arm/RH56 路径只有一次针对特定配置和运动范围的 60.105 秒真机 PASS；没有 300 秒 PASS。
- 最新 output-acceleration 修正已离线测试，但尚未完成修正后的有界真机验证。
- 早期 J4 collision alarm 的根因仍未解决；修正记录中的 payload 不匹配不等于证明了唯一原因。
- 平移和旋转只覆盖了部分有界真机范围，不能推断完整 workspace 或 orientation envelope。
- TCP1--TCP10 仍记录为零，不能声称 TCP calibration 已完成。
- controller health 使用唯一 JAKA SDK session 的轻量 polling；没有通过主动制造 collision 或 E-stop 来验证。
- Quest tracking/controller invalidity 属于 hard liveness stop，不得转换为普通 `HOLD_REJECTED`。

## RH56DFX

- PC-direct identity、read-only feedback、有界命令和短时 hand-only 有真机证据，但长期 Quest-driven hand teleop 和完整 target/feedback characterization 尚未完成。
- `ANGLE_ACT` 是六个 commanded actuator axis 的 feedback；`CURRENT`、`FORCE_ACT`、`ERROR` 和 `STATUS` 是 raw register，不是完整 passive-joint、tactile、slip 或 calibrated contact-force state。
- 非零 `STATUS` 的语义没有验证；不能猜测 code 含义。
- fixed 40 Hz scheduler 有界测试证据存在，但全 safe command range 的 physical continuity、feedback latency 和行为仍未完成刻画。

## 仿真、相机和数据

- MuJoCo hand 是六 position actuator 的近似，不模拟 tendon compliance、backlash、current/force control、calibrated force limit 或完整 underactuation。
- viewer 中的 provisional table/mounting geometry 不属于 shared pre-acceptance collision authority，也不能证明真机 workspace clearance。
- 双 D435 serial/profile、camera-to-robot extrinsic、跨设备时间同步和 end-to-end physical episode capture 尚未真机验证。
- episode validation、manifest/statistics 和 exporter 已有离线支持，但不代表 production dataset 或 policy training 已验证。
- 仓库没有当前维护的 ACT、Diffusion Policy 或 OpenPI/pi0 trainer，也没有 Jetson Thor deployment PASS。

## 外部事实

- Quest Unity/APK/runtime 版本和 headset 中实际安装的 build 是外部事实，源码审计不能证明。
- iPhone/MediaPipe 仍是实验路径，不是当前 Quest/JAKA production entry。
- vendor reference source 只是保留的外部资料，不保证是可导入的项目模块。

以上限制由[验证矩阵](validation_matrix.md)和[当前状态](current_status.md)汇总；
[历史索引](../history/README.md)中的 dated evidence 只作为证据，不能覆盖当前 source 或 safety policy。
