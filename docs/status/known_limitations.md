# Known limitations

- The production shared output-acceleration feasibility correction and
  transition-limited/true-hold classification have bounded physical evidence,
  but the latest complete direction run ended in retained producer-liveness
  timeout rather than an overall run PASS. The new 20 ms producer budget passes
  that input offline but has not received a post-fix physical validation.
- The prior J4 collision alarm's cause is unresolved; payload mismatch was
  corrected by the operator but is not proven to be the sole cause.
- Controller-health fault propagation is covered offline. The sole-session
  lightweight polling timing path completed a bounded physical run, but an
  actual induced controller collision/estop event was not used as a validation
  method.
- TCP1–TCP10 are recorded as zero; no completed TCP calibration is claimed.
- RH56 PC-direct actual-device identity/read-only feedback, bounded commands,
  and short Quest hand-only teleoperation have physical evidence. Raw nonzero
  `STATUS` meanings remain unvalidated.
- The combined arm+hand entry passes offline fake/gate tests and has multiple
  short physical run records, but it has no complete, long-duration,
  all-gates PASS.
- Quest tracking loss stops teleoperation under the retained liveness policy.
- An RH56 PC-direct worker failure is persisted only as a generic fault; its
  original exception, traceback, serial operation, and errno are not retained
  in the episode summary.
- Combined termination reporting does not yet guarantee immutable
  first-cause preservation; a later transport or pause observation may replace
  the primary terminal reason.
- `rh56_commands` currently counts register-write attempts, not independently
  confirmed successful commands.
- Physical hand target/feedback continuity and target-to-feedback dynamics are
  only partially characterized.
- RealSense profile fallback is offline-tested but not validated on the target
  dual-camera hardware/profile combinations.
- Physical JAKA full-envelope translation/orientation is only partially
  validated and must not be expanded from historical small/bounded results.
- Quest Unity/APK/runtime version and current headset installation remain
  external facts; repository source audits do not prove the deployed build.
- The digital-twin workspace remains “Integrated Workspace,” not “Simulation
  Ready”; its documented failed trajectories and calibration tasks remain.
- HEBI teleoperation is retired but remains in the tree as compatibility
  reference code. The iPhone path remains experimental; neither is a current
  Quest/JAKA production entry.
- Vendor reference sources are retained as supplied and are not necessarily
  importable project modules.

---

# 中文版：已知限制

- 共享输出加速度修复及 transition-limited/true-hold 分类已有受限真机证据，但最新完整
  方向运行以保留的 producer-liveness timeout 结束，不能视为整次运行 PASS。
  新增 20 ms producer budget 已用该输入通过离线回放，但尚未进行修复后真机验证。
- 先前 J4 collision alarm 原因仍未解决；payload 不匹配已由操作者修正，但未证明是唯一
  原因。
- 控制器健康故障传播有离线覆盖，唯一 SDK 会话轻量轮询已通过一次受限真机时序运行；
  没有通过故意触发 collision/estop 来验证。
- TCP1–TCP10 记录为零，没有完成 TCP 标定。
- RH56 PC-direct 已有实际设备身份/read-only feedback、受限 command 和短时 Quest
  hand-only 真机证据；非零 raw `STATUS` 含义仍未验证。
- combined arm+hand 入口已通过离线 fake/gate 测试并有多次短时真机运行记录，但尚无
  完整、长期、所有 gate 均 PASS 的验证。
- Quest tracking 丢失会按保留的 liveness policy 停止遥操作。
- RH56 PC-direct worker failure 在 episode summary 中只保留通用 fault，原始 exception、
  traceback、serial operation 和 errno 尚未持久化。
- combined termination reporting 尚未保证 immutable first-cause preservation；后续
  transport 或 pause observation 仍可能覆盖 primary terminal reason。
- `rh56_commands` 当前统计 register-write 尝试次数，不是独立确认的成功 command 数。
- 真手 target/feedback 连续性及 target-to-feedback dynamics 只完成部分刻画。
- RealSense profile fallback 仅完成离线测试，尚未在目标双相机/profile 组合上验证。
- JAKA 全范围平移/旋转只完成部分真机验证，不能从历史小范围结果推断完整 envelope。
- Quest Unity/APK/runtime 版本和头显当前安装属于外部事实，源码审计不能证明已部署版本。
- 数字孪生仍为 “Integrated Workspace”，尚未达到 “Simulation Ready”。
- HEBI 遥操作已经停用，仅保留兼容参考代码；iPhone 路径仍为实验路径，两者都不是当前
  Quest/JAKA 生产入口。
- vendor 参考源码按原样保留，不一定能作为项目 Python 模块直接导入。
