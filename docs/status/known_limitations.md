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
- Quest-driven physical RH56 teleoperation is not validated in the shared path.
- RH56 PC-direct identity, read-only feedback, bounded command behavior, and
  raw status/error meanings have not been validated on the physical hand. The
  combined arm+hand entry exists and passes offline fake/gate tests, but has not
  been run against either physical device as a combined system.
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
- Quest 驱动真机 RH56 尚未验证。
- RH56 PC-direct 的实际设备身份、只读 feedback、受限 command 行为以及 raw
  status/error 含义均未在真手上验证。combined arm+hand 入口已实现并通过离线 fake/gate
  测试，但尚未作为联合系统连接两台真机验证。
- JAKA 全范围平移/旋转只完成部分真机验证，不能从历史小范围结果推断完整 envelope。
- Quest Unity/APK/runtime 版本和头显当前安装属于外部事实，源码审计不能证明已部署版本。
- 数字孪生仍为 “Integrated Workspace”，尚未达到 “Simulation Ready”。
- HEBI 遥操作已经停用，仅保留兼容参考代码；iPhone 路径仍为实验路径，两者都不是当前
  Quest/JAKA 生产入口。
- vendor 参考源码按原样保留，不一定能作为项目 Python 模块直接导入。
