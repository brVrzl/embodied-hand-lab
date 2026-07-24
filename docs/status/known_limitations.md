# Known limitations

- The shared output-acceleration feasibility correction at current HEAD is
  offline tested but not physically validated.
- The prior J4 collision alarm's cause is unresolved; payload mismatch was
  corrected by the operator but is not proven to be the sole cause.
- Controller-health fault propagation is covered offline. The sole-session
  lightweight polling timing path completed a bounded physical run, but an
  actual induced controller collision/estop event was not used as a validation
  method.
- TCP1–TCP10 are recorded as zero; no completed TCP calibration is claimed.
- Quest-driven physical RH56 teleoperation is not validated in the shared path.
- Physical JAKA full-envelope translation/orientation is only partially
  validated and must not be expanded from historical small/bounded results.
- Quest Unity/APK/runtime version and current headset installation remain
  external facts; repository source audits do not prove the deployed build.
- The digital-twin workspace remains “Integrated Workspace,” not “Simulation
  Ready”; its documented failed trajectories and calibration tasks remain.
- HEBI and iPhone paths remain parallel/compatibility workflows and do not
  share every Quest/JAKA contract.
- Vendor reference sources are retained as supplied and are not necessarily
  importable project modules.

---

# 中文版：已知限制

- 共享输出加速度修复已经离线测试，但尚未完成真机验证。
- 先前 J4 collision alarm 原因仍未解决；payload 不匹配已由操作者修正，但未证明是唯一
  原因。
- 控制器健康故障传播有离线覆盖，唯一 SDK 会话轻量轮询已通过一次受限真机时序运行；
  没有通过故意触发 collision/estop 来验证。
- TCP1–TCP10 记录为零，没有完成 TCP 标定。
- Quest 驱动真机 RH56 尚未验证。
- JAKA 全范围平移/旋转只完成部分真机验证，不能从历史小范围结果推断完整 envelope。
- Quest Unity/APK/runtime 版本和头显当前安装属于外部事实，源码审计不能证明已部署版本。
- 数字孪生仍为 “Integrated Workspace”，尚未达到 “Simulation Ready”。
- HEBI 和 iPhone 路径是并行/兼容流程，不共享全部 Quest/JAKA 契约。
- vendor 参考源码按原样保留，不一定能作为项目 Python 模块直接导入。
