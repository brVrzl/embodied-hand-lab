# Validation matrix

Status is scoped to the current implementation and preserved evidence. “Offline
tested” and “simulation validated” do not mean physically passed.

| Capability | Implementation/offline | Simulation | Physical | Latest evidence / limitation |
|---|---|---|---|---|
| Quest packet/input parsing | implemented; tested | validated | input previously observed | HTS/CTRL provider tests; deployed APK still external |
| Wrist pose | implemented; tested | validated | partially observed | bounded historical Quest/JAKA runs |
| Controller clutch | implemented; tested | validated | partially validated | release-before-press and bounded runs |
| Reference capture/startup continuity | implemented; tested | validated | partially validated | post-EDG `q_hold`; no-jump contract |
| Coordinate mapping | implemented; tested | validated | partially validated | full envelope not proven |
| Translation/quaternion filters | implemented; tested | validated | partially exercised | current YAML and filter tests |
| Shared IK | implemented; tested | validated | partially exercised | MuJoCo model used plant-free |
| Continuation/branch policy | implemented; tested | validated | partially exercised | max 5 backtracks, min 1/32 |
| Jacobian singularity policy | implemented; tested | validated | not fully validated | J5 15° is warning only |
| Output velocity feasibility | implemented; tested | validated | partially exercised | checked pre-acceptance |
| Output acceleration feasibility | implemented; tested/replayed | validated offline | **not yet validated** | latest 4π rad/s² correction |
| `HOLD_REJECTED` recovery | implemented; tested | validated | not yet validated for acceleration fix | holds with fresh heartbeat |
| Native resampler | implemented; tested/fake worker | replay validated | partially validated | 125 Hz, 8 ms, latest destination |
| Native joint worker / zero native IK | implemented; tested | fake-worker validated | partially validated | joint mode `kine_inverse` count zero |
| Controller-health monitor | implemented; tested | fake-worker validated | timing path passed bounded run | sole-session lightweight polling |
| MuJoCo arm | implemented; tested | validated | n/a | shared accepted-target adapter |
| RH56 integrated simulation | implemented; tested | validated | n/a | 6 arm + 6 hand actuators; relative six-channel grip retarget |
| JAKA-only arm model | implemented; tested | validated | n/a | exactly 6 JAKA actuators; RH56 command path absent |
| Physical JAKA translation | implemented; tested offline | validated | partial | larger run ended in J4 collision |
| Physical JAKA orientation | implemented; tested offline | validated | partial | do not infer full envelope |
| Clutch release/cleanup | implemented; tested | validated | partial | historical bounded use |
| Collision-event propagation | implemented; tested offline | fake-worker validated | not intentionally validated | collision remains hard stop |
| Payload-corrected post-fix path | implemented through acceleration fix | replay validated | incomplete | polling timing passed; acceleration fix pending |
| TCP calibration | interfaces exist | model frames tested | not validated | TCP1–TCP10 recorded zero |
| RH56 physical teleoperation | command-priority scheduler tested offline | simulation hand validated | hand-only validated at 15/30/40/50 Hz; combined failed on arm timing | fast40 selected; zero RH56 serial faults |
| Foundation J6 gates | historical implementation | n/a | passed for exact +0.25°/+5° gates | July 16 evidence; not full teleop |
| Digital-twin workspace | implemented; tested | integrated workspace | not applicable | 3 failed trajectories; calibration pending |

---

# 中文版：验证矩阵

状态只针对当前实现和已保存证据。“离线测试通过”和“仿真验证通过”不等于真机 PASS。

| 能力 | 实现/离线 | 仿真 | 真机 | 最新边界 |
|---|---|---|---|---|
| Quest 包和输入解析 | 已实现、测试 | 已验证 | 曾观察输入 | 部署 APK 仍是外部事实 |
| Wrist pose / controller clutch | 已实现、测试 | 已验证 | 部分验证 | release-before-press，历史受限运行 |
| 参考捕获/启动连续性 | 已实现、测试 | 已验证 | 部分验证 | post-EDG `q_hold`，无跳变契约 |
| 坐标映射和滤波 | 已实现、测试 | 已验证 | 部分验证 | 全范围未证明 |
| 共享 IK / continuation / branch | 已实现、测试 | 已验证 | 部分运行 | 最多五次回退，最小 1/32 |
| Jacobian 奇异性策略 | 已实现、测试 | 已验证 | 未完全验证 | J5 15° 仅 warning |
| 输出速度可行性 | 已实现、测试 | 已验证 | 部分运行 | acceptance 前检查 |
| 输出加速度可行性 | 已实现、测试/回放 | 离线验证 | **未验证修复** | 当前 4π rad/s² |
| `HOLD_REJECTED` 恢复 | 已实现、测试 | 已验证 | 加速度修复未验证 | 新鲜 heartbeat 保持 |
| Native resampler | 已实现、fake 测试 | 回放验证 | 部分验证 | 125 Hz、8 ms、latest destination |
| Native zero-IK joint worker | 已实现、测试 | fake 验证 | 部分验证 | `kine_inverse` 调用为零 |
| 控制器健康监控 | 已实现、测试 | fake 验证 | 受限时序通过 | 单 SDK 会话轻量轮询 |
| MuJoCo 集成机械臂/RH56 | 已实现、测试 | 已验证 | 不适用 | 6 arm + 6 hand，相对式六通道 grip |
| JAKA-only arm model | 已实现、测试 | 已验证 | 不适用 | 严格 6 个 JAKA actuator，无 RH56 command |
| JAKA 平移/旋转 | 离线实现、测试 | 已验证 | 部分 | 较大运行触发 J4 collision |
| Clutch release/cleanup | 已实现、测试 | 已验证 | 部分 | 历史受限运行 |
| 碰撞事件传播 | 离线测试 | fake 验证 | 未故意触发验证 | collision 是 hard stop |
| Payload 修正后的修复路径 | 加速度修复已实现 | 回放验证 | 未完成 | 下一受限 gate |
| TCP 标定 | 接口存在 | 模型 frame 测试 | 未验证 | TCP1–TCP10 记录为零 |
| Quest 到真机 RH56 | command-priority scheduler 已离线测试 | 仿真手已验证 | hand-only 已验证 15/30/40/50 Hz；combined 因 arm timing 失败 | 选择 fast40；RH56 串口零故障 |
| 历史 J6 foundation gate | 历史实现 | 不适用 | 精确 +0.25°/+5° 通过 | 不代表完整遥操作 |
| 数字孪生工作区 | 已实现、测试 | Integrated Workspace | 不适用 | 三条失败 trajectory，标定待完成 |
