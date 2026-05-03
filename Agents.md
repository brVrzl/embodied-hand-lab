# Embodied Lab Agent 约束

本文档约束后续在本项目中工作的编码代理、研究代理和协作者。目标是让每次改动都服务当前研究主线，并避免破坏真实机器人安全、数据一致性和可发表性。

## 1. 项目目标

当前项目不是通用大模型仓库，也不是单纯仿真 demo。它的近期目标是：

1. 稳定运行 `JAKA mini2 + RH56` 桌面操作实验。
2. 记录可复现 episode 数据。
3. 支撑当前第一篇真实机器人论文主线：`Palm-Frame Hand-Code Transfer for Data-Efficient Dexterous Grasping on JAKA mini2 + Inspire RH56`。

当前唯一有效的研究与控制计划是：

- `docs/active_research_and_control_plan.md`

旧的 failure-aware envelope-grasp pipeline 文档只作为历史参考，不再作为第一篇论文主创新。

任何新增功能都应明确说明它服务以下哪一项：

- 真实 bring-up
- 安全控制
- episode 采集
- 数据导出
- hand-code / palm-frame / pseudo-tactile baseline 训练评测
- 论文实验
- 文档复现

若不能服务上述目标，默认暂缓。

## 2. 安全约束

- 真实机器人运动命令必须默认 dry-run；只有显式传入 `--execute` 才允许下发运动。
- 新增真实运动脚本必须先做只读状态检查，再做限位/急停/静止检查，再允许执行。
- 不允许在脚本中硬编码危险速度、最大力或不受限轨迹。
- 不允许把仿真坐标、示例 preset 或未验证位姿直接当作实机安全位姿。
- 对 JAKA 的 `move_pose()`、关节空间运动、工具端 RS485 模式切换要保持保守；不确定语义必须写入文档或报错。
- JAKA trajectory mode 和 EDG servo mode 必须互斥；新增控制代码必须有 mode manager 或等价保护。
- JAKA EDG servo 真实执行必须先低速、小幅、短时测试，再提升频率；默认从 `step_num=2` 或更保守参数开始。
- RH56 通过 JAKA 工具端 RS485 控制时，不要手动绕过项目 backend 反复切 raw/Modbus。
- RH56 论文实验主链路默认是 PC direct USB-RS485；JAKA tool RS485 只作为备用/演示链路。
- 任何 `estop`、碰撞、限位、通信异常都必须进入 failure reason 或 operator notes。

## 3. 数据约束

- Episode 必须包含 `task_name`、自然语言指令、开始/结束时间、success/failure、failure_reason、operator_notes 和逐步 observation/action。
- Dataset schema version 必须明确写入 metadata；新主线建议使用 `jaka_rh56_palm_handcode_v0.1`，不要沿用旧 pick-cube schema 表示新实验。
- Episode 必须记录 `failure_mode`，合法值固定为：`none`、`fail_late_close`、`fail_early_close`、`fail_lateral_offset`、`fail_low_grip`、`fail_object_slip`、`fail_collision`、`fail_timeout`、`unknown`；成功 episode 必须使用 `none`。
- `PickCubeJakaRH56-v1` 的 `success=true` 不能只看 oracle 阶段是否完成；当前还必须满足 `final_object_height >= 0.08m`。
- `final_object_height >= 0.08m` 只是自动启发式，不是唯一成功标准；后续训练过滤必须优先使用 replay 人工复核结果。
- 旧的 `jaka_rh56_pickcube_v0_2_20eps` 只有 state_dict，没有原始 RGB/D，状态重绘 replay 不能作为人工 BC 复核依据；它只用于 schema/debug。
- 新的 BC 候选数据必须保存原始 RGB 帧并导出 MP4 replay，默认路径为 `data/episodes/<dataset>/<episode_id>/rgb/third_person/frame_000000.png`，structured export 中必须写 `observation.rgb_path` 或 `observation.rgb_paths.third_person`。
- 仿真 oracle 的 RGB replay 帧是 `post_step_after_oracle_carry_review`，用于人工复核 kinematic carry 后的画面；不要把它当成严格同步的部署相机 observation。
- 真实数据、仿真数据和 mock 数据必须在 metadata 中区分。
- 不得把 mock 或 sim 结果描述成真实机器人结果。
- 缺失传感器字段必须保留为 null/empty/unavailable，不得伪造数值或语义。
- 时间戳应单调递增；新增 recorder 或 exporter 后必须增加相应检查或测试。
- 数据导出应优先保持 JSONL/manifest 可读，不引入难以审计的二进制主格式。
- 任何训练/评测 split 必须记录随机种子、对象列表和任务列表。
- 仿真和真实机必须共用 canonical action。当前主线默认 policy output 为 `delta_palm_pose + hand_code + close_strength`；兼容字段可保留 `ee_delta + hand_delta_cmd`。hand 顺序固定为 `[index, middle, ring, pinky, thumb_close, thumb_lateral]`，不得使用非 canonical 指名。
- 上层 policy 不允许直接依赖 MJCF joint 顺序或 RH56 SDK/寄存器顺序；顺序转换必须在 adapter/schema 层完成。
- 不得直接假设 RH56 raw angle 与 0-1 线性等价；每个自由度必须保留 `raw_open`、`raw_close`、`direction_sign`、`safe_min`、`safe_max`、`default_speed`、`default_force_limit`。
- 手部动作优先使用 low-dimensional hand-code；continuous 6D hand command 和 command-based delta action 只作为 baseline 或兼容字段。
- RH56 PC direct 数据必须保留 `target_angle - actual_angle`、force/current、status/error/temp，供 pseudo-tactile correction 和失败分析使用。
- replay 人工复核标准固定为：`strong_success` 表示物体被手稳定夹住并明显离桌且 episode 末尾仍保持；`weak_success` 表示短暂离桌或轻微抬起但不稳定、高度不足或末尾不可靠；`near_failure` 表示几乎没有成功抬起或只是贴地滑动/轻微扰动；`invalid` 表示穿模、瞬移、明显 kinematic artifact 或数据损坏。
- 第一版学习实验优先比较 hand-code / palm-frame / pseudo-tactile 变体。训练过滤必须优先使用 MP4 replay 人工复核后的 `manual_review.use_for_bc=true`；如果缺少 `manual_review.yaml`，才退化使用 `episode_quality=strong_success` 且 `failure_mode=none`。`weak_success` 和 `near_failure` 暂时只用于 debug，不进入主训练集。
- `action.ee_delta` 当前定义在 base frame；平移部分按 per-axis clip，即 `abs(dx/dy/dz) <= 0.02m`，不是按 vector norm clip，因此 translation norm 最大可能约为 `sqrt(3) * 0.02 = 0.03464m`。
- 旋转部分当前按 `euler_xyz` 的 `[droll, dpitch, dyaw]` 记录。当前 pick-cube oracle 的 rotation delta 实际为 0，因此 `euler_xyz` 暂时可接受；未来如果需要学习非零 wrist rotation，应迁移到 `rotvec`，即 `ee_delta = [dx, dy, dz, dRx, dRy, dRz]`。
- metadata 必须写 `ee_delta_frame`、`ee_translation_delta_limit_type=per_axis`、`ee_translation_delta_limit_m=0.02`、`rotation_delta_type`、`action_delta_base`、`hand_delta_cmd_clipped`、`hand_delta_state_clipped` 和 `hand_delta_state_raw_available`。
- 末端 `ee_delta` 默认平移 per-axis 限幅为 `[-0.02m, +0.02m]`，旋转限幅为 `[-5deg, +5deg]`。
- 手部 observation 至少预留 sparse proxy 字段：`currents[6]`、`forces[6]`、`position_error[6]`、`velocity[6]`、`contact_binary[6]`、`slip_binary`。
- Structured export 必须通过 `tools/validate_episode_schema.py`，尤其检查 metadata 一致性、手部 canonical order、delta 限幅、`robot_q_current/desired` 维度和 failure mode 枚举。

## 4. 研究约束

- 第一篇默认主线是 palm-frame hand-code transfer，并使用 RH56 low-level feedback 做 pseudo-tactile correction。
- 仿真默认主线只使用官方 `PickCube-v1` 和自定义 `PickCubeJakaRH56-v1`，不新增无法对齐官方 benchmark 的临时任务。
- 当前 JAKA+RH56 oracle 是 privileged / kinematic carry，只用于 schema、导出和训练链路验证，不代表真实物理抓取成功率。
- MuJoCo 最小调试链路可用于 RH56 hand close、指尖接触、撞桌和 cube 抖动诊断；它当前是 debug viewer，不是训练数据主链路。
- MuJoCo 交互调姿优先使用 `tools/teleop_mujoco_jaka_rh56.py`：鼠标/键盘移动蓝色 mocap target，JAKA 末端通过 IK 跟随，RH56 6 DoF 按 canonical order 微调。调出的姿态只能作为仿真调试候选，不能直接作为真实机器人安全位姿。
- 当前不优先做 OpenVLA、大规模 RL、高速在手操作、全手触觉皮肤路线或购买第二只手作为必要条件。
- 论文 claim 必须和实验规模匹配，不使用 `general-purpose`、`human-level`、`arbitrary objects` 等过大表述。
- 每个方法实验至少考虑这些 baseline 或消融：fixed palm、continuous 6D hand command、no public data、no pseudo-tactile correction、learning/retrieval method。
- 真实机器人实验必须包含失败分析。
- 仿真可用于开发、预演和辅助消融，但不能替代真实结果。

## 5. 工程约束

- 优先沿用现有模块：
  - `jaka_driver_adapter`
  - `rh56_driver`
  - `data_recorder`
  - `sim_maniskill`
  - `teleop_tools`
  - `evaluation`
  - `lerobot_bridge`
- 不要绕开统一接口直接在任务脚本里调用供应商 SDK，除非是一次性诊断工具。
- Backend 差异应封装在 backend 内，不泄漏到 recorder、evaluation 或 task 层。
- JAKA EDG servo、MoveIt/trajectory、RH56 PC direct、RH56 JAKA tool RS485 都必须通过 adapter/backend 或 ROS2 node 封装，任务脚本不得直接散落供应商 SDK 调用。
- 配置优先放在 `configs/`，不要把实验参数散落在脚本里。
- 新增 CLI 应有 `--config`，真实执行应有 `--execute`。
- 保持 Python 3.10 / Ubuntu 22.04 / ROS2 Humble 兼容性。
- 新依赖必须说明用途；不要为小功能引入大型框架。
- 文件编辑保持小范围，不做无关重构。

## 6. 测试与验证

修改代码后优先运行：

```bash
.venv/bin/python -m pytest
```

若修改了某个模块，可先跑相关测试，再跑全量测试。

新增或修改以下行为时必须补测试：

- 配置加载
- backend 状态解析
- episode schema
- exporter 输出
- evaluation 聚合
- ManiSkill 环境注册或 reset 行为
- JAKA/RH56 工具端 RS485 命令编码

如果因为缺少硬件无法测试真实路径，必须写明：

- 已测试 mock/sim 路径。
- 未测试真实路径。
- 真实验证命令是什么。

## 7. 文档约束

- 用户可执行流程必须写成命令，不只写概念。
- 实机流程必须明确安全前置条件。
- 研究路线文档必须区分“当前已完成”“下一步”“暂缓”。
- 报告和论文素材中必须区分真实、仿真、mock。
- 不要删除已有 bring-up、安全检查和数据协议说明；需要调整时做增量更新。

## 8. 文件与目录约定

- `docs/active_research_and_control_plan.md`：当前唯一有效研究与工程推进计划。
- 根目录 `Agents.md`：本约束文档。
- `docs/`：稳定文档和对外复现说明。
- `configs/`：机器人、手、相机、仿真、任务配置。
- `scripts/`：面向用户的一键入口。
- `tools/`：诊断、检查和一次性辅助工具。
- `src/`：可复用模块源码。
- `data/episodes/`：原始 episode。
- `data/exports/`：导出样本、manifest、报告。

## 9. 代理工作方式

每次任务开始前应先确认：

1. 本次任务属于哪条主线。
2. 是否会触碰真实硬件路径。
3. 是否会改变数据 schema。
4. 是否需要同步更新文档或测试。

编码代理应优先完成闭环：

1. 读相关代码和文档。
2. 做最小必要改动。
3. 跑相关测试。
4. 汇报改动、验证结果和未覆盖风险。

研究代理应优先输出可执行实验协议：

1. 明确假设。
2. 明确对象集和任务。
3. 明确 baseline。
4. 明确指标。
5. 明确失败模式。
6. 明确论文 claim 边界。

## 10. 暂缓事项

以下事项只有在 active plan 的 JAKA EDG servo、RH56 PC direct、palm-frame hand-code 和 pseudo-tactile correction 跑通后再重新评估：

- 临时盒子放置、长时序搬运等自造任务。
- 复杂 VLA / OpenVLA 接入。
- 大规模 cross-embodiment 数据混合。
- 高分辨率触觉皮肤硬件改造。
- 四足机器狗作为论文主线。
- 高速动态在手操作。
- 多用户 HRI 实验平台。
