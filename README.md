# Embodied Lab

面向真实世界具身智能实验的第一版工程仓库。目标不是一次性做“大一统大模型”，而是围绕 `JAKA mini2 + Inspire RH56` 建立可控、可记录、可复现实验栈，并支撑当前研究主线：

**Palm-Frame Hand-Code Transfer for Data-Efficient Dexterous Grasping on JAKA mini2 + Inspire RH56**。

当前唯一有效的研究与控制计划见：

- `docs/active_research_and_control_plan.md`

## 项目目标

- 支持桌面操作链路：`JAKA mini2 + Inspire RH56`
- 支持 JAKA trajectory mode 与后续 EDG servo mode 并存
- 支持 RH56 PC direct USB-RS485 主链路与 JAKA tool RS485 备用链路
- 支持统一 episode 数据采集、导出、评测
- 兼容 `ROS2 Humble + Ubuntu 22.04 + Python 3.10`
- 数据组织尽量贴近 LeRobot，便于后续 hand-code、PocketDP3/DP3 和轻量策略研究

## 当前支持硬件

- 机械臂：JAKA mini2
  - 当前状态：统一接口 + mock backend + `jkrc` SDK backend 骨架已完成
  - 当前段式控制：`joint_move` / `linear_move`
  - 计划新增高频控制：JAKA EDG servo，服务 palm-frame 微调和视觉闭环
  - 本地 SDK 资料已定位
- 灵巧手：因时 Inspire RH56
  - 当前状态：高级夹爪模式最小接口 + mock backend + RS485 serial backend + JAKA tool RS485 backend 已完成
  - 论文实验主链路：PC direct USB-RS485，读取 angle / force / current / status / temp
  - 备用链路：JAKA tool RS485，仅用于简化部署或演示
  - 官方 ROS2 service 接口已整理进配置
- RGB-D 相机：Orbbec / RealSense 风格抽象
  - 当前状态：mock camera + placeholder adapter 已完成
  - 厂商 SDK：`待替换适配点`

## 当前阶段功能

- mock 模式下整栈可启动
- JAKA 与 RH56 可通过统一 Python 接口调用
- episode 可开始、记录、停止并落盘
- episode 可导出为结构化样本与 LeRobot 风格 stub
- 提供 bring-up、数据协议、研究路线图文档
- 提供 ManiSkill 仿真采集入口，可先跑桌面任务假数据链路
- 已形成 RH56 hand-code / external dataset / MuJoCo replay 的前置研究材料

## 快速开始

```bash
cd /home/w/projects/embodied_lab
/usr/bin/python3.10 -m venv .venv
source .venv/bin/activate
pip install -e ".[dev]"
./scripts/start_mock_stack.sh
./scripts/start_data_recording.sh --task pick_and_place --instruction "pick the cube and place it in tray"
./scripts/start_maniskill_recording.sh --config configs/sim/maniskill_pick_cube.yaml
./scripts/start_maniskill_recording.sh --config configs/sim/maniskill_jaka_rh56_pick_cube.yaml
./scripts/start_maniskill_recording.sh --config configs/sim/maniskill_jaka_rh56_pick_cube_state.yaml
./scripts/export_maniskill_scene_preview.sh --config configs/sim/maniskill_jaka_rh56_scene_preview.yaml
./scripts/check_jaka_connection.sh --ip 192.168.1.100
./scripts/check_jaka_zero_motion.sh --ip 192.168.1.100
./scripts/check_rh56_connection.sh --port /dev/ttyUSB0
./scripts/check_rh56_via_jaka.sh --ip 192.168.1.100
./scripts/save_jaka_preset.sh --preset-name upright --joints 0 0 0 0 0 0
./scripts/arm_hand_smoke_test.sh --ip 192.168.1.100 --preset-name upright --hand-id 1 --execute
pytest
```

## 仓库结构

```text
docs/                 文档
scripts/              启动脚本
configs/              YAML 配置
src/                  各模块源码
launch/               ROS2 风格 launch 入口
tests/                最小测试
data/                 默认数据目录
tools/                辅助工具
```

关键模块：

- `src/robot_bringup`：统一 bring-up 入口
- `src/jaka_driver_adapter`：JAKA 统一接口层
- `src/rh56_driver`：RH56 最小驱动层
- `src/vision_interface`：相机抽象层
- `src/data_recorder`：统一 episode recorder
- `src/sim_maniskill`：ManiSkill 仿真采集入口
- `src/task_library`：任务模板与元数据
- `src/evaluation`：最小评测
- `src/lerobot_bridge`：LeRobot 导出 stub

真实设备模板配置：

- `configs/robot/jaka_mini2_real.yaml`
- `configs/hand/rh56_real.yaml`

建议保留原有 `mock` 配置不动，真实 bring-up 一律显式传 `--config`。

## 典型工作流

1. 用 `scripts/start_mock_stack.sh` 验证全链路。
2. 用 `scripts/start_arm_hand_stack.sh` 按模块 bring-up。
3. 用 `scripts/start_data_recording.sh` 启动一次 episode 记录。
4. 若先走仿真，使用 `scripts/start_maniskill_recording.sh` 采集桌面任务 episode。
5. 用 `scripts/export_lerobot_dataset.sh` 导出结构化样本。
6. 用 `python -m evaluation.report ...` 生成评测报告。
7. 对接真实硬件时，仅替换对应 adapter backend，不改 recorder 和任务层。
8. JAKA/RH56 的本地官方资料路径已在各模块 README 中写明，可直接照着替换。
9. 新实现优先服务 `docs/active_research_and_control_plan.md`：JAKA EDG servo、RH56 PC direct、palm-frame hand-code、pseudo-tactile correction。

## ManiSkill 仿真采集

适用场景：

- 还没买 RGB-D 相机
- 想先把桌面任务的采集与训练链路跑通
- 不急着一开始就还原 `JAKA mini2 + RH56`

最小入口：

```bash
cd /home/w/projects/embodied_lab
source .venv/bin/activate
pip install gymnasium mani_skill torch
./scripts/start_maniskill_recording.sh --config configs/sim/maniskill_pick_cube.yaml
```

默认配置会：

- 创建 `PickCube-v1`
- 使用 `rgbd` 观测与 `pd_ee_delta_pose` 控制
- 采 1 条 episode 到 `data/episodes/maniskill_pick_cube`
- 导出结构化样本到 `data/exports/structured/maniskill_pick_cube`

JAKA + RH56 仿真替换入口：

```bash
./scripts/start_maniskill_recording.sh --config configs/sim/maniskill_jaka_rh56_pick_cube.yaml
```

如果本机 ManiSkill/SAPIEN 的 renderer 还没配好，先用 state-only 验证机器人替换：

```bash
./scripts/start_maniskill_recording.sh --config configs/sim/maniskill_jaka_rh56_pick_cube_state.yaml
```

如果要先验证“JAKA+RH56 task schema -> episode -> structured export”闭环，可以使用特权 scripted oracle：

```bash
source .venv/bin/activate
export PYTHONPATH=$PWD/src
export MPLCONFIGDIR=/tmp/matplotlib
python tools/collect_jaka_rh56_pickcube_privileged_oracle.py \
  --episodes 20 \
  --output-dir data/episodes/jaka_rh56_pickcube_privileged_oracle \
  --export-dir data/exports/structured/jaka_rh56_pickcube_privileged_oracle
```

注意：

- 这个 oracle 会在 scripted close 阶段后用特权方式移动 cube，只用于验证数据格式、训练脚本和评测链路。
- 不要把该数据报告为物理抓取成功率。
- 当前本机无可用 SAPIEN GPU/CPU renderer，`--rgbd` 采集会失败；state-only 采集可用。
- 真实物理抓取 oracle 下一步要继续修 RH56 手指接触、抓取偏置和 `is_grasping` 判定。

如果你想先看场景是否和实机摆位接近，直接导出一份预览：

```bash
./scripts/export_maniskill_scene_preview.sh --config configs/sim/maniskill_jaka_rh56_scene_preview.yaml
```

如果你想直接盯着窗口看，不要再用录制命令，改用这个常驻查看器：

```bash
./scripts/view_maniskill_scene.sh --config configs/sim/maniskill_jaka_rh56_scene_preview.yaml
```

说明：

- 这个命令会一直停在初始场景，直到你手动关窗或 `Ctrl+C`
- 如果你想让仿真持续刷新但不动作，可以加 `--step-zero`

默认会在 `data/previews/maniskill_jaka_rh56_scene_preview` 下写出：

- `scene_summary.json`：机器人 base pose、TCP、cube、goal、qpos
- `human_render.ppm`：如果本机 renderer 可用，就导出第三人称视角
- `sensor_rgb.ppm`：如果 `obs_mode=rgbd` 且本机 renderer 可用，就导出任务相机视角

说明：

- 这会注册一个专用的 `PickCubeJakaRH56-v1` 任务，而不是继续硬套官方 `PickCube-v1`
- 机器人使用本地 `RoboTwin/robot_sim` 中的 `JAKA + RH56` MJCF 组合模型
- 当前第一版先使用 `pd_joint_delta_pos` 联调，不直接做末端位姿控制
- 本地模型里的手部安装位姿当前按 9mm 法兰厚度保留为 `0.009m`
- 工作台按实物近似为 `1.20m x 0.60m`，机器人安装点距右侧边约 `0.25m`、距前侧边约 `0.30m`
- 机器人 base pose 现在固定为朝向工作台，`cube_spawn_center` 放在机器人左侧内侧工作区，默认距 base 约 `0.5m`
- 当前仿真主线只保留 `PickCubeJakaRH56-v1`。临时盒子放置任务已移除，避免偏离 ManiSkill 官方 `PickCube-v1` 的任务语义和 baseline 生态。
- 键盘 teleop 控制：`w/s/a/d/r/v` 微调 TCP，`u/j i/k o/l 7/4 8/5` 可在 joint 模式微调 1-5 关节，`q/e` 旋转第 6 轴末端 roll，`g` 合手，`f` 开手，`p` 标记成功退出，`x` 标记失败退出。

说明：

- 当前入口优先验证“仿真采数据 -> 导出 -> 后续训练”链路
- 默认策略是 `random`，只适合联调，不适合作为高质量 imitation 数据
- 若当前 `.venv` 实际绑定的是 Python 3.13，先删除并用 `/usr/bin/python3.10 -m venv .venv` 重建

实机安全检查入口：

- `scripts/check_jaka_connection.sh`
- `scripts/check_jaka_zero_motion.sh`
- `scripts/check_rh56_connection.sh`
- `scripts/check_rh56_via_jaka.sh`
- `scripts/save_jaka_preset.sh`
- `scripts/arm_hand_smoke_test.sh`

其中：

- `check_jaka_connection.sh` 只做连接与状态读取
- `check_jaka_zero_motion.sh` 默认也只做预检查，只有加 `--execute` 才会下发零位移 `move_joints`
- `check_rh56_connection.sh` 只做连接与状态读取
- `check_rh56_via_jaka.sh` 通过 JAKA 工具端 RS485 向 RH56 发送 open/close 测试帧
- `save_jaka_preset.sh` 把当前关节或显式关节角保存到 `configs/robot/jaka_mini2.yaml`
- `arm_hand_smoke_test.sh` 按指定机械臂 preset 执行最小组合测试：`move_joints -> hand open -> hand close`

## 下一阶段路线图

- 第一阶段：完成 JAKA trajectory + EDG servo 双模式 bring-up，完成 RH56 PC direct USB-RS485 主链路。
- 第二阶段：完成 ROS2 Humble arm / hand / camera / TF / recorder 数据总线。
- 第三阶段：验证 palm-frame hand-code transfer 与 pseudo-tactile correction。
- 第四阶段：在少样本真实任务上接入 PocketDP3/DP3 或轻量策略 baseline。

详细说明见：

- [Active Research and Control Plan](docs/active_research_and_control_plan.md)
- [Hardware Bring-Up Checklist](docs/hardware_bringup_checklist.md)
- [Data Protocol](docs/data_protocol.md)
