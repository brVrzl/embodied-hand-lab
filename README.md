# Embodied Lab

`embodied_lab` 是一个正在重建中的 `JAKA mini2 + Inspire RH56` 操作栈仓库。当前仓库同时包含真实机器人 bring-up、遥操作、数据记录、仿真资产和预抓取工具。由于早期仿真内容曾被误删，当前项目应被视为恢复和重新规划中的工作区，而不是完成版数字孪生。

## 当前状态

当前可用内容：

- JAKA mini2 配置、preset motion 辅助脚本、mock backend、servo-jog 安全控制工具。
- Inspire RH56 命令 schema、mock/serial backend、ROS2 JSON bridge、手指顺序和命令映射。
- Xbox、HEBI Mobile I/O、iPhone camera、RViz shadow 等遥操作实验工具。
- 当前 JAKA-mounted MuJoCo 文件 `data/sim_assets/jaka_rh56.xml`，仍被 IK、预览、benchmark、ManiSkill 等路径引用。
- Correll RH56DFX 参考 MuJoCo 手部资产 `data/sim_assets/correll_rh56dfx/`，用于浮动手 FK 规划和指尖 force/torque scene 验证。
- `src/pregrasp` 中的确定性 geometry-based RH56 预抓取候选生成流程。

关键说明：

`data/sim_assets/jaka_rh56.xml` 需要保留，是因为当前项目仍依赖它作为恢复锚点和 mounted-arm 集成模型。这不表示该模型完整、已标定或是最终真值。它需要继续审计和改进，不能把它当作已经验证过的 RH56 数字孪生。

## 目录结构

```text
configs/          机器人、手、相机、遥操作、仿真和工作空间配置
data/sim_assets/  当前 IK、预览、规划和测试使用的 MuJoCo 资产
docs/             项目说明、协议和文献/资产审计
launch/           ROS2 launch 文件
scripts/          面向人的 shell 入口
src/              Python 包源码
tests/            回归和 smoke 测试
third_party/      vendor 或外部代码快照
tools/            脚本和实验调用的 Python 工具
```

核心模块：

- `src/rh56_driver`：RH56 command schema、backend adapter、ROS2 bridge helper。
- `src/jaka_driver_adapter`：JAKA SDK/mock、掌心目标 IK、servo-jog 安全逻辑。
- `src/teleop_tools`：Xbox、HEBI/iPhone、RViz shadow、relative-pose teleop。
- `src/robot_bringup`：真实机械臂和手的 bridge 编排。
- `src/pregrasp`：物体几何、RH56 预抓取 primitives、Correll FK adapter、触觉/力反馈修正。
- `src/sim_maniskill`：使用当前 JAKA+RH56 资产的 ManiSkill task 和 agent。
- `src/data_recorder`：episode 记录工具。
- `src/vision_interface`：相机 adapter 和 mock。

关键文档：

- `docs/project_rebuild_status.md`：当前项目重建状态、验证等级和优先级。
- `docs/rh56dfx_correll_integration_assessment.md`：Correll RH56DFX 资产与原项目资产的对比和整合结论。
- `data/sim_assets/README.md`：当前仿真资产角色边界。

## 仿真资产边界

当前有两类 RH56 资产角色：

- `data/sim_assets/jaka_rh56.xml`
  - 当前 JAKA+RH56 mounted model。
  - 被 palm-target IK、RViz/teleop 预览、MuJoCo benchmark、ManiSkill 集成使用。
  - runtime hand collision 默认注入 Correll RH56DFX collision mesh，旧 analytic proxy 仅作为对比/回退模式。
  - 仍需要继续审计。不要把它理解为完整硬件表征模型。

- `data/sim_assets/correll_rh56dfx/`
  - 来自 Correll Robotics Lab RH56DFX 工作的参考资产。
  - 用作浮动手 FK 规划和指尖 force/torque sensor scene 的参考模型。
  - 通过 `pregrasp.correll_rh56dfx` 暴露给项目代码，并通过 `sim_maniskill.rh56_collision` 作为 mounted hand 的默认 collision mesh 来源。

不要随意合并这两类角色。用 Correll 浮动手模型替换 JAKA-mounted 模型，需要单独迁移 mount frame、joint/body name、actuator order 和下游测试。

## 环境

```bash
cd /home/thor/projects/embodied_lab
python3 -m venv .venv
source .venv/bin/activate
pip install -e ".[dev]"
```

可选依赖：

```bash
pip install -e ".[gamepad]"        # Xbox / pygame
pip install -e ".[phone-teleop]"   # HEBI Mobile I/O
pip install -e ".[vision-teleop]"  # MediaPipe / OpenCV hand tracking
pip install -e ".[sim]"            # ManiSkill stack，需要匹配本机 Python 环境
```

ROS2 Humble 路径可能还需要系统 ROS 包，不完全由 Python venv 管理。

## 基础验证

修改机器人控制、RH56 schema 或仿真资产前，优先跑这些聚焦测试：

```bash
.venv/bin/python -m pytest \
  tests/test_rh56_hand_schema.py \
  tests/test_rh56_ros2_bridge.py \
  tests/test_rh56_serial_backend.py \
  tests/test_jaka_servo_jog.py \
  tests/test_correll_rh56dfx_assets.py \
  tests/test_mujoco_rh56_collision_modes.py \
  tests/test_pregrasp_prediction.py
```

只验证新引入的 Correll RH56DFX 参考资产：

```bash
.venv/bin/python -m pytest tests/test_correll_rh56dfx_assets.py
```

该测试会确认 XML 能被 MuJoCo 编译、预期 actuator/site/sensor 存在、width-based FK planner 能输出可用命令。

## 硬件入口

使用真实硬件脚本前，先检查对应配置、IP 和串口路径。

JAKA：

```bash
./scripts/check_jaka_connection.sh --ip 192.168.1.100
./scripts/check_jaka_zero_motion.sh --ip 192.168.1.100
./scripts/check_jaka_edg_servo_capability.sh --config configs/robot/jaka_mini2_real.yaml
```

RH56：

```bash
./scripts/check_rh56_connection.sh --port /dev/ttyUSB0
./scripts/rh56_pc_direct_bringup.sh --config configs/hand/rh56_real.yaml --port /dev/ttyUSB0 --polls 20
```

遥操作和预览：

```bash
./scripts/run_jaka_rh56_rviz.sh
./scripts/run_xbox_rviz_shadow.sh
./scripts/check_hebi_mobile_io.sh --duration-sec 5 --hz 10
```

会移动硬件的脚本应要求显式 enable/execute 参数。使用前先看帮助：

```bash
./scripts/<script-name>.sh --help
```

## 预抓取流程

当前预抓取路径是确定性的 geometry-first 流程：

```bash
.venv/bin/python tools/predict_rh56_pregrasp.py \
  --geometry-json path/to/object_geometry.json \
  --top-k 3
```

相关文件：

- `configs/pregrasp/rh56_pregrasp.yaml`
- `src/pregrasp/primitives.py`
- `src/pregrasp/predictor.py`
- `src/pregrasp/correll_rh56dfx.py`
- `tools/generate_rh56_pregrasp_dataset.py`

Correll FK planner 会在适合的物体宽度下生成额外的 `correll_line_width` 候选。这是规划辅助，不替代真实 RH56 接触验证。

## 开发约定

- 外部资产要按来源和用途分目录，并保留 license。
- 替换仿真文件前，先补验证测试。
- 在所有依赖路径迁移完成前，不要删除 `data/sim_assets/jaka_rh56.xml`。
- 没有真实 replay 数据时，不要把仿真抓取成功率写成真实机器人性能。
- 如果某个资产只是临时恢复锚点，要在文档和配置注释中明确说明。
