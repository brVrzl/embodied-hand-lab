# Embodied Lab Agents Guide

本文件约束后续在本仓库中工作的编码代理和研究代理。当前项目已精简，任何新增/删除都必须先保护已调通的真实遥操作链路。

## 当前不可删除内容

- iPhone / HEBI Mobile I/O 遥操作：`src/teleop_tools/hebi_mobile_io.py`、`src/teleop_tools/relative_pose_lag_follow.py`、`src/teleop_tools/hebi_rviz_shadow.py`、`tools/*hebi*`、`scripts/*hebi*`、`configs/teleop/hebi_mobile_io_jaka_rh56.yaml`。
- iPhone / TeleDex arm 遥操作：`src/teleop_tools/teledex_*`、`src/teleop_tools/pose_teleop_config.py`、`tools/*teledex*`、`scripts/*teledex*`、`configs/teleop/teledex_jaka_arm.yaml`；实机必须有独立 frame calibration confirmation。
- iPhone camera / MediaPipe RH56 hand teleop：`src/teleop_tools/iphone_hand.py`、`src/teleop_tools/hand_depth.py`、`tools/iphone_*`、`scripts/*iphone*`。
- Xbox / RViz shadow：`src/teleop_tools/xbox_ros2.py`、`src/teleop_tools/xbox_rviz_shadow.py`、`src/teleop_tools/rviz_shadow_sync.py`、`tools/run_xbox_*`、`scripts/run_xbox_*`、`configs/teleop/xbox_jaka_rh56.yaml`。
- JAKA/RH56 real bridge：`src/jaka_driver_adapter`、`src/rh56_driver`、`src/robot_bringup` 中的 ROS2 bridge、servo jog、serial backend。
- `data/sim_assets/jaka_rh56_visual_coacd.xml`：遥操作 IK/RViz shadow 的默认 runtime asset；`jaka_rh56.xml` 作为 collision comparison 派生源同样保留。

## 当前项目边界

硬件：

- 机械臂：`JAKA mini2`
- 灵巧手：`Inspire RH56`
- 主控制环境：`Ubuntu 22.04 + ROS2 Humble + Python 3.10`
- 主手部链路：RH56 PC direct USB-RS485
- 备用手部链路：JAKA tool RS485

当前任务优先级：

1. 稳定 iPhone/HEBI、Xbox 和 RH56/JAKA 的真实遥操作。
2. 围绕固定网球完成可复现 grasp-and-hold 数据采集。
3. 用 ACT/DP 做小数据 imitation baseline。
4. VLA 只作为相关工作和后续接口方向，不作为第一轮真机复现主线。

## 安全底线

- 默认不要下发真实运动命令；真实执行必须由明确脚本、明确参数或用户明确要求触发。
- JAKA 真实运动前必须确认网络、使能、急停、工作空间和周围安全。
- RH56 真实控制前必须确认手周围安全，避免夹住人体、线缆、刚性障碍物。
- 新增真实控制脚本必须有默认限幅、超时、deadman 或停止策略。
- 不得把仿真位姿、示例 preset 或未验证坐标直接当真实机器人安全位姿。
- 出现通信异常、限位、保护停、estop、碰撞或明显异常电流/力反馈时，应停止或降级，并写入日志。

## 接口约定

- `configs/` 放配置。
- `scripts/` 放用户直接运行的入口。
- `tools/` 放脚本调用的实现工具。
- `src/` 放可复用模块。
- `tests/` 放回归测试。
- `docs/` 放当前文档入口和可信文献索引。

RH56 canonical 手部顺序固定为：

```text
[index, middle, ring, pinky, thumb_close, thumb_lateral]
```

ROS2 JSON topic 应显式包含 schema/version、timestamp、source、单位和 canonical order。不要在 topic 中隐式猜测 normalized 与 raw count 的转换。

## 验证要求

修改遥操作、JAKA/RH56 bridge 或 IK 后，至少跑：

```bash
.venv/bin/python -m pytest \
  tests/test_xbox_ros2_teleop.py \
  tests/test_xbox_rviz_shadow.py \
  tests/test_rviz_shadow_sync.py \
  tests/test_jaka_servo_jog.py \
  tests/test_robot_bringup_ros2_bridge.py \
  tests/test_rh56_ros2_bridge.py \
  tests/test_rh56_serial_backend.py
```

脚本改动至少跑：

```bash
bash -n scripts/<script>.sh
```

真实硬件不可用时，最终说明必须写清已验证的 mock/dry-run 路径和未验证的真实路径。

## 研究文档

近期可信文献与方法边界见：

- `docs/literature/dexterous_grasping_recent_work_20260609.md`

研究结论必须区分：

- 已发表/强项目支撑的可信来源。
- arXiv 新 preprint 趋势。
- 本项目真实设备已验证结果。
- mock/sim/dry-run 结果。
