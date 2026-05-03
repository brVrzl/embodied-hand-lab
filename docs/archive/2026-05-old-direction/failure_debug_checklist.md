# Failure Debug Checklist

目标：定位为什么 JAKA mini2 + RH56 当前采不到稳定 success 数据。检查顺序必须从硬件/控制层开始，再到任务层，最后才讨论学习算法。

## A. Hardware and Control Layer

### JAKA mini2

- [ ] 末端位姿控制在低速下稳定，无明显抖动。
- [ ] `configs/robot/jaka_mini2_real.yaml` 中速度限制生效：`max_tcp_linear_speed_mps <= 0.10`。
- [ ] home pose、pregrasp pose、lift pose 都在安全工作空间内。
- [ ] 连续 10 次 move to pregrasp，末端位置重复误差肉眼不可见或 <5 mm。
- [ ] wrist orientation 固定时不会导致奇异位姿或关节限位。
- [ ] 工具 TCP offset 已更新，不只使用 `z: 0.009` 的临时值。

### Frames

- [ ] `jaka_base`、`jaka_tool0`、`hand_palm`、`object_fixture`、`camera_link` 命名统一。
- [ ] pregrasp pose 是基于 hand palm 接触几何定义，不是只看 JAKA flange。
- [ ] 物体治具坐标和机器人 base 坐标关系固定。
- [ ] 如果有相机，外参不参与第一周控制闭环，只用于复核。

### RH56 open / close calibration

- [ ] 每根手指 `raw_open=1000` 对应真实打开。
- [ ] 每根手指 `raw_close=0` 对应真实闭合。
- [ ] `direction_sign=-1` 对所有手指都正确。
- [ ] `safe_min/safe_max` 没有导致某根手指提前停住。
- [ ] `power_grasp: [150,150,200,200,200,450]` 在真实物体上不是过紧或过松。
- [ ] `close_strength <= 0.85`，第一周建议从 0.45、0.60、0.75 三档试。
- [ ] 手指闭合方向一致，没有某根手指反向或 lateral thumb 干扰包络。

### Hand latency and feedback

- [ ] 发送 hand command 后记录真实 hand_state 到达时间。
- [ ] 10 次 open/close 的平均延迟、最大延迟已记录。
- [ ] 没有丢包、卡顿、命令覆盖。
- [ ] `command_pause_sec: 0.8` 对采集节奏有显式等待，不在 hand 未闭合时 lift。
- [ ] `feedback_settle_sec: 0.3` 不足时增加到 0.5-1.0 s。

### Mounting and collision

- [ ] 手掌安装姿态让手指包住物体，而不是用指尖把物体推走。
- [ ] approach 时手指全开不会提前扫到物体。
- [ ] pre_shape 后最低手指离桌面有安全距离。
- [ ] close 过程中 palm 不压桌、不擦物体、不挤治具。
- [ ] lift 方向和重力方向一致，避免斜向拉出。

## B. Task Layer

### Object

- [ ] 第一对象是 50-60 mm 泡沫/海绵块，不是小、滑、硬、重物体。
- [ ] 质量 <30 g。
- [ ] 表面摩擦足够，不使用光滑塑料、玻璃、金属。
- [ ] 物体高度足够让 RH56 多指包络，不是薄片。
- [ ] 桌面不太滑，必要时换防滑垫。

### Initial condition

- [ ] 物体初始位置用纸框/治具固定。
- [ ] 物体姿态固定，cube 边与机器人 base 坐标对齐。
- [ ] 每次 episode 前拍一帧起始状态照片。
- [ ] 起点偏差超过 5 mm 的 episode 直接标 `use_for_bc=false`。

### Grasp geometry

- [ ] pregrasp pose 距物体中心和高度固定。
- [ ] palm center 对准物体几何中心略低处。
- [ ] wrist yaw 固定，优先让 4 指和拇指形成包络。
- [ ] close 前手指没有接触物体。
- [ ] close 后先等待，再 lift。

### Task simplification

- [ ] 第一阶段只要求 lift 3 cm。
- [ ] 第二阶段才 lift 8 cm。
- [ ] 第三阶段才 place。
- [ ] wrist orientation 固定。
- [ ] 不做 in-hand rotation。
- [ ] 不随机物体位置。

## C. Data Collection Layer

### Timing

- [ ] 每条 episode 采样频率统一：arm 10 Hz，hand 5 Hz，video 10-30 Hz。
- [ ] action timestamp、state timestamp、video timestamp 可对齐。
- [ ] stage boundary 写入 steps。

### Action representation

- [ ] action 明确是 absolute command 还是 delta command。
- [ ] 第一版 policy action 用 `delta_ee_pose + grasp_type + close_strength`。
- [ ] 6 指 raw command 记录为 state/debug，不作为第一版 policy 输出。

### State logging

- [ ] `robot_q_current`
- [ ] `ee_pose`
- [ ] `hand_cmd`
- [ ] `hand_state`
- [ ] `object_pose` 或 `fixture_id`
- [ ] `success`
- [ ] `failure_mode`
- [ ] `manual_success`
- [ ] `weak_success`
- [ ] `use_for_bc`
- [ ] `operator_notes`

### Video and replay

- [ ] 每条轨迹有侧视视频。
- [ ] 每条轨迹能 replay。
- [ ] clean success 至少抽样 replay 20%。
- [ ] replay 失败的轨迹标 `use_for_bc=false`。
- [ ] 人工复核写入 `manual_review.yaml`。

## D. Success Rate Engineering

### Mandatory gate rule

不得在当前阶段 success rate <60% 时进入下一阶段。

| Gate | Trials | 目标 | 进入下一阶段条件 |
|---|---:|---|---|
| open/close empty | 10 | 验证 RH56 命令和反馈 | 10/10 正常 |
| close-only on fixed object | 10 | 不推走物体并形成接触 | >=7/10 |
| lift 3 cm | 10 | 低高度稳定抬起 | >=7/10 |
| lift 8 cm | 20 | 标准 grasp-lift | >=14/20 |
| pick-and-place | 20 | 完整任务 | >=12/20 |

### If success rate <60%

按顺序处理：

1. 换更软、更大、更轻的对象。
2. 降低 lift 高度到 3 cm。
3. 固定 wrist orientation。
4. 增大 pregrasp clearance，避免 early collision。
5. 调整 palm height，让接触点更靠近物体中部。
6. 延长 close 后 settle 时间。
7. 降低机械臂速度。
8. 调整 close_strength。
9. 重新标定 raw_open/raw_close。
10. 暂停训练，只做工程诊断。
