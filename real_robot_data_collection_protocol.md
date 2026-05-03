# Real Robot Data Collection Protocol

适用硬件：JAKA mini2 + Inspire RH56。

当前 active plan 见 `docs/active_research_and_control_plan.md`。本协议保留 episode schema、manual review、success/failure 标注等数据规范，但控制假设已更新：

- JAKA 大范围运动使用 trajectory / MoveIt，精细 palm pose 微调使用 EDG servo。
- RH56 论文实验主链路使用 PC direct USB-RS485，以读取 angle / force / current / status / temp。
- JAKA tool RS485 只作为备用/演示链路；使用该链路时应在 metadata 中记录反馈受限和低频控制限制。

## 0. Episode Schema

每条 episode 必须保存：

- `episode_id`
- `task_name`
- `object_id`
- `object_pose_init`，如果没有 tracker，至少写治具位置编号。
- `robot_q_current`
- `ee_pose`
- `hand_cmd`
- `hand_state`
- `action`
- `stage`: `approach | close | lift | hold | place | release | correction`
- `auto_success`
- `manual_success`
- `success`
- `failure_mode`
- `weak_success`
- `use_for_bc`
- `operator_notes`
- side-view video path
- top-view or wrist-view video path, if available

推荐目录：

```text
data/real/jaka_rh56/YYYYMMDD/task_name/
  episode_0001/
    metadata.json
    steps.jsonl
    manual_review.yaml
    video_side.mp4
    video_top.mp4
```

## 1. Action Space

当前主线默认 action：

```yaml
action:
  delta_palm_pose:
    frame: object_or_base
    translation_m: [dx, dy, dz]
    rotation_rotvec: [dRx, dRy, dRz]
  hand_code: 0
  close_strength: 0.0-0.85
  correction:
    enabled: true
    source: angle_residual | force_current | none
```

兼容旧数据的低维 action：

```yaml
action:
  ee_delta_xyz_m: [dx, dy, dz]
  ee_delta_rpy_rad: [0, 0, 0]
  grasp_type: open | pre_shape | envelope_close | power_grasp | release
  close_strength: 0.0-0.85
```

不要把 6 指连续动作作为默认 policy action。可以记录 6 指 command/state，并将 continuous 6D hand command 作为 baseline；主方法应优先输出 hand-code。

## 2. Objects

第一对象：

- 50-60 mm 泡沫块或海绵块。
- 质量 <30 g。
- 表面高摩擦，不用硬塑料、不用金属、不用太滑的瓶子。
- 用双面胶/纸质定位框标出起始位置，但不要把物体粘住。

第二对象：

- EVA/泡沫圆柱，直径 45-60 mm，高 70-100 mm，质量 <80 g。

第三对象：

- 轻质塑料杯或小瓶，直径 55-70 mm，质量 <100 g。

## 3. Standard Stages

每条轨迹固定为：

1. `open`: RH56 全开，机械臂到 home。
2. `approach`: 末端到 pregrasp pose，速度低。
3. `pre_shape`: 手指到预形状，避免接近时戳到物体。
4. `close`: 执行 `envelope_close` 或 `power_grasp`。
5. `settle`: 等待 0.5-1.0 s，读取 hand_state。
6. `lift`: z 方向抬升 3 cm 或 8 cm。
7. `hold`: 保持 2-3 s。
8. `place`: 第二阶段才启用，移动到固定托盘。
9. `release`: 打开手。
10. `review`: 写入 success/failure/use_for_bc。

## 4. Success Definition

### Grasp-Lift 任务

`success = true` 仅当全部满足：

- 物体离桌高度 >= 5 cm。
- 保持 >= 2 s。
- 物体没有明显滑落。
- 没有人工接管。
- 没有机械臂/手碰撞报警。

`weak_success = true`：

- 抬起了但只保持 <2 s。
- 抬起高度不足 5 cm 但明显离桌。
- 抓取姿态极不稳定，肉眼判断不适合训练 BC。

`use_for_bc = true` 仅用于 clean success。weak success 默认不进入第一版 BC。

### Pick-and-Place 任务

`success = true` 仅当：

- 物体被抓起并移动到目标托盘。
- release 后物体中心在托盘内。
- 释放后稳定 >=2 s。

## 5. Failure Modes

固定枚举：

- `pregrasp_misaligned`
- `early_collision`
- `object_pushed_before_close`
- `grasp_empty`
- `insufficient_closure`
- `over_closure_ejected`
- `slip_during_lift`
- `object_rotated_out`
- `arm_pose_error`
- `hand_delay_or_no_response`
- `hand_state_mismatch`
- `place_collision`
- `manual_takeover`
- `estop_triggered`
- `unknown`

## 6. Step-by-Step Collection Gate

### Gate 1: 空载开合测试

- 10 trials。
- 只开合 RH56，不接触物体。
- 记录每根手指 raw command 和 hand_state。
- 通过标准：10/10 无丢包，无反向，无明显卡顿。

如果失败：不得采物体数据。先修 `raw_open/raw_close/safe_min/safe_max/direction_sign`。

### Gate 2: 固定物体 close-only

- 10 trials。
- 机械臂到 pregrasp，不抬升，只闭合。
- 通过标准：>=7/10 能形成稳定包络接触，且不把物体推出定位框。

如果 <60%：调整 pregrasp pose、手掌高度、手掌 yaw、close_strength、对象大小。

### Gate 3: lift 3 cm

- 10 trials。
- close 后抬升 3 cm，保持 2 s。
- 通过标准：>=7/10 success。

如果 <60%：不要进入 8 cm；换软/大/轻物体或降低 close speed/strength。

### Gate 4: lift 8 cm

- 20 trials。
- close 后抬升 8 cm，保持 2 s。
- 通过标准：>=14/20 success。

如果 <60%：回到 Gate 3；不要训练 policy。

### Gate 5: pick-and-place

- 20 trials。
- 放入固定托盘。
- 通过标准：>=12/20 success。

如果 <60%：先把 place 分开，做 grasp-lift-hold 数据集。

## 7. Per-Stage Metrics

每个 gate 都统计：

- `success_rate`
- `failure_mode_count`
- `mean_grasp_time_sec`
- `mean_final_height_cm`
- `slip_count`
- `hand_command_latency_ms`，如能测。
- `replay_success_rate`，抽样执行。

## 8. Training Start Rule

- 20 clean success：只允许 smoke test。
- 50 clean success：可以训练固定起点状态 BC。
- 100 clean success：可以做 BC ablation。
- 100 clean success + 稳定相机标定：再考虑 Diffusion Policy。

## 9. Manual Review YAML

```yaml
episode_id: episode_0001
task_name: fixed_foam_cube_lift_8cm
auto_success: false
manual_success: true
weak_success: false
use_for_bc: true
failure_mode: null
reviewer: operator_1
notes: "clean lift, no slip, good for BC"
```

失败例：

```yaml
episode_id: episode_0037
task_name: fixed_foam_cube_lift_8cm
auto_success: false
manual_success: false
weak_success: false
use_for_bc: false
failure_mode: slip_during_lift
reviewer: operator_1
notes: "contact too high on cube; slipped at 4 cm"
```
