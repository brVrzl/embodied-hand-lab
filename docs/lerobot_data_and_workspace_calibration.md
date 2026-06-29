# LeRobot Alignment and Workspace Calibration

目标：把本项目的 raw episode 格式稳定映射到 LeRobot/ACT/DP 可训练格式，同时把真实桌面、相机、机器人相对位置标定成可复现的仿真工作空间。

## Dataset Layers

保留两层数据，不混用：

1. Raw capture layer: `metadata.json + steps.jsonl + rgb/depth/*.npy`
   - 用于追溯、真机 replay、manual review、失败分析。
   - 保存所有原始值和标注。
2. Training export layer: LeRobot v3 style
   - 视频：每路相机导出 MP4 或 image sequence。
   - 表格：逐帧 state/action/reward/done 导出 Parquet。
   - 元数据：`meta/info.json`、`meta/stats.json`、`meta/tasks.*`、`meta/episodes.*`。

Raw layer 是项目内真实记录；training export layer 是给 ACT/DP/VLA 训练框架消费的视图。

## Canonical Features

第一版固定字段如下。

Observation:

```text
observation.images.side                 uint8 [H,W,3]
observation.images.wrist                uint8 [H,W,3], optional
observation.state.robot_q_current        float32 [6]
observation.state.ee_pose                float32 [7]  # xyz + xyzw
observation.state.hand_state_norm        float32 [6]
observation.state.last_hand_cmd_norm     float32 [6]
observation.state.object_pose            float32 [7], optional privileged
```

Continuous baseline action:

```text
action.ee_delta                          float32 [6]
action.hand_target_norm                  float32 [6]
```

Hand-code action:

```text
action.ee_delta                          float32 [6]
action.hand_code_id                      int64   []
action.close_strength                    float32 []
```

Shared frame fields:

```text
timestamp                                float64
frame_index                              int64
episode_index                            int64
task_index                               int64
next.done                                bool
reward                                   float32
```

Canonical hand order:

```text
[index, middle, ring, pinky, thumb_close, thumb_lateral]
```

Hand semantics:

```text
hand_target_norm: 0=open, 1=close
hand_target_raw_count: RH56 vendor count, 1000=open, 0=close
```

## Raw Step Requirements

每一帧 raw `steps.jsonl` 必须至少包含：

```json
{
  "timestamp": 0.0,
  "frame_index": 0,
  "stage": "approach",
  "rgb_paths": {"side": "rgb/000000_side.npy"},
  "depth_paths": {"side": "depth/000000_side.npy"},
  "arm_joint_states": {"positions": [0, 0, 0, 0, 0, 0]},
  "arm_ee_pose": {
    "frame_id": "jaka_base",
    "position": [0, 0, 0],
    "orientation_xyzw": [0, 0, 0, 1]
  },
  "hand_states": {
    "canonical_order": ["index", "middle", "ring", "pinky", "thumb_close", "thumb_lateral"],
    "normalized_positions": [0, 0, 0, 0, 0, 0],
    "raw_count": [1000, 1000, 1000, 1000, 1000, 1000],
    "current_count": [0, 0, 0, 0, 0, 0],
    "force_count": [0, 0, 0, 0, 0, 0]
  },
  "action": {
    "ee_delta": [0, 0, 0, 0, 0, 0],
    "hand_target_norm": [0, 0, 0, 0, 0, 0],
    "hand_delta_cmd_norm": [0, 0, 0, 0, 0, 0],
    "hand_target_raw_count": [1000, 1000, 1000, 1000, 1000, 1000],
    "hand_code_id": 0,
    "hand_code": "hold",
    "close_strength": 0.0,
    "source": "sim_replay"
  }
}
```

## Digital Twin Parameter Sheet

这些参数必须写入每条 sim/real episode 的 metadata，至少保存一个 `workspace_twin_id` 指向外参文件或 JSON。

```yaml
workspace_twin_id: jaka_rh56_tabletop_tennis_ball_v0
frames:
  world: sim_world
  robot_base: jaka_base
  table: worktable
  camera_side: camera_side_color_optical_frame
table:
  size_m: [1.20, 0.60, 0.04]
  top_z_in_robot_base_m: 0.0
  center_in_robot_base_m: [-0.015, 0.0, 0.0]
robot:
  base_pose_in_table_frame:
    xyz_m: [-0.25, -0.30, 0.0]
    quat_xyzw: [0, 0, 1, 0]
camera_side:
  intrinsics:
    width: 640
    height: 480
    fx: null
    fy: null
    cx: null
    cy: null
  T_camera_robot_base:
    source: hand_eye_or_apriltag
    xyz_m: null
    quat_xyzw: null
object:
  type: tennis_ball
  diameter_m: 0.067
  mass_kg: 0.058
fixture:
  object_init_xy_in_robot_base_m: [-0.12, 0.0]
  goal_xy_in_robot_base_m: [-0.02, 0.16]
```

## Calibration Procedure

### 1. Manual measurements

必须人工量或从 CAD/安装尺寸确认：

- 桌面长宽厚。
- 机器人 base 到桌边的 X/Y 偏移。
- 桌面高度相对机器人 base 的 Z。
- 网球直径、目标放置点相对桌面治具的位置。

相机不能可靠自动推断机器人 base 在桌面上的精确位置；这部分要人工量，再用视觉/触碰校正。

### 2. Camera intrinsics

RealSense 内参可以直接从相机 `camera_info` 或 SDK 读取：

- `width`
- `height`
- `fx`
- `fy`
- `cx`
- `cy`
- distortion model

这一步不需要人工标定，但要把相机 serial、分辨率、depth alignment 设置固定下来。

### 3. Camera extrinsics to robot base

推荐方法：AprilTag/Charuco 板 + 机器人触碰确认。

流程：

1. 把标定板固定在桌面，板坐标系相对桌面边缘的位置人工测量。
2. 相机检测标定板，得到 `T_camera_board`。
3. 根据人工测量得到 `T_robot_base_board`。
4. 解出 `T_camera_robot_base`。
5. 用机器人 TCP 触碰板上 3-5 个已知点，校验重投影误差和实际误差。

只靠单个 RGB-D 相机直接看桌面，可以估桌面平面和物体位置，但不能稳定得到完整 `camera -> robot_base` 高精度外参；必须引入标定板或机器人触碰点。

### 4. Table plane from depth

相机可以自动估：

- 桌面平面法向。
- 桌面高度的视觉估计。
- 网球中心位置。
- 目标标记位置，如果目标有 AprilTag/ArUco/颜色圆盘。

但这些应作为校正和运行时感知，不作为唯一 ground truth。

### 5. Robot-to-sim alignment

把标定结果写回仿真：

- MuJoCo: 修改 MJCF/worldbody 中 table、camera、object、goal pose。
- ManiSkill: 修改任务常量或 YAML `env_kwargs`。

推荐先保持一个 YAML/JSON workspace parameter file，再由 MuJoCo/ManiSkill 各自读取，避免两套仿真参数漂移。

## MuJoCo vs ManiSkill

MuJoCo 可以做这件事，尤其适合：

- JAKA+RH56 MJCF 已存在时的几何检查。
- 接触、碰撞、手型 codebook 静态验证。
- 真机 replay 前的轨迹可视化。

ManiSkill 当前更适合：

- Gym-style episode collection。
- RGB-D observation extraction。
- 直接复用 `EpisodeRecorder`。
- 后续训练数据导出 smoke test。

建议短期并行：

- MuJoCo: 做真实工作空间几何孪生和 replay preview。
- ManiSkill: 做 dataset/episode 采集和训练链路验证。

最终二者都读取同一个 workspace calibration file。
