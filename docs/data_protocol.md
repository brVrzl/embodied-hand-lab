# Data Protocol

Date: 2026-05-04

Current schema version: `jaka_rh56_palm_handcode_v0.1`

This project uses three data layers:

1. Raw synchronized runtime streams: ROS2 topics, preferably recorded as `rosbag2` with MCAP storage when available.
2. Episode-step JSONL: the project-owned lossless working format under `data/episodes`.
3. Training exports: LeRobot/RLDS/robomimic-compatible views generated from the episode-step data.

The important rule is that the project-owned format must not redefine common robotics-learning words in surprising ways. `observation` means the data available at time `t`; `action` means the command chosen at time `t` and applied after that observation; `reward`, `discount`, `is_first`, `is_last`, and `is_terminal` follow RLDS-style transition semantics.

## Mainstream Alignment

LeRobot v3 is the preferred training/export target. It uses Parquet for tabular state/action/timestamp data, MP4 for visual streams, and metadata for schema, statistics, task ids, and episode boundaries. Our final LeRobot exporter should therefore write `meta/info.json`, `meta/tasks.jsonl`, `meta/stats.json`, `meta/episodes/*`, `data/*`, and `videos/*`. The current JSONL exporter is only a compatibility preview.

RLDS is the semantic reference for transition keys:

```text
episode -> steps
step -> observation, action, reward, discount, is_first, is_last, is_terminal
```

robomimic/HDF5 remains useful for algorithm compatibility, but it expects trajectory groups with ordered `obs`, `next_obs`, `actions`, `rewards`, and `dones`. Images should be `uint8` channel-last, and action vectors should be explicitly normalized if exported for robomimic-style policies.

ROS bag is not the training schema. It is the raw stream replay/audit format for `/arm/*`, `/hand/*`, camera topics, `/tf`, `/tf_static`, policy commands, and safety events.

## Episode Definition

An episode is one complete task attempt. It must contain:

- `episode_id`: stable string id.
- `episode_index`: contiguous integer id within an export.
- `task_name`: short task id.
- `task_index`: contiguous integer task id within an export.
- `natural_language_instruction`: language condition.
- `start_time`, `end_time`, `duration_sec`.
- `success`: bool or null until reviewed.
- `failure_mode`: controlled vocabulary.
- `operator_notes`.
- one or more time-ordered steps.

`episode_id` is for long-term identity. `episode_index` is only an export-local integer and should not be used as a permanent id.

## Step Definition

Each step in a training export must contain:

- `index`: global zero-based integer row index within the export.
- `episode_index`: integer.
- `frame_index`: zero-based integer frame id within the episode.
- `task_index`: integer.
- `timestamp`: seconds.
- `observation`: dict of sensor/state values at time `t`.
- `action`: command selected at time `t`, applied after `observation`.
- `reward`: scalar.
- `discount`: scalar in `[0, 1]`.
- `is_first`: bool.
- `is_last`: bool.
- `is_terminal`: bool.

Boundary semantics:

- The first step of each episode has `is_first=true`.
- The last step has `is_last=true`, `is_terminal=true`, and `discount=0`.
- Non-last steps have `is_last=false`, `is_terminal=false`, and `discount=1`.
- For sparse success reward, only the final successful step uses `reward=1`; other steps use `0`.

## Observation Naming

Project JSONL keeps rich dictionaries for debuggability, but training exporters should map them to mainstream names:

| Project field | Training meaning |
| --- | --- |
| `observation.rgb_path` / `rgb_paths` | image paths or future `observation.images.*` |
| `observation.depth_path` / `depth_paths` | depth paths or future `observation.depth.*` |
| `observation.arm_joint_states` | arm proprioception source |
| `observation.arm_ee_pose` | end-effector pose source |
| `observation.hand_states` | full RH56 diagnostic state |
| `observation.state` | named low-dimensional state dict |
| `observation.extra_observation` | privileged or task-specific labels |

`observation.state` is a named dict in the project format. In final LeRobot v3 export it should become either a flat numeric `observation.state` tensor plus `features` metadata, or named low-dimensional keys such as `observation.state.arm_q`, `observation.state.ee_pose`, and `observation.state.hand_state`.

## Hand State Semantics

Canonical RH56 order:

```text
[index, middle, ring, pinky, thumb_close, thumb_lateral]
```

Official RH56 protocol order:

```text
[pinky, ring, middle, index, thumb_close, thumb_lateral]
```

Dataset-facing hand values must be canonical. Raw protocol values may be stored only under explicit raw/debug fields such as `hand_states.rh56_raw` or `/hand/raw_feedback`.

Recommended hand fields:

- `observation.state.hand_state`: normalized canonical `[0, 1]`, where `0=open`, `1=closed`.
- `observation.state.hand_cmd_last`: previous normalized canonical command.
- `observation.state.hand_error`: `hand_cmd_last - hand_state`.
- `action.hand_cmd`: next normalized canonical command.
- `action.hand_delta_cmd`: command-space delta from previous command.
- `action.hand_delta_state`: clipped state-space delta from observed hand state.
- `action.hand_order`: the canonical order list.

Do not call protocol raw values `hand_state` without a suffix. Use `raw`, `protocol`, or `rh56_raw` in the field name.

## Action Semantics

The default policy action is not raw JAKA joints plus raw six-finger commands. The preferred action object is:

```yaml
action:
  ee_delta: [dx, dy, dz, droll, dpitch, dyaw]
  hand_code_id: int | null
  hand_cmd: [6]
  hand_delta_cmd: [6]
  close_strength: float | null
```

Required metadata disambiguates `ee_delta`:

- `ee_delta_frame`: `base` or `ee_local`.
- `rotation_delta_type`: `euler_xyz` or `rotvec`.
- `ee_translation_delta_limit_type`: `per_axis` or `norm`.
- `action_timing`: `action_at_t_applied_after_observation_t`.

If an exporter needs a single continuous vector, define the vector layout in metadata and keep the named dict as the source of truth.

## File Organization

Raw episode:

```text
episode_xxx/
  metadata.json
  steps.jsonl
  rgb/
  depth/
```

Structured export:

```text
data/exports/structured/
  manifest.json
  samples.jsonl
```

LeRobot preview export:

```text
data/exports/lerobot/
  meta/info.json
  train/samples.jsonl
```

The preview export is intentionally not called official LeRobot v3 because it does not yet write Parquet/MP4 shards.

## Sampling Frequencies

- arm EDG servo internal loop: 62.5-125 Hz.
- arm ROS state: 50-100 Hz.
- RH56 PC-direct state: 20-50 Hz.
- RH56 command: 10-20 Hz.
- RGB-D: 15-30 Hz.
- policy: 5-20 Hz.
- aligned training export: 10-20 Hz.

## Topic Names

- `/arm/joint_states`
- `/arm/ee_pose`
- `/arm/edg_state`
- `/arm/command/joints`
- `/arm/command/pose`
- `/arm/follow_joint_trajectory`
- `/arm/servo/enable`
- `/arm/servo/command_delta_pose`
- `/arm/servo/command_twist`
- `/arm/servo/state`
- `/hand/state`
- `/hand/raw_feedback`
- `/hand/backend_mode`
- `/hand/command_code`
- `/hand/command_angles`
- `/hand/command_force`
- `/hand/execute_grasp`
- `/sensors/camera/color/image_raw`
- `/sensors/camera/depth/image_raw`
- `/sensors/camera/color/camera_info`

## Frame Names

- `jaka_base`
- `jaka_tool0`
- `rh56_palm`
- `camera_link`
- `camera_color_optical_frame`
- `camera_depth_optical_frame`
- `odom`
- `base_link`

## Failure Labels

Use the controlled set in `data_recorder.episode_recorder.FAILURE_MODES`:

- `none`
- `fail_late_close`
- `fail_early_close`
- `fail_lateral_offset`
- `fail_low_grip`
- `fail_object_slip`
- `fail_collision`
- `fail_timeout`
- `unknown`

Do not mix free-form failure names into training labels. Put extra explanation in `failure_reason` or `operator_notes`.

## RH56 ROS2 JSON Bridge

The current offline bridge uses `std_msgs/String` JSON payloads so it can run before custom ROS2 message generation is finalized. This is an integration layer, not the final message definition.

State topic payloads must include:

- `schema_version`
- `timestamp`
- `backend_mode`
- `canonical_hand_order`
- `hand.position`
- `hand.position_unit`
- `hand.order`

Command topic payloads must include explicit units:

```json
{"values": [1000, 1000, 1000, 1000, 1000, 1000], "unit": "rh56_angle_raw_0_1000", "order": "canonical"}
```

Normalized commands such as `normalized_0_1` must go through a calibrated policy adapter. They are intentionally rejected by `/hand/command_angles` to prevent silent raw/normalized confusion.

## References

- LeRobotDataset v3.0: https://huggingface.co/docs/lerobot/lerobot-dataset-v3
- RLDS: https://github.com/google-research/rlds
- robomimic dataset overview: https://robomimic.github.io/docs/v0.2/datasets/overview.html
- rosbag2: https://github.com/ros2/rosbag2
- Open X-Embodiment: https://arxiv.org/abs/2310.08864

# 中文版本

当前 schema 版本：`jaka_rh56_palm_handcode_v0.1`

本项目把数据分成三层：

1. 原始同步运行流：ROS2 topics，条件允许时用 `rosbag2` + MCAP 记录。
2. episode-step JSONL：项目内部的无损工作格式，位于 `data/episodes`。
3. 训练导出：从 episode-step 数据生成 LeRobot/RLDS/robomimic 兼容视图。

核心原则是不要重新发明一套容易和主流冲突的语义。`observation` 表示时间 `t` 可见的数据；`action` 表示时间 `t` 基于该 observation 选择、随后下发的命令；`reward`、`discount`、`is_first`、`is_last`、`is_terminal` 采用 RLDS 风格的 transition 语义。

## 与主流格式的关系

训练导出的首选目标是 LeRobot v3。LeRobot v3 使用 Parquet 存低维 state/action/timestamp，使用 MP4 存视觉流，并通过 metadata 管理 schema、统计量、task id 和 episode 边界。因此，最终正式 LeRobot exporter 应写出 `meta/info.json`、`meta/tasks.jsonl`、`meta/stats.json`、`meta/episodes/*`、`data/*`、`videos/*`。当前 JSONL exporter 只是兼容预览，不应声称是正式 LeRobot v3。

RLDS 是 transition key 的语义参考：

```text
episode -> steps
step -> observation, action, reward, discount, is_first, is_last, is_terminal
```

robomimic/HDF5 对算法兼容仍有价值，但它习惯使用 trajectory group，里面包含有序的 `obs`、`next_obs`、`actions`、`rewards`、`dones`。如果导出到 robomimic 风格，图像应是 `uint8`、channel-last，action vector 是否归一化必须写清楚。

ROS bag 不是训练 schema，而是原始同步流的回放和审计格式，用于记录 `/arm/*`、`/hand/*`、camera topics、`/tf`、`/tf_static`、policy command 和 safety events。

## Episode 定义

一个 episode 是一次完整任务尝试，必须包含：

- `episode_id`：稳定字符串 id。
- `episode_index`：导出内部连续整数 id。
- `task_name`：短任务名。
- `task_index`：导出内部连续整数 task id。
- `natural_language_instruction`：语言条件。
- `start_time`、`end_time`、`duration_sec`。
- `success`：bool，未复核前可为 null。
- `failure_mode`：受控枚举。
- `operator_notes`。
- 一个或多个按时间排序的 step。

`episode_id` 用于长期身份；`episode_index` 只是导出内部整数，不应用作永久 id。

## Step 定义

训练导出里的每个 step 必须包含：

- `index`：导出内部全局零基整数行号。
- `episode_index`：整数。
- `frame_index`：episode 内零基整数帧号。
- `task_index`：整数。
- `timestamp`：秒。
- `observation`：时间 `t` 的传感器和状态。
- `action`：时间 `t` 选择、随后执行的命令。
- `reward`：标量。
- `discount`：`[0, 1]` 标量。
- `is_first`：bool。
- `is_last`：bool。
- `is_terminal`：bool。

边界语义：

- 每个 episode 第一帧 `is_first=true`。
- 最后一帧 `is_last=true`、`is_terminal=true`、`discount=0`。
- 非最后一帧 `is_last=false`、`is_terminal=false`、`discount=1`。
- 如果使用稀疏成功奖励，只有成功 episode 的最后一帧 `reward=1`，其余为 `0`。

## Observation 命名

项目 JSONL 保留 rich dict，方便诊断；训练导出时再映射到主流字段：

| 项目字段 | 训练语义 |
| --- | --- |
| `observation.rgb_path` / `rgb_paths` | 图像路径，未来映射到 `observation.images.*` |
| `observation.depth_path` / `depth_paths` | 深度路径，未来映射到 `observation.depth.*` |
| `observation.arm_joint_states` | 机械臂 proprioception 源数据 |
| `observation.arm_ee_pose` | 末端位姿源数据 |
| `observation.hand_states` | RH56 完整诊断状态 |
| `observation.state` | 命名低维状态 dict |
| `observation.extra_observation` | privileged/task-specific 标签 |

项目格式中的 `observation.state` 是命名 dict。正式 LeRobot v3 导出时，可以转成一个 flat numeric `observation.state` tensor，并在 `features` metadata 中定义 layout；也可以拆成 `observation.state.arm_q`、`observation.state.ee_pose`、`observation.state.hand_state` 等命名低维键。

## Hand State 语义

RH56 canonical 顺序：

```text
[index, middle, ring, pinky, thumb_close, thumb_lateral]
```

RH56 官方协议顺序：

```text
[pinky, ring, middle, index, thumb_close, thumb_lateral]
```

面向数据集和策略的手部数值必须是 canonical。协议 raw 值只能放在显式 raw/debug 字段里，例如 `hand_states.rh56_raw` 或 `/hand/raw_feedback`。

推荐手部字段：

- `observation.state.hand_state`：归一化 canonical `[0, 1]`，`0=open`，`1=closed`。
- `observation.state.hand_cmd_last`：上一条归一化 canonical 命令。
- `observation.state.hand_error`：`hand_cmd_last - hand_state`。
- `action.hand_cmd`：下一条归一化 canonical 命令。
- `action.hand_delta_cmd`：相对上一条命令的 command-space delta。
- `action.hand_delta_state`：相对当前观测手状态的 state-space delta，经过安全裁剪。
- `action.hand_order`：canonical 顺序列表。

不要把协议 raw 值直接叫 `hand_state`。字段名里必须带 `raw`、`protocol` 或 `rh56_raw`。

## Action 语义

默认 policy action 不是 JAKA 原始关节加 RH56 原始六指命令。推荐 action object 是：

```yaml
action:
  ee_delta: [dx, dy, dz, droll, dpitch, dyaw]
  hand_code_id: int | null
  hand_cmd: [6]
  hand_delta_cmd: [6]
  close_strength: float | null
```

以下 metadata 必须写清楚 `ee_delta` 的含义：

- `ee_delta_frame`：`base` 或 `ee_local`。
- `rotation_delta_type`：`euler_xyz` 或 `rotvec`。
- `ee_translation_delta_limit_type`：`per_axis` 或 `norm`。
- `action_timing`：`action_at_t_applied_after_observation_t`。

如果某个 exporter 需要单个连续 action vector，必须在 metadata 中定义 vector layout，并以命名 dict 作为源格式。

## 文件组织

原始 episode：

```text
episode_xxx/
  metadata.json
  steps.jsonl
  rgb/
  depth/
```

结构化导出：

```text
data/exports/structured/
  manifest.json
  samples.jsonl
```

LeRobot 预览导出：

```text
data/exports/lerobot/
  meta/info.json
  train/samples.jsonl
```

这个预览导出不叫正式 LeRobot v3，因为它还没有写 Parquet/MP4 shards。

## 采样频率

- arm EDG servo internal loop：62.5-125 Hz。
- arm ROS state：50-100 Hz。
- RH56 PC-direct state：20-50 Hz。
- RH56 command：10-20 Hz。
- RGB-D：15-30 Hz。
- policy：5-20 Hz。
- aligned training export：10-20 Hz。

## Topic 名称

- `/arm/joint_states`
- `/arm/ee_pose`
- `/arm/edg_state`
- `/arm/command/joints`
- `/arm/command/pose`
- `/arm/follow_joint_trajectory`
- `/arm/servo/enable`
- `/arm/servo/command_delta_pose`
- `/arm/servo/command_twist`
- `/arm/servo/state`
- `/hand/state`
- `/hand/raw_feedback`
- `/hand/backend_mode`
- `/hand/command_code`
- `/hand/command_angles`
- `/hand/command_force`
- `/hand/execute_grasp`
- `/sensors/camera/color/image_raw`
- `/sensors/camera/depth/image_raw`
- `/sensors/camera/color/camera_info`

## Frame 名称

- `jaka_base`
- `jaka_tool0`
- `rh56_palm`
- `camera_link`
- `camera_color_optical_frame`
- `camera_depth_optical_frame`
- `odom`
- `base_link`

## Failure Label

使用 `data_recorder.episode_recorder.FAILURE_MODES` 中的受控集合：

- `none`
- `fail_late_close`
- `fail_early_close`
- `fail_lateral_offset`
- `fail_low_grip`
- `fail_object_slip`
- `fail_collision`
- `fail_timeout`
- `unknown`

不要把自由文本 failure name 混入训练标签。额外解释写入 `failure_reason` 或 `operator_notes`。

## RH56 ROS2 JSON Bridge

当前离线 bridge 使用 `std_msgs/String` JSON payload，这样在 custom ROS2 message 还没最终确定前也能运行。它是集成层，不是最终消息定义。

状态 topic payload 必须包含：

- `schema_version`
- `timestamp`
- `backend_mode`
- `canonical_hand_order`
- `hand.position`
- `hand.position_unit`
- `hand.order`

命令 topic payload 必须显式写单位：

```json
{"values": [1000, 1000, 1000, 1000, 1000, 1000], "unit": "rh56_angle_raw_0_1000", "order": "canonical"}
```

`normalized_0_1` 这类归一化命令必须先经过有明确标定的 policy adapter。`/hand/command_angles` 会故意拒绝 normalized 命令，避免 raw/normalized 被静默混用。
