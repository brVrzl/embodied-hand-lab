# Data Protocol

## Episode 定义

一个 episode 是一次完整任务尝试，必须包含：

- `task_name`
- `natural_language_instruction`
- `start_time`
- `end_time`
- `success`
- `failure_reason`
- `operator_notes`
- 每个时间步的 observation / action

## 采样频率建议

- arm EDG servo internal loop: 62.5-125 Hz
- arm ROS state: 50-100 Hz
- hand PC direct state: 20-50 Hz
- hand command: 10-20 Hz
- camera RGB-D: 15-30 Hz
- policy: 5-20 Hz
- recorder aligned export: 10-20 Hz

ROS2 用于协调、状态发布和记录；不要把普通 Python ROS node 当作硬实时控制环。JAKA 高频精细控制应由 EDG servo backend/node 负责。

## Topic 命名规范

- arm
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
- hand
  - `/hand/state`
  - `/hand/raw_feedback`
  - `/hand/backend_mode`
  - `/hand/command_code`
  - `/hand/command_angles`
  - `/hand/command_force`
  - `/hand/execute_grasp`
- camera
  - `/sensors/camera/color/image_raw`
  - `/sensors/camera/depth/image_raw`
  - `/sensors/camera/color/camera_info`

## Frame 命名规范

- `jaka_base`
- `jaka_tool0`
- `rh56_palm`
- `camera_link`
- `camera_color_optical_frame`
- `camera_depth_optical_frame`
- `odom`
- `base_link`

## 成功失败标注规范

- `success`: 任务目标满足且无人工接管
- `failure`: 任务目标未满足、发生错误或人工终止
- `partial_success`: 可保留在后续版本，第一版先写入 `operator_notes`

失败原因建议使用短语：

- `grasp_failed`
- `slip_detected`
- `pose_out_of_tolerance`
- `navigation_timeout`
- `manual_takeover`
- `estop_triggered`

## 自然语言任务描述规范

- 简短、可操作、面向机器人行为
- 包含对象、动作、目标位置或目标状态
- 避免高层模糊描述

推荐示例：

- `pick the red cube and place it into the left tray`
- `grasp the bottle and hold it upright for 5 seconds`
- `walk to waypoint B, inspect shelf 2, and return to dock`

## 文件组织

原始 episode：

```text
episode_xxx/
  metadata.json
  steps.jsonl
  rgb/
  depth/
```

导出数据：

- `data/exports/structured`
- `data/exports/lerobot`
