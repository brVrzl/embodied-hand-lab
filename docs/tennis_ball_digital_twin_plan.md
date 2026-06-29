# Tennis Ball Digital Twin Plan

目标：先在 ManiSkill 中还原 JAKA mini2 + RH56 的桌面工作空间，第一阶段生成“抓起网球并抬升”的候选轨迹；再用真机低速 replay 采集真实数据。放置任务保留为第二阶段。

## Data Contract

每条 episode 同时保留三类 hand 信息：

- Raw trace: `hand_target_raw_count`，RH56 vendor count，`1000=open`、`0=close`，只用于硬件回放和追溯。
- Training norm: `hand_target_norm`，canonical normalized，`0=open`、`1=close`，用于 continuous baseline。
- Code annotation: `hand_code_id`、`hand_code`、`close_strength`，用于 hand-code policy 和 ablation。

Canonical hand order 固定为：

```text
[index, middle, ring, pinky, thumb_close, thumb_lateral]
```

第一版训练导出两个 view：

```text
continuous baseline action = ee_delta_6 + hand_target_norm_6
hand-code action           = ee_delta_6 + hand_code_id + close_strength
```

## Workspace Twin Parameters

当前第一阶段配置在 `configs/sim/maniskill_jaka_rh56_tennis_ball_lift.yaml`，任务注册名：

```text
TennisBallLiftJakaRH56-v1
```

第二阶段放置任务配置保留在 `configs/sim/maniskill_jaka_rh56_tennis_ball_place.yaml`。

需要用真实测量替换或确认的参数：

- Table size: length, width, thickness.
- Robot base pose relative to table: base offset from table right/front edges.
- Tennis ball: diameter, mass, surface condition.
- Object fixture: initial XY center and randomization range.
- Lift threshold: first use ball radius + 8 cm.
- Goal position: tabletop XY target center and allowed radius, second phase only.
- Camera extrinsics: camera eye/target or full camera-to-base transform.
- Camera intrinsics: RealSense resolution, FOV, depth alignment mode.
- Lighting/background: only for visual domain gap notes, not physics.

## Initial Scene

当前初版：

- Table: `1.20 m x 0.60 m x 0.04 m`.
- Robot base: reuses existing `JAKA_RH56_BASE_POSE`.
- Object: tennis-ball-sized sphere, radius `0.0335 m`, nominal mass `0.058 kg`.
- Spawn center: `[-0.12, 0.0]`, randomization half-size `0.04 m`.
- First task: lift until ball center height is at least `0.1135 m`.
- Place target: unused in the first task.
- Camera: reuses existing `base_camera` workspace view.

## Real Workspace Notes From Current Photo

- Table size: `1.20 m x 0.60 m`.
- Aluminum profile frame cross-section: `0.03 m x 0.03 m`.
- Left camera upright is roughly aligned with the upper-left table corner.
- Left camera mount height is roughly `0.50 m`; lower-left base stack is roughly `0.53 m`.
- Robot-side frame is on the right side of the table; current rough offsets are about `0.29 m`.
- These dimensions are initial priors only. Do not use them as final robot-to-camera extrinsics.

## Current Sim Parameters

The current rough workspace twin is stored in:

```text
configs/workspace/tennis_ball_lift_current.yaml
configs/sim/maniskill_jaka_rh56_tennis_ball_lift.yaml
```

Current sensor camera initial guess:

```yaml
eye_xyz_m: [0.20, -0.22, 0.53]
target_xyz_m: [-0.12, 0.0, 0.055]
fov_y_rad: 0.78
```

Current ball spawn region:

```yaml
center_xy_m_in_sim: [-0.12, 0.0]
half_size_m: 0.04
```

This corresponds to an 8 cm x 8 cm placement region in simulation. In real data collection, first place the tennis ball at the center for smoke tests, then randomize inside the same physical square.

## Preview

```bash
python -m sim_maniskill.scene_preview \
  --config configs/sim/maniskill_jaka_rh56_tennis_ball_lift.yaml \
  --output-dir data/previews/tennis_ball_lift
```

The preview writes:

```text
data/previews/tennis_ball_lift/scene_summary.json
data/previews/tennis_ball_lift/sensor_rgb.ppm
data/previews/tennis_ball_lift/human_render.ppm
```

## Sim Collection

Current recommendation: use MuJoCo first. ManiSkill remains optional and is not the shortest path on the current machine.

MuJoCo workspace preview:

```bash
MUJOCO_GL=egl python tools/preview_mujoco_tennis_ball_lift.py \
  --config configs/sim/mujoco_jaka_rh56_tennis_ball_lift.yaml \
  --build-only \
  --snapshot-dir data/previews/mujoco_tennis_ball_lift
```

This writes:

```text
data/mujoco_debug/tennis_ball_lift_workspace.xml
data/previews/mujoco_tennis_ball_lift/real_view.png
data/previews/mujoco_tennis_ball_lift/contact_view.png
```

Interactive MuJoCo viewer with a two-camera side panel:

```bash
python tools/preview_mujoco_tennis_ball_lift.py \
  --config configs/sim/mujoco_jaka_rh56_tennis_ball_lift.yaml
```

The main MuJoCo viewer shows the full scene. The OpenCV side panel shows:

- `real_view`: rough match to the current RealSense workspace view.
- `contact_view`: close-up view of the hand-ball contact region.

If running headless, use the `MUJOCO_GL=egl ... --build-only` command above.

ManiSkill collection, once a compatible Python 3.10/3.11 environment exists:

```bash
python -m sim_maniskill.cli \
  --config configs/sim/maniskill_jaka_rh56_tennis_ball_lift.yaml \
  --episodes 10 \
  --max-steps 100 \
  --policy zero
```

This command currently requires a Python environment with `gymnasium`, `mani_skill`, `sapien`, and `torch`. The project machine used on 2026-06-16 only had Python 3.12 and lacked compatible ManiSkill dependencies, so the command should be run from a Python 3.10/3.11 ManiSkill environment.

## Data Collection Flow

1. Sim preview: verify table, object, camera, and goal are in frame.
2. Sim teleop or scripted collection: generate grasp-lift candidate trajectories.
3. Real dry replay: arm path only, hand open, low speed.
4. Real contact replay: close hand and lift at reduced speed.
5. Human correction: use HEBI/Xbox to correct failed replay.
6. Training set: use real clean success as primary data; use sim only for pretraining or candidate generation.

## Replay Safety

Do not replay simulation trajectories directly at normal speed. Real replay should enforce:

- Deadman input.
- Workspace clamp.
- Joint position, velocity, and acceleration limits.
- TCP target horizon clamp.
- RH56 max close strength and force/current stop thresholds.
- Manual review before `use_for_bc=true`.
