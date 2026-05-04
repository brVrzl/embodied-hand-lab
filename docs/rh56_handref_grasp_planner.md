# RH56 Hand-Ref Grasp Planner

Date: 2026-04-28

This planner implements the short-term method selected from `hand_ref.md`:

- Object-conditioned 6D wrist pose sampling.
- RH56-specific analytical width/contact planning for hand closure.
- Staged hybrid close: open, thumb/preshape, approach, slow final close, lift.
- MuJoCo contact/lift validation before exporting a real-hand preset candidate.
- Replay export and failure taxonomy for future BC/DP training data curation.
- The MuJoCo scene is shifted to a tabletop frame with table top at `z=0.80m`.

Run:

```bash
scripts/run_rh56_handref_grasps_smoke.sh
scripts/run_rh56_handref_grasps.sh
```

Outputs:

- `data/mujoco_handref_grasps/benchmark_summary.json`
- `data/mujoco_handref_grasps/<object>/summary.json`
- `data/mujoco_handref_grasps/<object>/candidates.json`
- `data/mujoco_handref_grasps/handref_presets.yaml`
- `data/mujoco_handref_grasps/replay_dataset.jsonl`
- `data/mujoco_handref_grasps/baseline_v0/baseline_summary.json`
- `data/mujoco_handref_grasps/baseline_v0/presets.yaml`
- `data/mujoco_handref_grasps/baseline_v0/success_replays.jsonl`

View the best candidate:

```bash
.venv/bin/python tools/view_mujoco_rh56_pose_contact.py --mode grasp --object foam_block_40mm
.venv/bin/python tools/view_mujoco_rh56_pose_contact.py --mode grasp --object light_cylinder_36mm
.venv/bin/python tools/view_mujoco_rh56_pose_contact.py --mode grasp --object light_can_50mm
```

View another ranked candidate:

```bash
.venv/bin/python tools/view_mujoco_rh56_pose_contact.py --mode grasp --object foam_block_40mm --rank 1
```

Compatibility aliases:

| Old name | Current object |
| --- | --- |
| `foam_cube` | `foam_block_40mm` |
| `paper_box` | `061_foam_brick` |
| `light_cylinder` | `light_cylinder_36mm` |
| `can` | `light_can_50mm` |
| `light_can` | `light_can_50mm` |
| `round_ball` | `056_tennis_ball` |

Default object set:

| Object | Family | Current best result |
| --- | --- | --- |
| `foam_block_40mm` | box precision pinch | lifts about 10.0 cm with thumb/index/middle pinch and no ring/pinky overwrap |
| `light_cylinder_36mm` | cylinder power envelope | lifts about 11.2 cm from a side/power grasp |
| `light_can_50mm` | horizontal can side grasp | lifts about 11.1 cm from a side/power grasp |

2026-05-04 smoke with the existing project-local `unifuc_pad_proxy` and 40 candidates per object:

| Object | Successes / Candidates | Best lift | Best wrist pose | Best close raw |
| --- | ---: | ---: | --- | --- |
| `foam_block_40mm` | 16 / 40 | `0.100 m` | `precision_yaw_left` | `[1000, 1000, 580, 550, 400, 0]` |
| `light_cylinder_36mm` | 10 / 40 | `0.112 m` | `power_center` | `[500, 500, 500, 500, 450, 0]` |
| `light_can_50mm` | 10 / 40 | `0.111 m` | `power_axis_left` | `[500, 500, 500, 500, 450, 0]` |

These are MuJoCo contact-filter results, not real-hardware claims. They are good enough to drive the next engineering step: export presets and replay a small subset on the real RH56 after PC-direct feedback is available.

The first strict audit + visual evidence + learned ranker baseline is recorded in:

- `docs/rh56_handref_baseline_report.md`

The can task is horizontal because the upright-can probe was pushed away by the current
arm approach before a stable opposing contact formed. That makes it a side-grasp task
instead of an upright top/side hybrid task.

Additional non-default benchmark objects are still available for later evaluation:

```text
062_dice
009_gelatin_box
061_foam_brick
004_sugar_box
005_tomato_soup_can
040_large_marker
056_tennis_ball
```

The exported raw presets use physical RH56 order:

```text
[pinky, ring, middle, index, thumb_bend, thumb_rotate]
```

The object XML uses two geoms per object: `bench_object_visual` for display, and
`bench_object` as a transparent analytic collision proxy. This avoids judging visible
mesh penetration from a collision surface that is smaller than the rendered object.

Planner outputs include per-candidate arm poses:

- `grasp_q`
- `approach_q`
- `lift_q`
- `wrist_pose_name`
- `wrist_delta`
- `wrist_rpy`
- `ik_error_m`
- `ik_rot_error`
- `failure_mode`

This is intentionally a grasp-pose generator plus physics filter, not a single shared
arm preset. It follows the pattern used by recent dexterous grasp work: generate multiple
robot-object spatial candidates, then reject penetrations, unstable contacts, pushed-away
objects, weak opposing contacts, slip-out trials, and failed lift trials before any BC/DP
policy training.

# 中文版本

## 目标

该 planner 是当前 RH56 hand-reference grasp 方向的短期实现。它的目标不是训练一个完整策略，而是先生成多个候选 grasp pose，并用 MuJoCo 接触和 lift 结果过滤掉明显不可执行的候选。

核心流程：

- 根据物体生成或读取点云。
- 估计物体宽度与主要几何方向。
- 采样 object-conditioned 6D wrist/palm pose。
- 选择 RH56-specific hand closure 或 hand-code。
- 在 MuJoCo 中检查碰撞、接触、滑落和 lift。
- 导出可 replay 的候选，用于后续真机 preset 或 BC/DP 数据。

## 输出内容

planner 会输出每个候选对应的 arm pose 和诊断信息，例如：

- `grasp_q`
- `approach_q`
- `lift_q`
- `wrist_pose_name`
- `wrist_delta`
- `wrist_rpy`
- `ik_error_m`
- `ik_rot_error`
- `failure_mode`

这些信息用于判断候选是否可达、是否接触合理、是否存在撞桌/自碰撞/弱接触/滑出等问题。

当前建议先跑：

```bash
scripts/run_rh56_handref_grasps_smoke.sh
scripts/run_rh56_handref_grasps.sh --objects foam_block_40mm light_cylinder_36mm light_can_50mm --max-candidates 40
```

然后用 viewer 看前三类物体的最佳候选：

```bash
DISPLAY=:1 scripts/view_mujoco_rh56_pose_contact.sh --mode grasp --object foam_block_40mm --show-contacts
DISPLAY=:1 scripts/view_mujoco_rh56_pose_contact.sh --mode grasp --object light_cylinder_36mm --show-contacts
DISPLAY=:1 scripts/view_mujoco_rh56_pose_contact.sh --mode grasp --object light_can_50mm --show-contacts
```

## 定位

该模块应被理解为：

```text
grasp-pose generator + physics filter
```

而不是单一固定 arm preset。它符合当前 palm-frame hand-code 方向：先生成多个 robot-object spatial candidate，再用物理过滤和真实 replay 验证，最后才考虑训练 BC/DP 策略。
