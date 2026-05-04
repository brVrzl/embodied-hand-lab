# MuJoCo RH56 Collision Diagnostics

Date: 2026-04-30

## Commands

Static pose collision check:

```bash
source .venv/bin/activate
python tools/check_mujoco_rh56_collision_modes.py \
  --out-dir data/collision_diagnostics/pose_modes
```

Old benchmark collision check:

```bash
source .venv/bin/activate
python tools/mujoco_rh56_grasp_benchmark.py \
  --objects foam_cube light_cylinder paper_box \
  --max-candidates 72 \
  --duration 3.5 \
  --success-lift 0.02 \
  --collision-mode proxy \
  --out-dir data/collision_diagnostics/full_proxy
```

Existing hand-ref planner summaries were also inspected under:

```text
data/mujoco_handref_grasps/*/summary.json
```

## Static Collision Findings

Static pose results:

| Collision mode | open | real_pinch_v4 | sim_best_pinch | power_close |
| --- | ---: | ---: | ---: | ---: |
| `proxy` | 0 contacts | 0 contacts | 5 contacts, 2 hand-self | 3 contacts, 2 hand-self |
| `mesh` | 4 total contacts, JAKA base/link noise | 4 total contacts, JAKA base/link noise | 6 total, 2 RH56 hand-self | 6 total, 2 RH56 hand-self |
| `mesh_proxy` | 4 total contacts, JAKA base/link noise | 4 total contacts, JAKA base/link noise | 9 total, 2 RH56 hand-self | 7 total, 2 RH56 hand-self |

Important contact pairs:

```text
sim_best_pinch:
  rh56_R_thumb_intermediate_collision <-> rh56_R_index_distal_collision
  rh56_R_thumb_distal_collision       <-> rh56_R_index_distal_collision
  thumb_pad_proxy                     <-> index_pad_proxy

power_close:
  rh56_R_thumb_proximal_collision     <-> rh56_R_index_distal_collision
  rh56_R_thumb_intermediate_collision <-> rh56_R_index_distal_collision
```

Interpretation:

- `open`, `thumb_rotate`, and real `pinch_box_v4` are clean under `proxy`.
- Aggressive `sim_best_pinch` and `power_close` create thumb-index self contact.
- The self-contact is likely meaningful: it matches the observed physical thumb-index blocking risk.
- `mesh`/`mesh_proxy` show persistent JAKA Link0-Link1 contacts. These are arm mesh noise and should be ignored or filtered for hand collision diagnostics.

## Grasp Benchmark Findings

The older `mujoco_rh56_grasp_benchmark.py` with 72 proxy candidates produced opposing contacts but failed the 2 cm lift criterion:

| Object | Best lift | Best contacts | Success |
| --- | ---: | --- | --- |
| `foam_cube` | 0.0088 m | thumb 3, index 2, middle 3 | false |
| `light_cylinder` | 0.0127 m | thumb 4, index 3, middle 3 | false |
| `paper_box` | 0.0030 m | thumb 1, index 3, middle 3 | false |

This old benchmark is not representative of the current best pipeline. The newer hand-ref planner has much better results:

| Object | Success count | Best lift | Best contacts |
| --- | ---: | ---: | --- |
| `foam_block_40mm` | 55 / 160 | 0.0965 m | thumb 2, index 1, middle 2 |
| `light_cylinder_36mm` | 16 / 160 | 0.1120 m | thumb 4, index 3, middle 3, ring/pinky 6 |
| `light_can_50mm` | 10 / 160 | 0.1111 m | thumb 7, index 3, middle 3, ring/pinky 6 |
| `paper_box` legacy | 67 / 80 | 0.0907 m | thumb 2, index 1, middle 2 |

Interpretation:

- Current MuJoCo collision is usable for candidate ranking when using the hand-ref planner.
- The old width-sweep benchmark should not be used as the main collision quality indicator.
- Current collision still needs calibration before making sim-to-real claims.

## UniFucGrasp Asset Comparison

Imported UniFucGrasp force-sensor meshes show finger pad dimensions roughly:

| Mesh family | Approx extents |
| --- | --- |
| index/middle/ring pad sensors | 10.8-11.6 mm x 14.3-15.8 mm x 1.7-1.9 mm |
| little/pinky pad sensors | 12.3 mm x 15.4-16.5 mm x 1.7-2.0 mm |
| thumb pad sensors | 6.0-11.7 mm x 3.9-15.8 mm x 2.5-2.9 mm |
| palm sensor | 50.0 mm x 30.1 mm x 2.0 mm |

Current local pad proxies in `mujoco_rh56_grasp_benchmark.py` are spheres:

```text
thumb_pad_proxy  radius 6.5 mm
index_pad_proxy  radius 6.5 mm
middle_pad_proxy radius 6.5 mm
ring_pad_proxy   radius 6.3 mm
pinky_pad_proxy  radius 6.0 mm
```

The radius is reasonable for pad width, but the spherical shape is too thick compared with UniFucGrasp's flat force-sensor pads.

## Current Collision Verdict

Current MuJoCo collision is acceptable for:

- kinematic hand pose screening;
- rejecting obvious self-collision;
- ranking grasp candidates in the hand-ref planner;
- collecting sim candidates for later real replay.

It is not yet sufficient for:

- claiming final force closure;
- predicting slip reliably;
- reporting sim-to-real success rate;
- direct execution of learned UniFucGrasp predictions.

## Recommended Next Fix

Use the new default collision option:

```text
unifuc_pad_proxy
```

Implementation direction:

- Keep the current RH56/JAKA kinematic chain and mount.
- Keep current segment capsules for coarse collision.
- Replace or supplement spherical `*_pad_proxy` geoms with the existing flat rectangular pads sized from UniFucGrasp force-sensor meshes:

```text
index/middle/ring: size approx 0.0055 0.0075 0.0012
pinky:             size approx 0.0062 0.0080 0.0012
thumb:             size approx 0.0055 0.0075 0.0015, adjusted per thumb link
```

Then re-run:

```bash
python tools/check_mujoco_rh56_collision_modes.py
python tools/rh56_handref_grasp_planner.py --objects foam_block_40mm light_cylinder_36mm light_can_50mm
```

Implemented direction:

- Default benchmark/viewer collision mode is the existing project-local `unifuc_pad_proxy`.
- Old spherical mode remains available as `proxy` for A/B comparison.
- Rectangular pads use the already validated distal `*_pad_proxy` names and centers for metrics and object placement.

# 中文版本

## 目标

该文档记录 RH56 MuJoCo 碰撞模型的诊断过程。它的用途是判断当前 RH56 模型在不同姿态下是否出现自碰撞、指尖代理碰撞过厚、撞桌或不合理接触。

## 主要命令

静态姿态碰撞检查：

```bash
source .venv/bin/activate
python tools/check_mujoco_rh56_collision_modes.py \
  --out-dir data/collision_diagnostics/pose_modes
```

grasp planner 碰撞检查：

```bash
python tools/rh56_handref_grasp_planner.py \
  --objects foam_block_40mm light_cylinder_36mm light_can_50mm
```

## 观察

当前球形 pad proxy 容易产生一些假阳性自碰撞，尤其是在拇指和食指附近。为了更接近真实 RH56 指尖/力传感器形状，建议使用更薄、更贴合指尖的 box-like proxy。

建议尺寸大致为：

```text
index/middle/ring: 约 0.0055 0.0075 0.0012
pinky:             约 0.0062 0.0080 0.0012
thumb:             约 0.0055 0.0075 0.0015，并根据拇指 link 单独调整
```

## 预期改进

- 减少人为 pad 厚度。
- 更真实地表示指尖接触面。
- 降低球形 pad 带来的假自碰撞。
- 更接近 UniFucGrasp Inspire force-sensor geometry。
