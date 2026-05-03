# Pinch Box V1 Trial

Date: 2026-04-28

Object: small paper box.

Grasp style: top-down pinch.

Observation:

- Manually adjusted wrist and hand posture.
- The box was lifted stably.
- `dz 10` after grasp did not loosen the object, according to operator observation.

Saved JAKA presets:

- `pinch_grasp_box_v1`: grasp-height pose after moving `dz -10` from the stable lifted pose.
- `pinch_lift_box_v1`: stable lifted pose.
- `pinch_grasp_box_v2`: updated successful grasp pose after further manual adjustment.

Current measured TCP poses:

`pinch_grasp_box_v1`:

```text
x  = -39.628 mm
y  = -482.018 mm
z  = 194.229 mm
rx = -3.045623 rad
ry = 0.823495 rad
rz = 1.455070 rad
```

`pinch_lift_box_v1`:

```text
x  = -39.627 mm
y  = -482.019 mm
z  = 204.233 mm
rx = -3.045622 rad
ry = 0.823485 rad
rz = 1.455078 rad
```

`pinch_grasp_box_v2`:

```text
x  = -37.483 mm
y  = -502.359 mm
z  = 193.981 mm
rx = -3.041737 rad
ry = 0.733695 rad
rz = 1.569001 rad
```

Saved RH56 preset:

- `pinch_box_v1`
- `pinch_box_v2`
- `pinch_box_thumb_rotate_v1`
- `pinch_box_v3`
- `pinch_box_thumb_rotate_v2`
- `pinch_box_v4`

```text
physical_dof_norm = [0.0, 0.0, 0.1, 0.1, 0.9, 1.0]
raw = [1000, 1000, 900, 900, 100, 0]
```

```text
pinch_box_v2 physical_dof_norm = [0.0, 0.0, 0.12, 0.15, 0.4, 1.0]
pinch_box_v2 raw = [1000, 1000, 880, 850, 600, 0]
```

Failure note:

- `pinch_box_v2` closed before the thumb had fully rotated into the collision-free pose.
- Result: thumb collided with or pushed the box before stable pinch.

Updated two-stage hand sequence:

```text
pinch_box_thumb_rotate_v1 physical_dof_norm = [0.0, 0.0, 0.0, 0.0, 1.0, 0.0]
pinch_box_thumb_rotate_v1 raw = [1000, 1000, 1000, 1000, 0, 1000]

pinch_box_v3 physical_dof_norm = [0.0, 0.0, 0.12, 0.15, 1.0, 1.0]
pinch_box_v3 raw = [1000, 1000, 880, 850, 0, 0]
```

Correction after replay:

- The previous DOF4/DOF5 labels were reversed.
- `pinch_box_thumb_rotate_v1` closed the thumb first, then `pinch_box_v3` rotated it, causing the wrong contact sequence.

Corrected two-stage hand sequence:

```text
pinch_box_thumb_rotate_v2 physical_dof_norm = [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
pinch_box_thumb_rotate_v2 raw = [1000, 1000, 1000, 1000, 1000, 0]

pinch_box_v4 physical_dof_norm = [0.0, 0.0, 0.12, 0.15, 0.4, 1.0]
pinch_box_v4 raw = [1000, 1000, 880, 850, 600, 0]
```

Evidence images:

- `data/real_debug/pinch_box_v1/pinch_side.jpg`
- `data/real_debug/pinch_box_v1/pinch_front.jpg`

Pending:

- Replay sequence:
  1. `open`
  2. move to `pinch_grasp_box_v2`
  3. send `pinch_box_thumb_rotate_v2`
  4. wait 0.8-1.0 s
  5. send `pinch_box_v4`
  6. wait 1 s
  7. move to `pinch_lift_box_v1`
  8. hold 2 s
  9. label success/failure

Physical DOF mapping under active CLI:

```text
0 = pinky_bend
1 = ring_bend
2 = middle_bend
3 = index_bend
4 = thumb_bend
5 = thumb_rotate
```

# 中文版本

## 试验记录

日期：2026-04-28

对象：小纸盒。

抓取方式：top-down pinch。

## 观察

- 通过人工调整 wrist 和 hand posture，纸盒被稳定抬起。
- 该 trial 是当前 RH56 真机上已经确认过的有效 primitive 证据。
- 它可以作为后续 MuJoCo 模型、hand-code 和 palm-frame grasp candidate 的校准 baseline。

## 推荐 replay 流程

1. `open`。
2. 移动到 `pinch_grasp_box_v2`。
3. 发送 `pinch_box_thumb_rotate_v2`。
4. 等待 0.8-1.0 秒。
5. 发送 `pinch_box_v4`。
6. 等待 1 秒。
7. 移动到 `pinch_lift_box_v1`。
8. 保持 2 秒。
9. 标注 success/failure。

## 当前 CLI 下的物理 DOF 映射

```text
0 = pinky_bend
1 = ring_bend
2 = middle_bend
3 = index_bend
4 = thumb_bend
5 = thumb_rotate
```
