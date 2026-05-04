# MuJoCo Simulation Mainline / MuJoCo 仿真主线

Date / 日期: 2026-05-04

## English

MuJoCo is now the main simulation layer for JAKA mini2 + Inspire RH56 grasp evaluation in this repository.

ManiSkill remains useful for episode schema, action representation, and data-export pipeline checks, but it should not be used as the main evidence for RH56 grasp contact quality until the JAKA+RH56 hand geometry and contact surfaces are visibly correct in the ManiSkill viewer.

### Why MuJoCo

- The current MuJoCo asset contains the JAKA mini2 + RH56 hand model.
- The MuJoCo tooling already has RH56 fingertip/pad contact proxies and visual calibration markers.
- Contact and lift outcomes can be inspected directly with MuJoCo viewer or recorded to MP4.
- The benchmark can export per-candidate XML files, contact summaries, object point clouds, and success metrics.

### Immediate Visual Gates

First confirm the hand pose and contact proxies:

```bash
DISPLAY=:1 ./scripts/view_mujoco_rh56_pose_contact.sh --mode poses --show-contacts
```

Check that the cyan rectangular active collision proxies sit near the real RH56 finger-pad/phalanx contact surfaces, especially in `real_pinch_v4`. If the active proxies are visibly on the joint root instead of the pad surface, grasp metrics are not meaningful yet.

Then inspect a simple hand-object interaction:

```bash
DISPLAY=:1 ./scripts/view_mujoco_jaka_rh56_grasp_debug.sh
```

The default scene is `cube_in_hand`: the cube is placed between the thumb/index/middle region and the hand cycles from open to close.

Alternative debug scenes:

```bash
DISPLAY=:1 ./scripts/view_mujoco_jaka_rh56_grasp_debug.sh --scenario hand_close --viewer
DISPLAY=:1 ./scripts/view_mujoco_jaka_rh56_grasp_debug.sh --scenario table_cube --viewer
```

For SSH sessions where you do not want to open a viewer window, record a short video:

```bash
./scripts/view_mujoco_jaka_rh56_grasp_debug.sh \
  --scenario cube_in_hand \
  --duration 3.0 \
  --record-mp4 data/replays/mujoco_jaka_rh56_debug/cube_in_hand.mp4
```

On the current workstation, the script automatically falls back to `DISPLAY=:1` when `DISPLAY` is unset and `/tmp/.X11-unix/X1` exists. On a pure headless server, configure `MUJOCO_GL=egl` or `MUJOCO_GL=osmesa` before using offscreen rendering.

### Evaluation Gates

Run the current analytical candidate benchmark:

```bash
./scripts/run_mujoco_grasp_benchmark.sh
```

Inspect best candidate rollouts after the benchmark has produced candidate summaries:

```bash
DISPLAY=:1 ./scripts/view_mujoco_rh56_pose_contact.sh \
  --mode grasp \
  --object foam_cube \
  --rank 0 \
  --show-contacts
```

Current interpretation:

- A successful MuJoCo lift is a simulation contact-model result, not a real-hardware success claim.
- A failed MuJoCo lift is still useful if the contact visualization explains whether the failure is caused by pad proxy placement, palm pose, hand-code, or lift path.
- Do not tune RH56 pseudo-tactile thresholds from MuJoCo contact forces. Tune pseudo-tactile thresholds from PC-direct RH56 feedback on real hardware.

### Mainline Task Definition

The simulation task that currently fits the research direction is:

```text
object point cloud / object width
-> object-relative palm pose candidates
-> RH56 hand-code sequence
-> MuJoCo contact + lift verification
-> export top candidates as real-robot presets
```

This aligns better with the planned paper direction than a generic PickCube success rate because it keeps the paper's variables explicit: palm frame, hand-code, object geometry, and low-cost contact feedback.

## 中文

MuJoCo 现在作为本仓库中 `JAKA mini2 + Inspire RH56` 抓取评估的主仿真层。

ManiSkill 仍然可以用于 episode schema、action representation 和 data-export pipeline 检查，但在 ManiSkill viewer 中确认 JAKA+RH56 手部几何与接触面正确之前，不应把 ManiSkill 结果作为 RH56 抓取接触质量的主要证据。

### 为什么转向 MuJoCo

- 当前 MuJoCo asset 已经包含 JAKA mini2 + RH56 手部模型。
- MuJoCo 工具链已经有 RH56 指腹/指尖 contact proxy 和可视化校准 marker。
- 接触和 lift 过程可以直接用 MuJoCo viewer 查看，也可以录制成 MP4。
- benchmark 可以导出每个候选抓取的 XML、contact summary、物体点云和成功指标。

### 第一批可视化门槛

先确认手部姿态和接触 proxy：

```bash
DISPLAY=:1 ./scripts/view_mujoco_rh56_pose_contact.sh --mode poses --show-contacts
```

重点看 `real_pinch_v4`：cyan 矩形 active collision proxies 应该落在真实 RH56 指腹/指节接触面附近。如果它们明显落在关节根部而不是接触面，当前 grasp metric 就还没有意义。

然后看一个简单的手-物体交互过程：

```bash
DISPLAY=:1 ./scripts/view_mujoco_jaka_rh56_grasp_debug.sh
```

默认场景是 `cube_in_hand`：方块放在拇指、食指、中指之间，手会从张开循环到闭合。

其他调试场景：

```bash
DISPLAY=:1 ./scripts/view_mujoco_jaka_rh56_grasp_debug.sh --scenario hand_close --viewer
DISPLAY=:1 ./scripts/view_mujoco_jaka_rh56_grasp_debug.sh --scenario table_cube --viewer
```

如果远程 SSH 不想打开 viewer 窗口，可以录一段短视频：

```bash
./scripts/view_mujoco_jaka_rh56_grasp_debug.sh \
  --scenario cube_in_hand \
  --duration 3.0 \
  --record-mp4 data/replays/mujoco_jaka_rh56_debug/cube_in_hand.mp4
```

当前工作站如果没有显式设置 `DISPLAY`，脚本会在存在 `/tmp/.X11-unix/X1` 时自动使用 `DISPLAY=:1`。如果换到纯 headless 服务器，需要先配置 `MUJOCO_GL=egl` 或 `MUJOCO_GL=osmesa`。

### 评估门槛

运行当前解析式候选抓取 benchmark：

```bash
./scripts/run_mujoco_grasp_benchmark.sh
```

benchmark 产生候选 summary 后，可以查看候选 rollout：

```bash
DISPLAY=:1 ./scripts/view_mujoco_rh56_pose_contact.sh \
  --mode grasp \
  --object foam_cube \
  --rank 0 \
  --show-contacts
```

当前解释方式：

- MuJoCo lift 成功代表仿真接触模型下的成功，不等同于真实硬件成功。
- MuJoCo lift 失败仍然有价值，只要 contact visualization 能说明失败来自 pad proxy、palm pose、hand-code 还是 lift path。
- 不要从 MuJoCo 接触力调 RH56 pseudo-tactile 阈值。pseudo-tactile 阈值应来自真实 RH56 PC-direct feedback。

### 当前主线任务定义

当前更适合研究主线的仿真任务是：

```text
物体点云 / 物体宽度
-> object-relative palm pose candidates
-> RH56 hand-code sequence
-> MuJoCo contact + lift verification
-> 导出 top candidates 作为真机 preset
```

这比泛泛地报告 PickCube 成功率更贴合当前论文方向，因为它明确保留了 palm frame、hand-code、object geometry 和低成本接触反馈这些变量。
