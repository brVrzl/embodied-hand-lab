# Simulation Task Assessment

Date: 2026-05-04

Task under review: `PickCubeJakaRH56-v1`

## Verdict

`PickCubeJakaRH56-v1` is suitable only for a narrow offline pipeline purpose:

- validate the episode/data schema.
- validate palm-frame action fields.
- collect state-only privileged demonstrations.
- train a small state-only BC or retrieval baseline.
- exercise structured export and schema validation before hardware is connected.

It is not suitable for claims about real RH56 grasp physics, pseudo-tactile force thresholds, or robust dexterous manipulation.

After visual inspection on 2026-05-04, the ManiSkill JAKA+RH56 viewer path should not be treated as the main grasp-evaluation layer because the hand geometry/contact representation is not reliable enough for RH56 grasp inspection. The active contact-evaluation path is now MuJoCo. See `docs/mujoco_simulation_mainline.md`.

## Why It Is Useful Now

The task already has the right embodiment and table geometry for the current project:

- JAKA mini2 + RH56 combined model.
- desktop-size table and object workspace.
- 6 arm joints and a 6-DOF controlled RH56 hand interface.
- pregrasp start pose that makes short-horizon scripted palm motion practical.
- state-only mode that does not depend on local RGB-D renderer support.

This makes it a good gate for the engineering question:

```text
Can the project produce valid episodes with observation/action/reward/discount/is_first/is_last semantics and palm-frame hand actions?
```

## Main Limitation

The current privileged oracle may kinematically carry the cube after the scripted close phase. That is acceptable for pipeline validation, but it means success is not a physical grasp success.

Treat its success rate as:

```text
schema + action representation + task flow success
```

not:

```text
RH56 simulated grasp performance
```

## Recommended Next Gate

Run a small state-only palm-frame oracle smoke:

```bash
PYTHONPATH=src:tools .venv/bin/python tools/collect_jaka_rh56_pickcube_privileged_oracle.py \
  --episodes 5 \
  --max-steps 120 \
  --output-dir data/episodes/jaka_rh56_pickcube_palm_frame_smoke \
  --export-dir data/exports/structured/jaka_rh56_pickcube_palm_frame_smoke

PYTHONPATH=src .venv/bin/python tools/validate_episode_schema.py \
  --export-root data/exports/structured/jaka_rh56_pickcube_palm_frame_smoke
```

Pass condition:

- structured export exists.
- schema validation passes.
- all successful episodes have `failure_mode=none`.
- action records contain `delta_palm_pose`, `hand_code_id`, `hand_cmd`, and `close_strength`.

## Later Gates

After the smoke gate passes:

1. Train a state-only BC baseline on the structured export.
2. Run the non-kinematic contact-only `LiftCubeJakaRH56-v1` task and compare it against the privileged oracle.
3. Add object-width and approach-direction variation.
4. Add RGB-D only after renderer support is confirmed.
5. Move pseudo-tactile threshold tuning to real RH56 feedback, not simulated contact forces.

## ManiSkill Contact-Only Evaluation Task

`LiftCubeJakaRH56-v1` follows the standard benchmark pattern used by tasks such as ManiSkill `PickCube`, `StackCube`, and `PegInsertionSide`: randomized tabletop object state, bounded horizon, explicit success terms, and a policy-independent evaluator.

However, because the current ManiSkill viewer path does not provide a trustworthy RH56 hand/contact visualization, this task is now a schema/evaluator prototype rather than the main contact-grasp benchmark.

Success requires:

- cube height above `0.075 m`.
- simulated RH56 grasp contact.
- arm reasonably static.

Run:

```bash
./scripts/evaluate_jaka_rh56_lift_hold.sh \
  --episodes 5 \
  --max-steps 100 \
  --out data/reports/jaka_rh56_lift_hold_eval/summary.json
```

Open an interactive viewer for the scripted rollout:

```bash
./scripts/evaluate_jaka_rh56_lift_hold.sh \
  --episodes 1 \
  --max-steps 100 \
  --viewer \
  --fps 20
```

For SSH/headless sessions without a working display, export diagnostic frames and MP4 instead:

```bash
./scripts/evaluate_jaka_rh56_lift_hold.sh \
  --episodes 1 \
  --max-steps 100 \
  --save-frames data/reports/jaka_rh56_lift_hold_eval/frames \
  --save-video data/reports/jaka_rh56_lift_hold_eval/videos
```

Interpretation:

- High success means the scripted policy and current ManiSkill contact model can support a basic lift/hold evaluator prototype.
- Low success is still useful for debugging task flow, but it should not drive the RH56 grasp research plan.
- Do not tune real pseudo-tactile thresholds from simulated contact forces.

## MuJoCo Main Evaluation Path

Use MuJoCo for RH56 hand/contact inspection and candidate grasp evaluation:

```bash
DISPLAY=:1 ./scripts/view_mujoco_rh56_pose_contact.sh --mode poses --show-contacts
DISPLAY=:1 ./scripts/view_mujoco_jaka_rh56_grasp_debug.sh
./scripts/run_mujoco_grasp_benchmark.sh
```

The MuJoCo path is currently better aligned with the research question because it exposes the actual hand geometry, active contact proxies, per-candidate XML scenes, contact summaries, and lift results.

# 中文版本

评估对象：`PickCubeJakaRH56-v1`

## 结论

这个任务只适合继续作为离线数据链路验证，用途要收窄：

- 验证 episode/data schema。
- 验证 palm-frame action 字段。
- 采集 state-only privileged demonstration。
- 训练一个小的 state-only BC 或 retrieval baseline。
- 在真实设备接入前，先跑通 structured export 和 schema validation。

它不适合支撑真实 RH56 抓取物理、伪触觉力阈值、复杂灵巧操作鲁棒性等结论。

2026-05-04 人工查看 viewer 后，ManiSkill JAKA+RH56 路径不应再作为主抓取评估层，因为当前手部几何/接触可视化不足以支撑 RH56 抓取判断。当前接触评估主线转到 MuJoCo，见 `docs/mujoco_simulation_mainline.md`。

## 现在为什么有用

当前任务已经具备项目需要的基本形状：

- JAKA mini2 + RH56 组合模型。
- 桌面尺度工作台和物体工作区。
- 6 个机械臂关节和 6-DOF RH56 控制接口。
- pregrasp 起始位姿，适合短时域 palm-frame scripted motion。
- state-only 模式可用，不依赖本机 RGB-D renderer。

因此它适合回答工程问题：

```text
项目能否稳定产出带 observation/action/reward/discount/is_first/is_last 语义的 episode，并记录 palm-frame hand action？
```

## 主要限制

当前 privileged oracle 在 scripted close 之后可能会用 kinematic carry 移动物体。这对验证数据链路是可以接受的，但它不代表物理抓取成功。

它的成功率只能解释为：

```text
schema + action representation + task flow success
```

不能解释为：

```text
RH56 simulated grasp performance
```

## 推荐下一道门槛

先跑一个小的 state-only palm-frame oracle smoke：

```bash
PYTHONPATH=src:tools .venv/bin/python tools/collect_jaka_rh56_pickcube_privileged_oracle.py \
  --episodes 5 \
  --max-steps 120 \
  --output-dir data/episodes/jaka_rh56_pickcube_palm_frame_smoke \
  --export-dir data/exports/structured/jaka_rh56_pickcube_palm_frame_smoke

PYTHONPATH=src .venv/bin/python tools/validate_episode_schema.py \
  --export-root data/exports/structured/jaka_rh56_pickcube_palm_frame_smoke
```

通过条件：

- structured export 存在。
- schema validation 通过。
- 成功 episode 的 `failure_mode=none`。
- action 记录中包含 `delta_palm_pose`、`hand_code_id`、`hand_cmd`、`close_strength`。

## 后续门槛

smoke gate 通过后：

1. 在 structured export 上训练 state-only BC baseline。
2. 运行不使用 kinematic carry 的 `LiftCubeJakaRH56-v1` contact-only 任务，与 privileged oracle 对照。
3. 增加物体宽度和 approach direction 变化。
4. 只有确认 renderer 可用后再加入 RGB-D。
5. 伪触觉阈值调参放到真实 RH56 feedback 上，不从仿真接触力直接得出。

## ManiSkill Contact-Only 评估任务

`LiftCubeJakaRH56-v1` 参照 ManiSkill `PickCube`、`StackCube`、`PegInsertionSide` 这类主流任务的模式：桌面物体随机化、固定 horizon、明确 success terms、独立于 policy 的 evaluator。

但因为当前 ManiSkill viewer 路径无法提供可信的 RH56 手部/接触可视化，它现在只作为 schema/evaluator prototype，不作为主要 contact-grasp benchmark。

成功条件：

- 方块高度超过 `0.075 m`。
- RH56 仿真接触判定为 grasped。
- 机械臂基本静止。

运行：

```bash
./scripts/evaluate_jaka_rh56_lift_hold.sh \
  --episodes 5 \
  --max-steps 100 \
  --out data/reports/jaka_rh56_lift_hold_eval/summary.json
```

打开 viewer 查看 scripted rollout：

```bash
./scripts/evaluate_jaka_rh56_lift_hold.sh \
  --episodes 1 \
  --max-steps 100 \
  --viewer \
  --fps 20
```

远程 SSH 没有可用 display 时，导出离线诊断帧和 MP4：

```bash
./scripts/evaluate_jaka_rh56_lift_hold.sh \
  --episodes 1 \
  --max-steps 100 \
  --save-frames data/reports/jaka_rh56_lift_hold_eval/frames \
  --save-video data/reports/jaka_rh56_lift_hold_eval/videos
```

解释方式：

- 高成功率只能说明 scripted policy 和当前 ManiSkill contact model 可以支撑一个基础 lift/hold evaluator prototype。
- 低成功率仍可用于调试 task flow，但不应驱动 RH56 抓取研究路线。
- 不要从仿真接触力直接调真实 RH56 伪触觉阈值。

## MuJoCo 主评估路径

RH56 手部/接触检查和候选抓取评估改用 MuJoCo：

```bash
DISPLAY=:1 ./scripts/view_mujoco_rh56_pose_contact.sh --mode poses --show-contacts
DISPLAY=:1 ./scripts/view_mujoco_jaka_rh56_grasp_debug.sh
./scripts/run_mujoco_grasp_benchmark.sh
```

MuJoCo 路径当前更贴合研究问题，因为它能直接暴露手部几何、active contact proxy、每个候选抓取的 XML 场景、contact summary 和 lift result。
