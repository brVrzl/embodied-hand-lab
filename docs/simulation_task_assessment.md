# Simulation Task Assessment

Date: 2026-05-04

Task under review: `PickCubeJakaRH56-v1`

## Verdict

`PickCubeJakaRH56-v1` is suitable for the next offline simulation step, but only for a narrow purpose:

- validate the episode/data schema.
- validate palm-frame action fields.
- collect state-only privileged demonstrations.
- train a small state-only BC or retrieval baseline.
- exercise structured export and schema validation before hardware is connected.

It is not yet suitable for claims about real RH56 grasp physics, pseudo-tactile force thresholds, or robust dexterous manipulation.

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
2. Add a non-kinematic contact-only baseline and compare it against the privileged oracle.
3. Add object-width and approach-direction variation.
4. Add RGB-D only after renderer support is confirmed.
5. Move pseudo-tactile threshold tuning to real RH56 feedback, not simulated contact forces.

# 中文版本

评估对象：`PickCubeJakaRH56-v1`

## 结论

这个任务适合继续推进仿真层，但用途要收窄：

- 验证 episode/data schema。
- 验证 palm-frame action 字段。
- 采集 state-only privileged demonstration。
- 训练一个小的 state-only BC 或 retrieval baseline。
- 在真实设备接入前，先跑通 structured export 和 schema validation。

它暂时不适合支撑真实 RH56 抓取物理、伪触觉力阈值、复杂灵巧操作鲁棒性等结论。

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
2. 增加不使用 kinematic carry 的 contact-only baseline，与 privileged oracle 对照。
3. 增加物体宽度和 approach direction 变化。
4. 只有确认 renderer 可用后再加入 RGB-D。
5. 伪触觉阈值调参放到真实 RH56 feedback 上，不从仿真接触力直接得出。
