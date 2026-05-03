# MuJoCo RH56 Grasp Benchmark / MuJoCo RH56 抓取基准

Date / 日期: 2026-04-28

## Goal / 目标

中文：这个 benchmark 的目标是把项目推进到“先仿真、后真机迁移”的抓取流程。当前流程在 MuJoCo 中直接生成物体点云，从点云估计物体宽度，再基于 RH56 的宽度和对向接触先验生成候选抓取姿态，最后用 MuJoCo 的接触和 lift 结果验证候选。

English: This benchmark moves the project toward a simulation-first grasp pipeline. It generates object point clouds directly from MuJoCo primitives, estimates object width from the point cloud, generates RH56 grasp candidates using analytical width/contact priors, and validates candidates with MuJoCo contact and lift outcomes.

中文：这个方向参考了 RH56DFX analytical planning、AnyDexGrasp、D(R,O)/T(R,O) Grasp 等工作的思想，但当前实现刻意保持小而直接可运行，优先服务于本项目的 JAKA mini2 + RH56 环境调通。

English: The design is inspired by RH56DFX analytical planning, AnyDexGrasp, and D(R,O)/T(R,O) Grasp, but the implementation is intentionally small, deterministic, and directly runnable in this repository.

## Script / 脚本

中文：推荐使用下面的脚本运行完整四类物体 benchmark。

English: Use the following script to run the full four-object benchmark.

```bash
scripts/run_mujoco_grasp_benchmark.sh
```

等价命令 / Equivalent command:

```bash
.venv/bin/python tools/mujoco_rh56_grasp_benchmark.py --objects all --max-candidates 72 --duration 4.0
```

主要文件 / Main file:

- `tools/mujoco_rh56_grasp_benchmark.py`

输出文件 / Outputs:

- `data/mujoco_grasp_benchmark/benchmark_summary.json`
- `data/mujoco_grasp_benchmark/<object>/summary.json`
- `data/mujoco_grasp_benchmark/<object>/candidates.json`
- `data/mujoco_grasp_benchmark/<object>/object_point_cloud.npy`
- `data/mujoco_grasp_benchmark/<object>/*.xml`

## Environment Changes / 环境改动

中文：benchmark 不会覆盖基础机器人 MJCF，而是为每个候选抓取生成单独的 XML 场景。

English: The benchmark does not overwrite the base robot MJCF. It creates a separate XML scene for each grasp candidate.

生成场景中的改动 / Changes in generated scenes:

- 中文：禁用机器人原始 mesh collision，避免未校准 mesh 造成虚假碰撞。  
  English: Disable raw robot mesh collisions to avoid false contacts from uncalibrated meshes.
- 中文：在 RH56 distal links 上加入简单的球形 fingertip contact proxy。  
  English: Add simple spherical fingertip contact proxies to RH56 distal links.
- 中文：提高仿真中的 RH56 actuator stiffness，用于抓取验证。  
  English: Increase simulated RH56 actuator stiffness for grasp validation.
- 中文：每个场景加入一个物体和一个局部桌面。  
  English: Add one object and one local table per scene.
- 中文：用 hand-base position IK 计算 lift joint target，而不是依赖当前真机 lift preset。  
  English: Compute the lift joint target with hand-base position IK instead of relying on the current real-hardware lift preset.

中文：这些是仿真环境 bring-up 选择，不等价于真实硬件标定。

English: These are simulation bring-up choices, not true hardware calibration.

## Success Criterion / 成功标准

默认成功标准 / Default success criterion:

- 中文：物体抬升高度 >= `0.020 m`。  
  English: Object lift is >= `0.020 m`.
- 中文：拇指接触，并且至少一个对向手指接触。  
  English: Thumb contact plus at least one opposing finger contact.
- 中文：物体不再接触桌面。  
  English: The object is no longer touching the table.
- 中文：初始状态不能已经出现 hand-object 或 hand-table 穿模接触。  
  English: The initial state must not already contain hand-object or hand-table penetration contacts.

中文：`2 cm` lift 阈值是为了先调通环境，后续应提高到 `5 cm` 或 `8 cm`。

English: The `2 cm` lift threshold is intentionally low for environment bring-up. It should later be raised to `5 cm` or `8 cm`.

提高阈值示例 / Example with a stricter threshold:

```bash
.venv/bin/python tools/mujoco_rh56_grasp_benchmark.py --objects all --success-lift 0.050
```

## Contact Calibration / 接触校准

中文：2026-04-28 的人工观察发现，旧版非拇指 `pad_proxy` 放在 distal body 原点附近，视觉上更像第二指节根部，不是指腹/指尖。这会导致物体初始位置估计错误，并产生 hand-table-object 穿模的假成功。

English: Manual inspection on 2026-04-28 showed that the previous non-thumb `pad_proxy` spheres were placed near distal body origins, visually closer to the second knuckle/root than the usable fingertip pad. This caused bad object placement and false positives with hand-table-object penetration.

当前修正 / Current fix:

- 中文：active cyan spheres 已移动到更接近指腹的位置。  
  English: Active cyan spheres are moved closer to the fingertip/pad region.
- 中文：`--mode poses` 会额外显示黄/橙/红/紫候选校准点；这些 marker 只可视化，不参与碰撞。  
  English: `--mode poses` additionally shows yellow/orange/red/purple calibration markers. They are visual-only and do not participate in collision.
- 中文：benchmark 现在用 active `pad_proxy` 的世界坐标估计物体初始位置。  
  English: The benchmark now estimates object placement from active `pad_proxy` world positions.
- 中文：benchmark 会记录并过滤 `initial_penetration`。  
  English: The benchmark records and filters `initial_penetration`.

查看校准姿态 / View calibration poses:

```bash
scripts/view_mujoco_rh56_pose_contact.sh --mode poses
```

中文：先观察 `real_pinch_v4`，确认 cyan spheres 是否在真实指腹附近。`power_close` 只用于观察极限自碰，不应作为有效抓取姿态。

English: First inspect `real_pinch_v4` and check whether the cyan spheres sit near the real finger pads. `power_close` is only for extreme self-collision inspection and should not be treated as a valid grasp posture.

## Current Results / 当前结果

运行命令 / Run command:

```bash
scripts/run_mujoco_grasp_benchmark.sh
```

| Object / 物体 | Success Count / 成功候选数 | Best Lift / 最佳抬升 | Best Contacts / 最佳接触 | Best Physical Close Norm / 最佳物理闭合参数 |
| --- | ---: | ---: | --- | --- |
| `foam_cube` | 0 | `0.0107 m` | thumb + index + middle / 拇指 + 食指 + 中指 | `[0.1, 0.1, 0.55, 0.6, 0.68, 1.0]` |
| `paper_box` | 0 | `0.0043 m` | thumb + index / 拇指 + 食指 | `[0.08, 0.08, 0.45, 0.5, 0.64, 1.0]` |
| `light_cylinder` | 0 | `0.0059 m` | thumb + middle + ring/pinky / 拇指 + 中指 + 无名指/小指 | `[0.0, 0.0, 0.3, 0.34, 0.52, 1.0]` |
| `round_ball` | 0 | `-0.0029 m` | index only / 仅食指 | `[0.08, 0.08, 0.45, 0.5, 0.64, 1.0]` |

中文：校准 proxy 后，旧的成功结果被清零，这是合理现象：之前的成功主要来自错误接触点和穿模。下一步应先肉眼确认指腹 proxy，再重新设计 grasp family，而不是继续调旧候选。

English: After proxy calibration, the old successes disappear. This is expected: the previous successes were mainly caused by wrong contact points and penetration. The next step is to visually confirm pad proxies, then redesign grasp families instead of tuning the old candidates.

RH56 物理 DOF 顺序 / Physical RH56 DOF order:

```text
[pinky, ring, middle, index, thumb_bend, thumb_rotate]
```

中文：当前最优候选都采用“先旋转拇指，再闭合手指”的策略。

English: The current best candidates all use the strategy of rotating the thumb first and then closing the fingers.

```text
rotate_norm = [0, 0, 0, 0, 0, 1]
```

## Current Limitations / 当前限制

- 中文：RH56 collision model 仍然是 proxy model，不是精确标定后的 mesh/contact model。  
  English: The RH56 collision model is still a proxy model, not a calibrated mesh/contact model.
- 中文：物体点云来自已知 primitive 几何，不是渲染深度图或真实相机点云。  
  English: Object point clouds are generated from known primitive geometry, not from rendered depth or real camera data.
- 中文：候选生成器是解析式小搜索，不是学习式 D(R,O)/AnyDexGrasp 模型。  
  English: The candidate generator is a small analytical search, not a learned D(R,O)/AnyDexGrasp model.
- 中文：当前 grasp family 是 top-down pinch/envelope grasp；对圆柱、杯子、工具等还需要更多 grasp family。  
  English: The current grasp family is top-down pinch/envelope grasp; robust cylinders, cups, and tools need additional grasp families.
- 中文：`2 cm` lift 是环境调通指标，不是最终论文级成功指标。  
  English: Success at `2 cm` lift is a bring-up metric, not a final paper-level metric.

## Next Steps / 下一步

1. 中文：为每个物体的最佳候选生成截图或视频。  
   English: Generate screenshots or videos for the best candidate of each object.
2. 中文：加入第二类 grasp family，优先做 cylinder power grasp。  
   English: Add a second grasp family, starting with cylindrical power grasp.
3. 中文：lift path 稳定后，把 success threshold 从 `2 cm` 提高到 `5 cm`。  
   English: Raise the success threshold from `2 cm` to `5 cm` once the lift path is stable.
4. 中文：加入随机物体位姿扰动，每类物体跑 20 个 seeds 并报告成功率。  
   English: Add randomized object pose perturbations and report success over 20 seeds per object.
5. 中文：把最优仿真候选导出为真机可用 preset，包括 thumb rotation stage、close norm、object-relative pregrasp offset 和预期接触手指。  
   English: Export the best simulation candidates into real-hardware presets, including thumb rotation stage, close norm, object-relative pregrasp offset, and expected contact fingers.
