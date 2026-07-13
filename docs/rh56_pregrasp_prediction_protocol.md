# RH56 抓取前预测实验协议

更新日期：2026-07-08

## 目标

在 `JAKA mini2 + Inspire RH56` 上先实现可解释的抓取前预测闭环：由物体几何估计候选手型和接近位姿，用 RH56 有限手型库筛选，再用力/电流/接触状态做低速修正。

## 第一版接口

- `src/pregrasp/geometry.py`：从点云提取中心、尺寸、主轴和粗 shape hint。
- `src/pregrasp/primitives.py`：RH56 normalized canonical hand command codebook，顺序为 `[index, middle, ring, pinky, thumb_close, thumb_lateral]`。
- `src/pregrasp/predictor.py`：确定性 geometry-aware scorer，输出 top-k `PregraspCandidate`。
- `src/pregrasp/tactile.py`：根据 RH56 `inspire6.contact_binary/forces` 生成手指闭合和腕部微调建议。
- `tools/predict_rh56_pregrasp.py`：离线 JSON/.npy 入口，用于标注数据和调参。

## 推荐流程

0. 先跑 MuJoCo 碰撞 readiness audit，确认当前 collision mode 和 RH56 primitive 不存在静态自碰或深穿透：

   ```bash
   python tools/audit_mujoco_rh56_collision_readiness.py \
     --collision-mode unifuc_pad_proxy \
     --fail-on-blocker
   ```

   数据生成只能使用 audit 通过的 primitive/候选；极端闭合手型如果出现 thumb/index 自碰、深穿透，或触发硬件拇指-食指互相阻挡风险，应作为无效候选过滤，不能标成成功抓取。

1. 用 RealSense/Orbbec 分割目标点云，保存为 `.npy` 或 JSON `points`。
2. 运行：

   ```bash
   python tools/predict_rh56_pregrasp.py \
     --point-cloud data/object_cloud.npy \
     --primitive-config configs/pregrasp/rh56_pregrasp.yaml \
     --task-mode pick \
     --top-k 3
   ```

3. 将候选 `target_position_xyz` 送入掌心目标 IK，把 `hand_command` 作为 RH56 normalized command。
4. 实机执行必须低速接近，先停在预接触位，再分段闭合。
5. 每次接触后调用 `estimate_tactile_correction`，只允许毫米级腕部修正和小幅手指增量。

## MuJoCo 数据集生成

第一版数据集只围绕审计通过的 RH56 primitive 采样，不做全手型空间随机搜索：

```bash
python tools/generate_rh56_pregrasp_dataset.py \
  --objects all \
  --primitive-config configs/pregrasp/rh56_pregrasp.yaml \
  --collision-mode unifuc_pad_proxy \
  --offsets-per-primitive 5 \
  --out-dir data/rh56_pregrasp_dataset_v0
```

输出：

- `manifest.json`：数据集元信息、物体统计、成功数。
- `samples.jsonl`：每条 candidate 的物体几何、primitive、硬件约束、MuJoCo 结果和标签。
- `objects/*/object_point_cloud.npy`：用于训练/复现实验的对象点云。

正式生成前必须跑 readiness audit；生成器会再次过滤硬件阻挡风险，但不会替代资产审计。

## 数据记录

每条 episode 至少记录：

- 目标点云或几何摘要。
- predictor 输出的候选列表、实际执行候选 id、失败重试 id。
- JAKA TCP/关节状态。
- RH56 raw angle/current/force/contact，以及 canonical normalized command。
- 成功、失败模式和人工备注。

## 评估指标

- 首次候选成功率。
- top-3 重试内成功率。
- 接触后滑移/掉落率。
- 触觉修正次数和幅度。
- 相比固定 `power_grasp` 或 `tripod` 的提升。

## 边界

当前实现不是学习模型，也不声称高分辨率触觉。它是为真实数据采集和后续学习型 predictor 提供稳定接口与 baseline。

## RH56 硬件约束

RH56DFX 的拇指和食指在较深弯曲、且拇指侧摆/对掌较大时，真实硬件可能出现互相阻挡。相关论文也指出：耦合连杆将手指运动限制在弧线上，拇指工作空间与其他手指交集有限；闭合会引入 tilt 和指尖位移，naive close-to-width 会系统性错位。

当前代码把这点作为保守约束处理：

- `evaluate_rh56_hardware_constraints` 会对 `[index, middle, ring, pinky, thumb_close, thumb_lateral]` 命令估计 `thumb_index_blocking_risk`。
- `tools/audit_mujoco_rh56_collision_readiness.py` 会同时检查 MuJoCo 自碰/穿透和硬件阻挡风险。
- 后续数据集生成必须先过滤该约束，再做 MuJoCo grasp success 标签。
