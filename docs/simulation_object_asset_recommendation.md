# Simulation Object Asset Recommendation

Date: 2026-04-30

## Recommendation

Start with **ManiSkill YCB** for the next simulation data generation round.

Reason:

- YCB is the most common manipulation benchmark object set.
- ManiSkill already packages YCB for `PickSingleYCB` and `PickClutterYCB`.
- The ManiSkill YCB assets include `textured.obj`, `collision.ply`, bounding boxes, scales, and density metadata.
- It is small enough to debug our RH56 codebook replay pipeline before moving to thousands of objects.

Then expand to **Google Scanned Objects** for unseen-object generalization once YCB replay is stable.

## Object Sets Checked

### YCB / ManiSkill YCB

Use first.

YCB provides RGB-D/RGB images, segmentation masks, calibration data, and texture-mapped 3D mesh models. ManiSkill supports loading YCB through its actor builder and uses it in `PickSingleYCB-v1` / `PickClutterYCB-v1`.

Local download command:

```bash
.venv/bin/python -m mani_skill.utils.download_asset ycb
```

Optional clutter configs:

```bash
.venv/bin/python -m mani_skill.utils.download_asset pick_clutter_ycb_configs
```

Inspect after download:

```bash
.venv/bin/python tools/inspect_maniskill_ycb_assets.py
```

Current local status:

- Downloaded to `/home/w/.maniskill/data/assets/mani_skill2_ycb`.
- Manifest written to `data/external/maniskill_ycb_manifest.json`.
- Split written to `data/external/maniskill_ycb_grasp_splits.json`.
- Split counts: train 28, held-out 12, reserve 34.
- MuJoCo-ready assets written to `data/sim_assets/maniskill_ycb_mujoco`.
- MuJoCo conversion manifest: `data/external/maniskill_ycb_mujoco_assets.json`.
- Train + held-out conversion result: 40 prepared, 0 failed.

Important implementation detail: MuJoCo does not directly load the packaged `collision.ply` files, so the local pipeline exports scaled `collision.obj` meshes first:

```bash
.venv/bin/python tools/prepare_maniskill_ycb_mujoco_assets.py
```

Preview a converted object in the JAKA + RH56 MuJoCo scene:

```bash
scripts/view_mujoco_ycb_object.sh --object 002_master_chef_can --viewer
```

Headless spot checks passed for `002_master_chef_can`, `003_cracker_box`, `011_banana`, `025_mug`, and `062_dice`: each compiled, settled on the table, and had no object-robot collision in the preview scene.

Follow-up replay smoke:

- YCB mesh replay collector: `tools/collect_ycb_codebook_replay_dataset.py`
- Notes and results: `docs/ycb_codebook_replay_smoke_20260430.md`
- Important: use active code scanning for labels. Nearest-code projection can select an overly open hand state and fail even when another active code succeeds.

### Google Scanned Objects

Use second.

Google Scanned Objects contains 1030 scanned household objects, about 13 GB, under CC-BY 4.0. The assets include simulation-oriented metadata such as mass, inertia, collision information, and SDF format.

This is better for scale and visual diversity than YCB, but the native SDF/Gazebo representation needs conversion and MuJoCo collision validation.

### GraspNet Objects

Use as a comparison/reference set, not the first source.

GraspNet provides 88 objects and object 3D models, plus 6-DoF grasp labels. Object ids 0-32 and 71 are from YCB, so it overlaps with YCB. It is useful for seen/similar/novel object splits, but the licensing and asset size make it less convenient as the first MuJoCo training source.

### DexGraspNet Objects

Use later.

DexGraspNet has 5355 objects, 133 categories, and 1.32M ShadowHand grasps. It is directly relevant to dexterous grasping, but the grasps are ShadowHand-centric and the dataset is CC BY-NC 4.0. It is better as a later research-scale comparison than as the immediate RH56 MuJoCo source.

### EGAD / RoboVerse / Objaverse-Style Large Assets

Use after YCB + GSO.

RoboVerse reports ManiSkill EGAD with 4562 objects and ManiSkill YCB with 78 objects. These large geometry sets are useful for broad shape generalization, but they need collision and scale validation before they are useful for dexterous contact training.

## Initial Split

Use a YCB-first split:

- Train: about 24 objects across boxes, cans, bottles, cups/mugs, bowls, fruit, tools, and balls.
- Held-out: about 8 objects, intentionally holding out both object instances and shape categories.
- Exclude initially: `022_windex_bottle`, `028_skillet_lid`, `029_plate`, `059_chain`, matching ManiSkill's own hard/non-graspable exclusions for `PickSingleYCB`.

## Why Not Start With Thousands of Objects

The current bottleneck is not object count. It is:

- reliable MuJoCo collision conversion,
- RH56 codebook grasp replay,
- feature extraction and labeling,
- held-out evaluation protocol.

Starting with YCB lets us debug the full training loop with known assets. Once the loop works, expanding to GSO/EGAD is mechanical.

## Sources

- YCB object models: https://www.ycbbenchmarks.com/object-models/
- ManiSkill object loading docs: https://maniskill.readthedocs.io/en/latest/user_guide/tutorials/custom_tasks/loading_objects.html
- ManiSkill PickClutterYCB source docs: https://maniskill.readthedocs.io/en/latest/_modules/mani_skill/envs/tasks/tabletop/pick_clutter_ycb.html
- Google Scanned Objects: https://research.google/blog/scanned-objects-by-google-research-a-dataset-of-3d-scanned-common-household-items/
- GraspNet datasets: https://graspnet.net/datasets.html
- DexGraspNet: https://pku-epic.github.io/DexGraspNet/
- RoboVerse object overview: https://roboverse.wiki/dataset_benchmark/dataset/objects

# 中文版本

## 推荐

下一轮仿真数据生成建议从 **ManiSkill YCB** 开始。

原因：

- YCB 是机器人操作领域最常用的物体集之一。
- ManiSkill 已经为 `PickSingleYCB` 和 `PickClutterYCB` 打包了 YCB 资产。
- YCB 对象类别足够丰富，适合测试 foam/cylinder/cup/bottle 之外的日用品泛化。
- 使用 YCB 能让实验更容易和主流 manipulation benchmark 对齐。

## 为什么不一开始用几千个物体

当前瓶颈不是物体数量，而是：

- MuJoCo collision conversion 是否可靠。
- RH56 codebook grasp replay 是否稳定。
- feature extraction 和 labeling 是否一致。
- held-out evaluation protocol 是否清楚。

先从 YCB 开始，可以用已知资产调通完整训练和评估闭环。等流程稳定后，再扩展到 GSO、EGAD 或更大的物体库。

## 用法建议

第一阶段：

- 选 4-8 个 YCB 小物体。
- 做 grasp-lift-hold。
- 比较 fixed palm、object-relative palm、continuous 6D hand command、hand-code。

第二阶段：

- 加入 clutter 或功能性物体。
- 扩展到 functional grasp 和 place。

第三阶段：

- 再考虑 GSO、DexGraspNet 或更大规模合成资产。
