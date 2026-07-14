# Simulation Assets

本目录保存当前项目恢复后可用的仿真资产。这里的资产有不同角色和验证等级，不能混为同一个“最终模型”。

## 默认 Mounted Runtime Model

`jaka_rh56_visual_coacd.xml` 是当前默认 JAKA mini2 + RH56 MuJoCo runtime asset。它是从
`jaka_rh56.xml` 可重复派生的固定资产，RH56 body 中只包含：

- 13 个 collision-disabled vendor visual STL geoms；
- 148 个 active `visual_coacd` convex collision geoms；
- 7 个经过审阅的相邻内部 body exclusions。

它不包含 legacy analytic/proxy 或 Correll RH56 collision geoms。当前 148 个 hull 是已选定的
默认 collision baseline，不再自动重建、调参或修改。完整文件哈希和资产策略记录在
`jaka_rh56_visual_coacd.manifest.json`。

重新派生或验证 committed asset，不会运行 CoACD 或修改 STL：

```bash
.venv/bin/python tools/build_rh56_visual_coacd_runtime_asset.py
.venv/bin/python tools/build_rh56_visual_coacd_runtime_asset.py --check
```

## 派生源与比较模式

`jaka_rh56.xml` 保留为 mounted integration/derivation source，供以下显式比较和诊断模式使用：

- `tools/mujoco_rh56_grasp_benchmark.py`
- `tools/view_mujoco_rh56_pose_contact.py`
- Stage 1/Stage 2 collision diagnostics
- `correll_mesh`、`unifuc_pad_proxy` 和 legacy/proxy comparison modes

不要把 `jaka_rh56.xml` 直接作为普通 runtime 默认资产；它包含用于派生比较模式的历史几何。
`src/sim_maniskill/rh56_collision.py` 保留这些隔离比较模式，但其默认 patch 同样选择 `visual_coacd`。

后续仍需要审计：

- RH56 运动学和 coupling
- JAKA flange 到手基座的 mount transform
- collision proxy 位置
- fingertip/contact geometry
- actuator limit 和物理命令映射

`jaka_rh56.xml` 使用的本地 mesh 目录：

- `meshes/jaka_minicobo_meshes/`
- `meshes/rh56/`

## Correll RH56DFX Reference Assets

`correll_rh56dfx/` 包含从 Correll Robotics Lab 公开工作引入的 Inspire RH56DFX 资产：

- `inspire_grasp_scene.xml`：浮动手 FK/planning scene
- `inspire_force_scene.xml`：浮动手 force/torque sensor scene
- `inspire_scene.xml`：固定基座手 + object + fingertip sensors
- `inspire_right.xml`：固定基座 RH56 reference hand
- `assets/visual/` 和 `assets/collision/`：手部 mesh
- `LICENSE`：上游 MIT license

这些资产有两类用途：

- 通过 `pregrasp.correll_rh56dfx` 使用，用于参考 FK 规划和验证。
  - 通过 `src/sim_maniskill/rh56_collision.py` 注入到派生模型中，作为隔离的比较模式。

它们不能直接替换整个 `jaka_rh56.xml`，因为它们不包含 JAKA 机械臂、当前 mount transform，也不使用项目里的 `rh56_R_*` joint/body 命名。

## 验证

```bash
.venv/bin/python -m pytest tests/test_correll_rh56dfx_assets.py tests/test_mujoco_rh56_collision_modes.py
```

该测试覆盖：

- Correll XML 资产能被 MuJoCo 编译。
- Correll actuator、fingertip site、force/torque sensor 接口存在。
- `correll_mesh` 模式能在 mounted model 中编译并禁用旧 analytic proxy。
- committed `visual_coacd` runtime asset 与可重复派生结果及哈希清单一致。

## 资产约定

- 按来源和用途给资产分目录。
- 导入第三方资产时保留 license。
- 不要删除或破坏 `jaka_rh56.xml`，它仍是隔离比较模式的派生源。
- 文档中要明确资产是 reference model、mounted integration model，还是 temporary recovery artifact。
