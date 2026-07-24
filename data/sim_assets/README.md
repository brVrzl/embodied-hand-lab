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

## 派生源

`jaka_rh56.xml` 是 mounted integration 源模型，
`src/rh56_collision_model.py` 只支持派生已审阅的 `visual_coacd` runtime。
旧 analytic proxy、Correll mounted comparison 和多模式碰撞研究已移除。
不要直接把源模型当作运行时安全资产。

后续仍需要审计：

- RH56 运动学和 coupling
- JAKA flange 到手基座的 mount transform
- CoACD collision hull 保守性
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

它们仅作为上游许可证和几何/传感器接口参考，不是当前 mounted
runtime 碰撞模式。它们不包含 JAKA、当前 mount transform，也不使用
项目的 `rh56_R_*` body/joint 命名。

## 验证

```bash
.venv/bin/python -m pytest -q tests/test_rh56_visual_coacd_default_asset.py
```

该测试覆盖 committed `visual_coacd` runtime asset 的可重复派生、质量/惯量保持、
审阅后 exclusion 和哈希清单。

## 资产约定

- 按来源和用途给资产分目录。
- 导入第三方资产时保留 license。
- 不要删除或破坏 `jaka_rh56.xml`，它仍是 runtime asset 的派生源。
- 文档中要明确资产是 reference model、mounted integration model，还是 temporary recovery artifact。
