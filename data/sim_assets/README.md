# Simulation Assets

本目录保存当前项目恢复后可用的仿真资产。这里的资产有不同角色和验证等级，不能混为同一个“最终模型”。

## 当前 Mounted Model

`jaka_rh56.xml` 是当前 JAKA mini2 + RH56 的 MuJoCo mounted model，被以下路径引用：

- `src/jaka_driver_adapter/palm_target_ik.py`
- `src/sim_maniskill/agents/jaka_rh56.py`
- `tools/mujoco_rh56_grasp_benchmark.py`
- `tools/view_mujoco_rh56_pose_contact.py`
- 依赖 mounted palm frame 的 teleop/RViz 预览流程

这个文件需要保留，是因为当前代码仍依赖它。它应被视为恢复锚点和集成模型，而不是完整验证过的数字孪生。

当前默认 runtime 碰撞不是继续使用原来的 capsule/sphere/pad proxy，而是通过 `src/sim_maniskill/rh56_collision.py` 注入 Correll RH56DFX collision mesh。旧 proxy 仍保留为工具里的对比/回退模式。

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
- 通过 `src/sim_maniskill/rh56_collision.py` 注入到当前 mounted hand body 上，作为项目默认 hand collision mesh。

它们不能直接替换整个 `jaka_rh56.xml`，因为它们不包含 JAKA 机械臂、当前 mount transform，也不使用项目里的 `rh56_R_*` joint/body 命名。

## 验证

```bash
.venv/bin/python -m pytest tests/test_correll_rh56dfx_assets.py tests/test_mujoco_rh56_collision_modes.py
```

该测试覆盖：

- Correll XML 资产能被 MuJoCo 编译。
- Correll actuator、fingertip site、force/torque sensor 接口存在。
- `correll_mesh` 模式能在 mounted model 中编译并禁用旧 analytic proxy。

## 资产约定

- 按来源和用途给资产分目录。
- 导入第三方资产时保留 license。
- 不要在依赖工具完成迁移前替换 `jaka_rh56.xml`。
- 文档中要明确资产是 reference model、mounted integration model，还是 temporary recovery artifact。
