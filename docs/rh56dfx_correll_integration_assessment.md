# Correll RH56DFX 资产整合评估

日期：2026-07-09

评估对象：

- Paper: `Characterization, Analytical Planning, and Hybrid Force Control for the Inspire RH56DFX Hand`
- Project page: https://correlllab.github.io/rh56dfx.html
- `correlllab/rh56_controller`, branch `h12`, commit `3db830161fcae85635126a5b0d0de8449a32b870`
- `correlllab/h1_mujoco`, commit `331b81cc3811e55fbb46d5b47969e8d8a03898f0`

## 结论

Correll 的 RH56DFX 资产不是当前 JAKA-mounted model 的直接替代品，但它补上了项目原先最关键的缺口：

- 浮动手 FK/planning scene
- fingertip sites
- force/torque sensors
- 更明确的 RH56 actuator ctrl range
- 更合理的 underactuated coupling 模型
- width-to-grasp planning 代码参考

当前项目原先的 `data/sim_assets/jaka_rh56.xml` 仍要作为 mounted-arm current anchor 保留，因为它包含 JAKA arm 和当前下游命名依赖。但 hand collision 已切换为 Correll RH56DFX collision mesh；旧 analytic proxy 保留为禁用状态和工具回退/对比模式。该 mounted model 仍缺少 sites/sensors，且 coupling/contact 仍需审计。

## 逐项对比

| 项目能力 | 原项目现状 | Correll 资产 | 评估 |
|---|---|---|---|
| JAKA+RH56 mounted model | 有：`data/sim_assets/jaka_rh56.xml` | 无 JAKA mini2 mounted model | 原项目仍需要保留，Correll 不能直接替换。 |
| RH56 floating hand FK model | 缺失 | 有：`inspire_grasp_scene.xml` | Correll 明显更好，已整合。 |
| fingertip sites | `jaka_rh56.xml` 中没有 site | `right_thumb_tip`, `right_index_tip`, etc. | Correll 补上关键缺口，已测试。 |
| fingertip force/torque sensors | 缺失 | `inspire_force_scene.xml`, `inspire_scene.xml` 有 10 个 sensor | Correll 补上关键缺口，已测试。 |
| hand visual/collision mesh separation | 原 mounted model 主要依赖本地 vendor STL + analytic proxy | Correll 分 `assets/visual` 和 `assets/collision` | 已将 Correll collision mesh 注入 mounted model，作为项目默认 hand collision。 |
| underactuated coupling | 原模型是较简单 mimic：distal=1.0*MCP，thumb PIP/DIP=0.6/0.8*bend | Correll 使用 offset + slope coupling：例如 index intermediate = -0.05 + 1.1169*proximal | Correll 对 planner 更有用；mounted model 是否迁移需另做几何验证。 |
| actuator ctrl ranges | 原 mounted model 与项目命名绑定 | Correll 明确 `[pinky, ring, middle, index, thumb_proximal, thumb_yaw]` ranges | 已通过 adapter 映射到项目 canonical order。 |
| width-to-grasp planning | 原项目主要是手写 primitives | Correll 有 FK sweep / width planning 参考 | 已整合最小 2-finger line-width planner。 |
| force closure / wrench analysis | 原项目缺少可用 sensor scene | Correll 有 `mujoco_bridge.py` 参考 | 目前只整合资产和基础传感器验证；完整 wrench analysis 暂不引入。 |
| UI/Tkinter/UR5/H1/ROS2 workflow | 原项目已有自己的 JAKA/RH56 工具 | Correll 有 UR5/H1/Tkinter/ROS2 多套 workflow | 暂不引入，依赖和机器人形态不匹配。 |

## 已整合内容

资产：

- `data/sim_assets/correll_rh56dfx/inspire_right.xml`
- `data/sim_assets/correll_rh56dfx/inspire_grasp_scene.xml`
- `data/sim_assets/correll_rh56dfx/inspire_force_scene.xml`
- `data/sim_assets/correll_rh56dfx/inspire_scene.xml`
- `data/sim_assets/correll_rh56dfx/assets/visual/`
- `data/sim_assets/correll_rh56dfx/assets/collision/`
- `data/sim_assets/correll_rh56dfx/LICENSE`

代码：

- `src/pregrasp/correll_rh56dfx.py`
  - XML path resolution
  - MuJoCo compile/interface validation
  - Correll actuator order 和项目 canonical order 互转
  - 基于 `inspire_grasp_scene.xml` 的 2-finger line width planner
- `src/pregrasp/predictor.py`
  - 在合适 object width 下加入 `correll_line_width` 候选
- `src/sim_maniskill/rh56_collision.py`
  - 将 Correll RH56DFX collision mesh 注入当前 `rh56_R_*` mounted hand bodies
  - 禁用原项目 analytic proxy collision
- `data/sim_assets/jaka_rh56.xml`
  - 已包含 Correll collision mesh assets/geoms
  - 旧 `*_collision` proxy 保留但 `contype=0`

测试：

- `tests/test_correll_rh56dfx_assets.py`

## 当前测试覆盖

测试已覆盖：

- 4 个 Correll XML 能被 MuJoCo 编译。
- expected actuators、fingertip sites、force/torque sensors 存在。
- Correll actuator order 和项目 canonical command 能 round-trip。
- line-width planner 可在典型 40 mm / 80 mm 宽度下生成毫米级误差的候选。
- pregrasp predictor 能把 `correll_line_width` candidate 集成到候选列表。
- `correll_mesh` collision mode 可编译，并确认旧 proxy 被禁用。

推荐命令：

```bash
.venv/bin/python -m pytest tests/test_correll_rh56dfx_assets.py tests/test_pregrasp_prediction.py
```

## 未整合内容

暂不整合：

- Correll Tkinter UI
- UR5/H1/H1-2 workflows
- ROS2 launch 和 bridge 代码
- force-control UI 和 Magpie force-control 依赖
- 大量 experiment CSV/video artifacts

原因：

- 与当前 JAKA mini2 栈不匹配。
- 依赖面大，容易污染当前重建目标。
- 当前最缺的是 hand FK/sensor/reference asset，而不是 UI 或 H1/UR5 部署逻辑。

## 下一步建议

1. 给 `jaka_rh56.xml` 增加只读审计脚本，输出 joint/body/site/sensor/actuator 差异。
2. 把 Correll fingertip site 概念迁移到 mounted model，但不要直接改 body 命名。
3. 用真实 RH56 照片或点位检查 Correll collision mesh 在 mounted body frame 下的贴合程度。
4. 用真实 RH56 照片或点位检查 Correll coupling 与当前手的物理一致性。
5. 将 `mujoco_bridge.py` 中的 contact/wrench analysis 只作为算法参考，按项目命名重写最小版本。
