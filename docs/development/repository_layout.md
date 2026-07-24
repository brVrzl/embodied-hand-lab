# Repository layout

| Path | Role | Status |
|---|---|---|
| `src/motion_input` | device-neutral input model, HTS/CTRL providers, replay | current |
| `src/quest_jaka_sim` | clutch, mapping, filters, continuation/IK, simulation | current primary |
| `src/teleoperation` | accepted-target, output feasibility, JAKA adapter | current primary |
| `native/jaka_servo_worker` | physical transport/safety worker | current, gated |
| `src/rh56_driver` | RH56 schema and backends | current parallel path |
| `src/jaka_driver_adapter`, `src/robot_bringup` | bring-up/legacy adapters | current or compatibility, not shared Quest authority |
| `src/teleop_tools` | HEBI/iPhone experiments | active parallel/legacy |
| `src/vision_interface` | perception and RealSense calibration | active parallel |
| `configs` | versioned examples and policy | current; local device facts must be verified |
| `data/sim_assets`, `models` | simulation assets | current; see their READMEs |
| `tests` | default offline test suite | current |
| `docs/history` | evidence and superseded narrative | historical, immutable outcomes |
| `third_party` | vendor/reference snapshots | do not treat as project style or edit casually |

The repository retains only parallel paths that still have an identified use.
They do not override the primary Quest/JAKA contracts.

---

# 中文版：仓库布局

| 路径 | 作用 | 状态 |
|---|---|---|
| `src/motion_input` | 设备无关输入、HTS/CTRL provider、回放 | 当前 |
| `src/quest_jaka_sim` | clutch、映射、滤波、continuation/IK、仿真 | 当前主链 |
| `src/teleoperation` | accepted target、输出可行性、JAKA adapter | 当前主链 |
| `native/jaka_servo_worker` | 真机传输/安全 worker | 当前，需 gate |
| `src/rh56_driver` | RH56 schema 和 backend | 当前并行路径 |
| `src/jaka_driver_adapter`、`src/robot_bringup` | bring-up/兼容 adapter | 非共享 Quest 权威 |
| `src/teleop_tools` | HEBI/iPhone 实验 | 并行/兼容 |
| `src/vision_interface` | 感知和 RealSense 标定 | 并行 |
| `configs` | 版本化示例和策略 | 当前；设备事实需现场核实 |
| `data/sim_assets`、`models` | 仿真资产 | 当前 |
| `tests` | 默认离线测试 | 当前 |
| `docs/history` | 证据和已取代叙述 | 历史 |
| `third_party` | vendor/reference snapshot | 不作为项目风格随意修改 |

仓库只保留仍有明确用途的并行路径；它们不能覆盖 Quest/JAKA 主链契约。
