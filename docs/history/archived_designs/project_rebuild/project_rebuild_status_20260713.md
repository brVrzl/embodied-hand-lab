# 项目重建状态总览

日期：2026-07-09

本项目当前处于 JAKA mini2 + Inspire RH56 操作栈重建阶段。早期仿真资产曾被误删，所以本文用于固定当前事实边界：哪些内容可用，哪些只是恢复锚点，哪些是外部参考，哪些仍是计划。

## 状态标记

- `validated`：有自动化测试覆盖，或近期真实硬件检查记录可复现。
- `current anchor`：当前代码依赖，但模型或流程仍需审计。
- `reference`：有用的外部资产/方法，不直接等同于当前机器人栈。
- `plan`：计划或建议流程，尚未验证。

## 核心内容

| 区域 | 状态 | 说明 | 主要入口 |
|---|---|---|---|
| RH56 hand schema/order mapping | validated | canonical order、protocol order、raw/normalized 转换有测试覆盖。 | `src/rh56_driver/hand_schema.py`, `tests/test_rh56_hand_schema.py` |
| RH56 ROS2 JSON bridge | validated by unit tests | JSON command/state 解析和基础桥接逻辑有测试；真实 ROS2 部署仍需现场检查。 | `src/rh56_driver/ros2_bridge.py`, `tests/test_rh56_ros2_bridge.py` |
| RH56 serial backend | validated by unit tests | backend wrapper 逻辑有测试；实际 USB-RS485 依赖硬件检查。 | `src/rh56_driver/serial_backend.py`, `scripts/check_rh56_connection.sh` |
| JAKA servo-jog safety logic | validated by unit tests | 命令解析、限幅和 watchdog 逻辑有测试；真实控制前仍需零运动检查。 | `src/jaka_driver_adapter/servo_jog.py`, `tests/test_jaka_servo_jog.py` |
| RH56 MuJoCo collision baseline | validated default asset | `jaka_rh56_visual_coacd.xml` 固定使用 13 个 disabled visuals + 148 个 active CoACD hulls + 7 个 reviewed exclusions；有清单、哈希和回归测试。 | `data/sim_assets/jaka_rh56_visual_coacd.xml`, `tests/test_rh56_visual_coacd_default_asset.py` |
| `data/sim_assets/jaka_rh56.xml` | derivation anchor | 保留 mounted kinematics 和历史 geometry，仅用于显式派生 Correll/proxy comparison modes。 | `data/sim_assets/jaka_rh56.xml` |
| Correll RH56DFX assets | reference + validated interface | 浮动手 FK 和指尖 force/torque scene 已引入，并有 MuJoCo 编译/interface 测试。 | `data/sim_assets/correll_rh56dfx/`, `src/pregrasp/correll_rh56dfx.py` |
| Pregrasp predictor | validated by unit tests | 几何到候选手型的确定性 pipeline 有测试；真实 grasp 成功率未验证。 | `src/pregrasp`, `tools/predict_rh56_pregrasp.py` |
| ManiSkill JAKA+RH56 tasks | current anchor | 仍基于当前 mounted model；可用于软件管线检查，不可作为真实抓取性能证据。 | `src/sim_maniskill` |
| Real teleop scripts | current anchor | 入口存在，使用前需逐脚本看 help 和 config；真实运动安全不靠 README 保证。 | `scripts/`, `src/teleop_tools` |
| Data recording | current anchor | 记录 schema 和工具存在；真实数据质量依赖相机/机器人/手状态同步检查。 | `src/data_recorder`, `real_robot_data_collection_protocol.md` |
| D435 RGB-D and point cloud | validated capture + unit-tested geometry | top D435 原子帧/点云已实机只读检查；桌面算法有合成测试。camera-to-base 外参和真实物体点云仍 pending。 | `src/vision_interface`, `docs/d435_depth_pointcloud_readiness.md` |

## 当前优先级

1. 普通仿真使用 `jaka_rh56_visual_coacd.xml`；保持 `jaka_rh56.xml` 可加载以支持隔离 comparison modes。
2. 用 Correll RH56DFX reference assets 补足当前项目缺失的浮动手 FK、fingertip sites、force/torque sensors。
3. 将 Correll planner 结果通过 adapter 映射到项目 canonical RH56 order，而不是直接替换 mounted model 命名体系。
4. 不再调整当前 148-hull collision geometry；整机 mount、coupling、actuator 和真实硬件映射仍按各自证据审计。
5. 任何 sim success 都必须标记为候选生成或软件验证，不写成真实机器人性能。

## 推荐验证命令

```bash
.venv/bin/python -m pytest \
  tests/test_rh56_hand_schema.py \
  tests/test_rh56_ros2_bridge.py \
  tests/test_rh56_serial_backend.py \
  tests/test_jaka_servo_jog.py \
  tests/test_correll_rh56dfx_assets.py \
  tests/test_mujoco_rh56_collision_modes.py \
  tests/test_pregrasp_prediction.py \
  tests/test_rh56_pregrasp_dataset_generator.py
```

## 后续文档规则

- 如果文档提到“可用”，必须说明是测试可用、仿真可用、还是真机可用。
- 如果文档提到 `jaka_rh56.xml`，必须说明它是 comparison-mode derivation anchor，不是普通 runtime 默认资产。
- 如果文档提到 Correll 资产，必须说明它是 reference hand model，不能直接替换 JAKA-mounted model。
