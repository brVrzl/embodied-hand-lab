# Simulation

## English

Run the headless smoke without devices:

```bash
.venv/bin/embodied-lab sim smoke
./scripts/run_quest_jaka_sim_demo.sh --help
```

The maintained model is `assets/jaka_rh56_visual_coacd.xml`. Quest simulation
uses the same accepted-target boundary as the physical adapter, but it does
not imply physical calibration or sim-to-real equivalence. The interactive
viewer is `tools/debug_mujoco_jaka_rh56_viewer.py`; it is an offline viewer,
not a hardware controller.

## 中文

无设备运行 headless smoke：

```bash
.venv/bin/embodied-lab sim smoke
./scripts/run_quest_jaka_sim_demo.sh --help
```

当前维护的模型是 `assets/jaka_rh56_visual_coacd.xml`。Quest 仿真与物理适配器共用 accepted-target boundary，
但不代表真机标定或 sim-to-real 等价。交互 viewer 是 `tools/debug_mujoco_jaka_rh56_viewer.py`；它是离线
viewer，不是硬件 controller。
