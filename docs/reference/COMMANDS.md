# Command reference

## English

The main offline CLI is:

```bash
.venv/bin/embodied-lab --help
.venv/bin/embodied-lab doctor
.venv/bin/embodied-lab sim --help
.venv/bin/embodied-lab dataset --help
```

Current wrappers:

| Function | Entry point |
| --- | --- |
| Simulation | `scripts/run_quest_jaka_sim_demo.sh` |
| Quest CTRL input gate | `tools/quest_controller_transport_gate.py` |
| RH56 offline/explicit test | `scripts/run_quest_rh56_hand_test.sh` |
| RGB-D checks | `tools/check_realsense_stream.py`, `tools/process_rgbd_tabletop.py` |
| ACT/LeRobot training | `scripts/train_physical_bottle_lerobot.sh` |
| π0.5 training | `training/pi05/scripts/train_weekend.sh` |
| Training status | `training/pi05/scripts/status.sh` |

Read each entry's `--help` before using it. Help and simulation do not grant
hardware authority.

## 中文

主离线 CLI 为：

```bash
.venv/bin/embodied-lab --help
.venv/bin/embodied-lab doctor
.venv/bin/embodied-lab sim --help
.venv/bin/embodied-lab dataset --help
```

当前 wrapper：

| 功能 | 入口 |
| --- | --- |
| 仿真 | `scripts/run_quest_jaka_sim_demo.sh` |
| Quest CTRL input gate | `tools/quest_controller_transport_gate.py` |
| RH56 离线/显式测试 | `scripts/run_quest_rh56_hand_test.sh` |
| RGB-D 检查 | `tools/check_realsense_stream.py`、`tools/process_rgbd_tabletop.py` |
| ACT/LeRobot 训练 | `scripts/train_physical_bottle_lerobot.sh` |
| π0.5 训练 | `training/pi05/scripts/train_weekend.sh` |
| Training status | `training/pi05/scripts/status.sh` |

使用前先阅读对应入口的 `--help`。help 和仿真不会授予硬件操作权限。
