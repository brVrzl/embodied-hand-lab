# Quest/JAKA MuJoCo simulation

This is the recommended first operational workflow. It receives Quest network
input and drives only MuJoCo. It does not import, initialize, log in to, or
command the JAKA or RH56 physical SDKs.

## Prerequisites

```bash
.venv/bin/python -m pip install -e ".[dev]"
./scripts/run_quest_jaka_sim_demo.sh --help
```

Use a Quest Hand Tracking Streamer build with the CTRL v1 left-controller
sidecar. The ordinary upstream hand-only streamer cannot engage the current
controller clutch. Configure the Quest sender to the host IPv4 address and UDP
port 9000 (or the value explicitly selected on both ends).

## Live simulation

From a graphical host:

```bash
./scripts/run_quest_jaka_sim_demo.sh \
  --config configs/sim/quest_hts_jaka_mini2_live_demo.yaml \
  --bind 0.0.0.0 \
  --port 9000 \
  --project-ip <HOST_IPV4> \
  --duration-sec 600 \
  --telemetry-hz 2 \
  --viewer
```

Left index is release-before-press arm clutch/reference capture and hold-to-run.
Left grip independently controls the simulated RH56 hand. Release the trigger
before the first engagement. Keep the first capture still; the arm must not
jump. Candidate rejection displays/records `HOLD_REJECTED` while preserving
the last safe target.

The wrapper can discover a local graphical session when invoked over SSH.
Prefer explicit `--display` and `--xauthority` if discovery is ambiguous.

## Offline and replay modes

Inspect exact options:

```bash
.venv/bin/python tools/quest_jaka_mujoco_sim.py --help
.venv/bin/python tools/quest_jaka_mujoco_sim.py replay --help
.venv/bin/python tools/quest_jaka_mujoco_sim.py replay-6dof --help
.venv/bin/python tools/quest_jaka_mujoco_sim.py live-6dof --help
```

Use committed regression fixtures or an explicitly selected local recording.
Recordings are not automatically committed. Add `--ik-debug` only when detailed
joint, TCP, singularity, continuation, and rejection diagnostics are needed.

## Acceptance checks

- No physical SDK import or connection occurs.
- First engagement is stationary and jump-free.
- Translation and orientation follow the documented frame mapping.
- Release holds/stops the session as documented.
- Recoverable infeasibility holds safely and retreat can recover.
- Input loss or timeout stops rather than replaying stale motion.

See [troubleshooting](troubleshooting.md) for packet, viewer, and rejection
diagnostics.

---

# 中文版：Quest/JAKA MuJoCo 仿真

这是推荐的第一操作流程。它接收 Quest 网络输入，只驱动 MuJoCo，不导入、初始化、登录
或命令 JAKA/RH56 真机 SDK。

## 前置条件

```bash
.venv/bin/python -m pip install -e ".[dev]"
./scripts/run_quest_jaka_sim_demo.sh --help
```

需要使用带 CTRL v1 左控制器 sidecar 的 Quest Hand Tracking Streamer。普通的上游
hand-only streamer 不能触发当前 clutch。将 Quest 发送地址设置为主机 IPv4，UDP 端口
通常为 9000；两端必须一致。

## 实时仿真

在图形桌面运行：

```bash
./scripts/run_quest_jaka_sim_demo.sh \
  --config configs/sim/quest_hts_jaka_mini2_live_demo.yaml \
  --bind 0.0.0.0 \
  --port 9000 \
  --project-ip <HOST_IPV4> \
  --duration-sec 600 \
  --telemetry-hz 2 \
  --viewer
```

左手食指是机械臂 release-before-press clutch/reference capture，并采用 hold-to-run。
左 grip 独立控制仿真 RH56。首次 engage 前必须先完全释放 trigger，捕获参考时保持静止，
机械臂不得跳变。候选被拒绝时记录 `HOLD_REJECTED` 并保持最后安全目标。

通过 SSH 启动时 wrapper 可以发现本地图形会话；如果识别不明确，应显式传入 `--display`
和 `--xauthority`。

## 离线和回放

```bash
.venv/bin/python tools/quest_jaka_mujoco_sim.py --help
.venv/bin/python tools/quest_jaka_mujoco_sim.py replay --help
.venv/bin/python tools/quest_jaka_mujoco_sim.py replay-6dof --help
.venv/bin/python tools/quest_jaka_mujoco_sim.py live-6dof --help
```

使用已提交的回归 fixture 或明确选择的本地记录。采集不会自动进入 Git。只有需要详细的
关节、TCP、奇异性、continuation 和拒绝诊断时才使用 `--ik-debug`。

## 验收检查

- 未发生真机 SDK 导入或连接；
- 第一次 engage 静止且无跳变；
- 平移/旋转符合坐标映射；
- release 按文档保持或停止；
- 可恢复不可行性安全保持且允许撤退恢复；
- 输入丢失或超时会停止，不会重放旧运动。

数据包、viewer 和拒绝问题见[故障排查](troubleshooting.md)。
