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

The finalized single-arm policy is selected by default. It uses six joint speed
limits of `pi` rad/s, the 60 Hz target-update feasibility interval (16.667 ms),
the unchanged 8 ms ServoJ contract period, and the existing 500 Hz MuJoCo
control loop. `pi` rad/s is the official outer theoretical/legal boundary used
here as a simulation ceiling; it is not a project-calibrated normal teleoperation
speed and does not change production hardware parameters.

With the committed live configuration, the integrated entry creates six JAKA
and six RH56 actuators. Left index controls only the arm and left grip controls
only the simulated hand. No RH56 hardware backend is reachable. The explicit
arm-only build used by the JAKA-only regression removes all RH56 actuators and
commands and still contains exactly six JAKA actuators.

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
Left grip independently captures a relative simulated-RH56 reference and
controls the hand while held. Release both controls before their first
engagement. Keep the first arm capture still; the arm must not jump. Candidate
rejection displays/records `HOLD_REJECTED` while preserving the last safe
target.

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

The accepted timing diagnosis is: the viewer needed `handle.sync()` to display
live MuJoCo state, and 60 Hz target replacement must not be evaluated with the
8 ms acceleration interval. The finalized single-arm policy is
`root_cause_fix`; low-latency/raw comparison profiles are not normal operation
settings.

Known limits remain: MuJoCo dynamics are approximate, Mini2 dynamics were not
identified here, no physical ServoJ speed calibration was performed, and the
official theoretical boundary is not a validated daily working speed.

## Effective `root_cause_fix` policy

The current launcher and Python CLI select `root_cause_fix` by default. The
effective values come from
`configs/sim/quest_hts_jaka_mini2_live_demo.yaml` plus the named overlay:

| Contract | Effective value |
|---|---:|
| input interpolation delay | 20 ms |
| target generation / IK | 60 Hz / 60 Hz |
| MuJoCo control | 500 Hz |
| JAKA transport / ServoJ contract | 125 Hz / 8,000,000 ns |
| output-feasibility acceleration interval | 16,666,667 ns |
| output joint velocity boundary | `pi` rad/s on all six joints |
| output joint acceleration boundary | `4*pi` rad/s² |
| command jerk diagnostic boundary | `20*pi` rad/s³ |
| TCP linear / angular cap | 1.0 m/s / 5.0 rad/s |
| translation One Euro | min cutoff 1.2, beta 18.0, derivative cutoff 1.0 |
| rotation One Euro | min cutoff 1.5, beta 4.0, derivative cutoff 1.0 |
| continuation | at most 5 backtracks; minimum fraction 1/32 |

The acceleration and jerk values are project-selected feasibility/diagnostic
boundaries, not published Mini2 hardware limits. The overlay changes the
acceleration evaluation interval; it does not alter the 8 ms ServoJ contract or
write any controller speed setting.

## Acceptance checks

- No physical SDK import or connection occurs.
- The integrated model reports six JAKA and six RH56 actuators; the explicit
  arm-only regression model reports six JAKA actuators and no RH56 command path.
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

最终默认策略为 `root_cause_fix`：六轴仿真速度上限为 `pi` rad/s，60 Hz target 的输出
可行性加速度评估周期为 16.667 ms，真机 ServoJ contract 仍为 8 ms，MuJoCo 控制周期为
2 ms（500 Hz）。`pi` 是官方外层理论/合法性边界，仅作为仿真上限；它不是本项目真机标定
出的日常遥操作速度，也不会自动修改 production 真机参数。已提交的实时配置会创建
6 个 JAKA 与 6 个 RH56 actuator；左 index 只控制机械臂，左 grip 只控制仿真手，且没有
任何 RH56 真机 backend 可达。显式 arm-only builder 用于 JAKA-only 回归，会移除全部
RH56 actuator/command，并仍然只有 6 个 JAKA actuator。

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

左手食指是机械臂 release-before-press clutch/reference capture，并采用 hold-to-run；
左 grip 独立捕获仿真 RH56 的相对参考并在按住时控制手。两者首次 engage 前都必须先完全
释放；机械臂捕获参考时保持静止，不得跳变。候选被拒绝时记录 `HOLD_REJECTED` 并保持最后
安全目标。

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
- 集成模型报告 6 个 JAKA + 6 个 RH56 actuator；显式 arm-only 回归模型报告 6 个
  JAKA actuator 且没有 RH56 command path；
- 第一次 engage 静止且无跳变；
- 平移/旋转符合坐标映射；
- release 按文档保持或停止；
- 可恢复不可行性安全保持且允许撤退恢复；
- 输入丢失或超时会停止，不会重放旧运动。

数据包、viewer 和拒绝问题见[故障排查](troubleshooting.md)。

## `root_cause_fix` 有效策略

当前 launcher 与 Python CLI 默认选择 `root_cause_fix`。有效值来自
`configs/sim/quest_hts_jaka_mini2_live_demo.yaml` 和该 overlay：

| 契约 | 有效值 |
|---|---:|
| 输入插值延迟 | 20 ms |
| target generation / IK | 60 Hz / 60 Hz |
| MuJoCo control | 500 Hz |
| JAKA transport / ServoJ contract | 125 Hz / 8,000,000 ns |
| 输出可行性加速度评估周期 | 16,666,667 ns |
| 六轴输出速度边界 | 每轴 `pi` rad/s |
| 输出加速度边界 | `4*pi` rad/s² |
| command jerk 诊断边界 | `20*pi` rad/s³ |
| TCP 线速度 / 角速度上限 | 1.0 m/s / 5.0 rad/s |
| 平移 One Euro | min cutoff 1.2、beta 18.0、derivative cutoff 1.0 |
| 旋转 One Euro | min cutoff 1.5、beta 4.0、derivative cutoff 1.0 |
| continuation | 最多 5 次 backtrack，最小 fraction 1/32 |

加速度和 jerk 是项目选择的可行性/诊断边界，不是公开的 Mini2 真机上限。overlay 只改变
加速度评估周期，不改变 8 ms ServoJ contract，也不会写入控制器速度设置。
