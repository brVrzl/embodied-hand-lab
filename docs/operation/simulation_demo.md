# Quest/JAKA MuJoCo recording, replay, and 125 Hz simulation

## 中文摘要

这是推荐的首个操作流程：接收 Quest 网络输入并只驱动 MuJoCo，不导入、初始化、登录
或控制 JAKA/RH56 真机 SDK。先使用 `--help`、回放和离线报告验证配置，再进行仿真；
仿真结果不能表述为真机 PASS。

This is the recommended first operational workflow. It receives Quest network
input and drives only MuJoCo. It does not import, initialize, log in to, or
command the JAKA or RH56 physical SDKs.

## Prerequisites

```bash
.venv/bin/python -m pip install -e ".[dev]"
./scripts/run_quest_jaka_sim_demo.sh --help
```

On the current workstation, the Quest-facing IPv4 address and local graphical
display are persisted in `~/.bashrc` as `HOST_IPV4` and `DISPLAY`. Load and
check them in every new terminal before starting the viewer:

```bash
source ~/.bashrc
printf 'HOST_IPV4=%s\nDISPLAY=%s\n' "$HOST_IPV4" "$DISPLAY"
```

These are host-local operator settings, not repository configuration. Update
the shell profile if the workstation address or desktop session changes; do
not hard-code them into the simulation YAML.

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

The viewer scene reuses the provisional physical tabletop dimensions from
`digital_twin/configs/workspace.yaml`: 0.73 x 1.38 x 0.02 m with a 0.75 m
tabletop height. The generated simulation world is expressed directly in the
JAKA base frame (`+X` forward, `-Y` right, `+Z` up). The base remains upright at
identity; the existing P-frame table and mounting members are transformed into
that base frame, placing the table workspace on base `+Y` (the robot's left).
At J1=+90 degrees the RH56 palm/TCP forward projection points into that
workspace. This scene transform is visualization/plant-only and does not
rotate the shared Quest mapping or alter accepted joint targets.

Every live run is also a joint arm/RH56 recording. Choose a new `RUN_NAME` for
each run: capture files use exclusive creation and an existing path is never
overwritten.

From a graphical host, run the default shaped 500 Hz path:

```bash
source ~/.bashrc
RUN_NAME=live_shaped_001
./scripts/run_quest_jaka_sim_demo.sh \
  --project-ip "$HOST_IPV4" \
  --display "$DISPLAY" \
  --arm-output-mode shaped-500hz \
  --output "logs/quest_jaka_sim/${RUN_NAME}.hts.jsonl" \
  --events "logs/quest_jaka_sim/${RUN_NAME}.events.jsonl" \
  --report "logs/quest_jaka_sim/${RUN_NAME}.report.json"
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

## Selectable arm output

`live-6dof` and `replay-6dof` share the same two post-`AcceptedArmTarget`
adapters. The default remains the existing 500 Hz command shaper shown in the
live-recording command above; omitting `--arm-output-mode` has the same default.

The optional JAKA-equivalent mode uses the production native
latest-destination/PWL resampler and its bounded transition logic. It updates
only the six arm controls on 8 ms simulation deadlines while MuJoCo continues
four 2 ms physics steps per emitted command. It bypasses the 500 Hz arm command
shaper; RH56 retains its existing independent simulation path:

```bash
cmake -S native/jaka_servo_worker -B build/jaka_servo_worker
cmake --build build/jaka_servo_worker -j

source ~/.bashrc
RUN_NAME=live_125hz_001
./scripts/run_quest_jaka_sim_demo.sh \
  --project-ip "$HOST_IPV4" \
  --display "$DISPLAY" \
  --arm-output-mode jaka-equivalent-125hz \
  --output "logs/quest_jaka_sim/${RUN_NAME}.hts.jsonl" \
  --events "logs/quest_jaka_sim/${RUN_NAME}.events.jsonl" \
  --report "logs/quest_jaka_sim/${RUN_NAME}.report.json" \
  --arm-emitted-events \
    "logs/quest_jaka_sim/${RUN_NAME}.arm_emitted_125hz.jsonl"
```

Raw recordings contain both HTS and CTRL datagrams. Replay routes recorded
CTRL index/grip samples through the same controller router used by live input;
known older HTS-only recordings retain the explicitly labelled deterministic
CLI clutch schedule. A recording never selects or restricts its downstream arm
output mode.

### Replay one recording through either arm adapter

Set `RECORDING` once, then select the output adapter independently. These two
commands rebuild the same controller routing, arm/hand clutch edges, reference
captures, mapping/filter/continuation IK, feasibility checks, and RH56 relative
retargeting from the same raw input.

Shaped 500 Hz replay:

```bash
RECORDING=logs/quest_jaka_sim/live_shaped_001.hts.jsonl
REPLAY_NAME=replay_shaped_001
PYTHONPATH=src .venv/bin/python tools/quest_jaka_mujoco_sim.py \
  replay-6dof "$RECORDING" \
  --arm-output-mode shaped-500hz \
  --viewer \
  --realtime \
  --events "logs/quest_jaka_sim/${REPLAY_NAME}.events.jsonl" \
  --report "logs/quest_jaka_sim/${REPLAY_NAME}.report.json"
```

JAKA-equivalent 125 Hz replay of the same recording:

```bash
REPLAY_NAME=replay_125hz_001
PYTHONPATH=src .venv/bin/python tools/quest_jaka_mujoco_sim.py \
  replay-6dof "$RECORDING" \
  --arm-output-mode jaka-equivalent-125hz \
  --viewer \
  --realtime \
  --events "logs/quest_jaka_sim/${REPLAY_NAME}.events.jsonl" \
  --report "logs/quest_jaka_sim/${REPLAY_NAME}.report.json" \
  --arm-emitted-events \
    "logs/quest_jaka_sim/${REPLAY_NAME}.arm_emitted_125hz.jsonl"
```

Omit both `--viewer` and `--realtime` for a fast headless offline replay. Do
not add deterministic `--engage-at-sec` or clutch-cycle options to a current
HTS+CTRL recording: those options exist only for explicitly selected legacy
HTS-only fixtures.

### 125 Hz startup and clock contract

- Before arm clutch engagement, the adapter holds the current MuJoCo arm
  `qpos`; it does not advance an old or zero destination.
- A new arm clutch/reference generation resets the native resampler from the
  current MuJoCo `qpos`, clears the previous destination/sequence, and then
  accepts the new 60 Hz target. A still first capture is therefore continuous.
- The first 125 Hz deadline is aligned to the current simulation time. Startup
  never catches up ticks from an earlier wall-clock or simulation epoch.
- Each emitted arm command is applied to `data.ctrl` for four 2 ms MuJoCo
  physics steps. RH56 remains on its independent path but shares the same
  simulation clock.
- The mode uses the SDK-free production native latest-destination/PWL and
  bounded-transition logic. It does not load the JAKA SDK and does not pass
  through the shaped 500 Hz arm command trajectory shaper.

### Recording and log files

| File | Contents |
|---|---|
| `*.hts.jsonl` | Raw timestamped HTS and CTRL datagrams, including wrist/head/hand input, index/grip, and validity |
| `*.events.jsonl` | Shared 60 Hz arm and RH56 control events plus sampled MuJoCo state |
| `*.report.json` | Summary and `quest_jaka_joint_recording.v1` manifest: commit/config hash, duration, orders, rates, paths, and `simulation_only=true` |
| `*.arm_emitted_125hz.jsonl` | 125 Hz accepted-source/emitted sequences and times, q/dq/ddq/jerk, destination/PWL/transition state, actual q, and tracking error |

The 125 Hz emitted log exists only when that output mode and
`--arm-emitted-events` are selected. Generated captures and logs remain local
and are ignored by default; do not add operator recordings to Git casually.

If a live run ends through a safety exception before normal finalization, the
raw capture may be the only completed artifact. Preserve it and the terminal
error. Do not edit a raw JSONL file to manufacture missing events or a report.

### Viewer and stopping

The viewer retains the desired/actual TCP markers and the terminal prints the
selected arm output mode. Use `Ctrl-C` or close the viewer for normal cleanup.
Stale input, clutch release, joint/collision/singularity/velocity/acceleration/
jerk policy and hard fault behavior remain active in both modes; do not relax
them to complete a replay. See [troubleshooting](../TROUBLESHOOTING.md) for packet,
viewer, rejection, and stop diagnostics.

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
- Input loss or recovery timeout enters a no-motion disengaged hold rather than
  replaying stale motion; actual producer/IPC death remains a hard stop.

Current validation level: both arm output modes and joint arm/RH56 replay are
offline tested, and the Quest/MuJoCo path is simulation validated. A fresh
post-fix Quest live run is not implied by offline replay, and none of these
commands constitutes physical JAKA or RH56 validation.

See [troubleshooting](../TROUBLESHOOTING.md) for packet, viewer, and rejection
diagnostics.

---

# 中文版：Quest/JAKA MuJoCo 录制、回放与 125 Hz 仿真

这是推荐的第一操作流程。它接收 Quest 网络输入，只驱动 MuJoCo，不导入、初始化、登录
或命令 JAKA/RH56 真机 SDK。

## 前置条件

```bash
.venv/bin/python -m pip install -e ".[dev]"
./scripts/run_quest_jaka_sim_demo.sh --help
```

当前工作站已在 `~/.bashrc` 持久化 Quest 接收地址 `HOST_IPV4` 和本地图形会话
`DISPLAY`。每个新 terminal 启动 viewer 前先加载并检查：

```bash
source ~/.bashrc
printf 'HOST_IPV4=%s\nDISPLAY=%s\n' "$HOST_IPV4" "$DISPLAY"
```

这两个值是本机操作环境，不是项目配置。工作站地址或桌面会话变化时应更新 shell
profile，不要把它们硬编码进仿真 YAML。

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

viewer 场景复用 `digital_twin/configs/workspace.yaml` 中已有的 provisional 现场桌面尺寸：
`0.73 x 1.38 x 0.02 m`，桌面高 `0.75 m`。生成后的仿真 world 直接使用 JAKA base
坐标（`+X` 前、`-Y` 右、`+Z` 上）；base 保持竖直 identity，已有 P-frame 桌面和安装梁
被变换到 base frame，桌面工作区位于 base `+Y`（机器人左侧）。J1=+90 度时，RH56
palm/TCP 的 forward 水平投影指向该工作区。这个 scene transform 只属于可视化/plant，
不会再次旋转共享 Quest mapping，也不会改变 accepted joint target。

每次 live 运行都会同时生成 arm/RH56 联合录制。每次必须使用新的 `RUN_NAME`；capture
采用排他创建，不会覆盖已经存在的文件。

在图形桌面运行默认 shaped 500 Hz 链路：

```bash
source ~/.bashrc
RUN_NAME=live_shaped_001
./scripts/run_quest_jaka_sim_demo.sh \
  --project-ip "$HOST_IPV4" \
  --display "$DISPLAY" \
  --arm-output-mode shaped-500hz \
  --output "logs/quest_jaka_sim/${RUN_NAME}.hts.jsonl" \
  --events "logs/quest_jaka_sim/${RUN_NAME}.events.jsonl" \
  --report "logs/quest_jaka_sim/${RUN_NAME}.report.json"
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

## 可选机械臂输出

`live-6dof` 与 `replay-6dof` 共用同一组 `AcceptedArmTarget` 后置 adapter。默认仍是
前文 live-recording 命令中的 500 Hz command shaper；省略 `--arm-output-mode` 时也使用
该默认值。

可选的 JAKA-equivalent 模式复用 production native latest-destination/PWL resampler
及其有界 transition。它按仿真 deadline 每 8 ms 只更新 6 个机械臂 control；MuJoCo 在
每个 emitted command 间仍执行 4 个 2 ms physics step。该模式旁路 500 Hz arm command
shaper，RH56 继续使用原有独立仿真路径：

```bash
cmake -S native/jaka_servo_worker -B build/jaka_servo_worker
cmake --build build/jaka_servo_worker -j

source ~/.bashrc
RUN_NAME=live_125hz_001
./scripts/run_quest_jaka_sim_demo.sh \
  --project-ip "$HOST_IPV4" \
  --display "$DISPLAY" \
  --arm-output-mode jaka-equivalent-125hz \
  --output "logs/quest_jaka_sim/${RUN_NAME}.hts.jsonl" \
  --events "logs/quest_jaka_sim/${RUN_NAME}.events.jsonl" \
  --report "logs/quest_jaka_sim/${RUN_NAME}.report.json" \
  --arm-emitted-events \
    "logs/quest_jaka_sim/${RUN_NAME}.arm_emitted_125hz.jsonl"
```

raw 录制同时包含 HTS 与 CTRL datagram。回放会把录下的 index/grip 送入与 live 相同的
controller router；明确不含 CTRL 的旧 HTS-only 录制仍使用标记清楚的 CLI 确定性 clutch
时序。录制文件不会选择或限制下游 arm output mode。

### 同一录制通过两种 arm adapter 回放

只设置一次 `RECORDING`，然后独立选择输出 adapter。以下两个命令会从同一份 raw input
重建相同的 controller routing、arm/hand clutch edge、reference capture、mapping/filter/
continuation IK、feasibility 和 RH56 relative retargeting。

shaped 500 Hz 回放：

```bash
RECORDING=logs/quest_jaka_sim/live_shaped_001.hts.jsonl
REPLAY_NAME=replay_shaped_001
PYTHONPATH=src .venv/bin/python tools/quest_jaka_mujoco_sim.py \
  replay-6dof "$RECORDING" \
  --arm-output-mode shaped-500hz \
  --viewer \
  --realtime \
  --events "logs/quest_jaka_sim/${REPLAY_NAME}.events.jsonl" \
  --report "logs/quest_jaka_sim/${REPLAY_NAME}.report.json"
```

同一录制的 JAKA-equivalent 125 Hz 回放：

```bash
REPLAY_NAME=replay_125hz_001
PYTHONPATH=src .venv/bin/python tools/quest_jaka_mujoco_sim.py \
  replay-6dof "$RECORDING" \
  --arm-output-mode jaka-equivalent-125hz \
  --viewer \
  --realtime \
  --events "logs/quest_jaka_sim/${REPLAY_NAME}.events.jsonl" \
  --report "logs/quest_jaka_sim/${REPLAY_NAME}.report.json" \
  --arm-emitted-events \
    "logs/quest_jaka_sim/${REPLAY_NAME}.arm_emitted_125hz.jsonl"
```

快速无界面离线回放时同时去掉 `--viewer` 和 `--realtime`。当前 HTS+CTRL 录制不要再加
`--engage-at-sec` 或 clutch-cycle 参数；这些参数只用于显式选择的旧 HTS-only fixture。

### 125 Hz 启动与时钟契约

- arm clutch 未 engage 时，adapter 保持当前 MuJoCo arm `qpos`，不推进旧目标或零位目标；
- 每个新的 arm clutch/reference generation 都以当前 MuJoCo `qpos` reset native
  resampler，清除旧 destination/sequence，然后接收新的 60 Hz target；静止首次捕获连续；
- 第一个 125 Hz deadline 对齐当前 simulation time，启动时不会补跑旧 wall-clock 或旧
  simulation epoch 的 tick；
- 每个 emitted arm command 在 `data.ctrl` 保持 4 个 2 ms MuJoCo physics step；RH56
  使用独立控制路径，但共享同一 simulation clock；
- 该模式复用不依赖 SDK 的 production native latest-destination/PWL 与有界 transition；
  不加载 JAKA SDK，也不经过 shaped 500 Hz arm command trajectory shaper。

### 录制与日志文件

| 文件 | 内容 |
|---|---|
| `*.hts.jsonl` | 带时间戳的 raw HTS/CTRL datagram，包括 wrist/head/hand 输入、index/grip 与 validity |
| `*.events.jsonl` | 共享 60 Hz arm/RH56 control event 与采样的 MuJoCo state |
| `*.report.json` | summary 与 `quest_jaka_joint_recording.v1` manifest：commit/config hash、时长、顺序、频率、路径、`simulation_only=true` |
| `*.arm_emitted_125hz.jsonl` | 125 Hz accepted-source/emitted 序列和时间、q/dq/ddq/jerk、destination/PWL/transition、actual q 与 tracking error |

只有选择 125 Hz mode 并传入 `--arm-emitted-events` 时才生成 emitted 日志。capture/log
是本地生成物，默认被 Git 忽略；不要随意把操作者录制加入仓库。

如果 live 运行因安全异常在正常 finalization 前结束，raw capture 可能是唯一完整落盘的
文件。应保留 raw 和 terminal error，禁止编辑 raw JSONL 去伪造缺失的 events/report。

### Viewer 与停止

viewer 保留 desired/actual TCP marker，terminal 会打印所选 arm output mode。正常停止使用
`Ctrl-C` 或关闭 viewer。stale input、clutch release、joint/collision/singularity/velocity/
acceleration/jerk policy 和 hard fault 在两种 mode 中都保持有效；不得为完成回放而放宽。
数据包、viewer、拒绝和停止诊断见[故障排查](../TROUBLESHOOTING.md)。

## 验收检查

- 未发生真机 SDK 导入或连接；
- 集成模型报告 6 个 JAKA + 6 个 RH56 actuator；显式 arm-only 回归模型报告 6 个
  JAKA actuator 且没有 RH56 command path；
- 第一次 engage 静止且无跳变；
- 平移/旋转符合坐标映射；
- release 按文档保持或停止；
- 可恢复不可行性安全保持且允许撤退恢复；
- 输入丢失或超时会停止，不会重放旧运动。

当前验证等级：两种 arm output mode 与 arm/RH56 联合回放均已离线测试，Quest/MuJoCo
路径已通过仿真验证。离线 replay 不代表已重新完成修复后的 Quest live 操作，也不构成
JAKA 或 RH56 真机验证。

数据包、viewer 和拒绝问题见[故障排查](../TROUBLESHOOTING.md)。

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
