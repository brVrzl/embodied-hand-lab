# Configuration

This page is the authoritative configuration guide for maintained code.
Configuration never opens hardware. Runtime safety prerequisites and operator
confirmations are command-line inputs, never reusable YAML defaults.

## Loading and precedence

There is no repository-wide implicit configuration merge and no `.env`
loader. Each entry point owns a documented YAML schema and command-line
surface. For a field supported by that entry point, the intended precedence is:

```text
code schema/default
  < value in the explicitly selected YAML
  < explicit command-line value
```

Environment variables participate only where the code or launcher names them
explicitly. They do not override arbitrary YAML keys.

Examples:

- `embodied-lab sim smoke --config PATH --duration-sec SEC` selects a replay
  YAML and explicitly overrides smoke duration.
- the physical Quest/JAKA wrappers require robot IP, bounded duration, output
  limits, and runtime safety confirmations on the command line;
- `quest_rh56_hand_test.py --device PATH` owns the actual serial device even
  though the selected hand YAML describes protocol policy;

Do not assume an older standalone tool implements this hierarchy. Its current
`--help` and loader are authoritative.

## YAML validation

The shared loader requires the YAML root to be a mapping. Subsystem loaders add
their own required fields, units, paths, enumerations, and bounds; some reject
unknown keys while others do not. A syntactically valid YAML file is therefore
not necessarily a valid runtime configuration.

The read-only doctor parses every `configs/**/*.yaml`:

```bash
.venv/bin/embodied-lab doctor --json
```

Exercise the owning loader for semantic validation:

```bash
.venv/bin/embodied-lab sim smoke \
  --config configs/sim/quest_hts_jaka_mini2_offline.yaml

.venv/bin/python -m pytest -q tests/test_configs.py
```

The simulation command is offline. Hardware config validation must use the
owning tool's no-connect/preflight mode where one exists; a YAML parse is not
permission to open a device.

## Maintained configuration inventory

### Simulation, motion input, and collection

| File | Owner and status |
| --- | --- |
| `configs/sim/quest_hts_jaka_mini2_offline.yaml` | Default headless/offline replay and unified simulation smoke |
| `configs/sim/quest_hts_jaka_mini2_live_demo.yaml` | Shared live Quest target-generation, MuJoCo, and native-adapter policy |
| `configs/data_collection/physical_collection.yaml` | Unified physical and simulation-backed collection config; runtime and recorder/camera settings are kept in one reviewed file |

The live Quest YAML is the default policy before the output adapter. It contains
input freshness, clutch semantics, frames, provisional calibration, filters,
continuation, IK, singularity checks, output feasibility, MuJoCo settings, and
the thin native adapter contract. Its input-recovery window is capped at
10 seconds and does not alter the native 100 ms producer watchdog. It is not controller state and must not be
used to write payload, TCP, installation, or safety settings.

The offline and live Quest configurations are deliberately different.
The offline file uses a small, uncalibrated simulation-only displacement
envelope and orientation disabled. The live file retains its
`maximum_target_displacement_m` for simulation/replay. The physical collection
runtime explicitly sets `enforce_clutch_target_displacement_limit: false`, so
that field is not used as a per-clutch task-travel limit there. IK, joint-limit,
singularity, collision, output, controller, timing, liveness, and operator
workspace gates remain active. The live file remains provisional where marked
and is not a claim of full physical calibration.

### RH56

| File | Owner and status |
| --- | --- |
| `configs/hand/rh56_pc_direct_teleop.yaml` | Maintained PC-direct protocol, scheduler, feedback, bounds, channel order, and safety policy |
| `configs/hand/quest_rh56_real_retarget.yaml` | Maintained live Quest feature calibration for hand-only and combined physical RH56, and the live simulation default; does not own protocol travel |

The canonical six-channel order is:

```text
[index, middle, ring, pinky, thumb_close, thumb_lateral]
```

The MuJoCo hand actuator order is:

```text
[thumb_lateral, thumb_close, index, middle, ring, pinky]
```

For the Quest RH56 retarget YAML, `calibration.palm_normalization_scale`
controls the shared palm-width denominator used by `thumb_lateral`; it is not
a sixth feature scale. `calibration.digit_scale` is exactly five finite,
positive values in `[index, middle, ring, pinky, thumb_close]` order. The
loader requires these fields directly and does not compose a runtime global
scale with local scales or accept legacy aliases.

The protocol order is:

```text
[pinky, ring, middle, index, thumb_close, thumb_lateral]
```

Do not interpret `ANGLE_ACT` as all coupled passive-joint angles.
`ANGLE_ACT`, `CURRENT`, `FORCE_ACT`, `ERROR`, and `STATUS` are device register
feedback fields. They are not a tactile array, slip sensor, or complete
kinematic state. Nonzero `STATUS` semantics have not been validated and must
remain raw rather than guessed.

`rh56_pc_direct_teleop.yaml` contains serial protocol settings but no device
path. The actual stable device is selected by the physical runtime
configuration and injected by the entry point before backend creation.
Speed/force values in YAML are software command policy, not permission to write runtime
registers. Runtime configuration writes have a separate explicit operation and
configuration-write confirmation.

### Perception and collection preparation

| File | Owner and status |
| --- | --- |
| `configs/perception/d435_tabletop.yaml` | Offline tabletop depth/point-cloud processing and uncalibrated target-frame placeholder |
| `configs/data_collection/physical_collection.yaml` | Unified runtime, dual-D435 acquisition, and episode-writer schema; bind workspace/wrist roles by serial |

The maintained collection file is the single source for this repository's
physical and simulation-backed capture. Its `runtime` mapping is consumed by
the physical wrapper; its `dataset`, `cameras`, and `calibration` mappings are
consumed by the episode recorder. Edit the host/device values in this file and
verify them before any separately acknowledged physical run:

```bash
${EDITOR:-vi} configs/data_collection/physical_collection.yaml
```

Its explicit `runtime.enforce_clutch_target_displacement_limit: false` setting
keeps the live-demo 0.20 m value available to simulation/replay while preventing
that clutch-relative envelope from stopping a collection task. This setting
does not disable the remaining target-generation, JAKA, RH56, controller,
operator-workspace, or cleanup safety contracts.

Replace both camera serial placeholders and attach calibration snapshot
identity before collection. Camera roles are assigned by serial, never
`/dev/video*` order. Camera acquisition settings live in this collection
schema; there is no second production camera profile under
`configs/camera/`. The current consumer records real Quest and D435 input
against simulated arm/hand state; a copied config does not make physical
JAKA/RH56 collection implemented or validate the cameras, calibration, or
synchronization on a target host.

The small mock RGB-D YAML used by `tests/test_configs.py` is kept under
`tests/fixtures/`, because it is test data rather than an operator or device
configuration. The tabletop YAML is intentionally separate: it configures
offline depth/point-cloud analysis after capture and is not read by the
episode recorder.

The tabletop perception config uses meters. Its
`target_from_camera_npy: null` and `calibration_status: uncalibrated` values
are intentional blockers; do not substitute an identity transform.

### Historical and digital-twin policy

`docs/history/gates/jaka_foundation_20260716/jaka_foundation.yaml` records a
dated foundation-gate policy beside its evidence. It is not the current
production arm configuration.

## Units and naming

Use field suffixes as part of the schema:

- `_m`, `_m_s`: meters and meters per second;
- `_rad`, `_rad_s`, `_rad_s2`, `_rad_s3`: radians and derivatives;
- `_deg`: degrees;
- `_sec`, `_s`, `_ms`, `_ns`: explicit time units;
- `_hz`: cycles per second;
- `_bytes`: byte counts;
- `_xyzw`: quaternion order;
- RH56 raw command/register values: dimensionless device counts, normally
  0--1000 where the owning schema says so.

Do not mix a configured MuJoCo actuator radian with RH56 raw counts.
Do not treat `CURRENT` or `FORCE_ACT` as SI torque/force unless a separately
validated calibration supplies that conversion.

Frame names follow the owning architecture document. In transform notation,
`T_A_B` maps coordinates expressed in frame B into frame A. A provisional
operator-to-robot basis, MuJoCo scene placement, camera preview pose, or
COLMAP registration is not automatically a calibrated robot/camera extrinsic.
See [coordinate frames](../architecture/coordinate_frames.md).

## Paths

The installed source tree is found from the `embodiment_core` package.
`EMBODIED_LAB_ROOT` may explicitly select another repository root. Relative
paths in maintained configs are interpreted relative to that repository or by
the owning tool as documented; run operator wrappers from any directory only
when the wrapper itself resolves the root.

Keep these roles separate:

- `assets/`: versioned MuJoCo assets;
- `configs/data_collection/physical_collection.yaml`: reviewed host/device and
  episode capture configuration;
- `data/episodes/`: canonical episode data, ignored by default;
- `logs/`: runtime evidence, ignored;
- `artifacts/`: generated reports, exports, local configs, checkpoints, and
  temporary experiment outputs, ignored;
- `configs/`: reviewed versioned policy/examples.

Do not place the only copy of calibration, raw data, or a checkpoint in a
temporary directory. Output commands should use unique run/episode paths and
must not overwrite raw evidence.

## Explicit environment variables

Only the following current variables have defined effects:

| Variable | Effect |
| --- | --- |
| `EMBODIED_LAB_ROOT` | Explicit repository-root override used by unified tooling |
| `EMBODIED_LAB_SOURCE_REVISION` | Source-bundle revision/provenance string for episode metadata when project Git metadata is absent |
| `DISPLAY`, `XAUTHORITY` | X11 viewer selection used by the simulation wrapper |
| `MUJOCO_GL` | MuJoCo rendering backend selected by MuJoCo, such as a tested `egl` or `osmesa` setup |
| `PYTHON_BIN` | Interpreter override supported by selected shell wrappers |
| `CUDA_VISIBLE_DEVICES` | Optional GPU visibility/order for host and MuJoCo diagnostics |

The doctor reports safe environment values and only the presence—not the
contents—of known credential variables. Do not store credentials in YAML,
shell history, logs, or examples.

## Device-specific values

### JAKA

The robot IP is an explicit command-line gate value. The production shared
YAML owns software limits and frame/transport expectations, not live
controller truth. Before a physical gate the operator must verify payload,
center of mass, installation, active TCP/user frame, collision state, and
safety limits at the controller. Software must not silently apply recorded
values.

### RH56

Select a stable device path explicitly. The YAML owns baud rate, address, the
single fixed scheduler, canonical/protocol mapping, stale thresholds, and
command bounds. An open transport performs no automatic configuration write or
safe-open.

### RealSense

Record camera role, serial, firmware, USB mode, stream profile, depth scale,
alignment policy, timestamp domains, and calibration snapshot. Serial and
calibration identity are data provenance, not optional convenience fields.

### Quest

Bind address, UDP port, project IP, and optional allowed-sender are command-line
network values. Freshness, required hand/head/controller state, clutch
hysteresis, frame mapping, and filters belong to the reviewed YAML/code
contract. Source timestamps and host receipt timestamps have different epochs.

## Configuration change review

Treat changes to any of the following as control or data-contract changes, not
cosmetic tuning:

- coordinate frames, axes, quaternion order, units, or calibration identity;
- joint, workspace, velocity, acceleration, jerk, singularity, collision, or
  stale thresholds;
- clutch and release-before-press behavior;
- RH56 channel order, register semantics, closure/delta/rate/feedback limits;
- camera role, serial, alignment, depth unit, timestamp skew, or extrinsic;
- observation/action schema, camera order, normalization, temporal horizon, or
  action chunk;
- distributed world/global batch, precision, learning rate, split manifest, or
  checkpoint compatibility.

Update the owning behavior test and documentation, parse all YAML, run the
offline smoke/replay that consumes it, and preserve the old configuration hash
with any historical result. Never weaken a robot-control safety boundary to
make a test pass.

For environment installation see
[Installation](../setup/INSTALLATION.md). For failure diagnosis see
[Troubleshooting](../TROUBLESHOOTING.md).

---

# 中文版：配置

本页是维护代码使用的权威配置说明。读取配置不会打开硬件。真机安全前置条件和操作者确认
必须通过命令行显式提供，不能作为可复用的 YAML 默认值。

## 加载和优先级

仓库没有隐式的全局配置合并，也没有 `.env` 自动加载。每个入口负责自己的 YAML schema 和
命令行参数。对于入口支持的字段，优先级是：

```text
代码 schema/default
  < 显式选择的 YAML 值
  < 显式命令行值
```

环境变量只有在代码或 launcher 明确读取时才有效，不会覆盖任意 YAML key。应以入口当前的
`--help` 和 loader 为准，不要根据旧工具猜测优先级。

## YAML 验证

共享 loader 要求 YAML 根节点是 mapping；各子系统 loader 继续检查必需字段、单位、路径、枚举
和范围，有的还拒绝未知 key。YAML 能解析不等于可以安全运行。

只读检查：

```bash
.venv/bin/embodied-lab doctor --json
.venv/bin/python -m pytest -q tests/test_configs.py
```

仿真配置可以直接由 `embodied-lab sim smoke` 读取。真机配置必须通过其所属入口的无连接或
preflight 模式验证；解析 YAML 不等于获得打开设备的权限。

## 当前配置清单

| 文件 | 作用 |
| --- | --- |
| `configs/sim/quest_hts_jaka_mini2_offline.yaml` | 默认无硬件离线回放和仿真 smoke |
| `configs/sim/quest_hts_jaka_mini2_live_demo.yaml` | Quest 目标生成、MuJoCo 和 native adapter 的共享 live policy |
| `configs/data_collection/physical_collection.yaml` | 真机/仿真采集的 runtime、双 D435 和 episode writer 配置 |
| `configs/hand/rh56_pc_direct_teleop.yaml` | RH56 PC-direct 协议、scheduler、feedback、channel order、bounds 和 safety policy |
| `configs/hand/quest_rh56_real_retarget.yaml` | Quest hand feature calibration；不负责 RH56 serial device 或协议写入 |
| `configs/perception/d435_tabletop.yaml` | 采集后的离线 tabletop depth/point-cloud 处理；不是 recorder 配置 |
| `tests/fixtures/default_rgbd.yaml` | 测试 fixture，不是 operator/device 配置 |

物理采集配置是采集流程的单一来源。两个相机 serial 必须显式填写且不同，角色不能按
`/dev/video*` 顺序推断。当前维护采集关闭 depth，只写 RGB MP4 和低维 state/action 表；Quest
packet、TCP 和 depth 不进入默认训练视图。

其中 `runtime.enforce_clutch_target_displacement_limit: false` 只表示采集流程不把 live-demo
配置中的 `maximum_target_displacement_m: 0.20` 当作单次 clutch 的任务行程上限；仿真、回放和其他
未显式覆盖的流程仍可使用该配置值。IK、joint-limit、奇异性、碰撞、输出、controller、时序、liveness
以及操作者 workspace 确认仍然有效。

RH56 软件顺序是：

```text
[index, middle, ring, pinky, thumb_close, thumb_lateral]
```

协议顺序是：

```text
[pinky, ring, middle, index, thumb_close, thumb_lateral]
```

`ANGLE_ACT`、`CURRENT`、`FORCE_ACT`、`ERROR` 和 `STATUS` 是原始寄存器反馈，不是完整被动
关节状态、触觉、滑移或标定后的接触力。非零 `STATUS` 的含义没有验证，不能猜测。

## 单位、路径和设备值

- `_m`、`_m_s` 表示米和米/秒；
- `_rad`、`_rad_s`、`_rad_s2`、`_rad_s3` 表示弧度及其导数；
- `_deg` 表示角度；`_sec`、`_s`、`_ms`、`_ns` 表示时间单位；
- `_hz` 表示频率，`_bytes` 表示字节数，`_xyzw` 表示四元数顺序；
- RH56 raw command/register 通常是 schema 指定的 0--1000 无量纲 device count。

不要混用 MuJoCo actuator 弧度和 RH56 raw count，也不要在没有独立标定时把 `CURRENT` 或
`FORCE_ACT` 当作 SI 扭矩/力。`T_A_B` 表示把 frame B 中的坐标映射到 frame A。

路径职责如下：

- `assets/`：版本化 MuJoCo 和机器人资产；
- `configs/`：审核过的 policy/example；
- `configs/data_collection/physical_collection.yaml`：采集 host/device 和 episode 配置；
- `data/episodes/`：默认忽略的 canonical episode 数据；
- `logs/`：默认忽略的运行证据；
- `artifacts/`：报告、导出物、checkpoint 和临时输出。

JAKA IP 是命令行 gate 值；RH56 要求显式稳定 serial path；相机要记录 serial、firmware、USB
mode、stream profile、depth scale、alignment、timestamp domain 和 calibration snapshot。软件
不能静默写入 payload、COM、installation、TCP、collision 或 controller safety。

## 配置变更规则

坐标系、单位、标定、joint/workspace/velocity/acceleration/jerk/singularity/collision/stale
阈值、clutch、RH56 channel/register 语义、相机身份、observation/action schema 或 split/normalization
都属于行为或数据契约变更，不是普通调参。变更后应更新行为测试，解析全部 YAML，运行对应的
离线 smoke/replay，并保留历史结果的来源信息。不得为了通过测试而放宽机器人安全边界。

安装见[安装](../setup/INSTALLATION.md)，故障诊断见[故障排查](../TROUBLESHOOTING.md)。
