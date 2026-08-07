# RH56 H0 MuJoCo self-test

## Purpose and safety boundary

H0 is a low-amplitude, simulation-only check of the six semantic Inspire
RH56DFX actuator channels in the mounted JAKA/RH56 MuJoCo model. It verifies
model loading, channel-to-actuator binding, command limits, finite state, log
output, and an unchanged arm target.

The current H0 call chain is:

```text
tools/rh56_h0_self_test.py
  -> rh56_sim.Rh56H0SelfTest
  -> MuJoCo model/data
  -> six RH56 MuJoCo position actuators
```

It has no Quest receiver, network transport, serial transport, robot SDK, or
physical actuator adapter. Running H0 does not authorize or validate JAKA or
RH56 hardware. Its result must be described as offline simulation evidence.

## Maintained model contract

H0 defaults to:

```text
model:      assets/jaka_rh56_visual_coacd.xml
arm config: configs/sim/quest_hts_jaka_mini2_live_demo.yaml
```

The model contains:

- six range-limited JAKA position actuators;
- 12 RH56 joints and six range-limited RH56 position actuators;
- 148 reviewed active convex RH56 collision hulls;
- collision-disabled vendor visual geometry;
- seven reviewed adjacent-body RH56 contact exclusions, plus the current
  JAKA Link 0/Link 1 exclusion;
- six equality constraints for passive joint following.

The six equality constraints are only an underactuated-hand approximation:

- thumb PIP and DIP follow the directly actuated thumb-close joint using cubic
  polynomials fitted to the local command/angle table;
- each non-thumb distal joint follows its directly actuated MCP joint.

This deterministic six-input model does not reproduce tendon compliance,
backlash, load-dependent coupling, calibrated current/force limits, passive
joint state, or contact sensing. The RH56 feedback names `ANGLE_ACT`,
`CURRENT`, `FORCE_ACT`, `ERROR`, and `STATUS` are not produced by H0, and the
model must not be presented as measuring them.

The fixed hand mount transform is inherited from the maintained combined
model. No independent physical mount calibration in this repository proves
that transform, so H0 validates the model as committed rather than physical
mount accuracy.

## Semantic channels

Upper layers use canonical names and must not infer semantics from MuJoCo
actuator order.

| Canonical channel | Direct actuator | Direct joint | Joint/control range (rad) | Positive motion |
|---|---|---|---:|---|
| `index` | `rh56_R_index_MCP_joint_act` | `rh56_R_index_MCP_joint` | 0--1.70 | close |
| `middle` | `rh56_R_middle_MCP_joint_act` | `rh56_R_middle_MCP_joint` | 0--1.68 | close |
| `ring` | `rh56_R_ring_MCP_joint_act` | `rh56_R_ring_MCP_joint` | 0--1.70 | close |
| `pinky` | `rh56_R_pinky_MCP_joint_act` | `rh56_R_pinky_MCP_joint` | 0--1.70 | close |
| `thumb_close` | `rh56_R_thumb_MCP_joint2_act` | `rh56_R_thumb_MCP_joint2` | 0--0.698132 | bend/close |
| `thumb_lateral` | `rh56_R_thumb_MCP_joint1_act` | `rh56_R_thumb_MCP_joint1` | 0--1.396263 | lateral/opposition |

The canonical execution order is:

```text
[index, middle, ring, pinky, thumb_close, thumb_lateral]
```

H0 resolves each actuator and joint by name, verifies that every direct joint
and actuator is limited, and commands only the intersection of those ranges.

## Initial contact boundary

The canonical runtime model has zero contacts after a direct MuJoCo
load/forward at model `qpos0`. H0 then applies the six configured initial arm
joints and open-hand targets before forwarding the model; the maintained
configuration also reports zero penetrating contacts.

Zero initial contact does not prove that every combined thumb-close and
thumb-lateral target is collision-free. Extreme commands can bring the
conservative thumb and index convex hulls into contact. H0 deliberately uses a
bounded low-amplitude, one-channel-at-a-time motion and is not a full hand
collision certification.

## Test sequence

For each channel, H0:

1. starts from the open/neutral hand target;
2. moves through a smoothstep trajectory toward a positive excursion;
3. returns to neutral;
4. attempts a negative excursion only when legal range exists;
5. returns to neutral before moving to the next channel.

The default excursion is 15% of each channel's effective range.
`--amplitude-scale` is rejected unless it is in `(0, 0.20]`. At the current
open lower-limit pose, a negative excursion is logged as
`negative_skipped_illegal` instead of commanding beyond the limit.

Throughout the sequence, the six JAKA actuator targets remain equal to their
configured initial targets. On normal completion, viewer close, or Ctrl-C, H0
restores the initial hand and arm targets in a `finally` path.

## Run and verify

Run these commands from the repository root. They are all offline.

Check that the generated runtime asset is current:

```bash
.venv/bin/python tools/build_rh56_visual_coacd_runtime_asset.py --check
```

Run a short headless smoke test:

```bash
PYTHONPATH=src .venv/bin/python tools/rh56_h0_self_test.py \
  --cycle-seconds 0.05 \
  --amplitude-scale 0.01 \
  --repeat 1 \
  --log-path artifacts/rh56_h0/smoke.jsonl
```

For optional visual inspection on a graphical workstation:

```bash
PYTHONPATH=src .venv/bin/python tools/rh56_h0_self_test.py --viewer
```

The default headless run uses `--cycle-seconds 2.0`,
`--amplitude-scale 0.15`, and `--repeat 1`. If `--log-path` is omitted, output
is written to timestamped JSONL under `logs/rh56_h0/`.

## Result interpretation

The process exits with status 0 only when:

- all six channel sequences complete;
- no non-finite command or state is observed; and
- the arm actuator target is unchanged.

Each JSONL record includes simulation and host monotonic time, repeat and phase,
canonical channel, actuator and joint names, requested and clipped control,
actual direct-joint position, joint/control ranges, saturation, finite-value
status, and phase progress.

A successful run supports only these claims:

- the maintained MuJoCo model loads;
- its six semantic direct-actuator mappings and limits are internally
  consistent;
- the bounded H0 sequence remains finite;
- the arm target is not changed by the H0 runner.

It does not validate physical direction, physical mount alignment, passive
joint motion, current or force feedback, collision safety over the full command
space, teleoperation retargeting, or any real-device behavior.

---

# 中文版：RH56 H0 MuJoCo 自检

## 目的和安全边界

H0 是挂载 JAKA/RH56 MuJoCo model 上六个 Inspire RH56DFX semantic actuator channel 的低幅度、
仅仿真检查。它验证 model load、channel-to-actuator binding、command limit、finite state、log
output 和 arm target 不变。调用链是 `tools/rh56_h0_self_test.py -> rh56_sim.Rh56H0SelfTest ->
MuJoCo model/data -> 六个 RH56 position actuator`。没有 Quest receiver、network、serial、robot
SDK 或 physical adapter，因此结果只能描述为离线仿真证据。

## Model contract 和通道

默认 model 是 `assets/jaka_rh56_visual_coacd.xml`，arm config 是
`configs/sim/quest_hts_jaka_mini2_live_demo.yaml`。model 有六个 JAKA actuator、12 个 RH56
joint、六个 RH56 actuator、reviewed collision hull、contact exclusion 和六个 equality constraint。
equality constraint 只近似 thumb PIP/DIP 和 finger DIP 的 coupled hand，不模拟 tendon compliance、
backlash、load-dependent coupling、current/force limit、passive state 或 contact sensing。

canonical execution order：

```text
[index, middle, ring, pinky, thumb_close, thumb_lateral]
```

上层不能从 MuJoCo actuator order 猜 channel 语义。H0 按名称解析 actuator/joint，确认每个对象
有 range，并只在这些 range 的交集内发 command。`ANGLE_ACT`、`CURRENT`、`FORCE_ACT`、`ERROR`、
`STATUS` 是物理 device register，H0 不会产生它们。

## 接触和序列

直接 load/forward 的 `qpos0` 以及配置的初始 arm/open-hand pose 当前应没有 penetrating contact；
这不证明所有 thumb-close/thumb-lateral target 都无碰撞。H0 使用单 channel、低幅度、有界 motion，
不是完整 collision certification。

每个 channel 从 open/neutral 开始，经过 smoothstep 正向 excursion，回 neutral；只有合法 range
允许时尝试负向 excursion，然后再进入下一个 channel。默认幅度是 effective range 的 15%，
`--amplitude-scale` 必须在 `(0, 0.20]`。整个过程 JAKA target 保持初始值；正常结束、viewer close
或 Ctrl+C 都在 `finally` 中恢复 hand/arm target。

## 离线运行

```bash
.venv/bin/python tools/build_rh56_visual_coacd_runtime_asset.py --check
PYTHONPATH=src .venv/bin/python tools/rh56_h0_self_test.py \
  --cycle-seconds 0.05 --amplitude-scale 0.01 --repeat 1 \
  --log-path artifacts/rh56_h0/smoke.jsonl
```

`--viewer` 只用于图形化仿真检查。status 0 只表示六路序列完成、command/state finite 且 arm
target 未改变。它不能证明物理方向、安装标定、被动关节、current/force feedback、完整 command
space collision safety、teleop retargeting 或任何真机行为。
