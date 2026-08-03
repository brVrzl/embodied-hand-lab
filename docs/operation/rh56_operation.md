# RH56 PC-direct operation

The maintained physical path is `RH56DFX -> USB/RS485 adapter -> Thor`. It does
not use JAKA tool-RS485 and never creates a JAKA SDK session. A 60 second Quest
hand-only run completed on 2026-07-29 with 903 feedback records, no timeout,
checksum, protocol, or hand fault, and zero JAKA sessions. Combined physical
operation remains unvalidated.

The control order is
`[index, middle, ring, pinky, thumb_close, thumb_lateral]`; the wire protocol
order is `[pinky, ring, middle, index, thumb_close, thumb_lateral]`. The driver
uses 115200 baud, device address 1, `ANGLE_SET=1486`, `ANGLE_ACT=1546`,
`FORCE_ACT=1582`, `CURRENT=1594`, `ERROR=1606`, and `STATUS=1612`. Position,
speed, and force register ranges are 0--1000. Production hand control defaults
to the physically tested `fast40` scheduler profile, with a maximum normalized
change of 0.05 per command and configured maximum closure 0.8. The 15 Hz
baseline remains selectable for comparison.

Opening the serial transport performs zero writes: it does not clear errors,
write speed/force, send a hold target, or open the hand. `ANGLE_ACT` is measured
feedback; command fields are never labelled as measured. Nonzero `ERROR`, a
read/protocol/checksum failure, feedback staleness, or disconnect enters
`HAND_FAULT`. The repository has no validated meaning for nonzero `STATUS`
codes, so they are recorded raw and a missing/invalid STATUS response faults;
no guessed status-code policy is applied.

## Current hand-only entry

The single authorization and operator procedure is maintained in
[RH56 hand-only session debug](rh56_session_debug.md). In brief, the default
entry is dry-run; a physical hand-only process requires
`--real --device /dev/serial/by-id/... --arm-session`. That in-memory arm covers
all poses in that process and expires at process exit. It never starts JAKA.

Use `--preflight-only` for identity and tty checks without opening the serial
transport. Prefer `/dev/serial/by-id/...`; the explicit CH341 fallback remains
available only after its VID:PID, driver, permissions, and occupancy checks.
The recommended physical progression is read-only, measured-relative bounded
single-channel, then Quest hand-only. The canonical page also contains the
prepared six-channel mapping check, with a five-second hold per pose.

The old per-command RH56 approval phrases are not used by these ordinary
hand-only modes. Exact approvals remain only for runtime configuration writes,
fault reset, and force-sensor calibration. JAKA arm-only and combined gates
retain their separate authorizations.

The following fake/offline check is useful before any physical hand gate:

```bash
.venv/bin/python -m pytest -q \
  tests/test_quest_live_controller_sim.py \
  tests/test_rh56_pc_direct_control.py \
  tests/test_quest_jaka_bounded_teleop_entry.py
```

All register-range, canonical/protocol ordering, scheduler/rate, per-cycle
delta, serial timeout, frame/checksum, feedback-stale, ERROR/STATUS,
measured/commanded separation, manual-stop, and cleanup protections remain
active.

## State and logging

```text
HAND_DISABLED -> HAND_HOLD -> HAND_ACTIVE
                      ^          |
                      +----------+  grip release/stale
                                 +-> HAND_FAULT
```

`HAND_HOLD` sends no new target and never opens automatically. Recovery needs
fresh feedback and a new grip enable. `rh56_pc_direct_episode.v1` records
selected and requested normalized targets separately, selected raw command,
measured raw/normalized position, current/load, raw status/error, transport and
control states, command/feedback timestamps, feedback read latency, fault, and
combined-episode validity.

---

# 中文版：RH56 PC-direct 操作

当前维护的真机链路是 `RH56DFX -> USB/RS485 转换器 -> Thor`，不使用 JAKA
tool-RS485，也不创建 JAKA SDK session。2026-07-29 已完成一次 60 秒 Quest hand-only
真机运行：903 条 feedback，无 timeout/checksum/protocol/hand fault，JAKA session 为 0；
联合真机运行仍未验证。

控制层规范顺序为 `[index, middle, ring, pinky, thumb_close, thumb_lateral]`，协议顺序为
`[pinky, ring, middle, index, thumb_close, thumb_lateral]`。协议为 115200 baud、地址 1；
`ANGLE_SET=1486`、`ANGLE_ACT=1546`、`FORCE_ACT=1582`、`CURRENT=1594`、
`ERROR=1606`、`STATUS=1612`。position/speed/force 寄存器范围是 0--1000。正式手部控制
默认使用已完成真机测试的 `fast40` scheduler profile；15 Hz baseline 仍可显式选择。
每次 normalized target 最大变化 0.05，配置的最大闭合量为 0.8。

打开串口时寄存器写入数为零：不 clear error、不写 speed/force、不发送 hold target、也不
自动张开。`ANGLE_ACT` 是实测反馈，command 不会伪装成 measured。非零 `ERROR`、读帧/
协议/checksum 错误、feedback stale 或断连都会进入 `HAND_FAULT`。仓库没有已验证的非零
`STATUS` code 含义，因此只记录 raw 值；STATUS 响应缺失或无效会 fault，不猜测 code。

## 中文当前入口

授权、设备前置检查、read-only/单通道阶段和下一步六通道映射命令统一见
[RH56 hand-only session debug](rh56_session_debug.md)。普通 hand-only 真机命令只需在进程
启动时一次传入 `--real --device ... --arm-session`；进程内不再逐姿势确认，退出后自动失效。
默认命令仍是 dry-run。旧的 per-command RH56 授权短语已从普通 hand-only 操作说明移除；
runtime config、clear error、force calibration 和 JAKA 联合运行继续使用独立 gate。
