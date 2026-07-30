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

## Required staged procedure

These steps are intentionally ordered. A later physical stage does not replace
an earlier one.

The officially recommended sequence remains read-only -> bounded single-channel
-> Quest hand-only. An operator with prior physical RH56 experience may make an
explicit, run-specific decision to skip the first two stages. That exception is
not the default procedure and does not disable any production hand limits.

1. Find the stable device path. Never use `/dev/ttyUSB0`.

   ```bash
   ls -l /dev/serial/by-id/
   export RH56_DEVICE=/dev/serial/by-id/REPLACE_WITH_ACTUAL_ADAPTER_ID
   ```

   Some Thor installations use the custom `usb_ch341` driver, whose tty name is
   `/dev/ttyCH341USB<N>`. The stock `60-serial.rules` matches only `ttyUSB*` and
   `ttyACM*`, so it may create no by-id link even when the sole CH340 adapter is
   present. After verifying VID:PID `1a86:7523`, driver `usb_ch341`, permissions,
   and zero occupants, the CLI supports this explicit fallback with
   `--allow-direct-ch341-device`. A stable administrator-managed udev alias is
   still preferred.

2. Record VID, PID, USB serial, and resolved tty without opening serial:

   ```bash
   ./scripts/run_quest_rh56_hand_test.sh \
     --device "$RH56_DEVICE" \
     --preflight-only \
     --summary logs/rh56_preflight.summary.json
   ```

3. Run the read-only probe. This mode writes no register; its summary must show
   `register_write_count=0`.

   ```bash
   ./scripts/run_quest_rh56_hand_test.sh \
     --device "$RH56_DEVICE" \
     --read-only \
     --duration-sec 10 \
     --approval I_AUTHORIZE_ONE_RH56_PC_DIRECT_READ_ONLY_PROBE \
     --jsonl logs/rh56_read_only.jsonl \
     --summary logs/rh56_read_only.summary.json
   ```

4. Review `ANGLE_ACT`, `CURRENT`, `FORCE_ACT`, `ERROR`, `STATUS`, feedback
   frequency, latency, repeat ratio, timeouts, checksum failures, protocol
   errors, identity, and channel-order conversion. Do not proceed on a fault.

5. Run one measured-relative, single-channel bounded test. Both `--channel`
   and `--delta` are mandatory; `|delta|` cannot exceed the production 0.05
   per-command limit. The first target is fresh measured `ANGLE_ACT`, never a
   fixed open/zero pose.

   ```bash
   ./scripts/run_quest_rh56_hand_test.sh \
     --device "$RH56_DEVICE" \
     --bounded-command \
     --channel index \
     --delta 0.03 \
     --duration-sec 2 \
     --hold-sec 2 \
     --approval I_AUTHORIZE_ONE_RH56_PC_DIRECT_BOUNDED_HAND_TEST \
     --manual-stop-accessible --workspace-clear --no-auto-retry \
     --jsonl logs/rh56_bounded_index.jsonl \
     --summary logs/rh56_bounded_index.summary.json
   ```

6. After Stage 1 and Stage 2 pass, run Quest hand-only teleoperation. It opens
   no JAKA path. Grip press captures fresh measured `ANGLE_ACT`; grip release
   holds the last safe target, does not open the hand, and the process continues.
   This uses production ranges/rate/delta, not the Stage 2 single-channel bound.

   ```bash
   ./scripts/run_quest_rh56_hand_test.sh \
     --device "$RH56_DEVICE" \
     --quest-teleop \
     --duration-sec 30 \
     --approval I_AUTHORIZE_ONE_RH56_PC_DIRECT_BOUNDED_HAND_TEST \
     --manual-stop-accessible --workspace-clear --no-auto-retry \
     --capture logs/rh56_quest_hand.hts.jsonl \
     --events logs/rh56_quest_hand.events.jsonl \
     --jsonl logs/rh56_quest_hand.telemetry.jsonl \
     --summary logs/rh56_quest_hand.summary.json
   ```

7. Complete the arm-only normal-teleop producer-budget retest described in
   [JAKA arm teleoperation](jaka_arm_teleoperation.md). It remains a separate
   physical gate and sends zero RH56 commands.

8. Run the deterministic fake combined checks before combined hardware:

   ```bash
   .venv/bin/python -m pytest -q \
     tests/test_quest_live_controller_sim.py \
     tests/test_rh56_pc_direct_control.py \
     tests/test_quest_jaka_bounded_teleop_entry.py
   ```

9. Run the first combined physical test with left index released throughout;
   operate grip only. See [combined teleoperation](jaka_rh56_combined_teleop.md).
   Arm commands while index is released must remain zero.

10. Only after Step 9 passes, perform a separately authorized combined run in
    which arm and hand may both be active.

11. Only after combined physical validation should the dual-camera single-
    episode capture be connected. This task does not begin that capture.

For an explicitly operator-approved direct Quest hand-only run, use unique
timestamped paths so none of the exclusive-create outputs can overwrite older
evidence:

```bash
timestamp="$(date +%Y%m%d_%H%M%S)"
./scripts/run_quest_rh56_hand_test.sh \
  --device "$RH56_DEVICE" \
  --allow-direct-ch341-device \
  --quest-teleop \
  --duration-sec 60 \
  --approval I_AUTHORIZE_ONE_RH56_PC_DIRECT_BOUNDED_HAND_TEST \
  --manual-stop-accessible --workspace-clear --no-auto-retry \
  --capture "logs/rh56_quest_hand_${timestamp}.hts.jsonl" \
  --events "logs/rh56_quest_hand_${timestamp}.events.jsonl" \
  --jsonl "logs/rh56_quest_hand_${timestamp}.telemetry.jsonl" \
  --summary "logs/rh56_quest_hand_${timestamp}.summary.json"
```

Normal hand teleoperation does not mean safety is disabled. Register range,
canonical/protocol ordering, selected scheduler rate, per-cycle delta, serial timeouts,
frame/checksum validation, feedback stale, ERROR/STATUS response gates, grip
stale, measured/commanded separation, and cleanup remain active.

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

## 必须按顺序执行的阶段

官方推荐顺序仍是 read-only -> bounded 单通道 -> Quest hand-only。已有 RH56 真机经验的
操作者可以针对某一次运行明确决定跳过前两阶段，但这不是默认流程，也不会关闭 production
hand 安全限制。

1. 用 `ls -l /dev/serial/by-id/` 找到稳定路径，并设置
   `RH56_DEVICE=/dev/serial/by-id/实际设备名`；禁止使用 `/dev/ttyUSB0`。
   若 Thor 使用自定义 `usb_ch341` 驱动，stock udev 规则可能不会为
   `/dev/ttyCH341USB<N>` 生成 by-id。只有核实 VID:PID `1a86:7523`、driver、权限和零占用后，
   才可显式增加 `--allow-direct-ch341-device`；仍优先使用管理员维护的稳定 udev alias。
2. 用上文 `--preflight-only --summary ...` 命令记录 VID、PID、USB serial 和 resolved tty，
   该命令不打开串口。
3. 用上文精确授权执行 `--read-only`。它不写任何寄存器，summary 必须为
   `register_write_count=0`。
4. 检查 `ANGLE_ACT`、`CURRENT`、`FORCE_ACT`、`ERROR`、`STATUS`、反馈频率/延迟、重复
   比例、timeout、checksum、protocol error 和通道转换；出现 fault 不得继续。
5. 用上文 `--bounded-command --channel index --delta 0.03` 模板执行单通道小幅测试。
   channel/delta 必须显式提供，首个 target 来自 fresh measured `ANGLE_ACT`。
6. Stage 1/2 通过后，才可用上文 `--quest-teleop` 运行 Quest hand-only。它不连接 JAKA；
   grip 释放只保持最后安全 target，不自动张开且程序继续。正式模式使用 production 范围/
   rate/delta，不继承 Stage 2 单通道限制。
7. 按[JAKA 机械臂遥操作](jaka_arm_teleoperation.md)完成 arm-only normal teleop 的 producer-
   budget 复测；该 gate 不发送 RH56 命令。
8. 用上文三个 pytest 文件执行 deterministic fake combined 检查。
9. 按[联合遥操作](jaka_rh56_combined_teleop.md)进行首次联合真机测试：全程不按 left-index，
   只操作 grip；index 未按时新增 arm motion target 必须为 0。
10. Step 9 通过且另行授权后，才执行 arm 与 hand 同时 ACTIVE。
11. 联合真机验证完成后，最后才接入双相机单 episode 数据采集；本任务不开始采集。

如果操作者基于既往 RH56 真机经验明确选择直接进行 Quest hand-only，可使用英文部分的
timestamp 模板。四个输出都采用唯一时间戳和 exclusive-create，避免覆盖旧证据。

正常 hand teleop 不等于取消安全限制。0--1000 范围、通道顺序、所选 scheduler rate、每周期 delta、
串口 timeout、帧/checksum、feedback stale、ERROR/STATUS 响应、grip stale、measured/commanded
区分和 cleanup 都继续生效。联合入口不会自动 clear error、写 speed、写 force 或张开手。
