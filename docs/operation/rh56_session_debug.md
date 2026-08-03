# RH56 hand-only session debug

This is the single current entry for operator-led RH56DFX PC-direct debugging.
It never starts or controls JAKA. Do not run the physical examples until the
operator has checked the device, workspace, hand stop, and emergency stop.

## Authorization model

The default command is dry-run and does not inspect or open a serial device.
The real hand path requires both an explicit device and one in-process session
arm:

```bash
./scripts/run_quest_rh56_hand_test.sh \
  --real --device /dev/serial/by-id/REPLACE_WITH_ACTUAL_ADAPTER_ID \
  --quest-teleop --arm-session \
  --duration-sec 60 \
  --manual-stop-accessible --workspace-clear --no-auto-retry
```

`--arm-session` creates an in-memory hand-only authorization for this process.
It covers all hand poses sent by that process, expires when the process exits,
and is never stored in a file, environment variable, or global service. A new
process must receive `--real --arm-session` again. Do not use `/dev/ttyUSB0`;
prefer the verified `/dev/serial/by-id/...` path.

Read-only device inspection remains available without arming:

```bash
export RH56_DEVICE=/dev/serial/by-id/REPLACE_WITH_ACTUAL_ADAPTER_ID
./scripts/run_quest_rh56_hand_test.sh \
  --real --device "$RH56_DEVICE" --preflight-only \
  --summary logs/rh56_preflight.summary.json
```

The legacy exact `--approval` option is no longer used for ordinary
read/position/Quest hand-only modes. It remains intentionally separate for
`--write-runtime-config`, `--clear-error`, and
`--force-sensor-calibration`. Combined JAKA+RH56 operation retains its own
dual physical gate and is not covered by this hand-only session arm.

## Six-channel mapping check

The following is prepared for the next physical check and has not been run by
this change. It commands only RH56, moves through existing rate and delta
limits, and holds every target for five seconds so the operator can mirror it
with the real hand:

```bash
timestamp="$(date +%Y%m%d_%H%M%S)"
./scripts/run_quest_rh56_hand_test.sh \
  --real --device "$RH56_DEVICE" --mapping-check --arm-session \
  --mapping-hold-sec 5 \
  --manual-stop-accessible --workspace-clear --no-auto-retry \
  --capture "logs/rh56_mapping_${timestamp}.hts.jsonl" \
  --events "logs/rh56_mapping_${timestamp}.events.jsonl" \
  --jsonl "logs/rh56_mapping_${timestamp}.telemetry.jsonl" \
  --summary "logs/rh56_mapping_${timestamp}.summary.json"
```

The sequence is open, index, middle, ring, little, thumb curve, thumb
lateral, and open return. It uses canonical order
`[index, middle, ring, pinky, thumb_close, thumb_lateral]`; the command path
still converts to the RH56 wire order. Each hold prints raw Quest thumb joint
data, raw/normalized thumb curve, thumb lateral, the six RH56 normalized
targets, and the latest Quest feature targets. Invalid/stale Quest input and
RH56 worker faults stop new commands and run cleanup.

## Safety that remains active

Session arming only removes repeated operator prompts. It does not bypass:

- six-channel canonical/protocol conversion and actuator range checks;
- configured maximum closure (`max_close=0.8`), per-command normalized delta
  limit (`0.05`), scheduler/rate limits, and existing command shaping;
- current/load contact protection, raw `ERROR` handling, feedback `STATUS`
  validation, checksum/protocol checks, stale-feedback checks, and disconnect
  handling;
- fresh `ANGLE_ACT` feedback at startup and measured activation before a target;
- manual stop, emergency stop, clear workspace, no automatic retry, and final
  hold/cleanup.

Opening the serial transport performs no register write. Grip release or a
mapping sequence finish holds the last safe target; it does not automatically
open the hand. The hand-only entry creates zero JAKA sessions and rejects any
arm `AcceptedArmTarget`.

## Offline checks

These checks do not connect to RH56:

```bash
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python tools/quest_rh56_hand_test.py --help
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python tools/quest_rh56_hand_test.py \
  --summary /tmp/rh56-dry-run-summary.json
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m pytest -q \
  tests/test_quest_rh56_hand_test_entry.py \
  tests/test_rh56_pc_direct_control.py \
  tests/test_rh56_worker_diagnostics.py
```

The dry-run summary must report `real_hardware: false` and
`rh56_connected: false`. No offline check is physical validation.

## 中文说明

普通 RH56 hand-only 调试只需要在当前进程命令中显式写出
`--real --device ... --arm-session` 一次；同一进程内的每个姿势不再重复确认。
进程退出后授权失效，重启必须重新传入参数，不使用 token 文件、环境变量或全局授权。
默认不带 `--real` 时是 dry-run。配置写入、清错、力传感器校准和 JAKA 联合运行仍使用
独立授权，不被本 session arm 覆盖。上面的六通道命令是下一步 hand-only 映射检查入口，
每个姿势保持至少 5 秒，操作者跟随并查看 Quest 原始/归一化特征和六个 RH56 target。
