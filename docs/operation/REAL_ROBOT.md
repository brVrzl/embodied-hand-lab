# Real-device operation boundary

## English

This page describes the current operator boundary; it does not authorize a
run. The physical JAKA/RH56 entries must be selected explicitly and must fail
closed before opening a device when prerequisites are missing.

Before a separately authorized run, the operator must verify the intended
device identity, controller state, payload/TCP/controller settings, workspace,
E-stop access, and the exact bounded command duration. Software must not infer
or write controller payload, TCP, installation, collision, or safety settings.

Current entrypoints are wrapped by:

```bash
./scripts/run_quest_jaka_bounded_teleop.sh --help
./scripts/run_quest_jaka_rh56_teleop.sh --help
./scripts/run_quest_rh56_hand_test.sh --help
```

A controller alarm, collision, emergency stop, SDK/transport error, watchdog
failure, hard command-limit violation, RH56 nonzero `ERROR`, or uncertain
cleanup is a hard stop. Candidate infeasibility is a bounded hold: no new
target is accepted and a fresh heartbeat keeps liveness distinct from target
validity. Physical status must be described literally as offline, simulation,
partial physical validation, passed, failed, or not validated.

## 中文

本页只描述当前 operator boundary，不构成运行授权。JAKA/RH56 真机入口必须显式选择；缺少前置条件时，
入口必须在打开设备前 fail closed。

独立授权的真机运行前，operator 必须核对设备 identity、controller state、payload/TCP/controller setting、
workspace、E-stop 可用性和精确的有界 command duration。软件不能推断或写入 controller 的 payload、TCP、
installation、collision 或 safety setting。

当前入口由以下 wrapper 提供：

```bash
./scripts/run_quest_jaka_bounded_teleop.sh --help
./scripts/run_quest_jaka_rh56_teleop.sh --help
./scripts/run_quest_rh56_hand_test.sh --help
```

controller alarm、collision、急停、SDK/transport error、watchdog failure、硬 command-limit violation、
RH56 非零 `ERROR` 或 cleanup 不确定都必须 hard stop。candidate infeasibility 属于 bounded hold：不接受
新的 target，并用 fresh heartbeat 将 liveness 与 target validity 分开。物理状态必须如实标记为 offline、
simulation、partial physical validation、passed、failed 或 not validated。
