# Teleoperation

## English

The maintained teleoperation entrypoints are:

```bash
./scripts/run_quest_jaka_sim_demo.sh --help
./scripts/run_quest_jaka_bounded_teleop.sh --help
./scripts/run_quest_jaka_rh56_teleop.sh --help
./scripts/run_quest_rh56_hand_test.sh --help
```

Use the simulation wrapper for offline/replay work. The bounded arm wrapper
and combined arm/RH56 wrapper are physical procedures and require the explicit
operator prerequisites in [REAL_ROBOT.md](REAL_ROBOT.md) and
[REAL_HARDWARE_SAFETY.md](../safety/REAL_HARDWARE_SAFETY.md).

The shared arm pipeline captures a reference only after release, rejects stale
or malformed input, applies continuation IK and feasibility checks, and emits
one immutable `AcceptedArmTarget`. RH56 command semantics remain the native
six-channel active-actuator representation. No wrapper may add a second IK,
frame-remapping, or automatic hardware-discovery path.

## 中文

当前维护的 teleoperation 入口为：

```bash
./scripts/run_quest_jaka_sim_demo.sh --help
./scripts/run_quest_jaka_bounded_teleop.sh --help
./scripts/run_quest_jaka_rh56_teleop.sh --help
./scripts/run_quest_rh56_hand_test.sh --help
```

仿真 wrapper 用于离线/replay；有界 arm wrapper 和 arm/RH56 联合 wrapper 属于真机流程，必须先满足
[REAL_ROBOT.md](REAL_ROBOT.md) 和 [REAL_HARDWARE_SAFETY.md](../safety/REAL_HARDWARE_SAFETY.md) 中的
operator prerequisites。

共享 arm 管线只有在 release 后才捕获 reference，会拒绝过期或格式错误的输入，执行 continuation IK 和
feasibility 检查，然后输出一个不可变 `AcceptedArmTarget`。RH56 command 语义仍是原生六通道 active-actuator
表示。任何 wrapper 都不能增加第二套 IK、frame remap 或自动硬件发现路径。
