# Real-hardware safety

## English

Repository maintenance, tests, replay, simulation, fake workers, and `--help`
do not connect to or command hardware. Physical operation is a separately
authorized activity.

The arm path is:

```text
validated input -> clutch/reference capture -> mapping/filtering
  -> continuation IK/feasibility -> AcceptedArmTarget
  -> native JAKA worker
```

The physical adapter does not remap, filter, solve IK, follow MuJoCo `qpos`, or
select another branch. The native worker owns final 8 ms command timing,
velocity/acceleration/jerk, watchdog, liveness, and cleanup checks. RH56 uses
the native six active-actuator channels; `ANGLE_ACT`, `CURRENT`, `FORCE_ACT`,
`ERROR`, and `STATUS` retain their raw meanings.

Hard-stop conditions include collision/alarm, emergency stop, SDK/transport
failure, watchdog/liveness loss, illegal command, hard joint/output limit,
RH56 fatal protocol or nonzero `ERROR`, and uncertain cleanup. An infeasible
candidate is rejected and held; it is not a hard-stop bypass or an automatic
retry.

Do not write controller payload, TCP, installation, collision, or safety
settings automatically. Do not claim physical PASS from simulation, replay,
fake workers, logs, or training results.

## 中文

仓库维护、测试、replay、仿真、fake worker 和 `--help` 都不会连接或控制硬件。真机操作必须单独获得授权。

机械臂管线为：

```text
已校验输入 -> clutch/reference capture -> mapping/filtering
  -> continuation IK/feasibility -> AcceptedArmTarget
  -> native JAKA worker
```

物理适配器不会 remap、filter、求 IK、跟随 MuJoCo `qpos` 或选择其他 branch。native worker 负责最终 8 ms
command timing、velocity/acceleration/jerk、watchdog、liveness 和 cleanup 检查。RH56 使用原生六个
active-actuator channel；`ANGLE_ACT`、`CURRENT`、`FORCE_ACT`、`ERROR`、`STATUS` 保留原始语义。

collision/alarm、急停、SDK/transport failure、watchdog/liveness loss、非法 command、硬 joint/output limit、
RH56 fatal protocol 或非零 `ERROR`、cleanup 不确定都属于 hard-stop。infeasible candidate 会被拒绝并 hold，
不能绕过 hard-stop，也不会自动 retry。

不要自动写入 controller payload、TCP、installation、collision 或 safety setting。不要把仿真、replay、fake
worker、log 或 training result 声称为真机 PASS。
