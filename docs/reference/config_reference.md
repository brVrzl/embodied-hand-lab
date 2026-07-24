# Configuration reference

| Configuration | Current role |
|---|---|
| `configs/sim/quest_hts_jaka_mini2_live_demo.yaml` | authoritative live Quest/JAKA shared pipeline |
| `configs/sim/quest_hts_jaka_mini2_offline.yaml` | recorded-input/offline simulation |
| `configs/motion_input/quest_hts_right_hand.yaml` | HTS receiver/provider settings |
| `configs/sim/quest_rh56_retarget.yaml` | simulated RH56 retargeting |
| `configs/robot/jaka_mini2_real.yaml` | physical connection example; not controller truth |
| `configs/hand/rh56_real.yaml` | RH56 physical example; separately gated |
| `configs/teleoperation/jaka_foundation.yaml` | dated foundation-gate policy |
| `digital_twin/configs/static_environment.yaml` | provisional integrated-workspace geometry |

The live Quest config documents freshness, clutch, mapping, filter,
continuation, IK, singularity, output velocity/acceleration, native period, and
safety timeouts. Comments in historical configs do not override current code.

Validate YAML syntactically and through existing loader tests. Do not
automatically translate a versioned sample IP, serial port, payload, or TCP
value into local device state.

---

# 中文版：配置参考

| 配置 | 当前用途 |
|---|---|
| `configs/sim/quest_hts_jaka_mini2_live_demo.yaml` | 权威的实时 Quest/JAKA 共享管线 |
| `configs/sim/quest_hts_jaka_mini2_offline.yaml` | 录制输入/离线仿真 |
| `configs/motion_input/quest_hts_right_hand.yaml` | HTS 接收器/provider 设置 |
| `configs/sim/quest_rh56_retarget.yaml` | 仿真 RH56 重定向 |
| `configs/robot/jaka_mini2_real.yaml` | 真机连接示例，不代表控制器真实配置 |
| `configs/hand/rh56_real.yaml` | RH56 真机示例，需单独 gate |
| `configs/teleoperation/jaka_foundation.yaml` | 带日期的基础 gate 策略 |
| `digital_twin/configs/static_environment.yaml` | 临时集成工作空间几何 |

实时 Quest 配置记录 freshness、clutch、映射、滤波、continuation、IK、奇异性、输出速度/
加速度、原生周期和安全超时。历史配置中的注释不能覆盖当前代码。

应通过 YAML 语法检查和现有 loader 测试验证配置。不能自动把版本化示例中的 IP、串口、
payload 或 TCP 值转化为本地设备状态。
