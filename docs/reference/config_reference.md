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
| `configs/teleoperation/teledex_jaka_arm_bounded.yaml` | older bounded TeleDex path |
| `configs/workspace/tennis_ball_lift_current.yaml` | integrated digital-twin workspace |

The live Quest config documents freshness, clutch, mapping, filter,
continuation, IK, singularity, output velocity/acceleration, native period, and
safety timeouts. Comments in historical configs do not override current code.

Validate YAML syntactically and through existing loader tests. Do not
automatically translate a versioned sample IP, serial port, payload, or TCP
value into local device state.
