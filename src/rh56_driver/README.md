# RH56 PC-direct boundary

This package implements the maintained six-channel RH56DFX PC-direct serial
boundary, command scheduling, telemetry, and canonical actuator ordering.

`ANGLE_ACT`, `CURRENT`, `FORCE_ACT`, `ERROR`, and `STATUS` are raw controller
feedback fields. They are not a full passive-joint state, tactile array, or
direct slip measurement.

The physical entry points are `tools/quest_rh56_hand_test.py` and the combined
Quest/JAKA tool. They require explicit device identity and physical gate
arguments. Importing this package never opens a serial port. The retired JAKA
TIO and ROS2 JSON bridge paths are not supported.
