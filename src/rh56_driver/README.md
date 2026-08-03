# RH56 PC-direct boundary

This package implements the maintained six-channel RH56DFX PC-direct serial
boundary, command scheduling, telemetry, and canonical actuator ordering.

`ANGLE_ACT`, `CURRENT`, `FORCE_ACT`, `ERROR`, and `STATUS` are raw controller
feedback fields. They are not a full passive-joint state, tactile array, or
direct slip measurement.

The physical hand-only entry point is `tools/quest_rh56_hand_test.py`. It is
dry-run by default and requires `--real --device ... --arm-session` to create
one in-memory authorization for the current process. Importing this package
never opens a serial port. Runtime configuration, fault reset, force
calibration, and combined Quest/JAKA operation retain separate gates. The
retired JAKA TIO and ROS2 JSON bridge paths are not supported.
