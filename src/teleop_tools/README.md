# teleop_tools

This package retains one experimental path and one retired compatibility path
outside the authoritative Quest/JAKA shared pipeline:

- retired HEBI Mobile I/O / iPhone ARKit relative-pose arm teleoperation;
- iPhone camera + MediaPipe RH56 hand teleoperation.

## HEBI Mobile I/O (retired)

The following commands are retained for historical reproduction and are not
current recommended or production entry points:

```bash
./scripts/check_hebi_mobile_io.sh --duration-sec 5 --hz 10
./scripts/record_hebi_mobile_io.sh --duration-sec 20
./scripts/run_hebi_rviz_shadow.sh
```

The retired real-arm entry point
`scripts/run_real_jaka_hebi_arm_teleop.sh` must not be treated as a current
physical workflow. Its deadman, reference, lag-follow, workspace, target-filter
and legacy ServoJ shaping settings are not authority for Quest/JAKA and must
not be copied into its post-`AcceptedArmTarget` adapter.

The reusable safety ideas are explicit reference capture and staged tracking
error handling (warning, hold, fault). The primary Quest/JAKA stack implements
these in its shared state machine and native worker under its own tested
contracts.

## iPhone camera / RH56 hand

Offline/help-only entry points include:

```bash
./scripts/check_iphone_camera_stream.sh --help
./scripts/run_iphone_hand_tracking_debug.sh --help
./scripts/run_iphone_mediapipe_hand_teleop.sh --help
./scripts/run_iphone_rh56_safety_gate.sh --help
```

Opening a camera or commanding RH56 remains a separately authorized physical
operation.

## Focused regression tests

```bash
.venv/bin/python -m pytest -q \
  tests/test_relative_pose_lag_follow.py \
  tests/test_jaka_servo_jog.py \
  tests/test_robot_bringup_ros2_bridge.py \
  tests/test_rh56_ros2_bridge.py \
  tests/test_rh56_serial_backend.py
```
