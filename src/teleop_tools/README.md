# teleop_tools

`teleop_tools` 是当前真实遥操作主模块，不能在项目精简中删除。

保留链路：

- TeleDex / iPhone ARKit arm teleop（新链路，要求独立 frame calibration）。
- HEBI Mobile I/O / iPhone ARKit 相对位姿 arm teleop。
- iPhone camera + MediaPipe RH56 hand teleop。
- Xbox palm-target jog teleop。
- RViz shadow / real-command mirror。

## TeleDex / iPhone ARKit

TeleDex 不复用旧 HEBI phone→robot 映射数值。先验流、三轴标定和 shadow 六方向确认：

```bash
./scripts/check_teledex_phone.sh --duration-sec 15
./scripts/calibrate_teledex_jaka_frame.sh
./scripts/run_teledex_rviz_shadow.sh
```

确认步骤和 arm-only 实机命令见
[`docs/teledex_jaka_arm_teleop.md`](../../docs/teledex_jaka_arm_teleop.md)。默认只启用平移，
RH56 hand retarget 暂不接入。

## HEBI Mobile I/O / iPhone ARKit

前提：

- iPhone 与 PC 在同一 WiFi。
- HEBI Mobile I/O app 中 `Family=HEBI`、`Name=mobileIO`。
- HEBI Mobile I/O 已允许相机权限，AR pose 可用。

只读检查：

```bash
./scripts/check_hebi_mobile_io.sh --duration-sec 5 --hz 10
```

记录手机 pose：

```bash
./scripts/record_hebi_mobile_io.sh --duration-sec 20
```

RViz shadow，不发布硬件命令：

```bash
./scripts/run_hebi_rviz_shadow.sh
```

实机 arm-only teleop：

```bash
./scripts/run_real_jaka_hebi_arm_teleop.sh \
  --enable-motion \
  --teleop-mode relative_pose_lag_follow \
  --teleop-profile practical \
  --jsonl-out logs/teleop/hebi_real_arm_$(date +%Y%m%d_%H%M%S).jsonl
```

控制约定：

- `B1` 是 deadman，并在启用时锁定当前手机 pose 作为相对参考。
- 手机平移通过 `configs/teleop/hebi_mobile_io_jaka_rh56.yaml` 的 `direction_calibration.phone_to_robot` 映射到掌心目标。
- `target_response_mode=direct` 表示当前 iPhone 相对位移直接成为 raw robot target。
- `lead limit`、`lag pause`、workspace limit 是实机安全门，不应删除。
- `A3` slider 做 precision 缩放。
- `B8` 退出 shadow 节点。

## iPhone Camera / RH56 Hand

先检查 iPhone camera stream：

```bash
./scripts/check_iphone_camera_stream.sh --url http://IPHONE_IP:PORT/video
```

手部跟踪调试：

```bash
./scripts/run_iphone_hand_tracking_debug.sh --url http://IPHONE_IP:PORT/video
```

MediaPipe hand teleop：

```bash
./scripts/run_iphone_mediapipe_hand_teleop.sh --url http://IPHONE_IP:PORT/video
```

带 RH56 safety gate 的实机入口：

```bash
./scripts/run_iphone_rh56_safety_gate.sh --url http://IPHONE_IP:PORT/video --ros2
```

## Xbox / RViz

RViz 预览：

```bash
./scripts/run_jaka_rh56_rviz.sh
./scripts/run_xbox_rviz_shadow.sh
```

只发布 Xbox intent：

```bash
./scripts/run_xbox_ros2_teleop.sh
```

实机一键入口：

```bash
./scripts/run_xbox_real_arm_with_rviz_shadow.sh --hand-port /dev/ttyUSB0
```

核心安全约定：

- `RB` 是机械臂 deadman。
- `A` 打开 RH56；`RB+B` 闭合；`RB+X` pinch preset。
- 实机 bridge 负责短 horizon IK、命令超时、关节速度/加速度限制和 saturation watchdog。

## 回归测试

```bash
.venv/bin/python -m pytest \
  tests/test_xbox_ros2_teleop.py \
  tests/test_xbox_rviz_shadow.py \
  tests/test_rviz_shadow_sync.py \
  tests/test_jaka_servo_jog.py \
  tests/test_robot_bringup_ros2_bridge.py \
  tests/test_rh56_ros2_bridge.py \
  tests/test_rh56_serial_backend.py
```
