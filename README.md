# Embodied Lab

面向 `JAKA mini2 + Inspire RH56` 的真实机器人遥操作、数据采集和灵巧抓取研究仓库。

当前原则：保留已经调通的实机链路，删除早期探索材料时不得影响遥操作、硬件 bring-up、ROS2 bridge、RViz shadow、RH56 PC direct 和支撑 IK/预览的 MuJoCo 资产。

## 保留主线

- `iPhone / HEBI Mobile I/O`：已调通的手机 ARKit 相对位姿遥操作链路，必须保留。
- `Xbox / RViz shadow`：手柄遥操作、实机命令镜像和 RViz 预览链路，必须保留。
- `JAKA mini2`：连接检查、preset、EDG servo 安全桥、掌心目标 IK。
- `Inspire RH56`：PC direct USB-RS485 主链路、ROS2 JSON bridge、基础 safety gate。
- `MuJoCo JAKA+RH56 asset`：`data/sim_assets/jaka_rh56.xml` 是遥操作 IK/RViz shadow 的依赖，必须保留。
- `文献与实验计划`：只保留当前论文主线和近期可信文献索引，旧探索材料不再作为入口。

## 快速环境

```bash
cd /home/w/projects/embodied_lab
/usr/bin/python3.10 -m venv .venv
source .venv/bin/activate
pip install -e ".[dev]"
```

可选依赖：

```bash
pip install -e ".[gamepad]"        # Xbox
pip install -e ".[phone-teleop]"   # HEBI Mobile I/O
pip install -e ".[vision-teleop]"  # iPhone camera + MediaPipe hand tracking
```

## 实机检查

```bash
./scripts/check_jaka_connection.sh --ip 192.168.1.100
./scripts/check_jaka_zero_motion.sh --ip 192.168.1.100
./scripts/check_jaka_edg_servo_capability.sh --config configs/robot/jaka_mini2_real.yaml
./scripts/check_rh56_connection.sh --port /dev/ttyUSB0
./scripts/rh56_pc_direct_bringup.sh --config configs/hand/rh56_real.yaml --port /dev/ttyUSB0 --polls 20
```

默认检查命令只读状态或做无运动检查；只有脚本参数显式带 `--execute` / `--enable-*` 时才会下发运动或手指命令。

## iPhone / HEBI 遥操作

HEBI Mobile I/O 是已调通链路。iPhone 与 PC 需在同一 WiFi；HEBI app 中 `Family=HEBI`、`Name=mobileIO`，并允许相机权限。

```bash
./scripts/check_hebi_mobile_io.sh --duration-sec 5 --hz 10
./scripts/record_hebi_mobile_io.sh --duration-sec 20
./scripts/run_hebi_rviz_shadow.sh
```

实机 arm-only 入口：

```bash
./scripts/run_real_jaka_hebi_arm_teleop.sh \
  --enable-motion \
  --teleop-mode relative_pose_lag_follow \
  --teleop-profile practical \
  --jsonl-out logs/teleop/hebi_real_arm_$(date +%Y%m%d_%H%M%S).jsonl
```

MediaPipe 手部跟踪入口：

```bash
./scripts/check_iphone_camera_stream.sh --url http://IPHONE_IP:PORT/video
./scripts/run_iphone_mediapipe_hand_teleop.sh --url http://IPHONE_IP:PORT/video
./scripts/run_iphone_rh56_safety_gate.sh --url http://IPHONE_IP:PORT/video --ros2
```

## Xbox / RViz 遥操作

```bash
./scripts/run_jaka_rh56_rviz.sh
./scripts/run_xbox_rviz_shadow.sh
./scripts/run_xbox_ros2_teleop.sh
```

实机推荐从一键入口开始，它会启动 real bridge、Xbox intent publisher、RViz 和 shadow mirror：

```bash
./scripts/run_xbox_real_arm_with_rviz_shadow.sh --hand-port /dev/ttyUSB0
```

核心安全约定：`RB` 是机械臂死人开关；短 horizon TCP velocity IK、关节限幅、命令超时和 saturation watchdog 不应删除。

## 结构

```text
configs/              当前硬件、手、遥操作、RViz 配置
data/sim_assets/      遥操作 IK 和预览需要的 JAKA+RH56 MuJoCo 资产
docs/                 当前文档入口和近期可信文献索引
scripts/              人直接运行的入口
src/                  Python 包源码
tests/                保留链路的回归测试
tools/                scripts 调用的实现工具
```

关键模块：

- `src/teleop_tools`：Xbox、iPhone/HEBI、RViz shadow、相对位姿跟随。
- `src/jaka_driver_adapter`：JAKA SDK/mock、掌心目标 IK、servo jog 安全解析。
- `src/rh56_driver`：RH56 schema、serial backend、ROS2 bridge。
- `src/robot_bringup`：实机 ROS2 bridge 和 RViz joint-state bridge。
- `src/sim_maniskill`：仅保留必要仿真/预览支持，不作为当前实机抓取成功率证据。

## 验证

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

当前项目精简后，新增或删除内容前先确认上述遥操作回归测试仍通过。
