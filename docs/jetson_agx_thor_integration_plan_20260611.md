# Jetson AGX Thor 集成准备清单

日期：2026-06-11

结论先行：Jetson AGX Thor 应作为本项目的边缘计算主机，直接接相机、USB-RS485、手柄/手机网络、ROS2 节点和模型推理。JAKA 控制柜只负责机械臂控制、控制柜/末端 I/O、TIO RS485、Modbus/PROFINET/EtherNet/IP 等工业通信。除非使用 JAKA 官方视觉包或 PLC 式 I/O 联动，不建议把 RGB-D 相机接到 JAKA 控制柜。

## 推荐拓扑

```text
                    lab LAN / WiFi
                         |
                  [Gigabit/2.5G/5G switch]
                         |
        +----------------+----------------+
        |                                 |
 [Jetson AGX Thor]                 [JAKA 控制柜]
        |                                 |
        | Ethernet: robot SDK/API         | robot body
        |                                 |
        +-- USB3: RealSense / Orbbec RGB-D
        +-- USB: Xbox receiver / debug devices
        +-- USB-RS485: RH56 direct path, preferred fallback
        +-- WiFi/LAN: iPhone / HEBI Mobile I/O
        +-- optional QSFP/25GbE: future high-speed camera or sensor bridge
```

如果 RH56 走 JAKA 末端 TIO RS485：

```text
Jetson --Ethernet--> JAKA 控制柜 --robot arm TIO RS485--> RH56
```

如果 RH56 走项目默认 PC-direct/host-direct 链路：

```text
Jetson --USB-RS485 adapter--> RH56
Jetson --Ethernet--> JAKA 控制柜
```

当前项目文档和 README 仍以 PC-direct USB-RS485 为主线，`configs/hand/rh56_real.yaml` 也保留了可切换到 JAKA TIO 的配置。

## 不建议“所有设备都接 JAKA 控制柜”

JAKA 控制柜适合：

- 机械臂本体控制和安全状态。
- 固定 I/O、扩展 I/O、Modbus RTU/TCP、PROFINET、EtherNet/IP。
- 末端 TIO：少量 DI/DO/AI、可配置 12V/24V 输出、RS485 通道，用于夹爪或类似末端设备。

Jetson 适合：

- RGB-D/USB/以太网相机采集。
- ROS2 图像 topic、TF、相机标定、数据录制。
- MediaPipe、视觉模型、抓取网络、TensorRT/Isaac ROS 推理。
- 手眼标定、点云处理、训练后的策略部署。

把通用相机接到控制柜的问题是：控制柜不是 ROS2 视觉主机，带宽、驱动、相机 SDK、时间戳、GPU 推理和数据录制都会受限。JAKA 官方 Lens 视觉包是例外，它更像“控制柜内置/配套视觉功能”，适合工业流程和简单识别，不适合本项目后续多相机 RGB-D、学习策略和数据集采集主线。

## Thor 上机前要准备

### 软件基线

- Jetson AGX Thor 当前应按 NVIDIA JetPack 7 系列准备；NVIDIA 文档显示 JetPack 7 基于 Ubuntu 24.04、Linux 6.8、CUDA 13，并支持 Thor。
- 当前项目 README 以 ROS2 Humble、Python 3.10 为默认环境。Thor/JetPack 7 的自然栈更接近 Ubuntu 24.04 + ROS2 Jazzy + Python 3.12。
- 因此需要提前做一次兼容性分叉：
  - 短期：保留现有 x86/Ubuntu 22.04/Humble 开发机作为稳定实机主机。
  - Thor 到货后：优先在 Docker/venv 中验证本仓库纯 Python 层、相机层和 JAKA SDK 可用性。
  - 中期：新增 Jazzy/Python 3.12 CI 或至少本地 smoke test，减少对 Humble-only API 的依赖。

### 依赖风险

- JAKA Python SDK 当前配置指向 x86_64 路径：`configs/robot/jaka_mini2_real.yaml` 的 `python_search_paths` 是 x86_64 SDK。Thor 是 ARM64/SBSA，需要确认 JAKA 是否提供 ARM64 Linux Python SDK，或者改走 TCP/ROS2 bridge/控制柜开放接口。
- RealSense/Orbbec 需要确认对应 JetPack 7/Ubuntu 24.04/aarch64 驱动。先用单相机 USB3 验证，再上多相机。
- Isaac ROS 当前主线与 ROS2 Jazzy 绑定更紧，适合以后把视觉加速迁过去；不要一开始就把全部控制链路依赖 Isaac ROS。

### 电源和网络

- Thor 开发套件高负载可到 130W，使用官方电源，避免从机器人控制柜取电。
- JAKA 控制柜、Jetson、开发笔记本放在同一有线交换机下；机器人控制网段建议固定 IP。
- 当前真实配置里 JAKA 控制柜 IP 是 `192.168.71.50`，到货后先保持同网段或更新 `configs/robot/jaka_mini2_real.yaml`。

建议网络：

```text
Jetson eth0:       192.168.71.10/24
JAKA controller:   192.168.71.50/24
dev laptop:        192.168.71.20/24
optional WiFi:     iPhone/HEBI only, not for hard real-time robot command
```

## 设备怎么接

### JAKA mini2

- JAKA 控制柜 LAN 接到交换机。
- Jetson RJ45 接到同一交换机。
- 在 Jetson 上跑：

```bash
./scripts/check_jaka_connection.sh --config configs/robot/jaka_mini2_real.yaml
./scripts/check_jaka_zero_motion.sh --config configs/robot/jaka_mini2_real.yaml
```

### RH56

优先方案：

- RH56 电源按厂商要求供电。
- RH56 RS485 接 USB-RS485 转接器。
- USB-RS485 插 Jetson USB-A/USB-C。
- 在 Jetson 上固定 udev 名称，例如 `/dev/rh56_hand`，不要长期依赖 `/dev/ttyUSB0`。

验证：

```bash
./scripts/check_rh56_connection.sh --port /dev/rh56_hand
./scripts/rh56_pc_direct_bringup.sh --config configs/hand/rh56_real.yaml --port /dev/rh56_hand --polls 20
```

备选方案：

- RH56 接 JAKA 末端 TIO RS485。
- Jetson 只通过 JAKA 控制柜网络接口间接读写 RH56。
- 适合减少拖线，但调试复杂度更高，且 TIO 信号数量和通信模式会限制连续反馈。

### RGB-D 相机

RealSense/Orbbec 单相机：

- 直接接 Jetson USB3。
- 不接 JAKA 控制柜。
- 使用当前项目 topic 命名：

```text
/sensors/camera/color/image_raw
/sensors/camera/depth/image_raw
/sensors/camera/color/camera_info
camera_link
camera_color_optical_frame
camera_depth_optical_frame
```

验证：

```bash
python3 -m pip install -e '.[realsense]'
python3 tools/check_realsense_stream.py --duration-sec 3 --width 640 --height 480 --fps 30
```

多相机：

- 优先用独立供电 USB3 hub 或以太网相机 + 交换机。
- 再考虑 Thor 的 QSFP/25GbE 或 NVIDIA Holoscan Sensor Bridge 路线。
- 每台相机必须有固定序列号、固定 frame、固定外参文件。

### iPhone / HEBI Mobile I/O

- iPhone 与 Jetson 放同一 WiFi/LAN。
- 只用于遥操作输入或视觉 debug，不作为低延迟安全链路。
- 先验证：

```bash
./scripts/check_hebi_mobile_io.sh --duration-sec 5 --hz 10
./scripts/check_iphone_camera_stream.sh --url http://IPHONE_IP:PORT/video
```

### Xbox

- USB 接收器直接接 Jetson。
- 保留当前 RB 死人开关和 RViz shadow 流程。

## Thor 到货后 bring-up 顺序

1. 安装 JetPack 7，更新固件，确认 `nvidia-smi`、CUDA、容器运行时。
2. 固定 Jetson IP，确认能 ping 到 JAKA 控制柜。
3. 跑 JAKA 只读连接检查和零运动检查。
4. 跑 RH56 单独连接检查，不接机械臂运动。
5. 跑单相机采集，确认 USB3 带宽、帧率、时间戳。
6. 启动 ROS2 bridge，只发布状态，不发运动命令。
7. 跑 RViz shadow。
8. 最后才启用低速、短 horizon、带死人开关的实机运动。

## 需要提前补的工程项

- 新增 `docs/jetson_agx_thor_integration_plan_20260611.md`，作为 Thor 上机入口。
- 增加 ARM64/Jazzy 兼容检查脚本，避免 Thor 到货后才发现 Python/ROS2 版本问题。
- 给 RH56 USB-RS485 增加 udev 规则文档。
- 给相机配置增加 serial/frame/extrinsics 字段，不只保存 width/height/fps。
- 把 JAKA SDK 依赖从 x86_64 路径中抽象出来：至少支持 `JAKA_SDK_PYTHONPATH` 环境变量；更好是支持 ARM64 SDK 或 TCP backend。
- 规划一个 `configs/robot/jaka_mini2_thor.yaml` 和 `configs/camera/realsense_thor.yaml`，等实机 IP 和相机型号确定后落地。

## 参考来源

- NVIDIA Jetson Thor 官方规格页：<https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-thor/>
- Jetson AGX Thor Developer Kit Quick Start：<https://docs.nvidia.com/jetson/agx-thor-devkit/user-guide/latest/quick_start.html>
- Jetson AGX Thor Hardware Layout：<https://docs.nvidia.com/jetson/agx-thor-devkit/user-guide/latest/hardware_layout.html>
- NVIDIA JetPack：<https://developer.nvidia.com/embedded/jetpack>
- NVIDIA Isaac ROS Getting Started：<https://nvidia-isaac-ros.github.io/getting_started/index.html>
- JAKA 控制柜通信/TIO 设置文档：<https://www.jaka.com/docs/cobo/320/3.0/EN/guide/9.Settings.html>
