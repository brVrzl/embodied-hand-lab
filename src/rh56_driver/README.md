# rh56_driver

Inspire RH56 最小驱动层，当前以“高级夹爪模式”优先。

已提供：

- `open`
- `close`
- `pinch`
- `preset grasp`
- 读手指状态
- force/contact 占位输出
- mock backend
- 基于本地 RH56 协议资料整理的 `serial_protocol` real backend
- 基于 JAKA TIO 的 `jaka_tool_rs485` real backend
- 官方 ROS2 service 命名约定配置
- `ros2_bridge.py`：mock-first JSON ROS2 bridge，用于先固定 topic 语义

真实设备接入说明：

- 通讯默认按 `RS485/serial` 抽象
- `mock_backend.py` 仅用于 bring-up 和 recorder 联调
- `serial_backend.py` 基于 `third_party/inspire_hand/rh56/rh56.py` 与 `inspire_hand_485_ros2/service_interfaces/*.srv` 整理
- `jaka_tool_backend.py` 用于 RH56 通过 JAKA 末端航插口接入：临时切 raw RS485 发送控制帧，切回 Modbus RTU 后重建 6 个角度信号量读取真实反馈
- 若你最终采用厂家 ROS2 工作空间，可把 `backend_type` 改为 `ros2_services` 并补齐对应 client

ROS2 JSON bridge:

- 入口：`./scripts/run_rh56_ros2_json_bridge.sh --config configs/hand/rh56.yaml --state-hz 20`
- 发布：
  - `/hand/state`
  - `/hand/raw_feedback`
  - `/hand/backend_mode`
- 订阅：
  - `/hand/command_angles`
  - `/hand/command_code`
  - `/hand/command_force`
- 所有 payload 暂时使用 `std_msgs/String` JSON。
- `/hand/command_angles` 只接受 canonical `rh56_angle_raw_0_1000`，不隐式接受 normalized command。

JAKA 工具端 RS485 说明：

- 当前仓库支持 `backend_type: jaka_tool_rs485`
- 控制链路基于 JAKA TIO raw RS485：
  - `set_tio_vout_param`
  - `set_tio_pin_mode`
  - `send_tio_rs_command`
- 反馈链路基于 JAKA TCP/IP JSON 的 `get_rs485_signal_list`
- JAKA TIO RS485 信号量最多支持 8 个；当前生产配置只常驻 6 个角度反馈：`rh56_angle_0..5`
- 角度反馈地址必须按 16-bit stride=2 添加：`1546,1548,1550,1552,1554,1556`
- Python SDK 的 `send_tio_rs_command()` 只能确认命令被控制器接受，不能返回 RH56 串口读回；真实角度反馈来自 JAKA 控制器侧 Modbus 信号量缓存
- 通道模式会在 raw 控制与 Modbus 反馈之间切换；切回 Modbus RTU 后必须重建 `rh56_angle_0..5`，后端会自动执行这一步
- 你的两条测试帧：
  - 全张开：`EB 90 01 0F 12 CE 05 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 77`
  - 全弯曲：`EB 90 01 0F 12 CE 05 00 00 00 00 00 00 00 00 00 00 00 00 F5`
  当前都对应 `hand_id = 1`，并且已和 App 中可读的 TIO Modbus 信号量配置对齐。

本地参考资料：

- `third_party/inspire_hand/rh56/rh56.py`
- `third_party/inspire_hand/rh56/readme.md`
- `src/rh56_driver/srv`
