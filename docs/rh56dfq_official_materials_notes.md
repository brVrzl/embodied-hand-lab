# RH56DFQ Official Materials Notes

Date: 2026-05-04

Source folder: `RH56DFQ系列最新资料/`

This folder is vendor reference material and should stay outside git. The useful parts for this project are the 485 protocol examples, the C API finger-order comments, the ROS2 service definitions, and the URDF/manual assets. Large build outputs, generated ROS2 install/log folders, PDFs, and archives should not be copied into the repository unless a small derived note is needed.

## Key Findings

The most important control detail is the official six-channel order used by the C API:

```text
[pinky, ring, middle, index, thumb_close, thumb_lateral]
```

The project policy/data order remains:

```text
[index, middle, ring, pinky, thumb_close, thumb_lateral]
```

Therefore every PC-direct RH56 backend must convert between canonical policy order and official protocol order. Without this conversion, learned hand codes and recorded feedback would silently assign the index/middle/ring/pinky channels to the wrong physical fingers.

The current implementation records the official protocol order in `hand_schema.protocol_order` and uses canonical order for `RH56SerialBackend.read_state()`, `set_canonical_angles()`, `set_canonical_speeds()`, and `set_canonical_forces()`. Low-level `set_angles()`, `set_speeds()`, `set_forces()`, `get_angles()`, `get_forces()`, and `get_currents()` remain protocol/raw register operations for debugging.

## 485 Register Map

The vendor Python and C/C++ examples consistently expose these addresses:

| Name | Address | Use |
| --- | ---: | --- |
| `ID` | 1000 | device ID |
| `baudrate` | 1001 | baudrate setting |
| `clearErr` | 1004 | clear errors |
| `forceClb` | 1009 | force sensor calibration |
| `posSet` | 1474 | position setpoint |
| `angleSet` | 1486 | angle setpoint |
| `forceSet` | 1498 | force threshold/limit |
| `speedSet` | 1522 | speed setpoint |
| `posAct` | 1534 | actual position |
| `angleAct` | 1546 | actual angle |
| `forceAct` | 1582 | actual force |
| `current` | 1594 | motor/current feedback |
| `errCode` | 1606 | error code |
| `statusCode` | 1612 | status code |
| `temp` | 1618 | temperature |
| `actionSeq` | 2320 | action sequence index |
| `actionRun` | 2322 | action sequence run trigger |

For the active research stack, the high-value read channels are `angleAct`, `forceAct`, `current`, `errCode`, `statusCode`, and `temp`. They are enough for first-version pseudo-tactile rules: closure residual, blocked closure, empty closure, over-current/over-force, and thermal/error safety gates.

## Frame Format

The vendor 485 examples use:

- TX frame header: `EB 90`.
- RX frame header: `90 EB`.
- Read command: `0x11`.
- Write command: `0x12`.
- Checksum: low 8 bits of the sum from the ID byte through the final payload byte.
- Multi-channel values are little-endian `uint16`.

The existing `RH56SerialBackend` already matches this frame convention.

## Vendor ROS2 Package

The vendor ROS2 485 package defines services named:

```text
Setangle, Setpos, Setspeed, Setforce, Setgestureno
Getangleact, Getangleset, Getposact, Getposset, Getspeedset
Getforceact, Getforceset, Getcurrentact, Geterror, Gettemp
```

The example node hardcodes `/dev/ttyUSB0`, uses 115200 baud, and repeatedly uses `rclcpp::WallRate(10.0)` in callbacks. This is useful as a protocol reference, but it should not be the main high-frequency data path. For this project, the better ROS2 Humble design is to wrap our Python/C++ backend in a project-owned `rh56_driver_node` that publishes continuous state at 20-50 Hz and accepts canonical commands at 10-20 Hz.

## Immediate Engineering Decisions

1. Keep `RH56DFQ系列最新资料/` ignored by git.
2. Treat PC direct USB-RS485 as the main backend for experiments.
3. Publish canonical hand state to ROS2 and datasets.
4. Preserve raw/protocol feedback in logs for diagnosis.
5. Use vendor ROS2 services only as a compatibility reference, not as the main runtime interface.
6. Add `forceClb`, `actionSeq`, and `actionRun` helpers later only if experiments need force recalibration or built-in gesture playback.

## Hardware Bring-Up Implications

When the hand is connected directly to the PC, the first write test should not be a full close. Use asymmetric low-amplitude commands that identify each channel safely:

```text
canonical test sequence:
[900, 1000, 1000, 1000, 1000, 1000]
[1000, 900, 1000, 1000, 1000, 1000]
[1000, 1000, 900, 1000, 1000, 1000]
[1000, 1000, 1000, 900, 1000, 1000]
```

The expected raw protocol vectors are:

```text
[1000, 1000, 1000, 900, 1000, 1000]
[1000, 1000, 900, 1000, 1000, 1000]
[1000, 900, 1000, 1000, 1000, 1000]
[900, 1000, 1000, 1000, 1000, 1000]
```

Only after the observed physical finger motion matches canonical order should preset grasps or hand-code replay be enabled.

# 中文版本

日期：2026-05-04

资料来源文件夹：`RH56DFQ系列最新资料/`

该文件夹是厂商资料，不应直接进入 git。当前项目真正需要吸收的是 485 协议示例、C API 中的手指顺序注释、ROS2 service 定义、URDF/手册等信息。大型 build 产物、ROS2 install/log 目录、PDF、压缩包不应提交，必要时只保留我们整理出的轻量笔记。

## 关键结论

最重要的控制细节是厂商 C API 使用的六通道顺序：

```text
[小指, 无名指, 中指, 食指, 拇指弯曲, 拇指侧摆]
```

项目内部策略和数据集顺序继续使用：

```text
[食指, 中指, 无名指, 小指, 拇指弯曲, 拇指侧摆]
```

因此 RH56 PC 直连后端必须在 canonical 顺序和官方协议顺序之间显式转换。否则后续 hand-code 学习、真实反馈记录、伪触觉规则都会把食指/中指/无名指/小指对应错。

当前代码已经把官方协议顺序写入 `hand_schema.protocol_order`。`RH56SerialBackend.read_state()`、`set_canonical_angles()`、`set_canonical_speeds()`、`set_canonical_forces()` 使用 canonical 顺序；底层 `set_angles()`、`set_speeds()`、`set_forces()`、`get_angles()`、`get_forces()`、`get_currents()` 保留为协议原始顺序，方便调试。

## 485 寄存器

厂商 Python 和 C/C++ 示例中一致出现的地址如下：

| 名称 | 地址 | 用途 |
| --- | ---: | --- |
| `ID` | 1000 | 设备 ID |
| `baudrate` | 1001 | 波特率设置 |
| `clearErr` | 1004 | 清除错误 |
| `forceClb` | 1009 | 力传感器校准 |
| `posSet` | 1474 | 位置设定 |
| `angleSet` | 1486 | 角度设定 |
| `forceSet` | 1498 | 力阈值/力限制 |
| `speedSet` | 1522 | 速度设定 |
| `posAct` | 1534 | 实际位置 |
| `angleAct` | 1546 | 实际角度 |
| `forceAct` | 1582 | 实际受力 |
| `current` | 1594 | 电流反馈 |
| `errCode` | 1606 | 错误码 |
| `statusCode` | 1612 | 状态码 |
| `temp` | 1618 | 温度 |
| `actionSeq` | 2320 | 动作序列索引 |
| `actionRun` | 2322 | 动作序列运行触发 |

对当前研究最有价值的读取通道是 `angleAct`、`forceAct`、`current`、`errCode`、`statusCode`、`temp`。它们足够支撑第一版伪触觉规则：闭合残差、空抓、被阻挡、过流/过力、温度和错误安全门。

## 帧格式

厂商 485 示例使用：

- 发送帧头：`EB 90`。
- 接收帧头：`90 EB`。
- 读命令：`0x11`。
- 写命令：`0x12`。
- 校验和：从 ID 字节到最后一个 payload 字节求和后取低 8 位。
- 六通道数值为小端 `uint16`。

现有 `RH56SerialBackend` 已经符合这一帧格式。

## 厂商 ROS2 包

厂商 ROS2 485 包定义的 service 名称是：

```text
Setangle, Setpos, Setspeed, Setforce, Setgestureno
Getangleact, Getangleset, Getposact, Getposset, Getspeedset
Getforceact, Getforceset, Getcurrentact, Geterror, Gettemp
```

示例节点硬编码 `/dev/ttyUSB0`，波特率 115200，并在多个 callback 中使用 `rclcpp::WallRate(10.0)`。它适合作为协议参考，但不适合作为本项目高频数据路径。更合理的 ROS2 Humble 方案是使用我们自己的 `rh56_driver_node` 包装 PC 直连后端，连续发布 20-50 Hz 手部状态，并接受 10-20 Hz canonical 命令。

## 直接工程决策

1. `RH56DFQ系列最新资料/` 保持 git ignore。
2. 实验主路径使用 PC direct USB-RS485。
3. ROS2 和数据集发布 canonical 手部状态。
4. 日志中同时保留 raw/protocol feedback，便于诊断。
5. 厂商 ROS2 service 只作为兼容参考，不作为主运行接口。
6. `forceClb`、`actionSeq`、`actionRun` 后续按实验需要再补 helper。

## 硬件联调影响

手直连 PC 后，第一次写入测试不应直接全闭合。应使用低幅度、非对称命令逐个确认通道：

```text
canonical test sequence:
[900, 1000, 1000, 1000, 1000, 1000]
[1000, 900, 1000, 1000, 1000, 1000]
[1000, 1000, 900, 1000, 1000, 1000]
[1000, 1000, 1000, 900, 1000, 1000]
```

对应的官方协议 raw 向量应为：

```text
[1000, 1000, 1000, 900, 1000, 1000]
[1000, 1000, 900, 1000, 1000, 1000]
[1000, 900, 1000, 1000, 1000, 1000]
[900, 1000, 1000, 1000, 1000, 1000]
```

只有实际物理手指运动与 canonical 顺序一致后，才应启用 preset grasp 或 hand-code replay。
