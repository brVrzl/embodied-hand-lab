# Motion Input Platform

Status: the device-neutral input platform and Quest HTS/CTRL providers are
implemented and integrated into the current Quest/JAKA simulation and shared
target pipeline. This package describes observations, never robot commands.

```text
Quest / future device
        |
device-isolated provider
        |
      UMIP 1.0
     /   |    \
record replay diagnostics/visualization
        |
explicit teleoperation consumer
```

## Components

- `src/motion_input/model.py`: immutable UMIP values and invariants.
- `src/motion_input/provider.py`: common live/replay lifecycle.
- `src/motion_input/quest.py`: Quest wire parser, UDP source, UMIP translator.
- `src/motion_input/hts_protocol.py`: strict HTS v1.1 CSV schema.
- `src/motion_input/hts_transport.py`: input-only UDP transport and raw replay.
- `src/motion_input/hts_canonical.py`: Unity/OpenXR canonical conversion.
- `src/motion_input/controller_provider.py`: CTRL v1 validation and freshness.
- `src/motion_input/recording.py`, `replay.py`, `diagnostics.py`, and
  `visualization.py`: device-neutral support.
- `integrations/quest_unity/`: input-only Unity publisher sources.

Current references:

- [UMIP observation contract](UMIP_PROTOCOL.md)
- [coordinate-frame contract](COORDINATE_FRAMES.md)
- [Quest controller host transport](QUEST_CONTROLLER_TRANSPORT_HOST.md)
- [Quest SDK/OpenXR review](QUEST_SDK_REVIEW.md)
- [current Quest host setup](../operation/quest_setup.md)

Dated repository audits, streamer integration gates, offline simulation gates,
and the initial dual-clutch design are preserved under
`docs/history/archived_designs/motion_input/`. They no longer define current
branches, local paths, test totals, or integration status.

## Safe usage

Inspect the input-only tools:

```bash
.venv/bin/python tools/umip_motion_input.py --help
.venv/bin/python tools/quest_hand_tracking_streamer.py --help
```

Recordings default under ignored `data/` paths and may contain personal motion
data. Select any sender address and output path for the current trusted network;
do not copy old example addresses as operational truth.

## Focused validation

```bash
.venv/bin/python -m pytest -q \
  tests/test_motion_input_protocol.py \
  tests/test_motion_input_recording_replay.py \
  tests/test_quest_motion_provider.py \
  tests/test_hts_protocol.py \
  tests/test_hts_canonical.py \
  tests/test_quest_controller_transport.py \
  tests/test_motion_input_diagnostics.py
```

---

# 中文版：运动输入平台

状态：设备无关输入平台和 Quest HTS/CTRL provider 已实现，并接入当前 Quest/JAKA 仿真
和共享目标管线。此 package 只描述 observation，不描述机器人命令。

```text
Quest / 未来设备
        |
设备隔离 provider
        |
      UMIP 1.0
     /   |    \
记录  回放  诊断/可视化
        |
显式 teleoperation consumer
```

## 组件

- `src/motion_input/model.py`：不可变 UMIP 值和 invariant。
- `src/motion_input/provider.py`：live/replay 生命周期。
- `src/motion_input/quest.py`：Quest wire parser、UDP source、UMIP translator。
- `src/motion_input/hts_protocol.py`：严格 HTS v1.1 CSV schema。
- `src/motion_input/hts_transport.py`：仅输入 UDP 和原始回放。
- `src/motion_input/hts_canonical.py`：Unity/OpenXR 规范化。
- `src/motion_input/controller_provider.py`：CTRL v1 验证和新鲜度。
- `recording.py`、`replay.py`、`diagnostics.py`、`visualization.py`：设备无关工具。
- `integrations/quest_unity/`：只发送输入的 Unity 源码。

参考文档：

- [UMIP observation contract](UMIP_PROTOCOL.md)
- [坐标契约](COORDINATE_FRAMES.md)
- [Quest 控制器传输](QUEST_CONTROLLER_TRANSPORT_HOST.md)
- [Quest SDK/OpenXR 审计](QUEST_SDK_REVIEW.md)
- [当前 Quest 主机设置](../operation/quest_setup.md)

历史输入审计和 gate 位于 `docs/history/archived_designs/motion_input/`，不再定义当前 branch、
路径、测试数量或集成状态。

## 安全使用

```bash
.venv/bin/python tools/umip_motion_input.py --help
.venv/bin/python tools/quest_hand_tracking_streamer.py --help
```

记录默认位于忽略的 `data/`，可能包含个人动作数据。为当前可信网络明确选择 sender 和输出
路径，不把旧示例地址当作运行真值。

## 聚焦验证

```bash
.venv/bin/python -m pytest -q \
  tests/test_motion_input_protocol.py \
  tests/test_motion_input_recording_replay.py \
  tests/test_quest_motion_provider.py \
  tests/test_hts_protocol.py \
  tests/test_hts_canonical.py \
  tests/test_quest_controller_transport.py \
  tests/test_motion_input_diagnostics.py
```
