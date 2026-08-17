# Unified Motion Input Protocol (UMIP) 1.0

## English

UMIP is the device-independent observation contract used by the current
motion-input package. It describes observations, not robot commands, targets,
trajectories, filtering, safety, inverse kinematics, or scaling.

The dependency boundary is:

```text
device SDK -> provider-private value -> MotionInputSample
                                      -> recorder
                                      -> replay provider
                                      -> visualization/diagnostics
```

Providers implement `MotionInputProvider` with `descriptor`, `open`,
`read(timeout_s)`, and `close`. `read` returns one immutable
`MotionInputSample` or `None` on timeout. Replay uses the same interface and
preserves recorded identity and timestamps while changing only delivery
timing.

Each sample carries a protocol version, sample and stream identity,
non-negative per-stream sequence number, capture/device/receive timestamps,
tracking state, optional confidence, an explicit coordinate-frame ID, device
descriptor, side, optional wrist/palm poses, motion kind, articulation, and
namespaced metadata/extensions. `Pose6D` uses meters and an `x,y,z,w` unit
quaternion; UMIP rejects non-finite values and non-unit quaternions instead of
normalizing them. Timestamps may be subtracted only when their `clock_id`
matches.

`tracking` requires a wrist pose. `not_tracking` and `disconnected` contain no
wrist or palm pose, while `limited` may carry a degraded but valid pose.
Providers preserve source order and do not reorder, smooth, extrapolate, or
generate observations. Sequence gaps and out-of-order samples are diagnostic
events.

Recordings use UTF-8 `.umip.jsonl`: one header, zero or more sample records,
and an optional footer. A missing footer means interrupted/unfinalized capture,
not automatic corruption. Device SDK objects, native handles, and engine
transforms do not enter serialized UMIP.

Use the maintained entrypoint for offline record/replay/diagnostics:

```bash
PYTHONPATH=src .venv/bin/python tools/umip_motion_input.py --help
```

World registration, calibration, filtering, scaling, safety, IK, target
generation, and command behavior remain downstream teleoperation concerns.

## 中文

UMIP 是当前 motion-input package 使用的设备无关 observation 契约。它描述 observation，不描述机器人
command、target、trajectory、filter、safety、inverse kinematics 或 scaling。

依赖边界为：

```text
device SDK -> provider 私有值 -> MotionInputSample
                                      -> recorder
                                      -> replay provider
                                      -> visualization/diagnostics
```

Provider 实现 `MotionInputProvider` 的 `descriptor`、`open`、`read(timeout_s)` 和 `close`。`read` 返回
一个不可变的 `MotionInputSample`，timeout 时返回 `None`。Replay 使用相同接口，保留记录中的 identity 和
timestamp，只改变 delivery timing。

每个 sample 包含 protocol version、sample/stream identity、每个 stream 的非负 sequence number、
capture/device/receive timestamp、tracking state、可选 confidence、显式 coordinate-frame ID、device
descriptor、side、可选 wrist/palm pose、motion kind、articulation 以及 namespaced metadata/extensions。
`Pose6D` 使用米和 `x,y,z,w` 顺序的 unit quaternion；UMIP 拒绝非 finite 值和非 unit quaternion，不自动
normalize。只有 `clock_id` 相同时 timestamp 才能相减。

`tracking` 必须有 wrist pose；`not_tracking` 和 `disconnected` 不得包含 wrist/palm pose；`limited` 可以
携带降级但有效的 pose。Provider 保持 source order，不 reorder、smooth、extrapolate 或生成 observation。
Sequence gap 和 out-of-order sample 只作为诊断事件。

记录使用 UTF-8 `.umip.jsonl`：一个 header、零个或多个 sample record 和可选 footer。缺失 footer 表示采集
中断或未 finalize，不自动等同于损坏。Device SDK object、native handle 和 engine transform 不进入序列化
UMIP。

离线 record/replay/diagnostics 使用当前维护入口：

```bash
PYTHONPATH=src .venv/bin/python tools/umip_motion_input.py --help
```

World registration、calibration、filter、scaling、safety、IK、target generation 和 command behavior
仍属于下游 teleoperation 边界。
