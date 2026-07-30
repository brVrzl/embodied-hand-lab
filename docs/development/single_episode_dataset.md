# Single-episode RGB-D dataset pipeline

## Validation level and safety boundary

This page describes an offline-tested acquisition data layer. No JAKA, RH56DFX,
Quest, or RealSense device was connected while implementing or testing it. The
code does not log in to JAKA, enable servo/EDG, or command either robot.

The current normal hardware entry supports combined JAKA + RH56 teleoperation,
but `SingleEpisodeCollector` is not yet wired into that physical loop.
Consequently this page does not claim that the combined physical collector is
ready. `SingleEpisodeCollector` is the instrumentation boundary
to call from the authoritative control loop after its existing reference,
accepted-target, measured-state, and fault decisions. A future physical wiring
change must retain the existing physical acknowledgement flags and safety gates.

## Infrastructure audit (2026-07-29)

| Source | Current authority | Finding |
|---|---|---|
| Quest HTS/CTRL raw | `motion_input.recording` and live HTS writers | Available with source and host receive timestamps |
| 60 Hz arm action | `SmoothQuestJakaSession.event_records` / `AcceptedArmTarget` | Available; this is the canonical arm action source |
| 125 Hz emitted command | native cycle telemetry and simulation equivalent | Available for raw diagnostics only |
| JAKA measured state | native cycle telemetry | measured q is available; measured dq is not emitted |
| TCP pose | shared/simulation events and native startup status | measured per-cycle physical TCP is unavailable; measured-q FK must be labeled estimated |
| RH56 target/state | simulation events and PC-direct RH56 driver | combined teleoperation emits commanded/measured hand telemetry; the dataset collector is not yet attached |
| RealSense | `vision_interface.realsense_adapter` | serial selection and frameset synchronization existed; raw depth, host monotonic time, and complete profile metadata were missing |
| ACT HDF5 | archived `feature/act-thor-data-infra` only | old custom 10 Hz/RGB-only format; not reusable as canonical |
| LeRobot | not installed in the project venv | no current export existed |
| Episode metadata | archived custom JSONL recorders | useful atomic-finalize ideas, but conflicting schema/timing |
| Preview | single-camera RealSense viewer | display code reusable; no dual recording overlay |

The archived `origin/feature/act-thor-data-infra` branch is heavily diverged
from `main`. Its `src/data_recorder` and `learned_policy/act` implementations
use 10 Hz, Unix timestamps, OpenCV device resolution, RGB-only NumPy frames,
and a custom episode schema. They are not merged or copied. The durable ideas
retained here are explicit provenance, invalid/aborted outcomes, validation,
and atomic finalization.

## Episode boundary

The state machine is `IDLE -> ARMING -> REC -> FINALIZING -> DONE`:

1. `IDLE` caches preview/state only. It does not create an episode directory.
2. A trigger press enters `ARMING`. The control owner must first establish the
   Quest reference and generate an accepted target.
3. Start succeeds only when measured arm q, both fresh causal camera frames,
   a non-default RH56 hold target, and the configured arm/hand continuity gates
   pass. Only then is the monotonic episode origin created and frame 0 written.
4. `REC` uses a fixed 30 Hz clock. Missed deadlines are skipped, never replayed
   in a catch-up burst. The clock origin never changes.
5. A release finalizes the last already-complete sample and immediately stops.
   There is no release tail.
6. Camera/control faults, stale data, timestamp regression, or write errors
   finalize as `aborted`/`invalid`. A rejected start writes only a rejection
   report; it leaves no partial episode directory.

## Time alignment

Every canonical source uses `latest sample at or before canonical timestamp`.
The signed offset is `source_timestamp - canonical_timestamp`, so a valid causal
offset is zero or negative. Future samples are never selected. The initial
camera limit is one 30 Hz period; the initial control limit is 20 ms. A stale
sample aborts rather than silently copying old data.

The primary action is the causally selected shared `AcceptedArmTarget`. A 125 Hz
transport point must never be mapped to `action.arm_q_target`.

## Capture-unit layout

```text
episode-<uuid>/
  metadata.json
  validation_report.json
  raw/
    <source>.jsonl
    cameras/{workspace,wrist}/{rgb,depth_raw,depth_aligned_to_rgb}/
  canonical/
    samples.jsonl
    frames/{workspace,wrist}/{rgb,depth_raw,depth_aligned_to_rgb}/
  calibration/
  exports/
```

Capture starts in `.episode-<uuid>.partial` and is renamed only after metadata,
validation report, and canonical index are flushed. Completed, aborted, and
invalid episodes are all literal; success remains `unlabeled`.

RGB is lossless HWC `uint8` NumPy staging input. `depth_raw` is device-unit
`uint16` NumPy data. Canonical frame files are hard links to the selected raw
frames, so the two layers retain separate indexes without duplicating payloads.
Aligned depth is stored under a different name and never replaces raw depth.
Per-frame device timestamps, host monotonic timestamps, frame numbers, and
signed canonical offsets live in `samples.jsonl` and raw camera JSONL. Camera
serial, actual profile/FPS/drop count, depth scale, intrinsics, distortion,
extrinsics, and firmware belong in the camera profile metadata snapshot.

The 25-dimensional observation vector order is six measured arm q, six arm dq,
TCP `[x,y,z,qx,qy,qz,qw]`, and six hand channels. The 12-dimensional action is:

```text
[J1,J2,J3,J4,J5,J6,H1,H2,H3,H4,H5,H6]
```

Arm quantities use radians/radians per second, TCP translation uses metres, and
the quaternion order is XYZW. Every hand value carries one of `measured`,
`commanded`, `estimated`, or `unavailable`; commanded state is never called
measured.

The currently wired end-to-end command is deliberately simulation-only. It
uses the authoritative Quest/MuJoCo shared pipeline plus two physical D435s,
shows the four-panel preview, records exactly one trigger-bounded episode, and
exits. Copy the example config and replace both serial placeholders first:

```bash
.venv/bin/python tools/quest_jaka_mujoco_sim.py live-6dof \
  --episode-data-config /path/to/local_dual_d435_episode.yaml \
  --episode-root /path/to/episodes \
  --task-name TASK --operator OPERATOR
```

This command cannot connect to JAKA or RH56; its metadata marks arm/hand state
as simulated. It is the safe manual gate for trigger boundaries, real dual-D435
capture, preview shutdown, disk throughput, and export readiness. A physical
command is intentionally not documented until the existing physical RH56 path
and read-only feedback are integrated into the authoritative hardware loop.

## LeRobot v3 audit and export

The project venv contains no LeRobot, PyArrow, PyAV, or FFmpeg. The official
LeRobot `main` inspected on 2026-07-29 was commit
`f37be3edbee60f3a09a5183788b91eb19f0c07d1`, package version 0.6.1. Its current
writer API is `LeRobotDataset.create()`, `add_frame()`, `save_episode()`, then
`finalize()`. Multiple RGB cameras are independent `video` features.

Current LeRobot also supports depth video, but raw `uint16`/float depth is
quantized to 12-bit codes for video. This pipeline therefore exports RGB and
low-dimensional features through the official v3 API while copying lossless
raw depth to `depth_sidecar/`. `meta/embodied_lab_depth_sidecar.json` explicitly
sets `official_lerobot_feature=false` and maps it one-to-one by frame index. The
sidecar is not described as native LeRobot depth.

Install the optional dependencies, then export offline:

```bash
.venv/bin/python -m pip install -e ".[dataset-export]"
.venv/bin/python tools/export_episode_dataset.py EPISODE lerobot-v3 OUTPUT \
  --repo-id LOCAL_NAMESPACE/DATASET_NAME
```

Each capture unit becomes a one-episode v3 dataset. Official LeRobot merge tools
can later combine identical-feature datasets while preserving episode records.

## ACT and OpenPI compatibility

ACT export preserves standard RGB/qpos/qvel/action paths and adds lossless depth
under `/observations/depth/{workspace,wrist}`:

```bash
.venv/bin/python tools/export_episode_dataset.py EPISODE act-hdf5 OUTPUT.hdf5
```

`qpos` is `[arm_q_measured(6), hand(6)]`; `qvel` is measured/estimated arm dq
only (6), rather than fabricating unavailable hand velocity. `action` uses the
12-dimensional order above. HDF5 attributes record ordering, units, completion
status, validity, and unlabeled success.

The sibling OpenPI checkout is commit `15a9616a00943ada6c20a0f158e3adb39df2ccac`
and pins LeRobot commit `0cf8648...`, using the older `lerobot.common` namespace.
Do not assume it can open a current 0.6.1 v3 dataset without updating/pinning its
loader. Once loader compatibility is resolved, the repo-specific transform is:

| Canonical field | OpenPI input |
|---|---|
| `observation.images.workspace.rgb` | `base_0_rgb` |
| `observation.images.wrist.rgb` | `left_wrist_0_rgb` |
| absent second wrist | zero image with false mask for π0 |
| 25-D observation state | pad/map to configured model state dimension |
| 12-D absolute action | choose the configured absolute-to-delta transform for J1-J6, keep hand semantics explicit, then pad to model action dimension |

OpenPI supports the two RGB views through a base and wrist slot. Its current
example converters ignore depth, so depth is not a default model input and must
not be silently inserted.

---

# 中文版：单 episode RGB-D 数据管线

本页描述的是只完成离线测试的数据采集层。本次实现和测试没有连接 JAKA、RH56DFX、
Quest 或 RealSense，也没有执行登录、使能、EDG 或运动命令。

当前正常真机入口已支持 JAKA + RH56 联合遥操作，但 `SingleEpisodeCollector` 尚未接入
该真机循环，因此不声称联合真机采集已经可用。`SingleEpisodeCollector` 只接收
权威控制循环已经做出的 reference、accepted target、measured state 和 fault 决策；以后
接入真机时必须保留原有真机确认 flags 和安全 gate。

采集边界严格为 `IDLE -> ARMING -> REC -> FINALIZING -> DONE`。IDLE 不创建 episode；
按下 index 后先完成 reference 和首个 accepted target，再检查 measured q、两路新鲜相机、
arm/hand 启动连续性，随后才建立时间原点并写 frame 0。REC 使用 30 Hz 固定时间轴，采用
`latest sample at or before canonical timestamp`，signed offset 定义为 source 减 canonical，
禁止未来样本和旧帧静默补齐。松开 index 后不再产生新 action，无 0.3 秒尾帧，直接落盘
并退出。断相机、stale、heartbeat/fault、时间倒退或写盘失败均写成 aborted/invalid。

原始 `depth_raw` 始终以 uint16 无损保存；`depth_aligned_to_rgb` 另名保存且不覆盖 raw。
官方当前 LeRobot 0.6.1 虽支持 depth video，但存储会做 12-bit 量化，所以本项目只让 RGB
和低维字段走官方 v3，原始 depth 使用逐 frame 对齐的无损 sidecar，并在 metadata 中明确
它不是官方原生 feature。ACT 导出保持标准 RGB/qpos/qvel/action 路径，depth 放在明确的
扩展 group。OpenPI 可映射 workspace+wrist 两路 RGB，但其当前示例默认不使用 depth，且
本地 checkout 固定的是旧 LeRobot namespace，训练前必须先解决 loader 版本兼容。

当前已接通的一命令端到端入口是 `tools/quest_jaka_mujoco_sim.py live-6dof
--episode-data-config ...`：Quest 与双 D435 使用真实输入，机械臂和手只在 MuJoCo 中运行。
它用于操作者手动检查 trigger 边界、四画面预览、双相机写盘和导出，不连接或命令真机。
联合遥操作已有 RH56 feedback telemetry，但在 dataset collector 正式接入权威硬件循环前，
不提供伪装成完整真机采集的命令。
