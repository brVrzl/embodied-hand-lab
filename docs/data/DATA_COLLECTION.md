# Physical episode collection entry

This is the one maintained operator entry for collecting physical episodes. It
combines live arm/hand teleoperation with recording; there is no separate
normal physical-teleoperation entry. Read the
[current status](../status/current_status.md), [real-hardware safety](../safety/REAL_HARDWARE_SAFETY.md),
and [combined teleoperation procedure](../operation/jaka_rh56_combined_teleop.md)
before opening any device. Documentation, `--help`, and offline validation do
not authorize a physical run.

## Prepare the unified collection configuration

```bash
${EDITOR:-vi} configs/data_collection/physical_collection.yaml
```

The `workspace` and `wrist` serials must be different and must match their
physical viewpoints. Camera roles are selected by explicit serial, never by
`/dev/video*` order. This page is the single authority for the collection
schema, staleness semantics, camera preflight, review, and conversion rules.

Configure host/device identity once in that YAML. Do not export robot, RH56,
camera, episode, or native-CPU values from `.bashrc`; the collection command
reads them from this file.

The runtime setting `enforce_clutch_target_displacement_limit: false` is
intentional: the live-demo YAML keeps its 0.20 m target envelope for
simulation/replay, but collection does not treat it as a per-clutch task-travel
limit. IK, joint-limit, singularity, collision, output, controller, timing,
liveness, and operator workspace checks remain active.

## Maintained collection entry

Use the production wrapper below. Host/device values and the verified native
control CPU are stored once in the ignored local runtime config; the command
does not require per-run substitution:

```bash
./scripts/run_quest_jaka_rh56_teleop.sh \
  --runtime-config configs/data_collection/physical_collection.yaml
```

The runtime config sets the bounded 300-second run, operator `01`, camera
episode config, robot/RH56 identity, native CPU, all-J1--J6 1.5 rad/s limits,
log directory, acceleration-transition recovery, and no preview. Set
`runtime.episode_preview: true` in the YAML only when preview is intentionally
needed; preview is not a required consumer. Normal J1--J6 run velocity is 1.5
rad/s.
This is a project-selected operating value, not a manufacturer maximum, and
all shared IK, collision, singularity, branch-continuity, joint-limit,
acceleration, timing, liveness, native-worker, and controller safety gates
remain active.

`ARM_CLUTCH` is not a separate legacy mode. During combined collection,
releasing the left index clutch places only the arm in bounded hold; pressing
again resumes after the existing fresh-reference rules. The hand grip clutch
controls the hand independently. The arm-only bounded wrapper remains only as
an isolation diagnostic and is not a competing collection entry.

Before starting, verify the JAKA controller state, payload/TCP/install state,
E-stop access, clear workspace, Quest boundary, RH56 prerequisites, camera
identity, free storage, and that no other control client is running. The
native-control CPU must be reserved for the native worker; camera, Python
control, RH56, recorder, and preview processes must not use it.

## What is written by the maintained physical entry

The tracked dual-D435 example selects `lerobot_staging_v1`. Collection writes
reviewable staging data only; it does not import PyArrow or create Parquet in
the live recorder:

```text
raw_episodes/
  meta/info.json
  meta/tasks.jsonl
  meta/episodes.jsonl
  meta/episodes/chunk-000/episode_000000.json
  data/chunk-000/episode_000000.jsonl
  videos/observation.images.workspace/chunk-000/episode_000000.mp4
  videos/observation.images.wrist/chunk-000/episode_000000.mp4
  audit/chunk-000/episode_000000/  # optional JAKA/RH56 records
```

`episode_000000.jsonl` is the aligned 30 Hz robot table. Row `i` and decoded
video frame `i` share the same episode sample. Its columns include
`frame_index`, `timestamp_ns`, `observation.state` (12 values), and `action`
(12 values):

- `observation.state[0:6]`: measured JAKA joint position in radians;
- `observation.state[6:12]`: measured RH56 six-channel normalized state;
- `action[0:6]`: accepted arm joint target sent to the adapter;
- `action[6:12]`: accepted RH56 target in normalized units.

TCP, Quest packets/events, and depth are deliberately not part of this
training view. Quest remains a live control input, but the maintained control
flow has no Quest recording sink and the episode writer rejects Quest raw
streams.
TCP can be derived offline from a reviewed model/calibration if later needed.
The RealSense workers run RGB-only (`capture_depth: false`), so no depth stream
or depth-sized payload is read or persisted.

Camera frames are still produced in independent processes and selected through
the bounded shared-memory rings; selection and video/JSONL writing remain
outside the control tick. An expired reference or dropped slot is represented
in the staging quality JSONL/metadata and never replaced by a newer image or
fabricated timestamp. Queues are bounded and preview cannot backpressure
capture or control.

Recording or camera failure invalidates or stops recording according to the
episode quality policy; it must not be converted into a healthy-robot
emergency stop. JAKA controller, native timing, liveness, collision, tracking,
or RH56 safety faults retain their normal hard-stop path. Do not retry a failed
run automatically; preserve its summary and episode state for diagnosis.

When both valid clutches are released continuously for five seconds, the
recorder finalizes that episode and opens the next numbered episode without
ending robot control. A new clutch press starts the next episode from fresh
state. The staging review page and approval/conversion commands are:

```bash
.venv/bin/embodied-lab dataset review-staging data/raw_episodes episode_000000
.venv/bin/embodied-lab dataset approve-staging data/raw_episodes episode_000000 \
  --status approved --notes "reviewed RGB and task outcome"
.venv/bin/embodied-lab dataset convert-staging data/raw_episodes episode_000000 \
  data/lerobot_dataset
```

The old canonical v1/v2 archive and `raw_episode_v1` writer remain available
for simulation and offline compatibility. They are not the default physical
collection format.

## Validate after collection

Use the episode index reported by the run summary and review it offline:

```bash
.venv/bin/embodied-lab dataset review-staging \
  /home/thor/projects/raw_episodes episode_000000
.venv/bin/embodied-lab dataset approve-staging \
  /home/thor/projects/raw_episodes episode_000000 --status approved
.venv/bin/embodied-lab dataset convert-staging \
  /home/thor/projects/raw_episodes episode_000000 data/lerobot_dataset
```

Only completed, human-approved episodes should be converted. Check that the
JSONL row count equals both MP4 frame counts and that `timestamp_ns` is strictly
increasing before conversion. The old `dataset validate`/`inspect` commands
remain for canonical and `raw_episode_v1` archives.

## Collection quality boundary

The recorder samples causally: a canonical row may select the newest source
frame at or before its timestamp, never a future frame. Stale frames, ring
reference expiry, queue drops, and preview lag are recorded as data-quality
metadata. They do not stop healthy JAKA/RH56 control. Persistent camera
acquisition failure may stop recording, while controller alarms, native timing
faults, liveness loss, tracking divergence, and RH56 fatal faults retain their
control stop behavior.

The live collection configuration keeps camera capture RGB-only. Depth and TCP
are not part of the maintained training view; TCP can be derived offline from a
reviewed model and calibration if a later dataset requires it. Quest packets
remain live input and are not copied into the episode.

The writer is deliberately review-first. A completed episode is not the same
as a successful task, and a structurally valid episode is not physical
calibration evidence. Keep rejected or degraded episodes for diagnosis, but do
not place them in a training manifest without an explicit policy.

---

# 中文版：物理 episode 采集入口

这是当前唯一维护的物理 episode 采集入口。它把 arm/hand 联合遥操作和 recording 放在同一
流程中，不再区分一个独立的普通真机遥操作入口。打开设备前必须阅读[当前状态](../status/current_status.md)、
[真机安全](../safety/REAL_HARDWARE_SAFETY.md)和[联合遥操作流程](../operation/jaka_rh56_combined_teleop.md)。
文档、`--help`、离线测试和仿真都不构成真机授权。

## 配置与启动

编辑唯一的运行配置：

```bash
${EDITOR:-vi} configs/data_collection/physical_collection.yaml
```

`workspace` 和 `wrist` serial 必须不同，并且要与实际视角一致。相机角色只按显式 serial
选择，不按 `/dev/video*` 顺序推断。本页是采集 schema、stale 语义、相机 preflight、review
和转换规则的唯一权威页。

当前维护入口：

```bash
./scripts/run_quest_jaka_rh56_teleop.sh \
  --runtime-config configs/data_collection/physical_collection.yaml
```

JAKA controller 状态、payload/TCP/install、E-stop、workspace、Quest boundary、RH56 前置条件、
相机身份、磁盘空间和 native control CPU 必须在真机 gate 中单独确认。不要通过 `.bashrc` 隐式
覆盖 robot、RH56、camera、episode 或 native CPU 值。preview 只是可选 review 功能，不能阻塞
recorder 或 control。

`runtime.enforce_clutch_target_displacement_limit: false` 是有意的采集策略：live-demo 配置仍保留
`maximum_target_displacement_m: 0.20` 供仿真/回放使用，但采集流程不把它当作单次 clutch 的任务行程
上限。IK、joint-limit、奇异性、碰撞、输出、controller、时序、liveness 及操作者 workspace 确认仍然有效。

释放左 index clutch 只会让 arm 进入 bounded hold；重新按下时按现有 fresh-reference 规则恢复。
hand grip clutch 独立控制 hand。

## 写入内容与对齐

采集阶段只写 review-first staging 数据，不在 live recorder 中生成 Parquet：

```text
raw_episodes/
  meta/info.json
  meta/tasks.jsonl
  meta/episodes.jsonl
  meta/episodes/chunk-000/episode_000000.json
  data/chunk-000/episode_000000.jsonl
  videos/observation.images.workspace/chunk-000/episode_000000.mp4
  videos/observation.images.wrist/chunk-000/episode_000000.mp4
  audit/chunk-000/episode_000000/       # 可选 JAKA/RH56 审计记录
```

`episode_000000.jsonl` 是对齐的 30 Hz 机器人表。第 `i` 行与两个视频解码出的第 `i` 帧属于同一个
episode sample。核心字段是 `frame_index`、`timestamp_ns`、`observation.state` 和 `action`：

- `observation.state[0:6]`：实测 JAKA 六关节弧度；
- `observation.state[6:12]`：实测 RH56 六通道归一化状态；
- `action[0:6]`：发送给 arm adapter 的 accepted arm joint target；
- `action[6:12]`：accepted RH56 normalized target。

默认训练视图不记录 Quest packet/event、TCP 和 depth。TCP 可以基于审核过的模型和标定离线计算；
当前 RealSense 采集配置关闭 depth，只保存 RGB MP4。stale frame、ring overwrite、recorder
queue drop 和 preview lag 记录为数据质量事件，不应停止健康的 robot control；持续的相机采集或
writer 故障可以停止 recording，但不能自动把健康机器人升级为 emergency stop。

## Episode 分割、review 和转换

两种有效 clutch 连续释放五秒后，当前 episode finalize，recorder 在同一进程中打开下一个递增
编号。下一次 press 从 fresh state 开始，frame index、canonical clock、writer handles 和 quality
counters 都重新开始。Ctrl+C 或真机 hard fault 只影响最后一个未完成 episode；partial episode 不得
加入 manifest 或导出。

采集后先生成 review 页面并人工确认：

```bash
.venv/bin/embodied-lab dataset review-staging data/raw_episodes episode_000000
.venv/bin/embodied-lab dataset approve-staging data/raw_episodes episode_000000 \
  --status approved --notes "reviewed RGB and task outcome"
.venv/bin/embodied-lab dataset convert-staging data/raw_episodes episode_000000 \
  data/lerobot_dataset
```

只有 `completed`、JSONL 非空且 `timestamp_ns` 严格递增、两路 MP4 帧数与 JSONL 行数一致、质量
信息可接受且人工标记为 approved 的 episode 才能转换。采集不直接生成 Parquet；转换只处理已经
确认的 staging episode。离线、仿真和 review 结果都不能替代真机 PASS。

## 采集质量边界

recorder 采用因果采样：canonical row 只能选择时间戳不晚于自身的最新 source frame，绝不选择未来
帧。stale frame、ring reference 过期、queue drop 和 preview lag 会写入数据质量 metadata，不会
停止健康的 JAKA/RH56 控制。持续相机采集失败可以只停止 recording；controller alarm、native timing
fault、liveness loss、tracking divergence 和 RH56 fatal fault 仍按控制安全规则停止。

当前 live collection 配置只保存 RGB。Depth 和 TCP 不属于维护的训练视图；如果后续需要 TCP，可
基于审核过的模型和标定离线计算。Quest packet 是 live input，不复制到 episode。

writer 采用 review-first 语义。episode 完成不等于任务成功，结构有效也不等于真机标定已经通过。
拒绝或 degraded episode 可保留用于诊断，但没有明确策略时不得放进训练 manifest。
