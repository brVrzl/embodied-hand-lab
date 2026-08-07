# Staged manual functional validation

This runbook was checked against the maintained command-line parsers, wrapper
scripts, configuration files, and safety documentation on 2026-07-31. It does
does not open or command hardware. Run the levels in order; a failure at one level
blocks every later level. Commands containing `<...>` are templates and must
not be pasted until every placeholder has been replaced from the current
configuration, controller, device identity, or vendor documentation.

Repository root for all commands:

```bash
cd /home/thor/projects/embodied_lab
```

The only implemented canonical episode producer combines real Quest and two
real D435 streams with a MuJoCo arm/hand. There is no maintained canonical
episode producer for the physical JAKA and RH56. Physical object-task
benchmarks are also not implemented. Those boundaries are called out below
instead of being replaced with invented commands.

## Universal stop rules

- Keep the operator outside the robot workspace and continuously within reach
  of the physical E-stop during every operator-initiated motion gate.
- Do not run two JAKA clients or two RH56 clients. Before each physical gate,
  run the residual-process check in [Stopping and recovery](#stopping-and-recovery).
- `Ctrl+C` is the normal software stop. A collision, alarm, tracking/timing
  hard fault, unexpected direction, unexpected joint, cable contact, person
  entering the workspace, loss of visibility, or unavailable E-stop requires
  immediate physical E-stop and no automatic retry.
- Never turn a fake/replay/simulation PASS into a physical PASS.

## Level 0 — environment and offline baseline

**Machine:** Jetson Thor or another supported Linux development host.
**Hardware state:** JAKA, RH56, Quest, and RealSense disconnected or unused.
**Outputs:** `build/validation/`; pytest and native build products remain under
ignored `build/`.

```bash
cd /home/thor/projects/embodied_lab
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -e ".[dev]"
.venv/bin/python -m pip check

mkdir -p build/validation
.venv/bin/embodied-lab doctor \
  --output build/validation/environment.json

PYTHONDONTWRITEBYTECODE=1 \
  .venv/bin/python -m compileall -q src tools tests
PYTHONDONTWRITEBYTECODE=1 \
  .venv/bin/python -m pytest -q -p no:cacheprovider

cmake -S native/jaka_servo_worker \
  -B build/jaka_servo_worker -DCMAKE_BUILD_TYPE=Release
cmake --build build/jaka_servo_worker -j2

.venv/bin/embodied-lab sim smoke
```

Pass criteria:

- `pip check`, compileall, CMake builds, and pytest exit zero; pytest has no
  failed or error outcome. Skips must be reviewed, not counted as passes.
- Doctor reports `ready_offline`, parses every maintained YAML, and records
  `device_connections_attempted=false` and `robot_commands_sent=false`.
- `native/jaka_servo_worker` currently has no CTest entries; its criterion is a
  successful build of the resampler and, on supported Linux, the SDK-linked
  worker.
- Sim smoke reports finite state, a loaded model, and bounded drift.
- Benchmark reports `status=passed`; inspect the JSON at
  `build/validation/benchmark.json` rather than accepting only exit status.

Failure evidence is in the terminal, `build/validation/environment.json`,
`build/validation/benchmark.json`, and the relevant CMake directory. Preserve
it before rebuilding.

Recoverable cleanup moves validation products aside instead of deleting them:

```bash
VALIDATION_ARCHIVE="/tmp/embodied-lab-validation-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$VALIDATION_ARCHIVE"
test ! -e build/validation || mv build/validation "$VALIDATION_ARCHIVE/"
```

## Level 1 — simulation manual validation

### 1. Model viewer and scene

```bash
.venv/bin/python tools/debug_mujoco_jaka_rh56_viewer.py \
  --scenario table_cube \
  --collision-mode visual_coacd \
  --duration 30 \
  --viewer \
  --out-xml build/validation/table_cube.xml
```

Expected screen: one upright JAKA/RH56 scene with the table and cube; no NaN,
exploding body, missing mesh, or unexpected initial penetration. Close the
viewer or press `Ctrl+C`. The generated XML is
`build/validation/table_cube.xml`.

### 2. RH56 six-actuator simulation

```bash
.venv/bin/python tools/rh56_h0_self_test.py \
  --viewer \
  --cycle-seconds 4 \
  --amplitude-scale 0.25 \
  --repeat 1 \
  --log-path build/validation/rh56_h0.jsonl
```

Expected screen/log: all six configured actuators move smoothly through the
H0 sequence, remain inside model/control intersections, and return without
NaN. A wrong actuator order, discontinuity, model penetration, or nonzero exit
is a failure. Stop with `Ctrl+C` or close the viewer.

### 3. Quest, JAKA joints, RH56, and combined simulation

First inspect the exact wrapper help:

```bash
./scripts/run_quest_jaka_sim_demo.sh --help
```

Then, on a graphical host with the Quest sender configured for the selected
host/port:

```bash
RUN_NAME="manual_sim_$(date +%Y%m%d_%H%M%S)"
./scripts/run_quest_jaka_sim_demo.sh \
  --project-ip <QUEST_FACING_HOST_IPV4> \
  --allowed-sender <QUEST_IPV4> \
  --display <DISPLAY> \
  --duration-sec 120 \
  --arm-output-mode shaped-500hz \
  --output "logs/quest_jaka_sim/${RUN_NAME}.hts.jsonl" \
  --events "logs/quest_jaka_sim/${RUN_NAME}.events.jsonl" \
  --report "logs/quest_jaka_sim/${RUN_NAME}.report.json"
```

Recommended operator order:

1. Start with left index and grip released.
2. Press/release left index while the hand is still; confirm J1--J6 move only
   after reference capture and hold immediately on release.
3. Keep index released; press grip and exercise the six simulated RH56
   channels. Confirm the arm remains still.
4. Engage both only after the isolated checks pass.
5. Approach an ordinary workspace/IK boundary slowly. The display/log must
   show `HOLD_REJECTED` and preserve the last accepted state; do not try to
   force through the boundary.
6. Release both controls, then stop with `Ctrl+C` or close the viewer.

Pass criteria: correct coordinate direction, no first-engagement jump, arm and
hand clutch isolation, finite state, bounded commands, fresh telemetry, and a
report with `simulation_only=true`. Logs are under `logs/quest_jaka_sim/`.

Repeat with the production-equivalent 125 Hz arm adapter only after the native
resampler build passes:

```bash
RUN_NAME="manual_sim_125hz_$(date +%Y%m%d_%H%M%S)"
./scripts/run_quest_jaka_sim_demo.sh \
  --project-ip <QUEST_FACING_HOST_IPV4> \
  --allowed-sender <QUEST_IPV4> \
  --display <DISPLAY> \
  --duration-sec 120 \
  --arm-output-mode jaka-equivalent-125hz \
  --output "logs/quest_jaka_sim/${RUN_NAME}.hts.jsonl" \
  --events "logs/quest_jaka_sim/${RUN_NAME}.events.jsonl" \
  --report "logs/quest_jaka_sim/${RUN_NAME}.report.json" \
  --arm-emitted-events \
    "logs/quest_jaka_sim/${RUN_NAME}.arm_emitted_125hz.jsonl"
```

### 4. Recording replay

Use a recording produced by the preceding simulation command; do not use a
historical file without reviewing its provenance.

```bash
RECORDING=<PATH_TO_REVIEWED_HTS_JSONL>
REPLAY_NAME="manual_replay_$(date +%Y%m%d_%H%M%S)"
PYTHONPATH=src .venv/bin/python tools/quest_jaka_mujoco_sim.py \
  replay-6dof "$RECORDING" \
  --arm-output-mode shaped-500hz \
  --viewer --realtime \
  --events "logs/quest_jaka_sim/${REPLAY_NAME}.events.jsonl" \
  --report "logs/quest_jaka_sim/${REPLAY_NAME}.report.json"
```

Pass criteria: the same clutch edges and accepted/rejected sequence recur, the
viewer remains finite, and the report identifies replay/simulation rather than
physical validation. Stop with `Ctrl+C` or close the viewer.

### 5. Reset, stop, watchdog, clipping, and rejection

The maintained viewer has no interactive reset key. A clean process restart is
the manual reset; `.venv/bin/embodied-lab sim smoke` independently exercises
model load/reset/step. `Ctrl+C` and viewer close are the supported simulation
stops. There is no simulated physical E-stop button.

Use behavior-level tests for deterministic fault injection:

```bash
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m pytest -q \
  tests/test_quest_controller_transport.py \
  tests/test_teleoperation_shaping_and_safety.py \
  tests/test_quest_jaka_shared_pipeline.py \
  tests/test_jaka_edg_resampler.py \
  tests/test_rh56_hand_schema.py
```

Pass criteria: stale/dropout paths stop or hold as specified, out-of-range and
nonfinite actions are rejected before state/SDK commit, and joint/velocity/
acceleration/jerk limits remain independent. This is deterministic offline
evidence, not a manual physical watchdog test.

- External/wrist simulated RGB-D camera views: **NOT IMPLEMENTED — DO NOT
CLAIM VALIDATED**. The canonical producer uses two physical D435 cameras.
- Pure-simulation canonical episode capture: **NOT IMPLEMENTED — DO NOT CLAIM
  VALIDATED**. Live simulation joint logs are not canonical episodes.
- Interactive action injection/reset/E-stop CLI: **NOT IMPLEMENTED — DO NOT
  CLAIM VALIDATED**. Use the maintained Quest entry, process restart, and
  deterministic tests above.

## Level 2 — real-device checks with no actuator motion

These checks may enumerate or read devices but must not command motion. Keep
the robot physically E-stopped where appropriate. Do not run them concurrently
with any control session.

### Workspace, E-stop, network, SDK, and gate parsing

The E-stop state, controller alarm state, payload/TCP/tool/user frames, and
clear workspace must be checked on the physical controller by the operator;
the repository has no authoritative no-motion CLI for those controller-owned
facts.

```bash
ip route get <CONFIRM_FROM_CURRENT_CONTROLLER>
ping -c 3 -W 1 <CONFIRM_FROM_CURRENT_CONTROLLER>

ldd build/jaka_servo_worker/jaka_servo_worker
readelf -d build/jaka_servo_worker/jaka_servo_worker

./scripts/run_quest_jaka_bounded_teleop.sh --help

PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m pytest -q \
  tests/test_quest_jaka_bounded_teleop_entry.py
```

The help command and offline test validate the maintained entry and its
YAML-owned runtime contract without opening sockets or hardware. `ping` is a
network reachability check; it does not load the SDK or command the robot.

### RH56 identity and feedback

Device identity without opening serial:

```bash
ls -l /dev/serial/by-id/
readlink -f /dev/serial/by-id/<CONFIRM_ADAPTER_ID>
udevadm info --query=property --name /dev/serial/by-id/<CONFIRM_ADAPTER_ID>
fuser -v /dev/serial/by-id/<CONFIRM_ADAPTER_ID> || true

./scripts/run_quest_rh56_hand_test.sh \
  --device /dev/serial/by-id/<CONFIRM_ADAPTER_ID> \
  --preflight-only \
  --summary logs/rh56_preflight.summary.json
```

Read-only serial feedback (opens the selected serial device but writes no
register and produces no commanded motion):

```bash
./scripts/run_quest_rh56_hand_test.sh \
  --device /dev/serial/by-id/<CONFIRM_ADAPTER_ID> \
  --duration-sec 10 \
  --jsonl logs/rh56_read_only.jsonl \
  --summary logs/rh56_read_only.summary.json
```

Pass criteria: stable identity, fresh `ANGLE_ACT`, `CURRENT`, `FORCE_ACT`,
`ERROR`, and `STATUS`; zero timeout/checksum/protocol error; and
`register_write_count=0`. Any nonzero error or unexpected write is a failure.

### Quest input only

```bash
.venv/bin/python tools/quest_controller_transport_gate.py \
  --bind 0.0.0.0 \
  --port 9000 \
  --project-ip <QUEST_FACING_HOST_IPV4> \
  --allowed-sender <QUEST_IPV4> \
  --duration-sec 30 \
  --log logs/quest_input_only.events.jsonl \
  --raw-log logs/quest_input_only.raw.jsonl \
  --report logs/quest_input_only.report.json
```

This entry imports no MuJoCo, IK, JAKA, or RH56 target/backend module. Pass if
HTS/CTRL sequences, index/grip, tracking validity, timestamps, and stale
transition are visible and no robot-control process exists.

### RealSense identity and streams

```bash
.venv/bin/python tools/check_realsense_stream.py --list-devices

.venv/bin/python tools/check_realsense_stream.py \
  --serial <WORKSPACE_SERIAL> --width 640 --height 480 --fps 30 \
  --duration-sec 5 --filter-profile raw --preview \
  --snapshot-dir artifacts/realsense_preflight/workspace

.venv/bin/python tools/check_realsense_stream.py \
  --serial <WRIST_SERIAL> --width 640 --height 480 --fps 30 \
  --duration-sec 5 --filter-profile raw --preview \
  --snapshot-dir artifacts/realsense_preflight/wrist
```

Pass criteria: two distinct serials, correct role/view, RGB and raw depth,
expected 640×480@30 profile, monotonic frame numbers, acceptable timestamp
skew, no freeze, and no USB-bandwidth failure. Press `q` to close preview.
These commands start sensor streams but cannot move an actuator.

### Config, storage, logs, and robot-mode boundary

```bash
.venv/bin/embodied-lab doctor --json \
  --output build/validation/hardware-host-doctor.json

mkdir -p data/validation_preflight
WRITE_PROBE="$(mktemp data/validation_preflight/write-probe.XXXXXX)"
printf 'embodied-lab-write-probe\n' > "$WRITE_PROBE"
sha256sum "$WRITE_PROBE"
mv "$WRITE_PROBE" /tmp/

git status --short
```

Pass criteria: configs parse, output paths are writable, no private serial/IP
is added to tracked configs, and physical wrappers still require explicit
runtime safety prerequisites. Doctor does not open any device.

## Level 3 — isolated low-risk actuator motion

Every command in this level is an operator-initiated, bounded physical run.
Do not substitute historical pose, IP, tool/user ID, or safety-controller
values.

### 1. RH56 isolated single channel

**PHYSICAL MOTION — OPERATOR-INITIATED RUN REQUIRED**

Start with the JAKA disconnected or physically E-stopped. The target is based
on fresh measured `ANGLE_ACT`; the configured per-command normalized delta
limit is 0.05. This template requests only `+0.03` on one channel, for 2 s plus
a 2 s hold. Operator stands outside the hand/cable sweep with the adapter
disconnect and E-stop accessible.

```bash
./scripts/run_quest_rh56_hand_test.sh \
  --device /dev/serial/by-id/<CONFIRM_ADAPTER_ID> \
  --bounded-command \
  --channel index \
  --delta 0.03 \
  --duration-sec 2 \
  --hold-sec 2 \
  --manual-stop-accessible --no-auto-retry \
  --jsonl "logs/rh56_bounded_index.jsonl" \
  --summary "logs/rh56_bounded_index.summary.json"
```

Run only one channel per run and inspect its summary before changing
`RH56_CHANNEL`. Abort on wrong channel/direction, unexpected coupling, cable
motion, nonzero `ERROR`, missing/stale feedback, checksum/protocol fault, or
inability to stop. Normal exit is duration expiry or `Ctrl+C`; grip release is
not an E-stop.

### 2. JAKA J6 +0.25 degree outward/hold/return

**PHYSICAL MOTION — OPERATOR-INITIATED RUN REQUIRED**

This maintained native diagnostic commands only J6: +0.25 degrees over 2 s,
0.4 s hold, 2 s return; planned peak bounds are 0.005 rad/s,
0.010 rad/s², and 0.040 rad/s³. It requires at least 5 degrees of software
joint-limit margin. The RH56 control process must be absent. Confirm the
current JAKA pose, positive-J6 cable sweep, tool/user IDs, payload/TCP,
controller safety, and physical E-stop before replacing placeholders.

```bash
cmake -S native/jaka_minimal_joint_probe \
  -B build/jaka_minimal_joint_probe -DCMAKE_BUILD_TYPE=Release
cmake --build build/jaka_minimal_joint_probe -j2

build/jaka_minimal_joint_probe/jaka_gate3c_motion_probe \
  --backend vendor \
  --physical-hardware \
  --robot-ip <CONFIRM_FROM_CURRENT_CONTROLLER> \
  --edg-state-ip <CONFIRM_FROM_CURRENT_NETWORK_CONFIG> \
  --expected-tool-id <CONFIRM_FROM_CONTROLLER> \
  --expected-user-frame-id <CONFIRM_FROM_CONTROLLER> \
  --estop-access-confirmed \
  --workspace-clear-confirmed \
  --no-person-in-workspace-confirmed \
  --cable-clearance-confirmed \
  --direction-understood \
  --ready-to-interrupt \
  --result-file logs/jaka_j6_0p25deg.result.json \
  --trajectory-csv logs/jaka_j6_0p25deg.trajectory.csv
```

Pass only if 551/551 commands complete, direction is correct, no non-J6 target
changes, hard timing/tracking/fault counts are zero, return error is reviewed,
and cleanup ends with servo disable, EDG exit, and logout. Abort immediately on
unexpected motion/direction, collision/alarm, tracking/timing hard fault, or
operator concern.

### 3–6. Required sequence boundaries

- JAKA multi-joint low-speed diagnostic: **NOT IMPLEMENTED — DO NOT CLAIM
  VALIDATED**. The normal teleoperation wrapper is not a substitute for a
  predetermined multi-joint diagnostic.
- RH56 six-channel sequence: repeat the isolated template above manually in
  canonical order; there is intentionally no unattended loop.
- Real physical watchdog/stale-command injection: no standalone low-risk
  command exists. First validate fake-worker stale/fault tests. A deliberate
  Quest loss during an operator-initiated motion session belongs to Level 5 and must
  stop/hold according to that subsystem's documented contract.
- Exit/disconnect stop: use `Ctrl+C`, verify cleanup logs, then use the residual
  process checks below. Do not unplug network/serial as a casual test while an
  actuator is active.

## Level 4 — sensors and data collection

### Static collection first

Run the two single-camera commands from Level 2 while the robots remain
E-stopped. Verify RGB/depth alignment, role, frame number, timestamp, FPS, and
USB bandwidth before a dual-camera episode.

Review the unified tracked collection config and verify both serials/private
calibration before the separately gated run:

```bash
mkdir -p data/episodes data/reports
${EDITOR:-vi} configs/data_collection/physical_collection.yaml
```

Replace both serial placeholders with distinct devices and review calibration
snapshot paths. The only implemented end-to-end capture uses real Quest and
real D435 cameras while commanding MuJoCo only:

```bash
.venv/bin/python tools/quest_jaka_mujoco_sim.py live-6dof \
  --episode-data-config configs/data_collection/physical_collection.yaml \
  --episode-root data/episodes \
  --task-name <TASK_ID> \
  --operator <PSEUDONYMOUS_OPERATOR_ID> \
  --duration-sec 180
```

Operator sequence: start with index released, check both previews, perform the
release-before-press reference capture, hold index for exactly one coherent
episode, and release it to finalize. A normal final directory is
`episode-<uuid>`; `.episode-<uuid>.partial` is an interrupted archive and must
not be renamed or exported. `Ctrl+C`, preview close, source stale, camera loss,
timestamp regression, or write failure should finalize as aborted/invalid or
write rejected-start evidence, never masquerade as success.

Validate, inspect, replay, split, and compute statistics:

```bash
export EPISODE=data/episodes/episode-<uuid>

.venv/bin/embodied-lab dataset validate "$EPISODE" \
  --output "data/reports/$(basename "$EPISODE").validation.json"

.venv/bin/embodied-lab dataset inspect "$EPISODE" \
  --output "artifacts/inspection/$(basename "$EPISODE").json" \
  --plot "artifacts/inspection/$(basename "$EPISODE").png" \
  --playback --playback-rate 1.0

.venv/bin/embodied-lab dataset manifest \
  data/episodes data/manifests/dataset-v1.json \
  --seed embodied-lab-v1 \
  --train-fraction 0.8 --validation-fraction 0.1

.venv/bin/embodied-lab dataset statistics \
  data/manifests/dataset-v1.json \
  data/manifests/dataset-v1.statistics.json
```

ACT-layout and LeRobot export require optional dependencies and an episode
with an independently reviewed explicit success/failure label:

```bash
.venv/bin/python -m pip install -e ".[dataset-export]"
.venv/bin/python -m pip check

.venv/bin/embodied-lab dataset export "$EPISODE" act-hdf5 \
  "data/exports/act/$(basename "$EPISODE").hdf5"

.venv/bin/embodied-lab dataset export "$EPISODE" lerobot-v3 \
  "data/exports/lerobot/$(basename "$EPISODE")" \
  --repo-id <LOCAL_NAMESPACE>/<DATASET_NAME>
```

The official LeRobot 0.6 default SVT-AV1 path requires each RGB stream to be
at least 32 pixels wide; the exporter rejects narrower synthetic frames before
encoding instead of entering an encoder that may not terminate. The maintained
640x480 D435 profile is above this bound. Treat any optional-environment
`pip check` warning as a failed environment check until it is explained. On
this Jetson Thor host the current NVIDIA cuSPARSELt wheel reports an upstream
internal `manylinux2014_sbsa` tag mismatch; the clean base `.[dev]`
environment nevertheless has no broken requirements.

Pass criteria: atomic finalized directory, deep `valid=true`, expected camera
roles/serials/profiles, monotonic frames, bounded causal offsets, zero missed
canonical slots for training eligibility, manifest split by episode, train-only
statistics, and exporter output ordering documented in `DATASET_SCHEMA.md`.
`completed` is not task success. There is currently no audited label-editing
command, no physical JAKA/RH56 canonical collector, and no multi-episode
LeRobot merge command.

## Level 5 — staged joint teleoperation

Complete Level 1 simulation first. The Quest-only command in Level 2 is the
required second step and produces no motion.

### JAKA only

**PHYSICAL MOTION — OPERATOR-INITIATED RUN REQUIRED**

```bash
./scripts/run_quest_jaka_bounded_teleop.sh \
  --runtime-config configs/data_collection/physical_collection.yaml \
  --rh56-command-path-absent
```

These run boundaries are project-selected, not vendor maximums. Start with
index released and RH56 disconnected. Verify coordinate directions one axis at
a time, reference continuity, 60 Hz producer/125 Hz worker timing, hold on
index release, immediate stale-input hold, 10-second recovery deadline
persistent disengaged hold, release-before-press recovery, separate producer
liveness hard stop, and cleanup.

### RH56 only

**PHYSICAL MOTION — OPERATOR-INITIATED RUN REQUIRED**

```bash
timestamp="$(date +%Y%m%d_%H%M%S)"
./scripts/run_quest_rh56_hand_test.sh \
  --device /dev/serial/by-id/<CONFIRM_ADAPTER_ID> \
  --quest-teleop \
  --duration-sec 30 \
  --manual-stop-accessible --no-auto-retry \
  --capture "logs/rh56_quest_hand_${timestamp}.hts.jsonl" \
  --events "logs/rh56_quest_hand_${timestamp}.events.jsonl" \
  --jsonl "logs/rh56_quest_hand_${timestamp}.telemetry.jsonl" \
  --summary "logs/rh56_quest_hand_${timestamp}.summary.json"
```

Keep JAKA disconnected/E-stopped. Grip press captures fresh measured
`ANGLE_ACT`; grip release holds without new writes and does not automatically
open. Abort on wrong mapping, stale/error/status/transport fault, or unexpected
motion.

### Combined arm and hand

**PHYSICAL MOTION — OPERATOR-INITIATED RUN REQUIRED**

```bash
./scripts/run_quest_jaka_rh56_teleop.sh \
  --runtime-config configs/data_collection/physical_collection.yaml
```

First combined run: keep left index released throughout and operate grip only;
new arm commands must remain zero. Only after reviewing that run should an
operator explicitly starts both operation modes. Check arm/hand clutch isolation, direction,
control frequency, latency, clipping/rejection, camera-independent logs,
stale/loss behavior, first terminal reason, and mutual hard-stop propagation.

External/wrist RealSense can be checked with the Level 2 stream commands, but
the physical teleoperation entry does not create a synchronized canonical
dual-camera episode. Adding cameras does not justify claiming physical dataset
collection. A physical short episode, object task, and stop/recovery trial are
**NOT IMPLEMENTED — DO NOT CLAIM VALIDATED** as canonical workflows.

## Level 6 — physical task boundary

The repository implements no physical object-task benchmark runner. Therefore
the required sequence has the following current status:

| Stage | Status |
|---|---|
| reach | NOT IMPLEMENTED — DO NOT CLAIM VALIDATED |
| pre-shape | NOT IMPLEMENTED — DO NOT CLAIM VALIDATED |
| grasp acquisition | NOT IMPLEMENTED — DO NOT CLAIM VALIDATED |
| lift | NOT IMPLEMENTED — DO NOT CLAIM VALIDATED |
| hold | NOT IMPLEMENTED — DO NOT CLAIM VALIDATED |
| transport | NOT IMPLEMENTED — DO NOT CLAIM VALIDATED |
| place | NOT IMPLEMENTED — DO NOT CLAIM VALIDATED |
| release | NOT IMPLEMENTED — DO NOT CLAIM VALIDATED |

No physical object-task runner is maintained here. Any future task runner must
define task/object/initial-pose distributions and record its success and
failure evidence separately from teleoperation and episode capture.

## Stopping and recovery

Normal exit is `Ctrl+C`, viewer close, clutch release followed by process exit,
or the configured duration. After every physical or simulation session:

```bash
ps -eo pid,etime,cmd | \
  grep -E 'quest_jaka|jaka_servo_worker|quest_rh56|rh56|realsense' | \
  grep -v grep || true

git status --short
```

Do not use `kill -9` as routine cleanup. If a maintained wrapper does not exit
after `Ctrl+C`, preserve logs, use the physical E-stop for any actuator risk,
and investigate cleanup before retrying. Confirm that JAKA servo/EDG is no
longer owned by the process, the RH56 serial path has no occupant, and camera
preview/stream processes have exited.

Key log locations:

- simulation: `logs/quest_jaka_sim/`;
- JAKA/combined teleoperation: timestamped files under `logs/`;
- RH56: `logs/rh56_*.jsonl` and `*.summary.json`;
- camera preflight: `artifacts/realsense_preflight/`;
- canonical episodes: `data/episodes/` and `data/reports/`;
- offline validation: `build/validation/`.

For a finalized episode, rerun deep validation before any transfer/export. For
an interrupted `.partial`, preserve it for forensics and recollect; do not
rename it to look complete. Move temporary validation outputs to a named `/tmp`
archive as shown in Level 0. Never delete datasets, recordings, calibration,
weights, or experimental results as cleanup.

---

# 中文版：分阶段手动功能验证

本页是维护的验证流程。任何真实设备操作都必须由操作者单独授权，并保留明确的设备身份、
有界时长/位移、E-stop 可达、workspace 检查和 cleanup 证据。Level 0/1、fake worker、replay 或
没有 gate 证据的日志都不能写成真机 PASS。

## 通用停止规则

遇到 controller alarm、collision、E-stop、power/enable loss、SDK fault、native hard timing、
watchdog/liveness loss、tracking divergence、actual joint/output violation、branch/winding fault、
RH56 `ERROR`/protocol fault 或 cleanup 不确定，立即停止并保留原始日志。不要自动 retry，不要提高
joint/velocity/acceleration/jerk/watchdog 限制，也不要把 JAKA controller 当作正常轨迹筛选层。

candidate infeasible、IK no solution、soft joint margin、singularity candidate rejection、Quest
transient loss、temporary feedback stale 或 recorder quality 问题必须按 bounded hold 或 recording
degraded 处理：不提交当前 candidate，保持最后安全目标/新鲜 heartbeat，允许操作者退出并从 fresh
measured state 重新 capture。孤立相机 stale、preview、event log 或统计问题不能停止健康控制。

## Level 0：环境和离线基线

从仓库根目录完成 `doctor`、YAML 解析、Python compile、核心 pytest、native build/help、MuJoCo
headless smoke、RH56 H0 simulation 和 dataset schema/episode validation。确认所有结果标注为
offline tested 或 simulation validated；不连接 JAKA、RH56、Quest 或 D435。

## Level 1：仿真

按顺序检查 model/viewer、六个 RH56 actuator、Quest 输入、JAKA 六关节、联合 arm/hand、recording
replay、clutch/recovery、reject/hold、watchdog 和 cleanup。确认 MuJoCo 与 JAKA adapter 共享
`AcceptedArmTarget`，物理 adapter 不读 `qpos`、不重算 IK，native joint mode 不调用 JAKA inverse kinematics。

每个 episode 必须独立 finalize；frame index 从 0 开始，timestamp 单调，state/action 与视频不串线。

## Level 2：真机无运动检查

在任何 actuator motion 前，独立确认 workspace、E-stop、网络、SDK gate、controller mode、payload/
COM/TCP/install 状态、JAKA/RH56 identity、Quest sender、RealSense serial、磁盘和日志目录。RH56
先做 identity/read-only feedback；相机先逐台按 serial preflight；Quest 先做 input-only transport。
这些步骤不等于可以发送 motion command。

## Level 3：隔离低风险动作

按维护的 gate 顺序分别进行 RH56 单 channel、JAKA 小范围指定关节和必要的静态 hold/return。每一项
都必须有独立的 stop access、明确边界、operator initiated start/stop 和 cleanup。任何实际 collision、
alarm、unexpected movement 或 feedback/command state 不确定都停止，不继续扩大范围。

## Level 4：相机和数据采集

先做双相机身份/USB/profile 检查，再做 robot disabled 的本地 recorder smoke，然后做 MuJoCo 加相机，
最后才考虑已授权的 physical staging。采集默认 RGB-only；Quest packet、TCP、depth 不进入默认训练视图。
检查 JSONL 与两路 MP4 帧数、strict timestamp、camera role、quality metadata、partial 文件和人工
review。相机 stale/drop/ring expiry/queue drop 只能降级 recording；持续 acquisition/writer failure
停止 recording，但不对健康 JAKA/RH56 emergency stop。

## Level 5：分阶段遥操作

先验证 JAKA-only、RH56-only，再验证 combined。使用现有 release-before-press clutch：transient
tracking loss 只 hold，恢复后 release-before-press 和 fresh measured reference；input/process/IPC/
native liveness loss 才可以 hard stop。先做短时、低幅度、有界任务；不要把完成一次采集写成长期或
完整 workspace PASS。

## Level 6：任务边界、停止和恢复

任务成功必须由人工独立标注，不能由 `completed` 推断。Ctrl+C 或自然结束只影响最后一条 partial
episode，前面已 finalized 的 episode 保留。事故发生时保存 raw logs、terminal reason、config revision、
device identity 和 cleanup 结果，隔离 partial 数据，按[事故响应](../safety/incident_response.md)处理。

所有结论必须明确写成 offline tested、simulation validated、partial physical、physical PASS、physical
FAIL 或 not validated；任何历史结果都不能自动授权下一次真机 gate。
