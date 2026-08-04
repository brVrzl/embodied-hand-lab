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

cmake -S native/teleop_shaping \
  -B build/teleop_shaping -DCMAKE_BUILD_TYPE=Release
cmake --build build/teleop_shaping -j2
ctest --test-dir build/teleop_shaping --output-on-failure

.venv/bin/embodied-lab sim smoke
.venv/bin/embodied-lab benchmark configs/benchmark/smoke.yaml
```

Pass criteria:

- `pip check`, compileall, CMake builds, and pytest exit zero; pytest has no
  failed or error outcome. Skips must be reviewed, not counted as passes.
- Doctor reports `ready_offline`, parses every maintained YAML, and records
  `device_connections_attempted=false` and `robot_commands_sent=false`.
- Teleop shaping reports 3/3 CTest. `native/jaka_servo_worker` currently has no
  CTest entries; its criterion is a successful build of the resampler and, on
  supported Linux, the SDK-linked worker.
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

### 6. Benchmark and unavailable simulation functions

```bash
.venv/bin/embodied-lab benchmark \
  configs/benchmark/smoke.yaml \
  --output build/validation/manual-benchmark.json
```

The current benchmark covers JAKA joint reach tracking and RH56 pre-shape
only. It does not implement grasp, lift, transport, place, or release.

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
export JAKA_IP=<CONFIRM_FROM_CURRENT_CONTROLLER>
ip route get "$JAKA_IP"
ping -c 3 -W 1 "$JAKA_IP"

ldd build/jaka_servo_worker/jaka_servo_worker
readelf -d build/jaka_servo_worker/jaka_servo_worker

./scripts/run_quest_jaka_bounded_teleop.sh \
  --robot-ip 192.0.2.10 \
  --output-generator pwl-8ms \
  --joint-velocity-limits-rad-s 1.5 1.5 1.5 1.2 1.2 1.2 \
  --no-auto-retry --estop-accessible --workspace-clear \
  --rh56-command-path-absent \
  --plant-free-no-network-check
```

The last command uses the TEST-NET address `192.0.2.10` and exits before
sockets or hardware. It validates only gate/config/worker parsing. `ping` is a
network reachability check; it does not load the SDK or command the robot.

### RH56 identity and feedback

Device identity without opening serial:

```bash
ls -l /dev/serial/by-id/
export RH56_DEVICE=/dev/serial/by-id/<CONFIRM_ADAPTER_ID>
readlink -f "$RH56_DEVICE"
udevadm info --query=property --name "$RH56_DEVICE"
fuser -v "$RH56_DEVICE" || true

./scripts/run_quest_rh56_hand_test.sh \
  --device "$RH56_DEVICE" \
  --preflight-only \
  --summary logs/rh56_preflight.summary.json
```

Read-only serial feedback (opens the selected serial device but writes no
register and produces no commanded motion):

```bash
./scripts/run_quest_rh56_hand_test.sh \
  --device "$RH56_DEVICE" \
  --read-only \
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
runtime safety prerequisites plus `--no-auto-retry`. Doctor does not open any
device.

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
export RH56_DEVICE=/dev/serial/by-id/<CONFIRM_ADAPTER_ID>
export RH56_CHANNEL=<index|middle|ring|pinky|thumb_close|thumb_lateral>

./scripts/run_quest_rh56_hand_test.sh \
  --device "$RH56_DEVICE" \
  --bounded-command \
  --channel "$RH56_CHANNEL" \
  --delta 0.03 \
  --duration-sec 2 \
  --hold-sec 2 \
  --manual-stop-accessible --workspace-clear --no-auto-retry \
  --jsonl "logs/rh56_bounded_${RH56_CHANNEL}.jsonl" \
  --summary "logs/rh56_bounded_${RH56_CHANNEL}.summary.json"
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

Prepare an ignored local config; never put serials/private calibration into the
tracked example:

```bash
mkdir -p data/local data/episodes data/reports
cp configs/data_collection/dual_d435_episode.example.yaml \
  data/local/dual_d435_episode.yaml
${EDITOR:-vi} data/local/dual_d435_episode.yaml
```

Replace both serial placeholders with distinct devices and review calibration
snapshot paths. The only implemented end-to-end capture uses real Quest and
real D435 cameras while commanding MuJoCo only:

```bash
.venv/bin/python tools/quest_jaka_mujoco_sim.py live-6dof \
  --episode-data-config data/local/dual_d435_episode.yaml \
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
  --robot-ip <CONFIRM_FROM_CURRENT_CONTROLLER> \
  --edg-state-ip <CONFIRM_FROM_CURRENT_NETWORK_CONFIG> \
  --allowed-sender <QUEST_IPV4> \
  --duration-sec 30 \
  --output-generator pwl-8ms \
  --joint-velocity-limits-rad-s 1.5 1.5 1.5 1.2 1.2 1.2 \
  --no-auto-retry --estop-accessible --workspace-clear \
  --rh56-command-path-absent
```

These run boundaries are project-selected, not vendor maximums. Start with
index released and RH56 disconnected. Verify coordinate directions one axis at
a time, reference continuity, 60 Hz producer/125 Hz worker timing, hold on
index release, immediate stale-input hold, 10-second recovery timeout hard
stop, release-before-press recovery, and cleanup.

### RH56 only

**PHYSICAL MOTION — OPERATOR-INITIATED RUN REQUIRED**

```bash
timestamp="$(date +%Y%m%d_%H%M%S)"
./scripts/run_quest_rh56_hand_test.sh \
  --device /dev/serial/by-id/<CONFIRM_ADAPTER_ID> \
  --quest-teleop \
  --duration-sec 30 \
  --manual-stop-accessible --workspace-clear --no-auto-retry \
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
  --robot-ip <CONFIRM_FROM_CURRENT_CONTROLLER> \
  --edg-state-ip <CONFIRM_FROM_CURRENT_NETWORK_CONFIG> \
  --rh56-device /dev/serial/by-id/<CONFIRM_ADAPTER_ID> \
  --allowed-sender <QUEST_IPV4> \
  --duration-sec 60 \
  --hand-prerequisites-complete --no-auto-retry \
  --estop-accessible --workspace-clear
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

## Level 6 — physical benchmark boundary

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

Do not repurpose teleoperation as a benchmark command. Before implementing a
runner, define task/object/initial-pose distributions and log at least:
`task_id`, `object_id`, initial pose, config hash, trial number, success,
failure reason, completion time, drop, command/feedback discrepancy, raw
status/error, and video/log paths. The existing offline benchmark remains:

```bash
.venv/bin/embodied-lab benchmark configs/benchmark/smoke.yaml \
  --output build/validation/offline-only-benchmark.json
```

It validates only simulated joint reach tracking and hand pre-shape.

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
