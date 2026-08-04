# Final repository review

Review date: 2026-07-31
Validation level: offline, replay, and MuJoCo simulation only
Physical devices accessed: none

This historical source-bundle review was independently re-run during the
Git-backed second-round integration on 2026-07-31. Its original macOS counts
and limitations remain historical evidence; the authoritative current results,
including Linux/aarch64 validation and the repaired regressions, are in the
second-round appendix at the end of this file and in
`INTEGRATION_VALIDATION_REPORT.md` in this directory.

## 1. Executive summary

This overhaul reduced the repository from 775 to 705 files and from 297 to
244 Python files. Python volume in the repeatable main-code scope (`src/`,
`tools/`, `tests/`, and protected `learned_policy/`) fell from 65,729 to
54,397 lines, a net reduction of 11,332 lines despite adding maintained
dataset, benchmark, doctor, CLI, and distributed-runtime modules.

The maintained system is now described consistently as:

```text
Quest HTS + CTRL
  -> validation and bounded input queue
  -> release-before-press reference capture
  -> mapping and filters
  -> shared continuation IK and feasibility
  -> immutable AcceptedArmTarget
  -> MuJoCo adapter OR physical JAKA joint adapter
```

The physical adapter still receives the same accepted J1--J6 radians as
MuJoCo. It does not follow MuJoCo `qpos`, remap/filter the target, or recompute
IK. Native joint mode still makes zero JAKA `kine_inverse` calls. Collision,
singularity, joint/command limits, timing, watchdog, liveness, and cleanup
contracts were retained.

The final offline suite passed with 562 tests and 114 declared
platform/hardware skips. Portable native code built, all three teleop-shaping
CTest targets passed, the default MuJoCo model passed a headless smoke test,
and the bounded joint reach/RH56 pre-shape benchmark passed. PyTorch 2.13 CPU
Gloo passed one- and two-process collectives and disjoint sampler checks. No
physical or GPU result is claimed.

No commit, push, rebase, merge, branch operation, or pull request was
performed.

The temporary validation virtual environment, native build tree, pytest
cache, editable-install metadata, Python bytecode, and empty retired-module
directories were removed before handoff.

## 2. Scope and count methodology

The downloaded source directory has no project-local `.git` metadata. Git
walks upward to an unrelated empty repository at `/Users/xuweihan`; it was not
used. Consequently an authoritative Git diff, original file hashes, branch,
upstream, or commit identity cannot be reconstructed.

Counts use the recorded initial inventory, final filesystem inventory,
creation/modification times, and the removal ledger. The 705-file final count
is the complete post-cleanup tree; no generated validation artifacts remain.
Same-path full rewrites are treated as modifications. Moves are conservatively
counted as one removed and one added path.

| Measure | Initial | Final | Change |
| --- | ---: | ---: | ---: |
| Non-generated files | 775 | 705 | -70 |
| Python files | 297 | 244 | -53 |
| Main-scope Python lines | 65,729 | 54,397 | -11,332 |
| Source-bundle size | about 133 MiB | 130.66 MiB | about -2.3 MiB |

“Main-scope Python” means files below `src/`, `tools/`, `tests/`, and
`learned_policy/`, matching the recorded baseline command. For completeness,
all 244 final Python files contain 56,973 lines when the standalone
`digital_twin/` package and vendored `third_party/` examples are also counted.

Conservative reconstructed path ledger:

- 93 files modified or rebuilt;
- 38 files added or relocated;
- 108 files removed.

These three counts reconcile the initial and final inventories, but they are a
maintenance ledger, not a substitute for a missing Git diff. The actual amount
of deleted Python code is greater than the 11,332-line net reduction because
new maintained code was added.

The large vendor JAKA libraries, simulation meshes, `artifacts/`,
`learned_policy/`, models, calibration material, captures, and experiment
evidence were preserved. The protected
`tools/teleop_mujoco_jaka_rh56.py` remained unchanged.

## 3. Initial repository problems

The initial source contained mature current Quest/JAKA work alongside several
superseded control stacks:

- a generic Python JAKA SDK/ServoJog/preset path parallel to the current native
  EDG accepted-joint path;
- HEBI MobileIO, RViz shadow, ROS 2 bridge, and real-arm experiments without a
  maintained role in the current authority;
- old RH56 JAKA tool-RS485 and ROS 2 routes parallel to the maintained
  PC-direct serial route;
- incomplete `robot_bringup` and RViz packages without a package manifest or
  maintained launch contract;
- one-off physical smoke, calibration, recording, and log-analysis tools;
- duplicated configuration aliases, viewer code, troubleshooting pages, and
  bilingual full-page copies;
- implicit hardware defaults such as `/dev/ttyUSB0`;
- unbounded retained 125 Hz adapter records and repeated linear statistics;
- data capture code without a complete durable validation/split/statistics
  boundary;
- no benchmark harness, host doctor, unified offline CLI, distributed smoke,
  Slurm templates, or current training integration authority;
- documentation that mixed current instructions, dated evidence, implemented
  code, and unvalidated future plans.

The initial full suite produced 575 passes, 9 failures, 124 errors, and 2
skips. Most errors came from platform assumptions, missing optional
dependencies, and retired modules rather than a single functional regression.

## 4. Removed content and evidence

Removal required more than a missing static import. Each group was checked
against dynamic imports, scripts, configs, docs, tests, package/build files,
and operator entry points.

| Removed group | Reason and evidence |
| --- | --- |
| HEBI MobileIO calibration, record, shadow, RViz, and real-arm path | Retired parallel research path; current authority is Quest HTS/CTRL. Its tools, configs, wrappers, package modules, and implementation-detail tests formed a closed obsolete group. |
| Generic JAKA SDK/ServoJog/preset modules and tools | Superseded by the native EDG accepted-joint worker. Current docs/scripts/build did not consume them. Active palm-target IK was retained. |
| JAKA tool-RS485 RH56 client/backend and ROS 2 bridges | Superseded by the maintained PC-direct USB/RS485 route; current configs, wrappers, and safety pages point only to PC-direct operation. |
| `robot_bringup`/RViz bridge group | No `package.xml`, ROS package, maintained launch contract, or current operator path; tests covered only the removed internal bridge. |
| Generic arm/hand smoke, preset move/save, force calibration, real-state recording, and old connection probes | Ungated, duplicated, or one-off physical utilities with no current operator contract. Current bounded wrappers and history retain the useful safety/evidence context. |
| One-off RH56 calibration/retarget log analyzers | Their durable conclusions already exist in current docs/history; no maintained workflow names them. |
| Duplicate RealSense viewer | Pure preview behavior was merged into `check_realsense_stream.py`; dynamic consumers and tests were updated. |
| Always-failing Orbbec adapter and typo camera alias | No working backend, export, config, entry point, or current documentation consumer. |
| Unused synthetic teleoperation runtime and old shaping helpers | No current caller after full reference search; current replay/simulation modules own the behavior. |
| Duplicate troubleshooting/current text and obsolete config aliases | Unique information was merged into the current authority and all links updated. |
| Tests tied only to deleted private implementations | Removed with their obsolete implementation; stable safety/data/control regressions were retained or extended. |

During cleanup, `tools/debug_mujoco_jaka_rh56_viewer.py` was initially
identified as apparently standalone. Dynamic-import inspection proved that
the protected teleoperation tool imports its constants and functions at
runtime. It was therefore restored, its API was exercised, and a short
headless smoke passed. This is the principal example of deletion being
reversed when evidence showed a real consumer.

## 5. Code refactoring and safety changes

### Shared limits and platform portability

- Added `embodiment_core.robot_limits` as the single Python authority for the
  conservative JAKA joint ranges used by IK, simulation, and dataset action
  validation.
- Corrected per-joint native velocity argument construction.
- Made socket/shared-library handling and affinity behavior portable on
  macOS without changing Linux real-time behavior.
- Changed the native JAKA CMake project to build the portable resampler on all
  hosts while keeping the real SDK worker Linux-only.
- Added a test-session path setup so the documented
  `.venv/bin/python -m pytest` form finds CMake/Ninja without shell activation.

### Control and output behavior

- Kept the accepted-target boundary immutable and adapter-neutral.
- Bounded retained 125 Hz output records to 8,192 entries while keeping
  cumulative statistics and a cursor for consumers.
- Replaced repeated linear scans with cumulative output statistics.
- Ensured composite output fan-out attempts every configured adapter rather
  than allowing one adapter to suppress cleanup/diagnostics for another.
- Preserved `HOLD_REJECTED` as a fresh-heartbeat hold of the last safe target,
  distinct from a terminal liveness or controller fault.
- Retained the reviewed JAKA Link0/Link1 3 mm vendor-mesh overlap exclusion;
  no broader collision suppression was added.

### RH56 behavior

- Required an explicit serial port; no default device is opened.
- Latched the first RH56 or arm terminal reason instead of overwriting it with
  later cleanup symptoms.
- Kept raw `ANGLE_ACT`, `CURRENT`, `FORCE_ACT`, `ERROR`, and `STATUS` semantics
  explicit. They are not passive-joint state, tactile arrays, direct slip
  sensors, or calibrated contact force.
- Preserved measured-first activation, bounded command range/rate/delta,
  stale/protocol/error stops, and deterministic cleanup.

### Network and input gates

- Made the JAKA LAN2 helper dry-run by default, removed `eval`, validated its
  interface/IP inputs, and retained separate execution and destructive-address
  replacement acknowledgements.
- Removed the hard-coded Quest project IP. The transport-only gate now
  requires an explicit address while still rejecting unsupported fake/keyboard
  modes before attempting any UDP work.

## 6. Dataset and collection changes

The canonical episode system now provides:

- strict `IDLE -> ARMING -> REC -> FINALIZING -> DONE` lifecycle;
- same-filesystem partial staging and atomic final rename;
- safe shutdown in IDLE, ARMING, recording, normal loop end, preview close,
  and Ctrl+C paths;
- cleanup if camera startup succeeds but later configuration, metadata, writer,
  or preview setup fails;
- causal source selection, fixed canonical clock, explicit missed-slot and
  timing metadata, and timestamp regression handling;
- finite non-negative startup continuity tolerances;
- unique calibration snapshot basenames and content hashes;
- deep structural/payload/hash validation;
- canonical UUID validation and exclusion of every copy of a duplicate UUID;
- deterministic episode-level split manifests;
- training eligibility only for complete, gap-free, explicitly labelled
  `success` or `failure` episodes;
- train-split-only state/action statistics tied to manifest, metadata, and
  canonical-index hashes;
- offline inspection summaries, plots, and optional local RGB/depth playback;
- ACT-style per-episode HDF5 and optional official-SDK LeRobot v3 export, with
  canonical data retained as authority.

The wired producer remains simulation-backed for arm/hand state. A future
physical schema must preserve RH56 raw register provenance and calibration; it
must not insert raw counts into the current radian fields.

The ACT-style file is not drop-in compatible with the unmodified official ACT
trainer. The pinned reference implementation hard-codes a 14-D ALOHA
state/action in multiple paths, while this project exports 12 dimensions.

## 7. Training and distributed changes

The repository intentionally does not contain a fake trainer. It now has a
small infrastructure boundary:

- strict parsing of the complete `LOCAL_RANK`, `RANK`, `WORLD_SIZE` triplet;
- explicit global-batch calculation;
- rank-zero-only atomic JSON output;
- lazy optional PyTorch import;
- Gloo/NCCL capability inspection;
- process-group, all-reduce, rank/device mapping, `DistributedSampler`,
  barrier, and cleanup smoke;
- one parameterized future training configuration;
- one single-node and one multi-node Slurm template.

DDP is the recommended future default only when the selected model fits one
GPU. FSDP and DeepSpeed were not added. OpenPI must retain its upstream-native
JAX or PyTorch distribution and checkpoint system.

Official-source review pinned the integration discussion to:

- ACT `742c753c0d4a5d87076c8f69e5628c79a8cc5488`;
- Diffusion Policy `5ba07ac6661db573af695b419a7947ecb704690f`;
- OpenPI `15a9616a00943ada6c20a0f158e3adb39df2ccac`.

That review corrected three important boundaries:

- ACT needs a reviewed 14-D-to-12-D source/model adaptation and train-only
  statistics rather than an unchanged upstream loader.
- Diffusion Policy uses observation `To`, prediction `Tp`, and execution
  `Ta`; its official sampler repeats boundary values and does not provide a
  standard padding/data-quality mask.
- OpenPI's native batch size is global, its pinned JAX trainer is not
  multi-node, and its pinned PyTorch implementation does not support
  π0-FAST, mixed precision, FSDP, LoRA, or EMA.

## 8. Benchmark and simulation changes

Added a deterministic offline benchmark with a small common task/result
contract, seed, config snapshot, bounds, failure reason, metrics, and atomic
JSON result. The implemented task measures:

- six simulated JAKA joint positions reaching one bounded target;
- six actuator-driven RH56 joints reaching one pre-shape;
- completion time, tracking error, commanded speed, TCP displacement, and
  diagnostic contact count.

It does not test object grasp, lift, hold, transport, placement, release,
disturbance resistance, passive RH56 joints, tactile sensing, physical current
or force, or sim-to-real transfer. Targets enter the MuJoCo adapter directly,
so this smoke is not a complete Quest mapping/shared-IK test.

The RH56 model remains an explicit six-command-axis approximation using
equality couplings. Tendon compliance, backlash, calibrated force control,
tactile sensing, and complete physical underactuation are not modelled.

## 9. Dependency and environment changes

`pyproject.toml` is the sole maintained Python environment definition.

| Extra | Purpose |
| --- | --- |
| base | NumPy and PyYAML core |
| `simulation` | MuJoCo runtime |
| `hardware` | pyserial only; never an authorization |
| `vision-teleop` | MediaPipe/OpenCV input |
| `realsense` | RealSense Python bindings |
| `dataset-export` | HDF5, LeRobot v3, and OpenCV export support |
| `training` | PyTorch and HDF5 training-server preparation |
| `asset-tools` | reconstruction/collision asset dependencies |
| `dev` | complete offline development/test environment |

Unused HEBI/phone-control dependencies were removed. Runtime, simulation,
hardware, dataset, training, and development concerns are separated.

No Docker image or Conda environment was added because neither could be
validated here and the current project does not yet have a trainer whose CUDA
stack can be pinned meaningfully. Documentation instead separates x86 Linux
training, ordinary workstation simulation, Linux JAKA SDK work, and ARM64
Jetson deployment. Slurm and Apptainer/Docker sections are templates that
require site-specific values.

No formatter, linter, type checker, or CI workflow is configured. None is
reported as passing.

## 10. Documentation changes

The documentation was rebuilt around one current authority per topic:

- root `README.md` for actual support, quick start, safety, and layout;
- `docs/README.md` as the current/history index;
- installation and configuration guides;
- system architecture and hardware/simulation boundaries;
- real-hardware safety and validation matrix;
- canonical data schema and collection/quality guide;
- training integration and distributed readiness;
- benchmark and experiment discipline;
- troubleshooting;
- phased execution roadmap;
- this final review and the concise overhaul log.

Historical reports remain under `docs/history/` and were not rewritten to
match current behavior. Duplicate full Chinese pages were reduced to short
Chinese orientation sections rather than maintaining a second drifting
manual.

## 11. User-experience changes

Editable installation provides one safe offline CLI:

```bash
embodied-lab doctor
embodied-lab sim smoke
embodied-lab dataset ...
embodied-lab benchmark ...
embodied-lab distributed-smoke ...
```

`doctor` records host, Python, packages, storage, CUDA/NVIDIA distinctions,
container/Slurm availability, YAML parsing, environment-variable presence, and
device-path inventory without opening devices. It distinguishes NVIDIA driver,
system CUDA toolkit, PyTorch build CUDA, and optional runtime availability.

Simulation, dataset, benchmark, and distributed tools have usable `--help`.
Physical commands remain outside the quick start and retain exact approvals,
bounded duration, workspace/E-stop checks, no-retry behavior, and cleanup.

## 12. Commands actually run

Representative commands, all from the repository root:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e ".[dev]"
.venv/bin/python -m pip install -e ".[training]"
.venv/bin/python -m pip check

PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m compileall -q src tools tests
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m pytest --collect-only -q -p no:cacheprovider
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m pytest -q -p no:cacheprovider

.venv/bin/cmake -S native/jaka_servo_worker -B build/jaka_servo_worker \
  -DCMAKE_BUILD_TYPE=Release
.venv/bin/cmake --build build/jaka_servo_worker -j
.venv/bin/cmake -S native/teleop_shaping -B build/teleop_shaping \
  -DCMAKE_BUILD_TYPE=Release
.venv/bin/cmake --build build/teleop_shaping -j
.venv/bin/ctest --test-dir build/teleop_shaping --output-on-failure

.venv/bin/embodied-lab doctor --json
.venv/bin/embodied-lab sim smoke --duration-sec 0.05
.venv/bin/embodied-lab benchmark configs/benchmark/smoke.yaml \
  --output build/benchmark-smoke.json
.venv/bin/embodied-lab distributed-smoke --check
.venv/bin/embodied-lab distributed-smoke \
  --device cpu --backend gloo \
  --result-json build/distributed-smoke-single.json

bash -n scripts/*.sh
bash -n scripts/slurm/*.sbatch
./scripts/setup_jaka_lan2_route.sh
```

The critical control subset was also run exactly against:

```text
tests/test_quest_jaka_shared_pipeline.py
tests/test_quest_jaka_output_feasibility.py
tests/test_quest_jaka_singularity_liveness.py
tests/test_jaka_edg_resampler.py
tests/test_native_jaka_servo_worker.py
```

No project-local `git diff --check` was possible because `.git` is absent.
The unrelated parent repository was not used.

## 13. Validation results

| Validation | Result |
| --- | --- |
| Editable `.[dev]` and `.[training]` installation | Passed |
| Dependency consistency | `pip check`: no broken requirements |
| Compileall | Passed |
| Test collection | 676 tests |
| Full pytest | 562 passed, 114 skipped, 0 failed/error |
| Critical Quest/JAKA subset | 61 passed, 54 Linux-only skipped |
| Native JAKA portable resampler | Built |
| Real Linux JAKA worker | Not built on macOS by design |
| Teleop shaping CTest | 3/3 passed |
| MuJoCo headless smoke | Passed; zero initial/final contacts, zero drift |
| Benchmark | Passed; arm max error 0.0004646 rad, hand max error 0.0010603 rad |
| YAML | 13/13 parsed as mappings |
| Shell/Slurm syntax | 11/11 passed |
| CLI help | 30 major help commands passed in the independent audit |
| Documentation links | 99 Markdown files; 194 relative links; 0 missing; 0 unclosed fences |
| Final residue scan | No merge markers, TODO/FIXME/HACK/XXX markers, temporary logs, backup files, or generated caches |
| One-process CPU/Gloo | Passed |
| Two-process CPU/Gloo | Passed; all-reduce 3.0, disjoint 8/8 shards, rank-zero write |
| Two-process macOS `torchrun` parent | Workers passed, parent did not reap/exit; manually terminated |

The 114 skips cover unavailable Linux real-time/procfs/SDK behavior and
optional hardware/platform paths. They are not failures and are not physical
validation.

## 14. Work not validated

- Any JAKA, RH56DFX, Quest, RealSense, or other hardware behavior in this
  session.
- Linux JAKA SDK linkage, real-time scheduling, EDG timing, and cleanup on a
  target controller.
- The latest output-acceleration correction on physical JAKA.
- CUDA, NCCL, one-GPU model training, multi-GPU DDP, multi-node, Slurm,
  Docker/Apptainer, or heterogeneous GPU behavior.
- ACT, Diffusion Policy, or OpenPI training, checkpoint recovery, model export,
  or Jetson Thor inference.
- Physical dual-D435 synchronization, camera/robot calibration, USB bandwidth,
  and physical dataset capture.
- Object-task benchmark, grasp/lift/hold/transport/place/release metrics, and
  sim-to-real equivalence.

## 15. Hardware risks

1. **Critical:** the earlier J4 collision cause remains unresolved.
2. **Critical:** the current output-acceleration correction is offline tested
   but has no bounded post-fix physical validation.
3. **High:** TCP calibration is incomplete; recorded TCP1--TCP10 values are
   zero and software must not silently write replacements.
4. **High:** no 300-second combined physical PASS exists. A 60.105-second PASS
   and a later correct 200.943-second liveness stop must remain distinct.
5. **High:** Quest controller validity, RH56 target/feedback characterization,
   and complete Quest-driven physical hand teleoperation remain incomplete.
6. **High:** camera geometry and clock alignment are not ready for production
   demonstrations.
7. **Medium:** RH56 current/load fields are raw proxies and passive joint state
   is unavailable.
8. **Medium:** provisional MuJoCo workspace geometry is not shared
   pre-acceptance physical collision authority.

Any future physical gate requires a new session, exact authorization,
controller-side payload/COM/installation/TCP/safety verification, E-stop
access, clear workspace, bounded motion/duration, and retained cleanup.

## 16. Data, training, and benchmark readiness

### Data collection readiness: Partially implemented

Evidence: atomic canonical writer, strict lifecycle, dual-camera interfaces,
deep validator, deterministic manifest, train-only statistics, inspection,
and exporters pass offline tests. Blockers: physical arm/hand state is not
wired into the schema, camera/time calibration is absent, and no physical
multimodal episode has passed acceptance.

### Training readiness: Planned to partially implemented

The data contract and external-framework boundaries are useful, but no model
trainer exists. ACT, Diffusion Policy, and OpenPI require separate pinned
adapters. No checkpoint/resume or inference-to-safe-target adapter exists.

### Benchmark readiness: Offline smoke verified

The deterministic MuJoCo joint/pre-shape benchmark passes and produces
reproducible results. A resettable object-task suite and physical benchmark
remain planned.

## 17. Distributed Training Readiness

Ratings use only `Verified`, `Implemented but not verified`, `Partially
implemented`, `Planned`, or `Blocked`.

| Area | Rating | Evidence or blocker |
| --- | --- | --- |
| Single-card training | Planned | PyTorch CPU/Gloo infrastructure works, but there is no trainer/model/optimizer loop, CUDA device, checkpoint, or one-GPU overfit result. |
| Single-node multi-card | Partially implemented | Rank/global-batch/sampler/rank-zero helpers exist and two CPU/Gloo processes passed; no DDP model or CUDA/NCCL test. macOS `torchrun` parent cleanup was abnormal. |
| Multi-node | Partially implemented | Rank/environment contract and parameterized launcher template exist; no nodes, fabric, shared storage, or inter-node collective was available. |
| Slurm | Implemented but not verified | Single- and multi-node templates pass `bash -n`; Slurm is absent and all site placeholders require administrator review. |
| Data I/O | Partially implemented | Canonical manifests, episode splits, train-only statistics, HDF5/LeRobot views exist; no distributed DataLoader, node cache, or shared-filesystem profile. |
| Checkpoint recovery | Planned | Rank-zero atomic JSON helper exists; model/optimizer/scheduler/scaler/EMA/RNG/sampler checkpoint and resume do not. |
| ACT multi-card | Planned | No ACT trainer; 12-D project layout needs a reviewed adaptation of pinned upstream 14-D assumptions before DDP. |
| Diffusion Policy multi-card | Planned | No adapter/trainer/EMA checkpoint; upstream reference workspace is not a ready project DDP implementation. |
| OpenPI/π0 large-model training | Planned | No project adapter or fine-tuned checkpoint; selected upstream backend must own distribution. Pinned backend limitations are documented. |
| Jetson Thor deployment | Planned | Handoff contract exists; no selected model, export, operator compatibility, latency, or replay equivalence evidence. |

No FSDP or DeepSpeed implementation was added. DDP remains the first future
GPU scaling path for ACT or a modest Diffusion Policy model that fits one
device. OpenPI must follow its selected upstream release.

## 18. Remaining issues by severity

### Critical

- Resolve the J4 collision cause.
- Complete TCP calibration and the separately authorized bounded physical
  validation of the latest acceleration correction.

### High

- Characterize RH56 actuator command/feedback timing and raw field behavior.
- Calibrate dual-D435 geometry and clocks.
- Produce and qualify a small physical pilot dataset.
- Add a real single-process trainer before any GPU-distribution layer.

### Medium

- Build an ACT-specific 12-D adapter and verify train-only split/statistics.
- Build a Diffusion Policy adapter with explicit `To/Tp/Ta` and reviewed
  boundary/data-quality behavior.
- Decide whether OpenPI integration cost is justified after a loader/forward
  test.
- Add a resettable MuJoCo object task only after defining observable success
  predicates and calibrated contacts.

### Low

- Add a container only after selecting the real trainer/CUDA stack.
- Consider formatter/linter/type-checker configuration in a separate scoped
  maintenance change rather than forcing unrelated churn here.

## 19. First commands on the next device

Do not run physical wrappers. First reproduce the offline source bundle:

```bash
cd /path/to/embodied-hand-lab-main
test -f AGENTS.md

python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -e ".[dev]"
.venv/bin/python -m pip check

mkdir -p build/validation
.venv/bin/embodied-lab doctor \
  --output build/validation/environment.json
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m compileall -q src tools tests
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m pytest \
  --collect-only -q -p no:cacheprovider
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m pytest \
  -q -p no:cacheprovider

.venv/bin/cmake -S native/jaka_servo_worker \
  -B build/jaka_servo_worker -DCMAKE_BUILD_TYPE=Release
.venv/bin/cmake --build build/jaka_servo_worker -j

.venv/bin/embodied-lab sim smoke
.venv/bin/embodied-lab benchmark \
  configs/benchmark/smoke.yaml \
  --output build/validation/benchmark-smoke.json
```

On a Linux training host, then install and inspect the optional runtime:

```bash
.venv/bin/python -m pip install -e ".[training]"
.venv/bin/embodied-lab distributed-smoke --check
.venv/bin/embodied-lab distributed-smoke \
  --device cpu \
  --backend gloo \
  --result-json build/validation/distributed-gloo-single.json
```

Only after that host reports a compatible CUDA PyTorch build should it run a
two-process NCCL smoke. Record `nvidia-smi`, topology, PyTorch/CUDA/NCCL,
storage, `/dev/shm`, network, and Slurm/container evidence first.

## 20. Recommended execution order

1. Reproduce this offline baseline on the transferred workstation.
2. On Linux, build native targets without connecting to a controller.
3. Resolve the J4/TCP/acceleration physical blockers in a new, separately
   authorized session.
4. Characterize RH56 in a separately gated hand-only session.
5. Calibrate both cameras and robot/device clocks.
6. Collect a small physical pilot with lifecycle drills.
7. Deep-validate, label, split, inspect, and freeze the pilot.
8. Build and overfit one ACT adapter on a single GPU.
9. Add checkpoint/resume and replay inference; then evaluate DDP.
10. Repeat with a separate Diffusion Policy adapter.
11. Make an evidence-based OpenPI go/no-go decision.
12. Extend the MuJoCo benchmark to an object task.
13. Export and validate the selected model on x86 replay, then Jetson replay.
14. Connect a learned policy to real hardware only through a new authorized
    gate after all earlier acceptance criteria pass.

## Second-round Git-backed validation appendix

The candidate archive was independently compared with the real repository
baseline, applied on an isolated integration branch, and validated on Linux
6.8.12-1021-tegra/aarch64 (Jetson Thor). The archive omitted two tracked model
XML files; they were restored from the protected baseline before validation.
`tools/teleop_mujoco_jaka_rh56.py`, `learned_policy/`, calibration, vendor, and
experiment assets were preserved. The reviewed deletions remain intentional:
HEBI, obsolete Python ServoJog/JAKA SDK paths, old RH56 JAKA-tool/ROS routes,
ungated physical utilities, and unmaintained bridge/RViz groups had no current
consumer after repository-wide reference checks.

The independent second-round suite collected 681 tests and finished with
680 passed, one headless-rendering skip, and zero failures/errors. It also
passed compileall, package import smoke (13 public packages; no socket or path
write side effect; one read-only GLFW capability subprocess), corrected CLI
help, native builds, 3/3 teleop CTest, MuJoCo smoke, and the benchmark. The
benchmark retained the original thresholds and reproduced arm maximum error
`0.0004646174241849099` rad and hand maximum error
`0.001060247811619841` rad.

Three implementation issues were repaired during this pass: the LeRobot extra
now declares `lerobot[dataset]` dependencies; the official exporter rejects
RGB widths below 32 pixels before the default SVT-AV1 encoder can hang (a
640x480 two-stream export and reload passed); and the native fake timing clock,
motion-probe help path, CLI delegated parser/default-output behavior, and
clean-build native pytest fixture were made deterministic and non-connecting.
The clean base `.[dev]` environment passes `pip check`. The optional
LeRobot/Torch environment has a platform-tag warning for the upstream
`nvidia-cusparselt-cu13` SBSA wheel; it is recorded as an environment issue,
not silently treated as a pass.

The second-round Linux distributed checks passed CPU/Gloo one- and two-process
`torchrun` (including parent reaping) and one-GPU NCCL. No multi-GPU,
multi-node, Slurm, ACT/Diffusion/OpenPI training, physical hardware, or
Jetson policy inference was claimed. See
`dev_tmp/maintenance/INTEGRATION_VALIDATION_REPORT.md` for the complete matrix.
