# Repository overhaul log

This is a concise engineering record for the repository overhaul started on
2026-07-31. It records evidence and decisions, not a transcript of internal
reasoning. All work in this session is offline: no JAKA, RH56DFX, Quest,
RealSense, or other actuator/device connection is authorized.

The source-bundle validation below is historical. The Git-backed second-round
integration and its corrected results are appended at the end of this log.

## Initial state

- Working directory: `/Users/xuweihan/Downloads/embodied-hand-lab-main`.
- The project copy has no `.git` directory. Git walks up to an unrelated,
  empty repository at `/Users/xuweihan` (`main`, no commits, no remote or
  upstream). Consequently repository-local branch, history, status, and diff
  evidence are unavailable. No operation will target the parent repository.
- Initial inventory: 775 files, 112 directories, approximately 133 MiB.
- Main file counts: 297 Python, 23 C++, 12 C++ headers, 25 shell, 93 Markdown,
  37 YAML, 28 JSON, 10 XML, 202 STL, 4 vendor shared libraries, and 2 vendor
  PDF files.
- Main source/document line counts: 65,729 Python lines across `src/`,
  `tools/`, `tests/`, and protected `learned_policy/`; 11,124 C++; 1,702 C++
  header; 1,202 shell; 12,751 Markdown; 2,227 YAML; and 2,896 XML lines.
- The largest files are the vendor JAKA SDK libraries (55 MiB x86_64 and
  47 MiB aarch64) plus simulation meshes. These are intentional platform
  dependencies/assets and are not deletion candidates.
- No `__pycache__`, `.pyc`, pytest/mypy/ruff cache, build directory, temporary
  log, or editor-backup file was present at baseline.
- `artifacts/`, `learned_policy/`, calibration, models, captures, and
  experiment evidence are treated as user-owned/preserved content.

## Baseline authority and safety

Read in full before edits:

- `AGENTS.md`
- `docs/README.md`
- `docs/status/current_status.md`

The authoritative arm path remains Quest HTS/CTRL validation, bounded queue,
release-before-press reference capture, mapping/filtering, shared continuation
IK and feasibility, immutable `AcceptedArmTarget`, then exactly one MuJoCo or
JAKA joint adapter. The physical adapter must not follow MuJoCo `qpos`, remap,
filter, or solve IK. Hardware entry points remain separately and explicitly
gated. Safety thresholds and stop/cleanup semantics are not cleanup targets.

## Baseline host and training resources

- Host: macOS 15.6.1, Darwin 24.6.0, arm64, Apple M1 (8 CPU and 8 integrated
  GPU cores), 16 GiB unified memory.
- Storage: repository filesystem has approximately 176 GiB free.
- Python: CPython 3.13.5, pip 25.1.1. No project virtual environment existed.
- Missing at baseline: NumPy, PyYAML, pyserial, MuJoCo, pytest, PyTorch,
  h5py, OpenCV, RealSense Python bindings, LeRobot, and MediaPipe.
- Unavailable commands: CMake, NVIDIA SMI, CUDA toolkit, Docker, Podman,
  Apptainer/Singularity, Slurm, PCI/InfiniBand diagnostic utilities, and
  `torchrun`.
- Available compiler tools: Apple clang 17 and `make`; this host cannot link
  or execute the Linux JAKA SDK workers.
- There is no NVIDIA driver, CUDA runtime/toolkit, cuDNN, NCCL, NVIDIA GPU
  topology, NVLink, InfiniBand/RoCE, `/dev/shm`, or Slurm allocation to
  measure on this host. Those checks must report “not available”, not infer
  values.
- User limits: 1,048,575 open files, 2,666 processes, 8 MiB stack, unlimited
  CPU time and address space. The active network is Wi-Fi; no reliable link
  bandwidth was inferred.

## Build, packaging, and validation baseline

- Python package: setuptools via `pyproject.toml`, Python >=3.10.
- Declared core dependencies: NumPy, MuJoCo, pyserial, PyYAML.
- Declared optional groups: development, vision teleoperation, RealSense,
  dataset export, phone teleoperation, motion-input visualization, asset
  tools.
- Tests: pytest under `tests/`; no configured formatter, linter, type checker,
  or CI workflow at baseline.
- Native build: CMake projects under `native/`; the documented primary build
  is `native/jaka_servo_worker`.
- ROS 2 is represented by a launch file and runtime bridges, but no ROS
  package manifest or colcon workspace is present.
- No Dockerfile, Conda environment, venv lock file, Slurm template, or
  distributed-training entry point existed at baseline.

## Initial issues requiring follow-up

- README/current status statements contain stale or internally inconsistent
  physical-validation summaries; English and Chinese duration statements also
  disagree in the current status page.
- The repository contains mature teleoperation/simulation code but only an
  inference-only protected OpenPI shadow directory; ACT, Diffusion Policy,
  general training, DDP, and benchmark training loops are not implemented.
- Data collection code exists for a single RGB-D episode, but its durable
  schema, integrity validator, training adapter boundary, and multi-episode
  lifecycle need audit against the requested learning workflow.
- The project has many operator scripts and historical/parallel paths. No file
  will be removed merely from lack of static imports; references, wrappers,
  docs, tests, build files, dynamic entry use, and historical evidence must be
  checked first.
- The Python baseline is 3.13 while several optional robotics/learning wheels
  commonly lag new Python releases. Installation results must distinguish a
  repository defect from a platform/wheel availability issue.

## Decision and validation ledger

| Area | Decision or result | Evidence / command |
| --- | --- | --- |
| Hardware | No connections or commands | Session authorization and safety rules |
| Git | Do not use parent `/Users/xuweihan/.git` | `git rev-parse --show-toplevel` |
| Large binaries | Preserve SDK libraries and simulation assets | file inventory and active build/model references |
| User content | Preserve `artifacts/`, `learned_policy/`, models, captures, calibration, experiments | `AGENTS.md` scope rules |
| Control boundary | Retain immutable accepted-target split and zero native IK joint mode | critical shared-pipeline tests and read-only call-chain audit |
| JAKA limits | Centralize the conservative six-joint limits in `embodiment_core.robot_limits` | configuration/model/native consistency tests |
| Collision model | Retain reviewed Link0/Link1 vendor-mesh overlap exclusion only | zero-start-contact MuJoCo smoke and collision tests |
| RH56 serial | Remove implicit `/dev/ttyUSB0`; require an explicit configured port | `tests/test_rh56_serial_backend.py` |
| Terminal faults | Preserve first arm/hand terminal reason | RH56 worker diagnostic regressions |
| Output records | Bound retained 125 Hz adapter records while keeping cumulative statistics | arm output-mode regressions |
| Data lifecycle | Keep partial writes atomic; reject gaps/unlabelled training input, duplicate UUIDs, invalid hashes, and calibration basename collisions | dataset tests and deep manifest/statistics validation |
| Data shutdown | Close idle/arming writers and all camera/preview resources on every loop exit | collection lifecycle tests |
| Distributed runtime | Implement rank parsing, global-batch calculation, rank-zero atomic JSON, collective/sampler smoke, and Slurm templates; do not invent a trainer | training-infrastructure tests and CPU/Gloo runs |
| Benchmark | Add deterministic joint-reach/RH56 pre-shape smoke only; make limitations part of its output | headless benchmark result |
| Documentation | Replace duplicate current pages with one indexed authority per topic and preserve dated evidence under `history/` | relative-link, command-help, YAML, and shell audits |

## Removal decisions

The following groups were removed only after repository-wide reference,
entry-point, build, configuration, test, script, and documentation checks:

- retired HEBI MobileIO/shadow/real-arm paths and their wrappers, configs, and
  tests;
- the superseded generic JAKA Python SDK/ServoJog/preset stack, retaining the
  active palm-target IK and native EDG joint path;
- obsolete JAKA tool-RS485 and ROS 2 RH56 bridges in favor of the maintained
  PC-direct route;
- the incomplete `robot_bringup`/RViz bridge group, which had no package
  manifest or maintained launch contract;
- ungated or one-off physical smoke/calibration/recording utilities whose
  behavior was covered by the current bounded wrappers or historical evidence;
- one-shot RH56 log analyzers, duplicate RealSense viewer code, unused
  synthetic runtime helpers, the always-failing Orbbec stub, duplicate
  troubleshooting text, obsolete aliases/config copies, and low-value tests
  tied only to those deleted implementations.

`tools/debug_mujoco_jaka_rh56_viewer.py` was restored after dynamic-import
inspection proved that the protected
`tools/teleop_mujoco_jaka_rh56.py` still consumes its public constants and
functions. This is recorded because it demonstrates why static non-use alone
was not accepted as deletion evidence.

## Final offline validation

The temporary validation environment installed `.[dev]` and later
`.[training]`. It contained MuJoCo 3.11.0, PyTorch 2.13.0, NumPy 2.5.1,
PyYAML 6.0.3, h5py 3.16.0, OpenCV 5, CMake 4.4.0, and pytest 9.1.1.

| Check | Result |
| --- | --- |
| Editable development/training install | Passed |
| `pip check` | No broken requirements |
| Python compileall | Passed |
| Pytest collection | 676 tests collected |
| Full offline pytest | 562 passed, 114 platform/hardware skips |
| Critical Quest/JAKA subset | 61 passed, 54 Linux-only skips |
| RH56 focused suite | 53 passed |
| Dataset/training focused suite after lifecycle fixes | Passed |
| Native teleop shaping build/CTest | 3 of 3 passed |
| Portable JAKA resampler build | Passed; real worker intentionally not built on macOS |
| MuJoCo headless smoke | Passed; zero initial/final contacts and zero drift |
| MuJoCo benchmark | Passed; arm max error 0.000465 rad, hand max error 0.001060 rad |
| One-process CPU/Gloo smoke | Passed |
| Two-process CPU/Gloo all-reduce/sampler | Passed with disjoint 8/8 shards and rank-zero result write |
| `torchrun` two-process launcher on this macOS/PyTorch build | Workers passed; parent did not reap/exit and was manually terminated, so launcher cleanup is not verified |
| YAML parse | 13 of 13 mappings passed |
| Shell syntax | 11 of 11 shell/Slurm files passed `bash -n` |
| Major CLI help | Passed without device access |
| LAN route wrapper | Help and default dry-run passed; no network state changed |

The final source bundle contains 705 files versus 775 at baseline, with no
generated validation artifacts retained. The same main-Python scope contains
54,397 lines versus 65,729. All 244 final Python files, including
`digital_twin/` and vendored examples under `third_party/`, contain 56,973
lines. Because project Git metadata is absent, the final report labels
reconstructed added/modified/deleted counts as a conservative path ledger
rather than presenting them as an authoritative Git diff.

Before handoff, the temporary `.venv/`, `build/`, pytest cache, editable-install
metadata, Python bytecode caches, and empty directories left by retired module
groups were removed. They are reproducible validation artifacts, not source or
evidence.

## Environment limitations and unverified work

- Physical arm, hand, Quest, and camera behavior will remain untested.
- Linux-only native SDK linkage and real-time scheduling cannot be validated
  on this macOS host.
- CPU/Gloo collectives were exercised after installing PyTorch, but GPU, CUDA,
  NCCL, model training, multi-GPU, multi-node, Slurm, container-runtime, and
  Jetson behavior remain unverified.
- The missing project Git metadata prevents accurate original-vs-final
  `git diff`, history-based deletion proof, commit identification, and
  upstream verification. Final counts will therefore use the recorded
  baseline inventory plus an explicit changed/added/deleted file ledger.

## Second-round Git-backed correction and validation

The candidate archive was matched to `/home/thor/projects/embodied_lab`, not
the unrelated `openpi` checkout. Its archive digest was
`b0931dc7ce3ec20c5cba9e61fe1f1bdcedfc99902c09afa1dc07dafa694e20a2`; the
candidate had 705 files, 244 Python files, and 56,973 Python lines. Git baseline
and `origin/main` both resolved to `04ba907df002730b80f15df1649e34565e205f78`.
The backup branch `backup/pre-overhaul-integration-20260731-092833` and local
tag `pre-overhaul-integration-20260731-092833` preserve that baseline.

The archive omitted tracked `models/digital_twin/scene.xml` and
`models/digital_twin/workspace_scene_sparse_debug.xml`; both were restored and
verified against the protected snapshot. No protected tool, policy, model,
calibration, vendor, or experiment asset was changed except the previously
reviewed two simulation XML/manifest files adding the narrow Link0/Link1
collision exclusion.

Confirmed fixes in this pass:

- declare `lerobot[dataset]` rather than an incomplete bare LeRobot dependency;
- reject LeRobot RGB widths below 32 px before the default SVT-AV1 path can
  hang; official 640x480 two-stream export/reload passed;
- make native fake timing deterministic without changing vendor timing;
- make native motion `--help` return before any backend construction;
- build the shared teleop C++ fixture once for a clean checkout;
- correct CLI delegated parser prefixes and benchmark default output handling.

Independent results: 681 tests collected, 680 passed, one headless-rendering
skip, zero failures/errors; compileall, imports, CLI help, native worker and
teleop builds, 3/3 CTest, MuJoCo smoke, benchmark, YAML, shell, docs, and
protected-asset checks passed. Linux/aarch64 PyTorch 2.11 Gloo single/two
process and one-GPU NCCL smoke passed; multi-GPU, multi-node, Slurm, trainer,
physical hardware, and policy inference remain unverified. The clean base
`.[dev]` pip check passes; the optional environment exposes the recorded
upstream NVIDIA cuSPARSELt SBSA wheel-tag warning.
