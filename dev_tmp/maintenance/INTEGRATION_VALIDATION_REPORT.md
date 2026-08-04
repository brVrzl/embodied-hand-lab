# Integration validation report

Date: 2026-07-31 (Asia/Shanghai)
Scope: independent candidate review, offline regression repair, Git
integration, and safe publish preparation. No physical device was connected or
commanded.

## 1. Assets and Git baseline

| Item | Recorded result |
|---|---|
| Workspace | `/home/thor/projects` |
| Git repository | `/home/thor/projects/embodied_lab` |
| Independent other repository | `/home/thor/projects/openpi` (not a target) |
| Candidate archive | `/home/thor/projects/embodied-hand-lab-main.zip` |
| Candidate SHA-256 | `b0931dc7ce3ec20c5cba9e61fe1f1bdcedfc99902c09afa1dc07dafa694e20a2` |
| Candidate extraction | `/tmp/embodied-lab-overhaul-review-20260731-092833/embodied-hand-lab-main` |
| Archive structure | 1,646 entries; one `embodied-hand-lab-main/` root plus `__MACOSX`; no `.git`, unsafe path, or symlink entry |
| Remote | `origin = git@github.com:brVrzl/embodied-hand-lab.git` |
| Main branch | `main` |
| Original local main | `04ba907df002730b80f15df1649e34565e205f78` |
| Original `origin/main` | `04ba907df002730b80f15df1649e34565e205f78` |
| Baseline state | **BASELINE MAIN VERIFIED AND PUSHED**; clean, 0 ahead/behind after fetch |
| Backup branch | `backup/pre-overhaul-integration-20260731-092833` (pushed and hash-verified) |
| Backup tag | `pre-overhaul-integration-20260731-092833` (local) |
| Protected snapshot | `/tmp/embodied-lab-integration-backup-20260731-092833/worktree-protected-snapshot.tar`, SHA-256 `64909a0ca34b807cca7c659cc8b28afc538b9e41d3e0758f42f6bc9aff639d92` |
| Integration branch | `integration/overhaul-validation-20260731-092833` |

The concurrent RH56 task's ten files and its `artifacts/rh56_physical/`
outputs were intentionally excluded from this integration and remain owned by
that task. They were neither overwritten nor staged.

## 2. Candidate comparison and deletion review

The candidate inventory is 705 files, 244 Python files, 56,973 Python lines,
99 Markdown files, 31 YAML files, and 11 maintained shell/Slurm scripts. A
Git-no-index comparison with the baseline reports A=41, D=111, M=87, R=3,
for 242 changed paths, 12,741 insertions, and 18,800 deletions. The previous
maintenance ledger (93 modified, 38 added/migrated, 108 deleted) uses a
different generated-file and rename-count convention; the discrepancy is
explained by Git path classification and the restored protected models.

The deletions were reviewed against source imports, scripts, configuration,
documentation, tests, and dynamic consumers. HEBI, obsolete Python ServoJog/
generic JAKA SDK, old RH56 JAKA-tool/ROS routes, unmaintained robot_bringup/
RViz, ungated one-off physical utilities, duplicate viewers, and stubs were
intentionally removed. `tools/debug_mujoco_jaka_rh56.py` was retained because
the protected MuJoCo teleoperation tool imports it dynamically.

The archive omitted tracked `models/digital_twin/scene.xml` and
`models/digital_twin/workspace_scene_sparse_debug.xml`. They were restored from
the immutable baseline before tests. The only expected protected simulation
changes are the reviewed Link0/Link1 XML and manifest updates.

## 3. Findings and repairs

| Priority | Finding | Action and status |
|---|---|---|
| P0 | None found | No hardware-danger, data-loss, or history-loss issue remains. |
| P1 | Candidate archive omitted two tracked model files | Restored from baseline; protected-asset comparison has 0 missing/unexpected files. **FIXED** |
| P2 | LeRobot extra omitted the `datasets` dependency and the exporter was only covered by a fake SDK | Declared `lerobot[dataset]>=0.6,<0.7`; imported official SDK and completed a real 640x480 two-stream export/reload. **FIXED** |
| P2 | Official SVT-AV1 can fail to terminate for synthetic RGB widths below 32 px | Added pre-encoder width validation; narrow input now fails explicitly. **FIXED** |
| P2 | Native fake timing tests depended on wall-clock scheduling | Added fake-only deterministic clock; vendor timing path is unchanged. **FIXED** |
| P2 | Native motion `--help` could construct a backend before help | Help is parsed and returned before backend construction; fake/vendor safety behavior unchanged. **FIXED** |
| P2 | Clean pytest checkout assumed a prebuilt teleop C++ library | Added a shared session fixture that builds the portable library once. **FIXED** |
| P2 | Delegated CLI help/default benchmark output was inconsistent | Corrected parser program prefixes and safe default output path. **FIXED** |
| P3 | Historical reports described the pre-integration macOS state as final | Added explicit second-round appendices and current Linux results. **FIXED** |

No safety limit, watchdog, stale-command, emergency-stop, ownership, cleanup,
or atomic-write protection was weakened. No secret, private configuration,
model, dataset, calibration, vendor SDK, recording, or experiment result was
added to the candidate commit.

## 4. Independent validation matrix

| Check | Previous report | Current independent result | Status |
|---|---|---|---|
| Package install | `.[dev]` reported pass | Fresh isolated `.[dev]` install completed | PASS |
| Base `pip check` | pass | `No broken requirements found` in fresh dev-only environment | PASS |
| Optional dataset/Torch environment | not independently checked | LeRobot 0.6 imports and real export passes; NVIDIA cuSPARSELt internal SBSA tag makes optional `pip check` warn | PASS with documented platform warning |
| Compileall | pass | `src tools tests` compiled with bytecode disabled | PASS |
| Public imports | not fully evidenced | 13 package imports; no socket/path writes; one read-only GLFW capability subprocess | PASS |
| CLI | help pass | root, 5 top-level, 5 dataset subcommands, and sim smoke help | PASS |
| Pytest | 562 passed, 114 skipped | Candidate validation tree: 681 collected, 680 passed, 1 headless-rendering skip, 0 failed/error. Final clean merge dev-only environment: 681 collected, 679 passed, 2 skips (headless rendering and PyTorch unavailable), 0 failed/error. | PASS; skips are identified and not counted as passes |
| Native JAKA worker | portable build pass | Linux/aarch64 worker and portable resampler build; no CTest entries in this target | PASS (CTest N/A) |
| Native teleop shaping | 3/3 CTest | 3/3 CTest | PASS |
| Native minimal JAKA probe | prior fake evidence | build, safe `--help`, and 21 tests | PASS (no vendor run) |
| MuJoCo smoke | pass | model nq=18, nv=18, nu=12; contacts 0→0; max drift 0 | PASS |
| Benchmark | arm 0.0004646, hand 0.0010603 rad | arm `0.0004646174241849099`, hand `0.001060247811619841`; thresholds unchanged | PASS |
| Gloo single | pass | all-reduce 1.0, 16-sample shard | PASS |
| Gloo two-process | pass; macOS parent uncertain | `torchrun` all-reduce 3.0, disjoint 8/8 shards, clean parent/worker exit | PASS |
| NCCL | not available before | one GPU, one-process all-reduce 1.0 on NVIDIA Thor | PASS (single GPU only) |
| YAML | 13/13 | 31 maintained YAML files, duplicate-key/type/path audit passed | PASS |
| Shell/Slurm | 11/11 syntax | 11/11 `bash -n`; shellcheck and Slurm runtime unavailable | PASS (syntax only) |
| Markdown | 99 files/194 links | current report adds one file; relative links rechecked after final docs | PASS |
| Protected assets | claimed unchanged | 2,210 selected files: 2 expected sim XML/manifest changes; 0 missing/unexpected | PASS |

The only pytest skip is
`tests/test_digital_twin_integrated_workspace.py:152`, because this host has
no configured headless MuJoCo rendering backend. It is not counted as a pass.

## 5. Unverified or unavailable scope

Real JAKA, RH56DFX, Quest 3, RealSense streams, Linux JAKA SDK worker against
a controller, physical teleoperation, physical data collection, policy
training, ACT/Diffusion/OpenPI trainers, multi-GPU NCCL, multi-node rendezvous,
Slurm execution, and Jetson policy inference remain unverified. The optional
NVIDIA cuSPARSELt wheel-tag issue must be resolved or explicitly pinned by the
training environment owner before claiming a clean optional `pip check`.

## 6. Git publish state

The integration branch was pushed ordinarily at
`fa881595778184502a4e8f357429f52651215c6d`. The first merge commit was
`c4c1241e7ac945188dac32ca1f20e090fd9668f8`; the candidate-overlap correction
was merged into main as `6f364063b5b7a3e56aaf2b324f3d74547443ba39`.
Before this report-only documentation update, local `main` and `origin/main`
both resolved to that merge hash and the clean temporary main worktree was
clean. The report update itself is an ordinary documentation-only commit; its
post-update hash is recorded in the final terminal handoff. The concurrent
RH56 task's dirty files remain untouched in the separate integration checkout.

## 7. Manual test entry points

The complete staged manual procedure is
[`docs/validation/MANUAL_FUNCTIONAL_VALIDATION.md`](../../docs/validation/MANUAL_FUNCTIONAL_VALIDATION.md).
It provides exact Level 0 offline commands, Level 1 simulation sequence,
Level 2 no-motion checks, and Levels 3–6 commands with the required
`PHYSICAL MOTION — MANUAL AUTHORIZATION REQUIRED` marker. Physical task stages
that are not implemented are explicitly labelled and must not be claimed as
validated.

Recommended next step: reproduce the Level 0 baseline, resolve the optional
training-wheel warning, then obtain separate human authorization for Level 2
device checks. Do not begin Level 3 or Level 5 motion until every prior gate
and the concurrent RH56 change set have been independently reviewed.
