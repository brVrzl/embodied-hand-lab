# Main documentation refresh — 2026-07-28

## Scope and evidence rules

This audit records removal of four explicitly abandoned linked worktrees and a
complete review of repository documentation. The inventory covers 86
repository documents after this report: 81 Markdown files and five tracked
`.txt` files (four CMake build files plus one vendor data file). Ignored
environment/package documentation under `.venv`, `.pytest_cache`, build trees,
and local artifacts is not project documentation and was excluded.

Facts were resolved in this order:

1. executable source;
2. selected configuration;
3. tests;
4. launchers and CLI help;
5. MuJoCo XML/runtime model construction;
6. current `main` history;
7. the latest audits;
8. historical research records.

No production code, configuration, test, model, real device, protected
worktree, local spreadsheet, or `.git/info/exclude` entry was changed.

## Removal gate and result

Before removal, local `main` and `origin/main` were both
`ce7735119505f51bfcf68479d5349b49a209a2ce`, with ahead/behind `0/0`. The main
worktree was clean, its stash was empty, and the local spreadsheet was hidden
only by the exact local exclude rule
`/关节角与0-1000 对应关系  .xls`.

The user explicitly abandoned every staged, modified, and untracked file in
the following worktrees:

| Removed path | Branch | HEAD | Staged / modified / untracked | Result |
|---|---|---|---:|---|
| `${PROJECTS_ROOT}/embodied_lab_quest_controller_transport` | `feature/quest-controller-transport-host` | `530d3a0aa75306248998f563fb981cb4a2c31790` | 0 / 8 / 6 | exact-path `git worktree remove --force` succeeded |
| `${PROJECTS_ROOT}/embodied_lab_quest_jaka_arm_audit` | `feature/quest-jaka-arm-teleop-audit` | `4a0b5e4465b47e753f1113c426b7ee3988e8ba0c` | 0 / 0 / 1 | exact-path removal succeeded |
| `${PROJECTS_ROOT}/embodied_lab_quest_jaka_dual_clutch` | `feature/quest-jaka-dual-clutch-checkpoint` | `4a0b5e4465b47e753f1113c426b7ee3988e8ba0c` | 0 / 5 / 2 | exact-path removal succeeded |
| `${PROJECTS_ROOT}/embodied_lab_quest_jaka_sim` | `feature/quest-jaka-offline-simulation` | `8f72e7b40c6ea31674c81aa9c82eabfe134d1095` | 0 / 7 / 16 | exact-path removal succeeded |

All four stashes were empty and all four current-branch unique-commit counts
relative to remotes were zero. Each HEAD was proven to be an ancestor of both
local and remote `main`. `git worktree prune --dry-run --verbose` printed no
remaining metadata candidate, so no prune mutation was needed.

The four local branches were deleted with `git branch -d`; `-D` was not
needed. The four same-name remote branches were not present when checked with
`git ls-remote --heads`, so no remote deletion was performed. A later fetched
ref check also found all four absent.

## Current project baseline

- PWL/root-cause-fix commit
  `0c30c2d2e550f7da7f3648e4d1849a78adaf38e1` is in local and remote `main`.
- RH56 completion
  `b2a4cec57837b027b7ac1aad9811c346d89cd112` and merge
  `8ea39c92a5523c73229c106ffb4c98382658a7f4` are in local and remote `main`.
- `root_cause_fix` is the normal launcher/CLI simulation profile.
- Target generation and IK are 60 Hz. Output-acceleration feasibility is
  evaluated over 16,666,667 ns; the ServoJ/EDG contract remains 8,000,000 ns.
- Shared output limits are `pi` rad/s and `4*pi` rad/s² per arm joint. The
  command jerk diagnostic boundary is `20*pi` rad/s³.
- Input interpolation delay is 20 ms. TCP linear/angular caps are 1.0 m/s and
  5.0 rad/s. The selected rotation filter is min cutoff 1.5, beta 4.0, and
  derivative cutoff 1.0.
- The integrated live configuration enables six JAKA and six RH56 simulation
  actuators. The explicit JAKA-only builder remains a separate invariant:
  exactly six JAKA actuators and no RH56 command path.
- Left index controls the arm; left grip controls only the simulated hand.
  Quest-to-physical-RH56 control is not validated.
- No post-acceleration-fix physical JAKA gate has been run. The historical J4
  collision cause remains unresolved and TCP remains recorded as zero.
- Teleoperation rearchitecture remains an independent active worktree, six
  commits ahead of its remote and dirty; it is not in the production baseline.
- MoveIt, Ruckig, ACT/Thor, TeleDex, and repository-cleanup remain remote
  archives. OpenPI remains a clean sibling checkout at
  `15a9616a00943ada6c20a0f158e3adb39df2ccac`, used only by the inference-only
  π0.5-DROID shadow path.

One source/documentation discrepancy remains: the live YAML and Python entry
enable RH56 simulation, but `scripts/run_quest_jaka_sim_demo.sh` still prints a
stale “GRIP disabled” banner. This documentation refresh records the
discrepancy but does not modify production code.

## Documentation inventory

The `last commit` column is the latest commit touching the file after the
logical documentation commits in this refresh. “Historical correction” means
only a status box, current pointer, broken link, or final disposition was
updated; original measurements and conclusions were preserved.

### Updated current documents (15)

| Path | Topic | Last commit | Stale claim/action |
|---|---|---|---|
| `README.md` | project entry and validation boundary | `274cd08` | integrated/arm-only RH56 distinction, archive and research state; `UPDATE` |
| `THIRD_PARTY_NOTICES.md` | dependency attribution | `96e5000` | new provenance/license boundary; `UPDATE` |
| `docs/README.md` | documentation navigation | this report commit (prior `96e5000`) | audit and attribution navigation; `UPDATE` |
| `docs/architecture/overview.md` | current control architecture | `274cd08` | nonexistent test and RH56 model role corrected; `UPDATE` |
| `docs/development/repository_layout.md` | source tree roles | `274cd08` | added `rh56_sim` and inference-only policy area; `UPDATE` |
| `docs/motion_input/QUEST_CONTROLLER_TRANSPORT_HOST.md` | CTRL input gate | `274cd08` | deleted-worktree command and fixed host IP corrected; `UPDATE` |
| `docs/motion_input/README.md` | input platform | `274cd08` | nonexistent test path corrected; `UPDATE` |
| `docs/operation/rh56_h0_self_test.md` | offline H0 | `274cd08` | deleted-worktree absolute invocation corrected; `UPDATE` |
| `docs/operation/rh56_operation.md` | RH56 roles | `274cd08` | integrated simulation, arm-only boundary, H0 entry added; `UPDATE` |
| `docs/operation/simulation_demo.md` | live/replay simulation | `274cd08` | 6+6 integrated model, 6-only arm model, effective PWL/filter limits, stale banner disclosed; `UPDATE` |
| `docs/reference/command_reference.md` | safe commands | `274cd08` | H0 help entry added; `UPDATE` |
| `docs/status/current_status.md` | authoritative state | `274cd08` | RH56 merge, archives, research worktree, OpenPI, worktree removal; `UPDATE` |
| `docs/status/known_limitations.md` | current limitations | `274cd08` | dynamic HEAD wording and stale launcher banner corrected; `UPDATE` |
| `docs/status/validation_matrix.md` | evidence levels | `274cd08` | integrated RH56 and arm-only validation split; `UPDATE` |
| `docs/audits/main_documentation_refresh_2026-07-28.md` | this audit | this report commit | new complete inventory; `UPDATE` |

### Marked historical (21)

| Path | Topic | Last commit | Action |
|---|---|---|---|
| `docs/audits/legacy_quest_worktrees_audit_2026-07-28.md` | legacy worktree evidence | `6c424c7` | status box and final user disposition; `MARK_HISTORICAL` |
| `docs/audits/projects_workspace_cleanup_2026-07-28.md` | earlier workspace state | `6c424c7` | current pointer; `MARK_HISTORICAL` |
| `docs/audits/projects_workspace_consolidation_2026-07-28.md` | pre-integration workspace | `6c424c7` | current pointer; `MARK_HISTORICAL` |
| `docs/d435_algorithm_selection_20260713.md` | dated D435 comparison | `6c424c7` | dated status and current pointer; `MARK_HISTORICAL` |
| `docs/d435_depth_quality_assessment_20260713.md` | dated depth evidence | `6c424c7` | dated status and current pointer; `MARK_HISTORICAL` |
| `docs/development/repository_consolidation_audit.md` | 2026-07-24 cleanup | `6c424c7` | status box/current layout pointer; `MARK_HISTORICAL` |
| `docs/history/archived_designs/motion_input/HAND_TRACKING_STREAMER_INTEGRATION.md` | streamer integration checkpoint | `6c424c7` | historical box; `MARK_HISTORICAL` |
| `docs/history/archived_designs/motion_input/QUEST_JAKA_OFFLINE_SIMULATION.md` | early offline simulation | `6c424c7` | historical box and current simulation link; `MARK_HISTORICAL` |
| `docs/history/archived_designs/motion_input/QUEST_JAKA_RH56_PRECISION_DUAL_CLUTCH.md` | dual-clutch design | `6c424c7` | historical box/current links; `MARK_HISTORICAL` |
| `docs/history/gates/jaka_foundation_20260716/jaka_control_foundation_gates_1_2.md` | foundation gates 1–2 | `6c424c7` | historical box; `MARK_HISTORICAL` |
| `docs/history/gates/jaka_foundation_20260716/jaka_control_foundation_gates_1_2_implementation_report_20260716.md` | gate implementation | `6c424c7` | historical box; `MARK_HISTORICAL` |
| `docs/history/gates/jaka_foundation_20260716/jaka_gate3a_readonly_validation_20260716.md` | read-only physical gate | `6c424c7` | bounded historical box; `MARK_HISTORICAL` |
| `docs/history/gates/jaka_foundation_20260716/jaka_gate3b_stage3_state_preparation_review_20260716.md` | SDK state review | `6c424c7` | historical box; `MARK_HISTORICAL` |
| `docs/history/gates/jaka_foundation_20260716/jaka_gate3b_zero_motion_validation_20260716.md` | zero-motion gate | `6c424c7` | bounded historical box; `MARK_HISTORICAL` |
| `docs/history/gates/jaka_foundation_20260716/jaka_gate3c_5degree_joint6_plan_20260716.md` | +5-degree J6 gate | `6c424c7` | bounded historical box; `MARK_HISTORICAL` |
| `docs/history/gates/jaka_foundation_20260716/jaka_gate3c_minimal_joint_validation_20260716.md` | minimal J6 gate | `6c424c7` | bounded historical box; `MARK_HISTORICAL` |
| `docs/history/incidents/quest_jaka_20260722_23/quest_jaka_output_feasibility_followup_20260723.md` | output incident | `6c424c7` | historical box/root-cause pointer; `MARK_HISTORICAL` |
| `docs/history/incidents/quest_jaka_20260722_23/quest_jaka_physical_parity_audit_20260722.md` | parity audit | `6c424c7` | historical box and broken guide reference corrected; `MARK_HISTORICAL` |
| `docs/history/incidents/quest_jaka_20260722_23/quest_jaka_physical_parity_followup_20260722.md` | parity follow-up | `6c424c7` | historical box; `MARK_HISTORICAL` |
| `docs/history/incidents/quest_jaka_20260722_23/quest_jaka_singularity_liveness_followup_20260723.md` | singularity/liveness incident | `6c424c7` | historical box; `MARK_HISTORICAL` |
| `learned_policy/pi05_shadow/VALIDATION_REPORT.md` | dated OpenPI host evidence | `6c424c7` | historical environment box/current safety pointer; `MARK_HISTORICAL` |

### Reviewed and kept unchanged (50)

| Path | Topic/type | Last commit | Action |
|---|---|---|---|
| `AGENTS.md` | repository working rules | `e7b111c` | `KEEP` |
| `data/sim_assets/README.md` | simulation assets and Correll attribution | `b2a4cec` | `KEEP` |
| `digital_twin/calibration/README.md` | calibration module | `ac7399b` | `KEEP` |
| `digital_twin/exporters/README.md` | exporters module | `ac7399b` | `KEEP` |
| `digital_twin/scene/README.md` | scene module | `ac7399b` | `KEEP` |
| `docs/architecture/coordinate_frames.md` | current coordinate contract | `2773988` | `KEEP` |
| `docs/architecture/shared_target_pipeline.md` | current target contract | `879848b` | `KEEP` |
| `docs/architecture/simulation_hardware_parity.md` | adapter parity | `2773988` | `KEEP` |
| `docs/d435_depth_pointcloud_readiness.md` | current D435 readiness | `ac7399b` | `KEEP` |
| `docs/development/build.md` | build guide | `2773988` | `KEEP` |
| `docs/development/configuration.md` | config precedence | `2773988` | `KEEP` |
| `docs/development/contribution_workflow.md` | contribution workflow | `2773988` | `KEEP` |
| `docs/development/logging_and_replay.md` | evidence retention | `2773988` | `KEEP` |
| `docs/development/setup.md` | development setup | `2773988` | `KEEP` |
| `docs/development/testing.md` | test guide | `2773988` | `KEEP` |
| `docs/digital_twin/BASE_REGISTRATION_REPORT.md` | explicitly historical registration evidence | `35ff114` | `KEEP` |
| `docs/digital_twin/CALIBRATION_PLAN.md` | current calibration plan | `ac7399b` | `KEEP` |
| `docs/digital_twin/CAPTURE_01_02_REPORT.md` | explicitly scoped capture evidence | `ac7399b` | `KEEP` |
| `docs/digital_twin/MEASUREMENT_REQUEST.md` | current measurement request | `ac7399b` | `KEEP` |
| `docs/digital_twin/README.md` | current digital-twin state | `2773988` | `KEEP` |
| `docs/history/README.md` | history retention index | `35ff114` | `KEEP` |
| `docs/motion_input/COORDINATE_FRAMES.md` | UMIP frames | `8ecd70b` | `KEEP` |
| `docs/motion_input/QUEST_SDK_REVIEW.md` | SDK/OpenXR review | `8ecd70b` | `KEEP` |
| `docs/motion_input/UMIP_PROTOCOL.md` | UMIP protocol | `35ff114` | `KEEP` |
| `docs/operation/hardware_prerequisites.md` | physical gate prerequisites | `2773988` | `KEEP` |
| `docs/operation/jaka_arm_teleoperation.md` | current gated arm entry | `5401f3c` | `KEEP` |
| `docs/operation/quest_setup.md` | current host/input setup | `2773988` | `KEEP` |
| `docs/operation/troubleshooting.md` | current diagnostics | `2773988` | `KEEP` |
| `docs/reference/config_reference.md` | config inventory | `2773988` | `KEEP` |
| `docs/reference/glossary.md` | terminology | `2773988` | `KEEP` |
| `docs/reference/log_schemas.md` | log schemas | `2773988` | `KEEP` |
| `docs/safety/controller_configuration.md` | controller write boundary | `2773988` | `KEEP` |
| `docs/safety/incident_response.md` | incident handling | `2773988` | `KEEP` |
| `docs/safety/physical_test_gates.md` | physical gate policy | `2773988` | `KEEP` |
| `docs/safety/safety_model.md` | safety states | `2773988` | `KEEP` |
| `integrations/quest_unity/README.md` | input-only Unity bridge | `8ecd70b` | `KEEP` |
| `learned_policy/pi05_shadow/README.md` | current inference-only boundary | `f270cf0` | `KEEP` |
| `native/jaka_minimal_joint_probe/CMakeLists.txt` | native build metadata | `bc15d55` | `KEEP` |
| `native/jaka_readonly_diagnostic/CMakeLists.txt` | native build metadata | `bc15d55` | `KEEP` |
| `native/jaka_servo_worker/CMakeLists.txt` | native build metadata | `bc15d55` | `KEEP` |
| `native/jaka_zero_motion_probe/CMakeLists.txt` | native build metadata | `bc15d55` | `KEEP` |
| `src/embodiment_core/README.md` | core package | `87684b2` | `KEEP` |
| `src/jaka_driver_adapter/README.md` | JAKA adapter package | `a807bc3` | `KEEP` |
| `src/rh56_driver/README.md` | RH56 driver package | `a807bc3` | `KEEP` |
| `src/robot_bringup/README.md` | bring-up package | `87684b2` | `KEEP` |
| `src/teleop_tools/LEGACY.md` | legacy boundary | `bc15d55` | `KEEP` |
| `src/teleop_tools/README.md` | parallel teleop tools | `35ff114` | `KEEP` |
| `src/vision_interface/README.md` | vision interface | `ac7399b` | `KEEP` |
| `third_party/inspire_hand/rh56/examples/DefaultAction.txt` | vendor data | `a807bc3` | `KEEP` |
| `third_party/inspire_hand/rh56/readme.md` | vendor design note | `a807bc3` | `KEEP` |

No document met the `DELETE_OBSOLETE` threshold. No document remains
`NEEDS_MANUAL_REVIEW` for current operating accuracy; the attribution risks
below remain explicitly bounded rather than silently resolved.

## Paths, links, commands, and attribution

- Current documentation contains no reference to the four removed worktree
  paths or the three removed reference-clone paths. Historical audits/designs
  retain such paths only as dated evidence.
- Two broken references to the retired
  `docs/quest_jaka_sim_teleoperation.md` path now point to
  `docs/operation/simulation_demo.md`.
- The transport-only gate now runs from repository root with
  `.venv/bin/python` and `<HOST_IPV4>`, rather than a sibling worktree and a
  recorded LAN address.
- The motion-input test list now names existing HTS protocol/canonical tests.
- A URL-decoding Markdown link check examined 154 local links and found zero
  missing targets after this report and its index link were added.
- Fifteen documented safe `--help` commands exited zero. No device connection
  or control command was executed.
- Correll reference assets retain their MIT license. The repository previously
  had no top-level third-party notice; `THIRD_PARTY_NOTICES.md` now records
  Correll attribution and explicitly marks the JAKA SDK and Inspire RH56 vendor
  snapshots as lacking redistributable license evidence in this checkout.
  OpenPI remains external and governed by its own checkout/license.

## Validation

| Check | Result |
|---|---|
| Full pytest | PASS — 616 passed, 1 skipped in 72.06 s |
| Python compileall (`src tools tests`) | PASS |
| All shell files under `scripts` and `tools` via `bash -n` | PASS |
| Markdown local links | PASS — 154 checked, 0 missing |
| Current-doc removed local paths | PASS — 0 |
| Safe CLI/help references | PASS — 15/15 |
| `git diff --check` | PASS |
| Sensitive-pattern scan of changed text | PASS |
| Binary additions | PASS — none |
| Local spreadsheet ignored and untracked | PASS |
| `.gitignore` diff | PASS — none |
| `.git/info/exclude` tracked/staged | PASS — no |
| JAKA-only six-actuator/root-cause timing invariants | PASS through full suite, including `test_quest_jaka_single_arm_final.py` |
| RH56 H0/H2/thumb and integrated simulation tests | PASS through full suite |

## Protected work

The RH56 worktree remained clean at
`b2a4cec57837b027b7ac1aad9811c346d89cd112`. The rearchitecture worktree
remained at `4e6cca5f8439e60d2137e7fcb1837d947508b6fb`, six commits ahead of its
remote, with its existing modified and untracked files. Neither was modified.
The `openpi` checkout remained clean at
`15a9616a00943ada6c20a0f158e3adb39df2ccac`.

## Remaining issue

The stale shell-wrapper “GRIP disabled” banner conflicts with the effective
hand-enabled live configuration. Correcting it requires a production-script
change and was intentionally left outside this documentation-only refresh.
