# Legacy Quest worktrees audit — 2026-07-28

## Scope and safety boundary

This report records:

1. verification of the completed RH56 merge;
2. deletion of three reference clones explicitly abandoned by the user; and
3. a read-only audit of four legacy linked worktrees.

No file in the four audited worktrees was modified, staged, committed,
stashed, restored, reset, cleaned, switched, merged, rebased, pushed, moved,
or deleted. No JAKA, Quest, or RH56 hardware was started or commanded.

## Main and RH56 merge state

After `git fetch --all --prune`:

| Check | Result |
| --- | --- |
| Main worktree branch | `main` |
| Local `main` | `8ea39c92a5523c73229c106ffb4c98382658a7f4` |
| `origin/main` | `8ea39c92a5523c73229c106ffb4c98382658a7f4` |
| `main...origin/main` | ahead 0 / behind 0 |
| Main worktree | clean |
| Local-only spreadsheet | still ignored precisely by `.git/info/exclude` and untracked |

The RH56 completion was identified from current history rather than inferred
from the old workspace snapshot:

| Item | Evidence |
| --- | --- |
| Completion branch | `feature/quest-rh56-hand-teleop-sim` |
| Completion commit | `b2a4cec57837b027b7ac1aad9811c346d89cd112` — `feat(sim): restore and calibrate Quest RH56 hand teleoperation` |
| Merge commit | `8ea39c92a5523c73229c106ffb4c98382658a7f4` — `merge: Quest RH56 hand teleoperation simulation` |
| Merge parents | `93cec76585d1fd2704d5fc2b7b33e21316409b40` and `b2a4cec57837b027b7ac1aad9811c346d89cd112` |
| Local `main` contains completion | yes; `merge-base --is-ancestor` returned success |
| `origin/main` contains completion | yes; `merge-base --is-ancestor` returned success |
| RH56 linked worktree | remains at `${PROJECTS_ROOT}/embodied-hand-lab-quest-rh56-hand-teleop-sim` |
| RH56 branch refs | local and `origin/feature/quest-rh56-hand-teleop-sim` both remain |
| RH56 worktree state | clean at the audit snapshot |

No RH56 ref or worktree was changed by this task.

## Approved reference deletion

For each path, the deletion gate proved that the path:

- existed as a directory;
- was not a symbolic link;
- resolved exactly to the named first-level child of
  `/home/thor/projects`;
- was neither `/home/thor/projects` nor the main repository; and
- was not registered by `git worktree list --porcelain`.

The user explicitly approved abandoning all dirty/staged/untracked local
content in these three directories. No backup or checkpoint was made.

| Deleted path | Branch | HEAD | Remote | Dirty state before deletion | Result |
| --- | --- | --- | --- | --- | --- |
| `/home/thor/projects/_reference_Open-Teach` | `main` | `32a7d44b33953066ff27312a7b2b4c294f4f52c5` | `https://github.com/aadhithya14/Open-Teach.git` | 12,042 staged deletions; no worktree files | deleted |
| `/home/thor/projects/_reference_hand-tracking-streamer` | `feature/quest-mixed-input-probe` | `5ff7c1cfea0ead1bb8a0e233bc7770d94d31feb5` | `https://github.com/wengmister/hand-tracking-streamer.git` | 1 modified Unity scene; 2 untracked probe files | deleted |
| `/home/thor/projects/_reference_AnyDexRetarget_sparse` | `master` | `77c0a1074ba6eb003159da37b2bd3cec41792523` | `https://github.com/qqsq12321/AnyDexRetarget.git` | clean | deleted |

Each directory was removed separately with an exact
`rm -r --one-file-system -- <path>` invocation and immediately checked for
absence. No wildcard deletion was used. `openpi` was not deleted.

## Four-worktree Git summary

Dirty counts are `staged/modified/untracked`.

| Worktree | Branch / HEAD | Upstream | HEAD vs `main` | Dirty | Stash | Current-branch commits outside remotes |
| --- | --- | --- | --- | ---: | ---: | ---: |
| `embodied_lab_quest_controller_transport` | `feature/quest-controller-transport-host` / `530d3a0aa75306248998f563fb981cb4a2c31790` | none | 0 ahead / 44 behind; ancestor of local and remote main | 0/8/6 | 0 | 0 |
| `embodied_lab_quest_jaka_arm_audit` | `feature/quest-jaka-arm-teleop-audit` / `4a0b5e4465b47e753f1113c426b7ee3988e8ba0c` | none | 0 ahead / 46 behind; ancestor of local and remote main | 0/0/1 | 0 | 0 |
| `embodied_lab_quest_jaka_dual_clutch` | `feature/quest-jaka-dual-clutch-checkpoint` / `4a0b5e4465b47e753f1113c426b7ee3988e8ba0c` | none | 0 ahead / 46 behind; ancestor of local and remote main | 0/5/2 | 0 | 0 |
| `embodied_lab_quest_jaka_sim` | `feature/quest-jaka-offline-simulation` / `8f72e7b40c6ea31674c81aa9c82eabfe134d1095` | none | 0 ahead / 47 behind; ancestor of local and remote main | 0/7/16 | 0 | 0 |

All four share the common Git directory
`/home/thor/projects/embodied_lab/.git`.

The conclusion “HEAD is in main” applies only to committed history. It does
not apply to modified or untracked files.

## A. Controller transport

### What the committed branch did

The complete committed history is already in both local and remote `main`:

- `9d97a8dadf651a2b3c40af7d3bff9daaa662a6e3`,
  `feat: add Quest controller transport gate`, added the controller wire
  protocol, provider abstraction, transport gate, CLI, documentation, and
  tests.
- `530d3a0aa75306248998f563fb981cb4a2c31790`,
  `Add precise dual-clutch Quest MuJoCo teleoperation`, added the live
  controller router, precise arm/hand dual-clutch simulation integration,
  configuration changes, and focused tests.

These commits remain reachable through `main`; the branch has no unpushed
commit.

### What remains unique in the working tree

All 14 dirty-path file blobs are absent from every currently reachable local
or remote commit. The exact snapshots are therefore locally unique. This does
not mean every individual line or idea is unique: the worktree is based 44
commits behind current main, and its eight tracked files diverge from later
PWL and RH56 implementations. Compared directly with current main those
tracked files would add 781 lines and remove 2,281 lines, so they must not be
copied or merged wholesale.

| Path | Kind | Function | Relationship to current main |
| --- | --- | --- | --- |
| `src/jaka_driver_adapter/servo_jog.py` | modified source; **physical arm command path** | Exposes IK tuning, prime state, saturation-watchdog reset behavior, and palm-target controller changes | Path exists but exact working blob is unique; current main contains later PWL/startup/trajectory changes that this snapshot lacks |
| `src/quest_jaka_sim/hand_retarget.py` | modified source | Adds canonical normalized/MuJoCo conversions used by the proposed real hand bridge | Path exists and current RH56 implementation has evolved substantially; selective semantic port only |
| `src/quest_jaka_sim/simulation.py` | modified source | Synchronizes authoritative real joints into mounted-palm FK and shared feasibility preview | Path exists; snapshot predates later PWL/root-cause and RH56 simulation work |
| `src/quest_jaka_sim/smooth_session.py` | modified source | Seeds hand hold from feedback and exposes command-update state for a physical sink | Path exists; snapshot omits later RH56 grip/thumb and telemetry behavior |
| `src/rh56_driver/jaka_tool_backend.py` | modified source; **physical hand command/feedback path** | Shares the existing JAKA SDK session with tool-RS485 hand control and reads hand feedback through SDK signals | Path exists; exact shared-session implementation is not in main |
| `src/robot_bringup/ros2_bridge.py` | modified source; **physical arm/hand bridge** | Injects one shared JAKA backend, exposes arm IK parameters, and wires cleanup | Path exists; exact changes are unique and affect real hardware lifecycle |
| `tests/test_jaka_servo_jog.py` | modified tests | Tests shared IK parameters and transient saturation-watchdog behavior | Main has later tests; these assertions require manual reconciliation |
| `tests/test_rh56_jaka_tool_backend.py` | modified tests | Tests shared injected JAKA backend and SDK feedback without a second login | Exact test is unique and is useful evidence for any future shared-session rewrite |
| `docs/motion_input/QUEST_JAKA_RH56_REAL_TELEOP.md` | untracked documentation | Describes real feedback path, frame finding, shared SDK fix, execution gates, and a dry-run result | Absent from main; historically useful, but contains dated physical procedure and host-specific paths |
| `scripts/quest_jaka_real_teleop.sh` | untracked launcher; **can start physical-output tool** | Sources ROS2 and launches the real teleop tool | Absent from main; contains a local absolute Python path |
| `src/quest_jaka_real/__init__.py` | untracked source | Exports fail-closed command-policy helpers | Absent from main |
| `src/quest_jaka_real/commands.py` | untracked source; physical command contract | Pure builders for deadman/target payloads and feedback-seeded, slew-limited RH56 commands; it opens no SDK/socket itself | Absent from main; valuable as a design/test unit but depends on older simulation APIs |
| `tests/test_quest_jaka_real_commands.py` | untracked tests | Tests zero-jump actual-joint sync, explicit stop, hand feedback seeding, dry-run default, acknowledgment gate, and no SDK connection on import | Absent from main; useful candidate after adapting to current APIs |
| `tools/quest_jaka_real_teleop.py` | untracked source; **publishes real ROS commands when explicitly enabled** | Reads Quest UDP and real ROS feedback; dry-run by default; with execute and acknowledgment flags publishes JAKA palm targets and RH56 commands | Absent from main; live Quest validation was still incomplete and it must not enter main without a fresh hardware/safety design review |

The untracked tool uses ROS2 topics and explicit
`--execute-arm`/`--execute-hand` plus `--acknowledge-real-hardware` gates. It
is the only file in this worktree that directly creates real command
publishers; the launcher can invoke it. The tracked servo/ROS/backend files
also sit on physical command paths.

### Disposition

**Classification: `NEEDS_REWRITE_BEFORE_MERGE`.**

The committed controller transport and simulation dual-clutch work is already
in main. The locally unique follow-up is a real arm/hand output prototype
based on an older control stack. Its pure command policy, shared-SDK regression
test, and safety documentation may be reusable, but the physical sender and
tracked patches require a deliberate port onto current main.

User options:

1. extract only the pure command policy, selected tests, and historical design
   document into a new review branch, then rewrite the physical adapter against
   current main;
2. checkpoint the complete dirty state to a remote archive branch without
   merging it; or
3. explicitly abandon the real-output prototype and remove the worktree.

## B. JAKA arm audit

### What the committed branch did

Its HEAD `4a0b5e4...` is the already-merged provider-independent dual-clutch
checkpoint. There are no additional commits and no code modifications.

### Unique document

The only dirty path is the 471-line, 44,648-byte untracked document:

`docs/motion_input/QUEST_JAKA_ARM_TELEOP_AUDIT_20260721.md`

It is absent from main and from every reachable commit. It records:

- the full Quest UDP → canonical pose → buffered/filtering → relative SE(3) →
  continuation IK → bounded MuJoCo actuator call chain;
- a pinned external-source ledger for Unitree, Open-Teach, PickNik,
  AnyTeleop, Quest2ROS2, XRoboToolkit, AnyDexRetarget, and dex-retargeting;
- a concern-by-concern comparison matrix;
- a 27-case test coverage map; and
- a 92-test provider-independent baseline.

The central conclusion was to preserve the relative transform order,
authoritative actual-state reference capture, latched head yaw,
quaternion-safe filtering, hold-to-run state machines, continuation IK, and
hold-last rejection behavior.

It identified two concrete checkpoint defects:

1. `rates.ik_hz` was not an independent scheduler; and
2. effective buffered-pose age was reported as latest raw observation age.

Current main now explicitly checks that `ik_hz` equals target generation rate
in the hardware entry, so the first finding is partly superseded as a
configuration-contract issue. The old `right_wrist_sample_age_ms` and
`arm_target_latency_ms` fields still use `right_wrist_age_s`, so the second
telemetry warning remains relevant to the legacy simulation report. Current
main also has newer pose-validation tests, PWL incident documents, and the two
workspace audits, but none contains this external comparison ledger or its
complete coverage matrix.

Deleting the worktree without extraction would lose that research provenance,
the dated call-chain snapshot, the two defect findings, and the explicit test
gap list. It would not lose executable code.

### Disposition

**Classification: `KEEP_AS_DOCUMENTATION`.**

User options:

1. submit the document as a clearly dated research/audit artifact after adding
   a short “current status” note;
2. checkpoint it to an archive branch without putting it in main; or
3. explicitly discard the historical audit and remove the worktree.

## C. Dual-clutch checkpoint

### What the committed branch did

Commit `4a0b5e4465b47e753f1113c426b7ee3988e8ba0c`,
`feat: checkpoint provider-independent Quest dual clutch`, is already in local
and remote main. It added the live simulation config, hand retarget config,
independent analog hold-to-run arm/hand clutch machines, precise SE(3)
mapping/filtering, RH56 retargeting, smooth session, tests, and documentation.
Later commits in main, including the controller transport, PWL, and completed
RH56 work, build on it.

### Unique working content

All seven working blobs are absent from reachable commits. The patch from its
old HEAD is 204 insertions and 8 deletions.

| Path | Kind | Function | Relationship to current main |
| --- | --- | --- | --- |
| `src/quest_jaka_sim/keyboard_clutch.py` | untracked source | Explicit simulation-only Space/H hold-to-run provider; focus loss and Escape fail released; arm-only mode disables hand | Completely absent from main; self-contained extraction candidate |
| `tests/test_keyboard_developer_clutch.py` | untracked tests | Tests hold-to-run rather than toggle, focus/exit release, arm-only behavior, release-before-press, and explicit CLI selection | Completely absent from main; accompanies keyboard provider |
| `src/quest_jaka_sim/__init__.py` | modified source | Exports the keyboard provider | Current main export surface has advanced; tiny manual port only |
| `tools/quest_jaka_mujoco_sim.py` | modified tool | Adds an explicit Tk keyboard window, `--clutch-provider keyboard-developer`, `--arm-only`, independent polling, and viewer disclosure | Current main tool has advanced significantly; manual integration required |
| `src/quest_jaka_sim/simulation.py` | modified source | Adds mean/p95/max desired-to-simulated TCP error metrics and keeps measurements across recapture | Current main simulation has PWL/RH56 changes; extract metrics concept only |
| `src/quest_jaka_sim/smooth_session.py` | modified source | Adds capture jump distributions, reject/fault counts, and keyboard-specific fault text | Current main session has substantially newer RH56 behavior; extract selected observability only |
| `tests/test_quest_jaka_smooth.py` | modified tests | Asserts zero position/orientation jump at reference capture | Current main has newer test structure; preserve the assertion concept rather than the old file |

This prototype is simulation-only and contains no hardware sender. The
already-merged Quest grip/trigger dual-clutch implementation does not make the
keyboard provider redundant: the keyboard provider is an explicit developer
fallback selected by CLI, not inferred Quest controller input.

### Disposition

**Classification: `EXTRACT_SELECTED_FILES`.**

User options:

1. port `keyboard_clutch.py`, its tests, and selected zero-jump/metric
   assertions onto current main;
2. checkpoint the complete prototype on a remote archive branch; or
3. explicitly abandon the developer-keyboard provider and remove the
   worktree.

## D. Offline simulation

### What the committed branch did

Commit `8f72e7b40c6ea31674c81aa9c82eabfe134d1095`,
`feat: add offline Quest to JAKA MuJoCo gate`, is already in local and remote
main. It introduced the recorded Quest → mapped TCP → feasibility/IK →
MuJoCo position-actuator pipeline, its configuration, tests, viewer tool, and
initial documentation. The later merged `4a0b5e4`, `530d3a0`, PWL commits, and
RH56 completion evolve this foundation.

### Working-content recoverability

There are 23 dirty paths: 7 modified and 16 untracked.

- 17 exact working blobs already exist in reachable Git history.
- Of those, three are also byte-identical to current main:
  `src/motion_input/hts_operator.py`, `src/quest_jaka_sim/mapping.py`, and
  `src/quest_jaka_sim/smooth_operator.py`.
- The other 14 recoverable blobs are historical versions now superseded by
  current main.
- Six exact snapshots are not reachable from any commit:
  three documents, the local `se3.py` snapshot, and two offline diagnostic
  tools.

| Path | State | Recoverability and meaning |
| --- | --- | --- |
| `docs/motion_input/QUEST_JAKA_OFFLINE_SIMULATION.md` | modified; unique blob | Historical translation/6-DoF replay results, axis observations, jitter diagnosis, and pending gates; absent from main |
| `src/motion_input/hts_operator.py` | modified relative to old HEAD | Byte-identical to current main; no unique content |
| `src/quest_jaka_sim/__init__.py` | modified | Exact historical blob is reachable; current main has newer exports |
| `src/quest_jaka_sim/mapping.py` | modified | Byte-identical to current main |
| `src/quest_jaka_sim/simulation.py` | modified | Exact historical blob is reachable; current main has later PWL/root-cause and RH56 behavior |
| `tests/test_quest_jaka_sim.py` | modified | Exact historical blob is reachable; current tests have advanced |
| `tools/quest_jaka_mujoco_sim.py` | modified | Exact historical blob is reachable; current tool has advanced |
| `configs/sim/quest_hts_jaka_mini2_live_demo.yaml` | untracked at this old HEAD | Exact blob is reachable; current main has a newer version |
| `configs/sim/quest_rh56_retarget.yaml` | untracked at this old HEAD | Exact blob is reachable; current main has completed RH56 calibration changes |
| `docs/motion_input/QUEST_JAKA_RH56_PRECISION_DUAL_CLUTCH.md` | untracked; unique blob | Historical design, external-reference review, bounded replay results, and remaining gaps; absent from main |
| `docs/motion_input/QUEST_RH56_ANYDEX_RETARGETING.md` | untracked; unique blob | AnyDex compatibility/licensing audit and project-native retarget design; absent from main |
| `src/quest_jaka_sim/clutch.py` | untracked at this old HEAD | Exact content is reachable in history; current implementation has evolved |
| `src/quest_jaka_sim/hand_retarget.py` | untracked at this old HEAD | Exact historical blob is reachable; current RH56 retargeting is newer |
| `src/quest_jaka_sim/precision_mapping.py` | untracked at this old HEAD | Exact historical blob is reachable; current mapping is newer |
| `src/quest_jaka_sim/se3.py` | untracked; unique blob | Older snapshot that lacks current main's swing/twist and bounded-pose-step functions; unique hash but functionally superseded |
| `src/quest_jaka_sim/smooth_operator.py` | untracked at this old HEAD | Byte-identical to current main |
| `src/quest_jaka_sim/smooth_session.py` | untracked at this old HEAD | Exact historical blob is reachable; current RH56/session implementation is newer |
| `tests/test_precision_dual_clutch.py` | untracked at this old HEAD | Exact historical blob is reachable; current tests are newer |
| `tests/test_quest_jaka_se3.py` | untracked at this old HEAD | Exact historical blob is reachable; current tests are newer |
| `tests/test_quest_jaka_smooth.py` | untracked at this old HEAD | Exact historical blob is reachable; current tests are newer |
| `tests/test_quest_rh56_retarget.py` | untracked at this old HEAD | Exact historical blob is reachable; current RH56 tests are newer |
| `tools/analyze_quest_jaka_jitter.py` | untracked; unique blob | Offline-only recording/event analyzer for packet timing, stationary jitter, target steps, actuator error, and discontinuity counts; no hardware or network opening |
| `tools/quest_rh56_skeleton_debug.py` | untracked; unique blob | Offline three-panel Quest landmarks / scaled targets / RH56 MuJoCo FK visualizer; no hardware path |

The bulk of the control implementation is therefore already in main in a
newer form. Copying this worktree wholesale would regress PWL and completed
RH56 behavior.

### Broken dependency after approved reference deletion

`docs/motion_input/QUEST_RH56_ANYDEX_RETARGETING.md` line 11 names the now
deleted exact path:

```text
/home/thor/projects/_reference_AnyDexRetarget_sparse
```

This is a documentation/provenance reference, not a runtime import. The same
document pins the remotely recoverable upstream commit
`77c0a1074ba6eb003159da37b2bd3cec41792523`. No code was changed to repair the
path, as required by this audit.

### Disposition

**Classification: `EXTRACT_SELECTED_FILES`.**

User options:

1. extract the two offline diagnostic tools and selected historical documents,
   updating the stale AnyDex local path and clearly labeling old thresholds and
   results as historical;
2. checkpoint the complete dirty worktree to a remote archive branch, then
   remove it without merging old control files; or
3. explicitly discard all remaining working content and remove the worktree.

## Recommendation summary

| Worktree | Classification | Reason |
| --- | --- | --- |
| Controller transport | `NEEDS_REWRITE_BEFORE_MERGE` | committed transport is already main; unique physical-output prototype is based on an older control stack |
| JAKA arm audit | `KEEP_AS_DOCUMENTATION` | unique 44 KiB evidence ledger and test-gap analysis; no code |
| Dual clutch | `EXTRACT_SELECTED_FILES` | unique simulation-only keyboard provider/tests and useful metrics; old integration files cannot be copied wholesale |
| Offline simulation | `EXTRACT_SELECTED_FILES` | core is already in main; two offline tools and historical documents remain unique |

## User decisions required

1. **Controller transport:** port selected pure policy/tests/documentation and
   rewrite the physical adapter, archive the entire dirty prototype, or
   abandon it.
2. **Arm audit:** retain the 471-line audit in main/research history, archive
   it, or discard it.
3. **Dual clutch:** port the developer-keyboard provider and selected metrics,
   archive the prototype, or discard it.
4. **Offline simulation:** extract the offline diagnostics/documents, archive
   the whole snapshot, or discard it.

No disposition above was executed. The four worktrees remain registered and
unchanged pending user instruction.
