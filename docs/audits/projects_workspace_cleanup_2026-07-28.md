# Projects workspace cleanup — 2026-07-28

## Scope and safety boundary

This report records the local convergence of the `embodied-hand-lab`
development workspace. `${PROJECTS_ROOT}` denotes the audited projects
directory. Host-specific paths are not production configuration.

The following worktrees were treated as read-only active development for the
entire cleanup:

- `${PROJECTS_ROOT}/embodied-hand-lab-quest-rh56-hand-teleop-sim`
- `${PROJECTS_ROOT}/embodied_lab_teleop_rearchitecture`

No hardware connection, robot command, controller-setting write, force push,
history rewrite, hard reset, Git clean, or forced branch deletion was used.

## Main consolidation

| Check | Result |
| --- | --- |
| Local `main` before consolidation | `891ae6f8c36c3c40a22a025cde288be3093fc5dc` |
| `origin/main` before consolidation | `891ae6f8c36c3c40a22a025cde288be3093fc5dc` |
| Source integration branch | `origin/chore/consolidate-completed-worktrees-20260728` |
| Integration HEAD | `54026d3c2bfe1d4007b771eb9e2548766d17112f` |
| Linear relationship | strict descendant, ahead 9 / behind 0 |
| Local integration method | `git merge --ff-only` |
| Merge commit created | no |
| Remote update | fast-forward push of `main` |
| `main` immediately after PWL push | `54026d3c2bfe1d4007b771eb9e2548766d17112f` |
| `main...origin/main` after push | 0 / 0 |

The nine preserved commits are the eight original PWL/root-cause-fix commits
through `0c30c2d2e550f7da7f3648e4d1849a78adaf38e1` plus the workspace
consolidation audit commit `54026d3c2bfe1d4007b771eb9e2548766d17112f`.

## Local-only spreadsheet rule

The user-designated local file is:

```text
关节角与0-1000 对应关系  .xls
```

The exact local `.git/info/exclude` rule is:

```gitignore
/关节角与0-1000 对应关系  .xls
```

Verification:

- `git check-ignore -v` attributes the match to `.git/info/exclude`;
- the file is not tracked;
- repository `.gitignore` and the index were unchanged;
- the file remained 800,768 bytes;
- SHA-256 remained
  `881d9693a01ee51086c928435df5ab66e12c8e886d100ebaf3dba8f950b305cc`;
- mtime remained `2026-07-28 09:59:49.317712857 +0800`.

The local exclude file is outside the commit and push range.

## Validation before pushing main

- `tests/test_quest_jaka_single_arm_final.py`: 1 passed.
- Default offline pytest suite: 576 passed, 1 skipped in 70.78 seconds.
- The skip is the existing headless MuJoCo rendering test when no rendering
  backend is configured.
- `python -m compileall -q src tools tests`: passed.
- `git diff --check origin/main..HEAD`: passed.
- `bash -n` passed for all three changed shell entry points.
- The normal arm-only model test asserted six JAKA actuators.
- `root_cause_fix` asserted a 16,666,667 ns feasibility-acceleration period.
- The independent ServoJ contract asserted 8,000,000 ns.
- The normal arm entry retained a disabled RH56 command path.
- No `configs/robot` or `configs/hand` path changed in the nine-commit range.
- The local spreadsheet was not tracked.
- The range contained no detected private key, common GitHub token, or
  host-specific absolute path.

## Removed temporary PWL branches

Remote `main` was proven to contain both `0c30c2d...` and `54026d3...` before
these refs were deleted.

Deleted locally with `git branch -d`:

- `feature/quest-jaka-pwl-acceleration-recovery`
- `chore/consolidate-completed-worktrees-20260728`

Deleted remotely without force:

- `origin/feature/quest-jaka-pwl-acceleration-recovery`
- `origin/chore/consolidate-completed-worktrees-20260728`

The local recovery ref
`backup/pre-consolidation-2026-07-28-0c30c2d` remains at
`0c30c2d2e550f7da7f3648e4d1849a78adaf38e1`.

## Archived results

The following clean local-only results were pushed without merging them into
main:

| Archive branch | Remote HEAD | Preserved local-only commits |
| --- | --- | ---: |
| `origin/feature/act-thor-data-infra` | `f5e6d538ab7b4c244dc718b3215eb7b2355dc932` | 2 |
| `origin/feature/teledex-bounded-live-teleoperation` | `52b67fab057afd0a63c67fa6b6f9333dc807113a` | 1 |
| `origin/chore/repository-cleanup` | `d797dc5ce27a0d7cb1ca34df7a3ef23093d75e79` | 12 |

Existing remote archives retained:

- `origin/feature/moveit-servo-jaka-evaluation`
- `origin/feature/ruckig-edg-otg-evaluation`
- `origin/feature/quest-jaka-teleop-rearchitecture`
- `private/integration/rh56-visual-coacd-default`

No Pull Request was created for an archive branch.

## Removed worktrees

Each entry was clean, had no stash, had no commit outside fetched remotes, and
was proven recoverable immediately before `git worktree remove`.

| Worktree | Branch | HEAD before removal | Recovery ref | Result |
| --- | --- | --- | --- | --- |
| `${PROJECTS_ROOT}/embodied_lab_act_thor` | `feature/act-thor-data-infra` | `f5e6d538ab7b4c244dc718b3215eb7b2355dc932` | `origin/feature/act-thor-data-infra` | removed |
| `${PROJECTS_ROOT}/embodied_lab_moveit` | `feature/moveit-servo-jaka-evaluation` | `aaebd66accaaf56a496c4eec37afe83c81771331` | `origin/feature/moveit-servo-jaka-evaluation` | removed |
| `${PROJECTS_ROOT}/embodied_lab_quest_input` | `feature/quest-hand-tracking-streamer-integration` | `7f4036eaffffde74c5ccb2698734e7c68094673d` | `origin/main` | removed |
| `${PROJECTS_ROOT}/embodied_lab_ruckig` | `feature/ruckig-edg-otg-evaluation` | `155a5f908ecec21e4dfe9743485e53452e3cc3a5` | `origin/feature/ruckig-edg-otg-evaluation` | removed |
| `${PROJECTS_ROOT}/embodied_lab_teledex_bounded_live` | `feature/teledex-bounded-live-teleoperation` | `52b67fab057afd0a63c67fa6b6f9333dc807113a` | `origin/feature/teledex-bounded-live-teleoperation` | removed |
| `<temporary-worktree>/embodied_lab_repository_cleanup` | `chore/repository-cleanup` | `d797dc5ce27a0d7cb1ca34df7a3ef23093d75e79` | `origin/chore/repository-cleanup` | removed |

## Removed local branches

In addition to the two temporary PWL refs, the following were deleted with
ordinary `git branch -d` after their worktrees were removed or their contents
were otherwise proven recoverable:

- `feature/act-thor-data-infra`
- `feature/moveit-servo-jaka-evaluation`
- `feature/quest-hand-tracking-streamer-integration`
- `feature/ruckig-edg-otg-evaluation`
- `feature/teledex-bounded-live-teleoperation`
- `chore/repository-cleanup`
- `feature/jaka-teledex-control-foundation`
- `feature/quest-motion-input-platform`
- `integration/rh56-visual-coacd-default`

## Reference-clone decisions

The following independent clones passed the pre-deletion checks. Their entries
were written here before deletion.

| Directory | Remote | Branch / local HEAD | Upstream proof | Deletion status |
| --- | --- | --- | --- | --- |
| `${PROJECTS_ROOT}/_reference_AnyDexRetarget` | `https://github.com/qqsq12321/AnyDexRetarget.git` | `master` / `77c0a1074ba6eb003159da37b2bd3cec41792523` | local HEAD is an ancestor of refreshed `origin/master` at `745a8358f0ff86e90991fc5eb7e85073e56a8818` | removed with controlled exact-path recursion |
| `${PROJECTS_ROOT}/_reference_FFTAI_teleoperation` | `https://github.com/FFTAI/teleoperation.git` | `devel` / `f90a64a5e18ce6c516cab6d81d9f212453522419` | equals refreshed `origin/devel` | removed with controlled exact-path recursion |
| `${PROJECTS_ROOT}/_reference_XRoboToolkit` | `https://github.com/XR-Robotics/XRoboToolkit-Teleop-Sample-Python.git` | `main` / `79e5cb8a56e3455515ce1b476e993c764ec58739` | equals refreshed `origin/main` | removed with controlled exact-path recursion |
| `${PROJECTS_ROOT}/_reference_xr_teleoperate` | `https://github.com/unitreerobotics/xr_teleoperate.git` | `main` / `7dc9aa1a6edbf4a9f4f887d8ab6fc449ea5135f6` | equals refreshed `origin/main` | removed with controlled exact-path recursion |

Exact-snapshot recovery commands:

```bash
git clone --filter=blob:none https://github.com/qqsq12321/AnyDexRetarget.git \
  "${PROJECTS_ROOT}/_reference_AnyDexRetarget"
git -C "${PROJECTS_ROOT}/_reference_AnyDexRetarget" switch --detach \
  77c0a1074ba6eb003159da37b2bd3cec41792523

git clone --filter=blob:none --branch devel \
  https://github.com/FFTAI/teleoperation.git \
  "${PROJECTS_ROOT}/_reference_FFTAI_teleoperation"

git clone --filter=blob:none \
  https://github.com/XR-Robotics/XRoboToolkit-Teleop-Sample-Python.git \
  "${PROJECTS_ROOT}/_reference_XRoboToolkit"

git clone --filter=blob:none \
  https://github.com/unitreerobotics/xr_teleoperate.git \
  "${PROJECTS_ROOT}/_reference_xr_teleoperate"
```

### References retained

- `_reference_AnyDexRetarget_sparse`: a retained dirty legacy worktree
  document currently names this exact local path. Removing it would break the
  documented development dependency.
- `_reference_Open-Teach`: does not meet the deletion gate; see the
  investigation below.
- `_reference_hand-tracking-streamer`: contains one modified Unity scene and
  two untracked probe files. It is active and not remotely recoverable.
- `openpi`: remains a fixed local dependency of the OpenPI shadow client and
  its documented execution environment.

## Open-Teach staged-deletion investigation

Observed state:

- branch `main`, HEAD
  `32a7d44b33953066ff27312a7b2b4c294f4f52c5`;
- remote `https://github.com/aadhithya14/Open-Teach.git`;
- 12,042 staged deletions;
- HEAD tree paths: 12,042;
- index paths: 0;
- non-Git worktree files: 0;
- `.git/index` is absent;
- sparse checkout is not enabled;
- the clone is partial (`blob:none`), which does not by itself explain an
  absent index and empty worktree;
- the directory is on the normal writable ext4 root filesystem;
- `git diff --cached --stat` did not finish within a bounded 20-second audit,
  while summary/name-status samples consistently showed deletion.

The evidence cannot distinguish a deliberate bulk index removal from index or
clone damage. No restore, reset, checkout, clean, commit, or delete was
performed. The directory is retained because the staged state cannot be
proven valueless.

## Protected and blocked work

Dirty counts are `staged/unstaged/untracked`.

| Directory | Branch / HEAD | Dirty | Remote-unreachable commits | Reason retained |
| --- | --- | ---: | ---: | --- |
| `${PROJECTS_ROOT}/embodied-hand-lab-quest-rh56-hand-teleop-sim` | `feature/quest-rh56-hand-teleop-sim` / `0c30c2d2e550f7da7f3648e4d1849a78adaf38e1` | 0/15/9 | 0 | active RH56 development; concurrent working changes are protected |
| `${PROJECTS_ROOT}/embodied_lab_teleop_rearchitecture` | `feature/quest-jaka-teleop-rearchitecture` / `4e6cca5f8439e60d2137e7fcb1837d947508b6fb` | 0/11/8 | 6 | active research; local commits and working files are not fully remote |
| `${PROJECTS_ROOT}/embodied_lab_quest_controller_transport` | `feature/quest-controller-transport-host` / `530d3a0aa75306248998f563fb981cb4a2c31790` | 0/8/6 | 0 | uncommitted physical-command-path work |
| `${PROJECTS_ROOT}/embodied_lab_quest_jaka_arm_audit` | `feature/quest-jaka-arm-teleop-audit` / `4a0b5e4465b47e753f1113c426b7ee3988e8ba0c` | 0/0/1 | 0 | untracked audit evidence |
| `${PROJECTS_ROOT}/embodied_lab_quest_jaka_dual_clutch` | `feature/quest-jaka-dual-clutch-checkpoint` / `4a0b5e4465b47e753f1113c426b7ee3988e8ba0c` | 0/5/2 | 0 | uncommitted clutch work |
| `${PROJECTS_ROOT}/embodied_lab_quest_jaka_sim` | `feature/quest-jaka-offline-simulation` / `8f72e7b40c6ea31674c81aa9c82eabfe134d1095` | 0/7/16 | 0 | uncommitted simulation/retargeting work |
| `${PROJECTS_ROOT}/_reference_AnyDexRetarget_sparse` | `master` / `77c0a1074ba6eb003159da37b2bd3cec41792523` | 0/0/0 | 0 | exact path is named by retained uncommitted AnyDex research documentation |
| `${PROJECTS_ROOT}/_reference_Open-Teach` | `main` / `32a7d44b33953066ff27312a7b2b4c294f4f52c5` | 12042/0/0 | 0 | absent index/empty worktree cause is not proven safe |
| `${PROJECTS_ROOT}/_reference_hand-tracking-streamer` | `feature/quest-mixed-input-probe` / `5ff7c1cfea0ead1bb8a0e233bc7770d94d31feb5` | 0/1/2 | 0 commits, 3 working paths | active Unity probe |
| `${PROJECTS_ROOT}/openpi` | `main` / `15a9616a00943ada6c20a0f158e3adb39df2ccac` | 0/0/0 | 0 | fixed path/commit dependency of the shadow client |

All stash counts above are zero. No protected worktree was switched, staged,
committed, reset, cleaned, merged, pushed, moved, or deleted by this cleanup.

## Final workspace and recovery

Final first-level directories:

```text
_reference_AnyDexRetarget_sparse
_reference_Open-Teach
_reference_hand-tracking-streamer
embodied-hand-lab-quest-rh56-hand-teleop-sim
embodied_lab
embodied_lab_quest_controller_transport
embodied_lab_quest_jaka_arm_audit
embodied_lab_quest_jaka_dual_clutch
embodied_lab_quest_jaka_sim
embodied_lab_teleop_rearchitecture
openpi
```

Final linked worktrees:

```text
embodied_lab                                      main
embodied-hand-lab-quest-rh56-hand-teleop-sim     feature/quest-rh56-hand-teleop-sim
embodied_lab_quest_controller_transport           feature/quest-controller-transport-host
embodied_lab_quest_jaka_arm_audit                 feature/quest-jaka-arm-teleop-audit
embodied_lab_quest_jaka_dual_clutch               feature/quest-jaka-dual-clutch-checkpoint
embodied_lab_quest_jaka_sim                       feature/quest-jaka-offline-simulation
embodied_lab_teleop_rearchitecture                feature/quest-jaka-teleop-rearchitecture
```

Final local branches before committing this report:

```text
backup/pre-consolidation-2026-07-28-0c30c2d
feature/quest-controller-transport-host
feature/quest-jaka-arm-teleop-audit
feature/quest-jaka-dual-clutch-checkpoint
feature/quest-jaka-offline-simulation
feature/quest-jaka-teleop-rearchitecture
feature/quest-rh56-hand-teleop-sim
main
```

Immediately before this report commit:

- local `main` and `origin/main` both equal
  `54026d3c2bfe1d4007b771eb9e2548766d17112f`;
- ahead/behind is 0/0;
- the only visible main-worktree change is this new report;
- `git worktree prune --dry-run --verbose` reports no stale metadata.

Committing this report advances `main` by one documentation-only commit. The
exact resulting commit is recorded in the final command-backed handoff (a Git
commit cannot embed its own final object ID without changing that ID).

Recovery principles:

- removed worktrees can be recreated from the recorded remote refs with
  `git worktree add`;
- removed reference clones can be recreated with the commands above;
- the local spreadsheet remains outside Git and is selected by its exact local
  exclude rule;
- the pre-consolidation backup branch remains until a later explicit cleanup;
- active/dirty worktrees were preserved rather than normalized.
