# Projects workspace consolidation audit — 2026-07-28

> **Status: historical snapshot dated 2026-07-28.** This report records the
> pre-integration workspace. PWL/root-cause-fix and RH56 simulation hand work
> have since entered `main`; several listed worktrees and reference clones have
> been removed. See
> [`docs/status/current_status.md`](../status/current_status.md) for current
> behavior.

## Scope and evidence boundary

This audit covers the 20 first-level Git directories under
`${PROJECTS_ROOT}` and the additional linked worktree registered at
`<temporary-worktree>/embodied_lab_repository_cleanup`. It is a repository/workspace
convergence audit, not a blanket merge plan.

The primary repository was refreshed with:

```bash
git fetch --all --prune
```

The independent reference clones were not fetched. Their ahead/behind values
below therefore compare against their existing local remote-tracking refs.
All repositories and worktrees reported an empty `git stash list`.

The user explicitly designated the untracked spreadsheet in `embodied_lab` as
a local-only file that must not be committed or pushed. It is treated as a
documented exception, not as an integration input.

## Executive summary

- Relevant directories: 21 (20 under `projects`, plus one linked temporary
  worktree).
- Primary development repository: 1 (`embodied_lab`).
- Active directories with uncommitted development or probe content: 7.
- Completed or archival experiment/result directories: 5.
- Explicitly rejected or superseded implementation lines: 3.
- Worktrees that satisfy the proposed local-removal safety conditions: 3.
- Work/result groups requiring manual review before archival or cleanup: 4.
- Invalid/prunable worktree metadata: 0. `git worktree prune --dry-run
  --verbose` produced no candidate.
- Commits reachable from local branches but from no fetched remote: 21.
- Recommended integration: preserve the eight exact commits from
  `origin/main..0c30c2d2e550f7da7f3648e4d1849a78adaf38e1`, plus this audit
  document. Do not merge the teleoperation rearchitecture prototype, ACT,
  repository-cleanup, or old TeleDex branch wholesale.

The selected eventual target is `origin/main` at
`891ae6f8c36c3c40a22a025cde288be3093fc5dc`. The completed PWL branch is a
strict linear descendant of that commit (ahead 8, behind 0), so the
consolidation branch is created at the protected PWL HEAD. This preserves the
original commit provenance without a merge commit, squash, rebase, or repeated
cherry-pick.

## Git topology

The following directories share one object database:

```text
${PROJECTS_ROOT}/embodied_lab/.git
├── ${PROJECTS_ROOT}/embodied_lab
├── ${PROJECTS_ROOT}/embodied-hand-lab-quest-rh56-hand-teleop-sim
├── ${PROJECTS_ROOT}/embodied_lab_act_thor
├── ${PROJECTS_ROOT}/embodied_lab_moveit
├── ${PROJECTS_ROOT}/embodied_lab_quest_controller_transport
├── ${PROJECTS_ROOT}/embodied_lab_quest_input
├── ${PROJECTS_ROOT}/embodied_lab_quest_jaka_arm_audit
├── ${PROJECTS_ROOT}/embodied_lab_quest_jaka_dual_clutch
├── ${PROJECTS_ROOT}/embodied_lab_quest_jaka_sim
├── ${PROJECTS_ROOT}/embodied_lab_ruckig
├── ${PROJECTS_ROOT}/embodied_lab_teledex_bounded_live
├── ${PROJECTS_ROOT}/embodied_lab_teleop_rearchitecture
└── <temporary-worktree>/embodied_lab_repository_cleanup
```

All `_reference_*` directories and `openpi` are independent clones with their
own object databases.

## Directory inventory

Dirty counts are `staged/unstaged/untracked`. `remote-only unique` means the
number of commits reachable from the local HEAD but from no fetched remote.

| Path | Type | Branch / HEAD | Remote status | Dirty | Remote-only unique | Classification | Recommended action and evidence |
| --- | --- | --- | --- | ---: | ---: | --- | --- |
| `${PROJECTS_ROOT}/_reference_AnyDexRetarget` | independent reference | `master` / `77c0a10` | `origin/master` 0/0 | 0/0/0 | 0 | read-only reference | Keep. |
| `${PROJECTS_ROOT}/_reference_AnyDexRetarget_sparse` | independent sparse reference | `master` / `77c0a10` | `origin/master` 0/0 | 0/0/0 | 0 | duplicate reference clone | Keep unless the user later approves reference de-duplication. |
| `${PROJECTS_ROOT}/_reference_FFTAI_teleoperation` | independent reference | `devel` / `f90a64a` | `origin/devel` 0/0 | 0/0/0 | 0 | read-only reference | Keep. |
| `${PROJECTS_ROOT}/_reference_Open-Teach` | independent reference | `main` / `32a7d44` | `origin/main` 0/0 | 12042/0/0 | 0 | dirty reference, cause unknown | Do not restore, commit, or delete. The index contains 12,042 staged deletions. |
| `${PROJECTS_ROOT}/_reference_XRoboToolkit` | independent reference | `main` / `79e5cb8` | `origin/main` 0/0 | 0/0/0 | 0 | read-only reference | Keep. |
| `${PROJECTS_ROOT}/_reference_hand-tracking-streamer` | independent modified reference/probe | `feature/quest-mixed-input-probe` / `5ff7c1c` | no upstream; HEAD contained by `origin/main` | 0/1/2 | 0 commits | active third-party probe | Keep. The Unity scene and `MixedInputCapabilityProbe.cs(.meta)` contain uncommitted work. |
| `${PROJECTS_ROOT}/_reference_xr_teleoperate` | independent reference | `main` / `7dc9aa1` | `origin/main` 0/0 | 0/0/0 | 0 | read-only reference | Keep. |
| `${PROJECTS_ROOT}/embodied-hand-lab-quest-rh56-hand-teleop-sim` | linked worktree | `feature/quest-rh56-hand-teleop-sim` / `0c30c2d` | no same-name remote; base recoverable from PWL remote | 0/13/7 | 0 commits, 20 unique working paths | `KEEP_ACTIVE`, protected RH56 work | Never reset, merge, or delete. The tracked diff is 984 insertions/184 deletions and all 20 working blobs differ from PWL/main. |
| `${PROJECTS_ROOT}/embodied_lab` | primary repository | completed PWL branch / `0c30c2d` at audit start | same-name origin 0/0 | 0/0/1 | 0 | main development directory / completed arm result | Keep. The one spreadsheet is an explicit local-only exception. |
| `${PROJECTS_ROOT}/embodied_lab_act_thor` | linked worktree | `feature/act-thor-data-infra` / `f5e6d53` | no upstream or same-name remote | 0/0/0 | 2 | clean but unpushed result | Manual review. Preserve and push/archive before any cleanup. Repository policy excludes `learned_policy/` from this consolidation. |
| `${PROJECTS_ROOT}/embodied_lab_moveit` | linked worktree | `feature/moveit-servo-jaka-evaluation` / `aaebd66` | same-name origin 0/0 | 0/0/0 | 0 | completed negative experiment | Keep the remote branch as archive. The local worktree is a removal candidate. |
| `${PROJECTS_ROOT}/embodied_lab_quest_controller_transport` | linked worktree | `feature/quest-controller-transport-host` / `530d3a0` | branch name absent remotely; HEAD in `origin/main` | 0/8/6 | 0 commits, 14 unique working paths | `KEEP_ACTIVE` | Keep. It contains uncommitted physical Quest/JAKA/RH56 command-path work. |
| `${PROJECTS_ROOT}/embodied_lab_quest_input` | linked worktree | `feature/quest-hand-tracking-streamer-integration` / `7f4036e` | branch name absent remotely; HEAD in `origin/main` | 0/0/0 | 0 | completed and fully incorporated | Safe local worktree/branch removal candidate. |
| `${PROJECTS_ROOT}/embodied_lab_quest_jaka_arm_audit` | linked worktree | `feature/quest-jaka-arm-teleop-audit` / `4a0b5e4` | HEAD in `origin/main` | 0/0/1 | 0 commits, one unique 44 KiB document | `KEEP_ACTIVE` / manual evidence | Keep. The untracked audit cannot be recovered remotely. |
| `${PROJECTS_ROOT}/embodied_lab_quest_jaka_dual_clutch` | linked worktree | `feature/quest-jaka-dual-clutch-checkpoint` / `4a0b5e4` | HEAD in `origin/main` | 0/5/2 | 0 commits, 7 unique working paths | `KEEP_ACTIVE` | Keep. Keyboard-clutch work is uncommitted. |
| `${PROJECTS_ROOT}/embodied_lab_quest_jaka_sim` | linked worktree | `feature/quest-jaka-offline-simulation` / `8f72e7b` | HEAD in `origin/main` | 0/7/16 | 0 commits, 23 working paths | `KEEP_ACTIVE` | Keep. Only 3 working blobs exactly match PWL; 20 remain different. |
| `${PROJECTS_ROOT}/embodied_lab_ruckig` | linked worktree | `feature/ruckig-edg-otg-evaluation` / `155a5f9` | same-name origin 0/0 | 0/0/0 | 0 | completed negative experiment | Keep the remote branch as archive. The local worktree is a removal candidate. |
| `${PROJECTS_ROOT}/embodied_lab_teledex_bounded_live` | linked worktree | `feature/teledex-bounded-live-teleoperation` / `52b67fa` | no same-name remote | 0/0/0 | 1 | superseded implementation with unpushed checkpoint | Archive the commit remotely before cleanup. `git cherry` proves its patch is not equivalent to a main commit. |
| `${PROJECTS_ROOT}/embodied_lab_teleop_rearchitecture` | linked worktree | `feature/quest-jaka-teleop-rearchitecture` / `4e6cca5` | same-name origin ahead 6, behind 0 | 0/11/8 | 6 commits plus 19 working paths | `KEEP_ACTIVE` candidate/research | Do not merge or clean. Every working blob differs from PWL/main. |
| `${PROJECTS_ROOT}/openpi` | independent external clone | `main` / `15a9616` | `origin/main` 0/0 | 0/0/0 | 0 | external dependency/reference | Keep. The primary repository pins this commit and documents this local path. |
| `<temporary-worktree>/embodied_lab_repository_cleanup` | linked worktree | `chore/repository-cleanup` / `d797dc5` | no upstream or same-name remote | 0/0/0 | 12 | clean but unpushed documentation series | Manual review. It diverges 38/12 from current main and must be archived before cleanup. |

## Local-only commit inventory

These 21 commits are not reachable from any fetched remote:

### Teleoperation rearchitecture (6)

- `6f08b09b3a3d7584c517f8aec1ee9306cbb5003b`
- `d7661cc4d8d6e5c835d801a62114d8220bee5364`
- `afb54b3326cca482b508607f78dd3ab0bf5bd786`
- `a53ece339b945a79accaa70687f22c8f853c9344`
- `4f60842f1ca1ba1abfd68887a5ec8d65067e2d50`
- `4e6cca5f8439e60d2137e7fcb1837d947508b6fb`

### ACT/Thor (2)

- `a8d6edea7719cf72b8d04073ca4b0fd1d975ef45`
- `f5e6d538ab7b4c244dc718b3215eb7b2355dc932`

### TeleDex (1)

- `52b67fab057afd0a63c67fa6b6f9333dc807113a`

### Repository cleanup (12)

- `e7b6686a23260503f602b7abc8a315a4d39f541a`
- `dd23d244030c9c540c2ff1297126fb5186414532`
- `8df15148fe2cffe80c3efce4dcafc51db0c0f407`
- `be4fcf349920e913e845ec6d9a0d21e582850944`
- `fbf520f5811189ce7b3c7b301b9c1b9649b1370a`
- `8f34898ac942bb0599f2e20ee940739d9d04e073`
- `b865059648822e208aa5652dddeec127b699083d`
- `6adec3bca9dcb0daafd43cd0122c4773fa9f452a`
- `b9bac0b1cf550088db7978340fc2397205a3c0c1`
- `cb75b5b3fd88449a5d6660c9010cf1688b3d1818`
- `d925f87d2af94eee2a9935d7f09f18f1413a648f`
- `d797dc5ce27a0d7cb1ca34df7a3ef23093d75e79`

## Branch disposition

### `KEEP_ACTIVE`

- `feature/quest-rh56-hand-teleop-sim`
- `feature/quest-controller-transport-host`
- `feature/quest-jaka-arm-teleop-audit`
- `feature/quest-jaka-dual-clutch-checkpoint`
- `feature/quest-jaka-offline-simulation`
- `feature/quest-jaka-teleop-rearchitecture`
- `_reference_hand-tracking-streamer:feature/quest-mixed-input-probe`
- `feature/act-thor-data-infra` until its two commits are archived

### `KEEP_REMOTE_ARCHIVE`

- `feature/moveit-servo-jaka-evaluation` at
  `aaebd66accaaf56a496c4eec37afe83c81771331`
- `feature/ruckig-edg-otg-evaluation` at
  `155a5f908ecec21e4dfe9743485e53452e3cc3a5`
- `integration/rh56-visual-coacd-default` at
  `6faa64b3776aa536ba699fe4967956f34e0865b5`
- remote teleoperation rearchitecture audit at
  `3e911f80ba8b02260fd68c1e7c8a9641521b3622`

### `MERGE_OR_CHERRY_PICK_TO_MAIN_DEV`

The exact protected range:

1. `8ee9729e3db4a2d620f14e4792d752790ce82977`
2. `879848ba38821875ae24addefdc1fe4c67de7b2d`
3. `4a1c94747f45c64bf7ec84746334527d7959f2c3`
4. `8e5ec55e396af1c00b0eb56fab6d2de10e85002f`
5. `5401f3c3b5a185e698b4416273ff4111b90b2b51`
6. `e619aeda62aae0bc6983bd277fdf814de6280bde`
7. `7a02aa6fbdb01efddcc09605bc8e1e22eaf3bc8b`
8. `0c30c2d2e550f7da7f3648e4d1849a78adaf38e1`

Because this is a strict linear range above `origin/main`, creating the
consolidation branch at the final commit preserves all eight originals and is
safer than reconstructing the same history with cherry-picks.

### `SUPERSEDED`

- `feature/jaka-teledex-control-foundation` at `f270cf0` is an ancestor of
  `origin/main`; its upstream is gone.
- `feature/quest-hand-tracking-streamer-integration` at `7f4036e` is an
  ancestor of `origin/main`.
- `feature/quest-motion-input-platform` at `8ecd70b` is an ancestor of
  `origin/main`.
- The committed tips `530d3a0`, `4a0b5e4`, and `8f72e7b` are ancestors of
  main, but this does not supersede the uncommitted files in their worktrees.
- The old TeleDex runtime entered main at `ac7399b` and was explicitly pruned
  at `35ff114`. The unpushed `52b67fa` must nevertheless be archived before
  deletion because its patch is not equivalent to a fetched remote commit.

### `SAFE_TO_DELETE_LOCAL`

Subject to one final status/stash/unpushed check immediately before execution:

- local branch `feature/jaka-teledex-control-foundation`
- local branch `feature/quest-motion-input-platform`
- worktree `embodied_lab_quest_input`, then its local branch
- worktree `embodied_lab_moveit` only; retain the remote branch
- worktree `embodied_lab_ruckig` only; retain the remote branch

### `NEEDS_MANUAL_REVIEW`

- ACT/Thor two-commit series
- repository-cleanup twelve-commit series
- TeleDex `52b67fa`
- Open-Teach staged deletion state
- teleoperation rearchitecture's six local commits and uncommitted work

## Protected arm result verification

`git cat-file -t 0c30c2d2e550f7da7f3648e4d1849a78adaf38e1`
returned `commit`. The exact commit is contained by:

- local `feature/quest-jaka-pwl-acceleration-recovery`
- local `feature/quest-rh56-hand-teleop-sim`
- `origin/feature/quest-jaka-pwl-acceleration-recovery`

The protected branch and its origin tracking branch are identical. All eight
commits are marked `+` by `git cherry -v origin/main`, so they have not been
patch-equivalently incorporated into main.

Static invariant evidence:

- `tests/test_quest_jaka_single_arm_final.py` selects `root_cause_fix`.
- It requires `feasibility_acceleration_period_ns == 16_666_667`, matching the
  60 Hz accepted-target replacement period.
- It independently requires `servo_period_ns == 8_000_000`, retaining the
  ServoJ contract.
- It builds the normal arm-only MuJoCo model and requires exactly six
  actuators, all prefixed `jaka_joint_`.
- `src/quest_jaka_sim/simulation.py` removes all `rh56_` actuators when
  producing the arm-only model.
- `tools/quest_jaka_mujoco_sim.py` reports that the RH56 command path is
  disabled in the final arm-only entry.
- `docs/operation/simulation_demo.md` states that the normal entry is six-axis,
  RH56 is visual-only, feasibility is evaluated at 60 Hz, and physical ServoJ
  remains 8 ms.
- The task did not perform physical speed calibration or write controller
  speed settings.

The source asset `data/sim_assets/jaka_rh56.xml` contains six arm and six hand
actuators. The six-actuator contract applies to the generated normal arm-only
runtime model, not to that source XML.

## MoveIt and Ruckig outcomes

The MoveIt evaluation is a completed negative experiment. Its document says:

> Keep the current production Quest mapping, full-pose continuation IK,
> AcceptedArmTarget, PWL/EDG adapter, health/liveness logic, and physical gates.
> Do not migrate to MoveIt or ros2_control yet.

The Ruckig evaluation is also a completed negative experiment. It reports
144–200 ms translational latency for the selected Ruckig point versus 8 ms for
the corresponding PWL result, and explicitly states that Ruckig must remain
experimental and must not replace PWL on this evidence.

Both branches are clean and exactly pushed. Their negative results and
reproducible experimental implementations belong in remote archive branches,
not in the production consolidation.

## Teleoperation rearchitecture disposition

The branch diverges from the protected PWL line at
`7a02aa6fbdb01efddcc09605bc8e1e22eaf3bc8b`.

- PWL side: final accepted simulation policy commit `0c30c2d`.
- Remote rearchitecture side: `893abf6` and research-audit commit `3e911f8`.
- Local committed rearchitecture side: six unpushed commits through
  `4e6cca5`.
- Working tree: 11 modified and 8 untracked paths.

Classification:

1. **Directly mainline-ready generic code:** none yet.
2. **Research record:** the pushed research document and result files through
   `3e911f8`; they are explicitly offline and made no JAKA, Quest, or RH56
   connection.
3. **Rewrite/extract before mainline:** the versioned command ABI, isolated
   shaping interface, health/telemetry ownership, and thin adapter contract.
4. **Superseded for the current acceleration incident:** using a full
   rearchitecture as the remedy. `root_cause_fix` already preserves the
   accepted 60 Hz feasibility and 8 ms ServoJ contract.
5. **Candidate only:** the B/C shaper selection, research thin worker,
   engagement shaper, and current production adapter wiring.

The remote prototype adds 1,128 lines. The six local commits add approximately
55,932 lines relative to the remote, much of it generated result JSON. The
working tree adds another 548 tracked lines plus eight untracked paths. A
whole-branch merge is not justified.

## Reference and attribution audit

The protected PWL HEAD has no top-level `LICENSE`, `LICENSE.md`, or
`THIRD_PARTY_NOTICES.md`.

- The Open-Teach intersection with the protected tree is two empty/placeholder
  blobs, not copied implementation code.
- The AnyDexRetarget intersections are zero blobs. The
  `hand_retarget.py` header says the design was AnyDexRetarget-informed and
  that no upstream source was copied.
- The protected tree shares six RH56 base-mesh blobs with `xr_teleoperate` and
  two with XRoboToolkit. The repository keeps these assets in
  `data/sim_assets/correll_rh56dfx`, attributes them to Correll Robotics Lab,
  and includes the Correll MIT license. The shared blobs indicate a common
  upstream mesh, not proof that code or assets were copied from Unitree or
  XRoboToolkit.
- Open-Teach declares MIT; `xr_teleoperate` declares Apache-2.0; XRoboToolkit
  declares MIT.

The rearchitecture branch introduces a top-level third-party notice, but that
notice belongs to an evolving research branch. A later dedicated attribution
change should reconcile the complete current tree and then manually transplant
the notice without importing prototype code.

## Proposed integration plan

| Source | Target | Method | Risk and behavior | Validation | Rollback |
| --- | --- | --- | --- | --- | --- |
| Eight commits in `origin/main..0c30c2d` | `chore/consolidate-completed-worktrees-20260728`, targeting `origin/main` | Create the branch at the exact protected HEAD; preserve linear history | No RH56 normal actuator path, no physical speed-setting write, 60 Hz feasibility and 8 ms ServoJ remain distinct | Targeted pytest, six-actuator MuJoCo test, fake native worker, shell syntax, diff and sensitive-data scan | Delete the unmerged integration branch; `origin/main` remains unchanged |
| This report | Same integration branch | Separate `docs:` commit | Documentation only | `git diff --check`, path and commit checks | Revert the documentation commit |
| Rear Architecture research/notice | Deferred | Later manual extraction from a clean pushed checkpoint | Current branch is ahead, dirty, and still changing | License/link/source audit | Keep remote archive |
| ACT local commits | Deferred | Push/archive first, then separate review | `learned_policy/` is outside this consolidation's allowed scope | ACT-specific tests and sensitive scan | Keep worktree |
| Repository cleanup commits | Deferred | Rewrite/select files against current main | Old base is 38 commits behind main and current documentation may conflict | Current-status/link review | Keep `/tmp` worktree |

## Proposed cleanup plan

No directory or branch is deleted by this audit.

The candidates meeting the current clean/stash/recoverability conditions are:

```text
${PROJECTS_ROOT}/embodied_lab_quest_input
${PROJECTS_ROOT}/embodied_lab_moveit
${PROJECTS_ROOT}/embodied_lab_ruckig
```

MoveIt and Ruckig must retain their remote archive branches. All reference
directories, the primary repository, the RH56 worktree, dirty worktrees,
worktrees with local-only commits, and unknown non-Git content are excluded
from automatic deletion.

Recommended commands are provided in the final audit handoff, but should be
run only after repeating:

```bash
git status --porcelain=v1
git stash list
git log --branches --not --remotes --oneline
```

for the exact target and shared repository.
