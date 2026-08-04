# Integration Validation Log

## Scope and safety boundary

- Validation started: `2026-07-31T09:28:33+08:00`
- Repository maintenance and offline validation only.
- No JAKA, RH56DFX, Quest, RealSense, serial, CAN, EtherCAT, or other hardware interface was opened.
- No physical-motion command was executed.
- Physical validation remains a separately authorized activity.

## Asset identification

| Item | Result |
|---|---|
| Workspace | `/home/thor/projects` |
| Primary repository | `/home/thor/projects/embodied_lab` |
| Why this repository | Package name `embodied-lab`, JAKA/RH56/Quest/MuJoCo layout, and `origin` URL `git@github.com:brVrzl/embodied-hand-lab.git` match the candidate archive. |
| Other repository | `/home/thor/projects/openpi`; independent upstream OpenPI checkout, not the integration target. |
| Candidate archive | `/home/thor/projects/embodied-hand-lab-main.zip` |
| Archive SHA-256 | `b0931dc7ce3ec20c5cba9e61fe1f1bdcedfc99902c09afa1dc07dafa694e20a2` |
| Archive size / mtime | `40,229,848` bytes; `2026-07-31 09:13:24.974451179 +0800` |
| Archive integrity | `unzip -t`: no errors |
| Archive structure | One project root, `embodied-hand-lab-main/`, plus macOS metadata root `__MACOSX/`; no `.git` entry. |
| Archive safety scan | 1,646 entries; 137,181,248 uncompressed bytes; no absolute path, `..`, backslash-path, NUL-path, or symbolic-link entry. |
| Candidate extraction | `/tmp/embodied-lab-overhaul-review-20260731-092833/embodied-hand-lab-main` (made read-only after extraction) |
| Candidate inventory | 705 files; 244 Python files; 56,973 Python lines; 99 Markdown files; 31 YAML files; 9 shell files. |

## Git baseline

| Item | Result |
|---|---|
| Default remote | `origin` |
| Main branch | `main` |
| Initial local `main` | `04ba907df002730b80f15df1649e34565e205f78` |
| Initial `origin/main` after `git fetch --all --prune` | `04ba907df002730b80f15df1649e34565e205f78` |
| Ahead / behind | `0 / 0`; no left/right commits and no diff |
| Initial worktree | Clean (`git status --short` empty) |
| Baseline status | **BASELINE MAIN VERIFIED AND PUSHED** |
| Backup branch | `backup/pre-overhaul-integration-20260731-092833` at the baseline hash; pushed and remote hash verified |
| Local backup tag | `pre-overhaul-integration-20260731-092833` at the baseline hash |
| Integration branch | `integration/overhaul-validation-20260731-092833` |

## Non-Git asset backup

`git status --ignored` revealed local ignored logs, digital-twin outputs,
episodes, exports, vendor material, caches, and environments. Before candidate
application, the repository content excluding `.git`, `.venv`, `build`, caches,
bytecode, egg-info, and `MUJOCO_LOG.TXT` was archived to:

`/tmp/embodied-lab-integration-backup-20260731-092833/worktree-protected-snapshot.tar`

- Size: `6,971,893,760` bytes
- Members: `3,435`
- SHA-256: `64909a0ca34b807cca7c659cc8b28afc538b9e41d3e0758f42f6bc9aff639d92`
- Verification: complete `tar -tf` traversal succeeded.

## Protected-asset baseline digests

| Asset | Baseline digest or inventory |
|---|---|
| `tools/teleop_mujoco_jaka_rh56.py` | `04d4c5028efcc050ff2a8e8b6d8dcd6078df4588ed4d6b2d82dc9daa9a94950c` |
| `learned_policy/` | 20 files, 1,354,070 bytes; aggregate digest `dd83f7b5bb889085d98d6f2f800ee8e778dc9bcd3be398ee40a3ad2cd697deb0` |
| `models/` | 4 files, 284,584 bytes; aggregate digest `3e28df0d70a26cfa92db0d65ff426afc012a69a0d2490dcae5ff8397fe84d6df` |
| `data/sim_assets/` | 215 files, 20,753,993 bytes; aggregate digest `26d2bba0efd76a7f29e1bcf0284c41810d4600d99ba1e80a091e21fae8f969c6` |
| `data/vendor/`, `third_party/`, `digital_twin/calibration/` combined | Aggregate digest `8f686a82d1b0c82ed037630ed4237be8617757be0899366f8d167399f2bbb128` |

The aggregate digests include `sha256sum` output paths as well as content and
are intended for exact post-integration comparison in the same worktree.

## Validation chronology

Further candidate-diff review, repairs, validation commands, Git integration,
and final remote verification are appended below as they are completed.

## Independent review and repairs

- Candidate application used copy-only synchronization with protected-path
  exclusions; the two missing tracked model XML files were restored from the
  baseline snapshot. No `.git`, ignored user assets, policy files, models,
  calibration, vendor SDK, recordings, or experiment outputs were removed.
- Reviewed deletions: HEBI, obsolete Python JAKA SDK/ServoJog, old RH56
  JAKA-tool/ROS routes, unmaintained bridge/RViz groups, ungated one-off
  physical tools, duplicate viewers, and stubs. Dynamic import inspection
  retained `tools/debug_mujoco_jaka_rh56.py`.
- Concurrent RH56 task files and physical artifacts were excluded by design;
  they belong to a separate commit/push.

Confirmed second-round findings and fixes:

1. P1 archive omission of two tracked model XML files: restored and verified.
2. P2 incomplete LeRobot optional dependency: changed to
   `lerobot[dataset]>=0.6,<0.7` and tested the official SDK.
3. P2 official SVT-AV1 non-termination on sub-32-pixel RGB widths: exporter
   now rejects the input before encoding; 640x480 two-stream export and reload
   passed.
4. P2 native fake timing wall-clock flake: added a fake-only deterministic
   clock; vendor timing path is unchanged.
5. P2 native motion `--help` backend construction: help exits before backend
   construction and has a regression test.
6. P2 clean checkout teleop fixture and CLI parser/default-output issues:
   fixed and covered by tests.

## Final offline validation evidence

| Check | Result |
|---|---|
| Fresh `.[dev]` install / pip check | PASS; clean venv reports no broken requirements |
| compileall | PASS |
| import smoke | PASS; 13 public packages, no socket/path writes; one read-only GLFW capability subprocess |
| CLI help | PASS; root, doctor, sim, dataset, distributed-smoke, benchmark, dataset subcommands |
| pytest | PASS; 681 collected, 680 passed, 1 headless-rendering skip, 0 failed/error |
| native JAKA worker | PASS build; CTest has no registered tests |
| native teleop shaping | PASS; 3/3 CTest |
| minimal JAKA probe | PASS build/help; 21 tests |
| MuJoCo smoke | PASS; nq=18/nv=18/nu=12, contacts 0→0, drift 0 |
| benchmark | PASS; arm max 0.0004646174241849099 rad, hand max 0.001060247811619841 rad |
| Gloo | PASS single and `torchrun` two-process; disjoint shards and clean parent reap |
| NCCL | PASS one-process on one visible NVIDIA Thor GPU; multi-GPU unavailable |
| YAML | PASS; 31 maintained files plus 6 preserved artifact snapshots, no duplicate keys/errors |
| shell/Slurm | PASS syntax 9 shell + 2 Slurm; shellcheck/Slurm runtime unavailable |
| Markdown | PASS; 114 repository Markdown files, 195 relative links, 0 broken |
| protected assets | PASS; 2,210 selected files, 2 expected sim XML/manifest changes, 0 missing/unexpected |

No physical command was executed. The exact staged manual procedure is in
`docs/validation/MANUAL_FUNCTIONAL_VALIDATION.md`.

## Git integration status

The isolated integration index/commit and final main/origin-main hashes are
recorded after the ordinary push and merge below. A clean temporary main
worktree is used for merge verification so the separate concurrent RH56 task
cannot be staged or overwritten.
