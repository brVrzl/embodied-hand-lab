# Repository consolidation audit

Audit date: 2026-07-23.

## Scope and starting state

The audited root was `/home/thor/projects/embodied_lab`, branch
`feature/jaka-teledex-control-foundation`, starting HEAD
`2f5a3bd57e3e50f123d50c0ada8a9b4dbe123d91`. It was 15 commits ahead of its
configured `origin` upstream before synchronization. No submodules or active
Git hooks were present. All linked worktrees were inventoried and left
untouched.

The starting worktree contained concurrent user changes:

- modified `tools/teleop_mujoco_jaka_rh56.py`;
- untracked `learned_policy/`;
- ignored local captures/media/logs and other runtime artifacts.

They were classified as user-owned and excluded from maintenance changes.

## Inventory findings

- 745 tracked files; 65 tracked Markdown/RST documents at audit start.
- 76 collected pytest modules and 627 collected tests at audit start.
- Current source includes 149 Python modules plus native worker/diagnostics.
- Tracked large binary content is primarily vendor JAKA SDK libraries and robot
  meshes; it is intentional source/vendor data, not generated build output.
- Physical gate and incident evidence is unique and was preserved.
- The `models/digital_twin/scene.xml` alias and repeated vendor headers/wrappers
  are intentional compatibility copies, not arbitrary duplicates.
- Local build, cache, media, model, capture, log, and artifact output is already
  ignored. No broad ignore pattern was added.
- The repository has no configured CI workflow, formatter, linter, or type
  checker beyond pytest/build commands.

## Documentation decisions

Current architecture, operation, safety, development, reference, and status
pages now have one indexed location. Dated gates/incidents moved with their raw
evidence. Old TeleDex handoffs/audits and the rebuild snapshot moved to
`docs/history/archived_designs/`. The generic motion-input, digital-twin,
vision, pregrasp, data, and literature areas remain because they are active
parallel work or useful current references.

Historical text was not rewritten to conceal stale decisions. Its history index
warns that old worktree paths, branch names, IPs, commands, test totals,
SPACE-clutch behavior, fixed J5 policy, repeat-latest resampling, and
MuJoCo-following concepts are not current authority.

## Move/rename manifest

| Previous location | New location | Classification/reason |
|---|---|---|
| `Agents.md` | `AGENTS.md` | standard scoped agent-instruction filename; rewritten as current authority |
| `docs/quest_jaka_sim_teleoperation.md` | `docs/operation/simulation_demo.md` | current topic moved into operation architecture and refreshed |
| root `PROJECT_HANDOFF.md` | `docs/history/archived_designs/teledex/iphone_rh56_collision_handoff.md` | dated collision/TeleDex handoff, not root authority |
| `docs/jaka_*gate*` and foundation reports | `docs/history/gates/jaka_foundation_20260716/` | unique physical gate evidence |
| `docs/gate3b_measurements/`, `docs/gate3c_measurements/` | corresponding foundation gate subdirectories | raw evidence kept with reports |
| `docs/quest_jaka_*audit/followup*` | `docs/history/incidents/quest_jaka_20260722_23/` | dated incident/correction chronology |
| `docs/measurements/` | incident `measurements/` | raw replay/fake-worker evidence kept with chronology |
| TeleDex `docs/handoffs`, `docs/reports`, `docs/teleoperation` files | `docs/history/archived_designs/teledex/` | superseded designs and handoffs |
| `docs/project_rebuild_status.md` | `docs/history/archived_designs/project_rebuild/project_rebuild_status_20260713.md` | dated pre-current-stack snapshot |
| dated motion-input audit/build/integration documents | `docs/history/archived_designs/motion_input/` | checkpoint evidence separated from current UMIP/transport references |
| root `real_robot_data_collection_protocol.md` | `docs/development/real_robot_data_collection.md` | active data schema moved out of repository root and linked to current safety/status |
| dated Jetson and initial tennis-ball digital-twin plans | `docs/history/archived_designs/plans/` | superseded planning paths separated from current digital-twin status |

## Deletion manifest

No tracked document, report, raw evidence file, test, source module, asset, or
configuration was deleted during consolidation. Empty source directories
created solely by moves were removed. Any later deletion must be added here
with active-reference proof and replacement.

## Test decision

Every collected test was mapped to an active, compatibility, historical
regression, simulation, native, or safety-gate contract. No test was removed:
the audit did not find a duplicate whose deletion could be proven coverage
neutral. Marker churn was also avoided because hardware execution is already
outside default pytest and a new marker taxonomy would not improve the current
single suite.

The audit started with 627 tests in 76 modules. Three cases were added in
`tests/test_repository_hygiene.py`: one prevents credentials embedded in
runtime source/config and two require an explicit input source for camera
tools. Final collection is 630 tests. The full run completed with 629 passed
and one expected skip because a headless MuJoCo rendering backend was not
configured.

## Code/configuration and artifact decisions

- Removed an embedded iPhone camera URL containing basic-auth credentials.
  Camera tools now require `--source` or an explicit `--realsense-serial`.
- Removed the unused `/home/w/Desktop/...` ManiSkill asset-prefix constant;
  runtime asset resolution already uses the repository-relative asset root.
- Kept sample robot/network configuration where it is an explicit example.
  No physical limits, mapping, payload, TCP, installation, or controller
  setting changed.
- Kept intentional duplicate vendor C/C++ headers, compatibility wrapper
  scripts, and the digital-twin scene alias. No duplicate current document or
  unique evidence was deleted.
- `.gitignore` was already conservative for builds, caches, logs, captures,
  models, and local artifacts, so it was not broadened. Existing ignored local
  media/log/artifact data and untracked `learned_policy/` were preserved.

## Final offline validation

The following completed without hardware access:

- native `jaka_servo_worker` configure/build and `--help`;
- Python `compileall` over `src`, `tools`, and `tests`;
- 630-test collection;
- 92/92 critical Quest/JAKA safety tests;
- full pytest: 629 passed, one expected headless-render skip;
- syntax check of all 50 shell scripts;
- parse of all 27 versioned YAML configs;
- tracked schema/evidence parse: 27 JSON, 5 JSONL, and 44 YAML files;
- zero broken Markdown links and zero missing current backtick paths;
- documented CLI `--help` checks, stale-current-path scan, credential/private
  key pattern scan, duplicate hash audit, and `git diff --check`.

No JAKA, RH56DFX, Quest, camera, robot SDK session, servo/EDG mode, payload
identification, controller write, or live physical gate was invoked.
