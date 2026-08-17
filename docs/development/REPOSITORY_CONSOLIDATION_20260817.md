# Repository consolidation audit — 2026-08-17

This is a maintenance audit for `brVrzl/embodied-hand-lab`. The local
checkout was the source of truth. No robot, camera, raw/master dataset, or
uncommitted user work was touched.

## Initial state

| Item | Observed state |
|---|---|
| Working branch | `research/thread-b-force-interaction` tracking `origin/research/thread-b-force-interaction` |
| Working tree | Clean; no staged, modified, or untracked repository files |
| Local/origin main | `fa194170e1599eb3678ef9cbd627f8966cbda144` |
| Local/origin research | `895c2e42a8e9d0a41530247b1f4c5b857b362ae2` |
| Source dataset boundary | `data/training/**` and raw/master inputs are preserved and read-only for this maintenance |

## Path audit before deletion

| Path or family | Classification | Entry/reference evidence | Replacement | Decision |
|---|---|---|---|---|
| `src/**`, `native/**`, `assets/**` | MAIN_MAINTAINED | Current architecture, safety docs, runtime imports, CMake and simulation assets | None | Keep current implementation on main |
| `src/episode_dataset/**` | MAIN_MAINTAINED | `embodied-lab` CLI, maintained collection/materialization commands, dataset tests | None | Keep; includes physical-bottle materialization and OpenPI mapping |
| `src/embodiment_core/act_contract.py`, `act_temporal_executor.py` | MAIN_MAINTAINED | Imported by maintained rollout/runtime code and protected by contract/executor tests | None | Keep |
| `src/rh56_driver/**`, `src/teleoperation/**` | MAIN_MAINTAINED | Current RH56/JAKA runtime and safety test suites | None | Keep; preserve current safety boundaries |
| `tools/act_live_shadow.py`, `act_shadow_model_worker.py`, `act_physical_rollout.py` | MAIN_MAINTAINED | Current physical rollout adapter and shadow worker imports/tests | None | Keep; no hardware execution during audit |
| `tools/lerobot_physical_bottle.py` | MAIN_MAINTAINED | `scripts/train_physical_bottle_lerobot.sh`, dataset docs and LeRobot tests | None | Keep |
| `tools/analyze_act_horizon_coverage.py` | MAIN_MAINTAINED | Current nominal52 checkpoint evaluator requires `analysis/horizon_coverage.json`; direct test | None | Keep as reusable offline evaluation tool |
| `tools/analyze_act_nominal_checkpoint.py` | MAIN_MAINTAINED | Current nominal52 checkpoint evaluator invokes it for every checkpoint; direct test | None | Keep |
| `tools/act_transition_execution_audit.py` | RESEARCH_ONLY | Only its own one-off test and dated transition audit evidence reference it | `src/embodiment_core/act_temporal_executor.py` for maintained behavior | Exclude from main; preserve on research |
| `tools/act_temporal_executor_replay.py` | RESEARCH_ONLY | Only dated replay evidence; no maintained CLI/docs/runtime import | `src/embodiment_core/act_temporal_executor.py` | Exclude from main; preserve on research |
| `tools/analyze_act_bottle_rollout.py` | RESEARCH_ONLY | Only its own analysis test; no current documented entrypoint | `tools/act_physical_rollout.py` for runtime | Exclude from main; preserve on research |
| `tools/analyze_physical_bottle_curation.py` | RESEARCH_ONLY | Only its own curation-analysis test; canonical materializer owns current workflow | `src/episode_dataset/physical_bottle_materialization.py` | Exclude from main; preserve on research |
| Matching one-off tests for transition/replay/bottle-rollout/curation | DEAD_WITH_TARGET / RESEARCH_ONLY | Each loads only the corresponding one-off tool | Maintained contract, executor, materializer and rollout tests | Exclude from main with targets; preserve research evidence where useful |
| `tests/test_act_horizon_coverage.py`, `test_act_nominal_checkpoint_analysis.py` | REGRESSION_WORTH_KEEPING | Current documented nominal52 evaluation pipeline calls both tools | None | Keep on main |
| `tests/test_act_checkpoint_contract.py`, `test_act_temporal_executor.py`, `test_act_physical_rollout_adapter.py` | SAFETY_CRITICAL / CORE_BEHAVIOR | Protect current checkpoint/action contract and temporal/runtime adapter | None | Keep on main |
| `configs/training/act/**`, `configs/training/pi05/**`, `configs/training/shared/**` | MAIN_MAINTAINED | Current training READMEs, launchers, config-layout tests and locked task split | Historical aliases | Keep canonical configs on main |
| `configs/training/*v1*`, old root LeRobot/OpenPI aliases | DELETE_DEAD / ARCHIVE_ONLY | Superseded by canonical `act/`, `pi05/`, and `shared/` layout; no current entrypoint | Git history/research branch | Remove from main |
| `configs/archive/training/**` | ARCHIVE_ONLY | Historical mixed/val4 snapshots; no active launcher consumes them | Research branch and Git history | Exclude from main |
| `research_log/**` | RESEARCH_ONLY | Dated evidence, audit JSON/figures, rollout narratives; not runtime imports | Current `docs/**` authorities | Exclude from main; preserve on research |
| `research/__init__.py` | DELETE_DEAD | Empty package with no import or meaningful sibling code | None | Delete from both maintained branches |
| `training/act/**`, `training/pi05/**` | MAIN_MAINTAINED | Current operator docs, launchers, configs and dependency checks | None | Keep on main |
| `.gitmodules`, `third_party/lerobot`, `third_party/openpi` | MAIN_MAINTAINED | Current launchers mount exact pinned submodules; notices and lock files reference them | None | Keep with license notices |
| `scripts/run_quest_*`, `scripts/check_*`, physical ACT/LeRobot launcher | MAIN_MAINTAINED | Operator docs and shell entrypoints | None | Keep; run only offline/help/smoke validation |
| `experiments/**` | DELETE_DEAD / SUPERSEDED | Current project code was moved to `training/pi05/**`; no tracked current tree remains | `training/pi05/**` | Do not reintroduce |
| ignored `outputs/**`, caches, logs, checkpoints | OUT_OF_SCOPE | Generated runtime artifacts, ignored by Git | None | Do not modify |

## Branch audit before deletion

| Branch | Evidence vs main | Decision |
|---|---|---|
| `main` | `fa19417`, baseline | Update only from validated integration branch |
| `research/thread-b-force-interaction` | 25 commits ahead, 0 behind; current production plus research evidence | Keep research; merge updated main into it after main push |
| `dev` | origin ahead 1; local ahead 2; content is ancestor of research | Delete local/origin after main receives approved snapshot |
| `archive/dev-teleop-dataset-pipeline-20260806` | 0 ahead, 8 behind | Safe delete after consolidation |
| `backup/full-local-state-20260804-181401` | 0 ahead, 38 behind; existing tag `backup/pre-repository-simplification-20260804-181401` | Delete branch; retain existing archival tag |
| `backup/dev-teleop-dataset-pipeline-20260807` | 3 unique backup commits; final tree contains superseded digital-twin/vendor/old validation material | Create archival tag, then delete branch after validating no current consumer |
| `research/broad-exploration` | Local-only research branch; 3 unique broad-research commits plus older superseded tree | Preserve its broad survey in research, tag its history, then delete stale local branch |

The three unique commits on `backup/dev-teleop-dataset-pipeline-20260807`
were inspected: they preserve an older teleoperation-pipeline snapshot,
vendor binaries/manuals, and a pre-digital-twin-removal worktree. The current
main/research source already follows the later simplified architecture; no
unique required production file from that snapshot is missing. The branch is
therefore archival rather than active.

## Final decisions and validation

The isolated integration snapshot is validated before it is applied to main.
It keeps the current hardware/simulation/data/runtime code, canonical ACT and
π0.5 training layout, current offline evaluation tools, and the pinned OpenPI
and LeRobot gitlinks. It removes the six superseded root training aliases, the
historical `configs/archive/training/**` family, the empty `research/` package,
the old `act_physical_bottle_validation.py` integration pair, and the four
one-off analysis/replay tools with their dedicated tests. The existing main
`research_log/**` files are removed from the final main tree; the research
branch retains the complete dated evidence and generated artifacts.

The final main tree has no `research_log/` or `research/` directory and no
maintained source reference to either. This is a deliberate removal of stale
paths already present on the old main, not an import of research material.

Validation on `integration/main-consolidation-20260817`:

- `700 passed, 2 warnings` with the repository's full offline pytest suite;
- Python compileall for `src`, `tools`, `tests`, and `training` passed;
- 700 tests collected before execution;
- all shell scripts under `scripts/` and `training/` passed `bash -n`;
- unified CLI and dataset CLI `--help` passed;
- pinned OpenPI runner source compiles, but its host-side help smoke requires
  the Thor OpenPI container dependencies (`flax` was not installed in the
  repository venv); the runbook continues to require that pinned container;
- `git diff --check` passed;
- no hardware connection, robot motion, raw/master dataset edit, or training
  job was performed.

Final branch SHAs, pushed branch decisions, archival tags, and any remaining
uncertain items are appended after the normal main/research push sequence.
