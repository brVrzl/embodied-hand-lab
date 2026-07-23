# Embodied Lab repository guide

## Purpose and current authority

This repository develops simulation, teleoperation, perception, and data tools
for a JAKA Mini2 arm with an Inspire RH56DFX hand. The primary current
teleoperation stack is Meta Quest 3 hand/wrist tracking plus a left Touch
controller, MuJoCo simulation, and an explicitly authorized physical JAKA
ServoJ/EDG adapter.

Read [docs/README.md](docs/README.md) and
[docs/status/current_status.md](docs/status/current_status.md) before changing
control code or interpreting an old report. Dated files under `docs/history/`
are evidence or design history, not current operating instructions.

The authoritative Quest-to-JAKA control path is:

```text
Quest HTS + CTRL packets
  -> validation and bounded input queue
  -> release-before-press clutch/reference capture
  -> frame mapping and filters
  -> shared continuation IK and feasibility checks
  -> immutable AcceptedArmTarget
  -> MuJoCo adapter OR JAKA joint adapter
```

The two adapters are identical before `AcceptedArmTarget`. Physical JAKA must
never follow MuJoCo `qpos`, and the physical adapter must not remap, filter, or
recompute IK. In joint-teleop mode the native worker must make zero JAKA
`kine_inverse` calls.

## Absolute safety rules

- Default to offline and simulation work. Never connect to or command a JAKA,
  RH56DFX, Quest headset, or other actuator unless the user separately and
  explicitly authorizes the exact physical gate in the current session.
- Repository maintenance, tests, `--help`, fake-worker replay, and static
  analysis do not authorize login, enable, servo mode, EDG, or motion.
- Never perform automatic payload identification or write payload, TCP,
  installation, collision, or controller safety settings.
- A physical procedure must retain exact acknowledgement flags, bounded
  duration/displacement, operator stop access, workspace checks, and cleanup.
- Controller collision, servo alarm, emergency stop, loss of power/enable,
  SDK error, command-loop hard timing fault, or actual liveness loss is a hard
  stop. Candidate infeasibility is different: `HOLD_REJECTED` keeps a fresh
  heartbeat and holds the last safe target.
- Do not weaken startup continuity, timeout, singularity, collision, joint
  limit, output velocity/acceleration, or cleanup contracts to make a test pass.
- Describe physical status literally: offline tested, simulation validated,
  partially physically validated, passed, failed, or not validated. Never turn
  implementation or replay evidence into a physical PASS.

Recorded operator state (not code-owned truth): payload 0.8 kg, COM
`[9.289, 12.427, 36.961]` mm, upright/floor installation with X=0° and Z=0°,
TCP1-TCP10 zero, and unchanged controller safety limits. Software must not
silently apply these values. Verify them at the controller before any future
authorized physical gate.

## Repository and worktree discipline

- Work only in the current worktree and branch. Inspect `git status`,
  `git worktree list`, remotes, and upstream state before broad work.
- Preserve unrelated changes and all intentionally untracked data. In
  particular, do not modify, stage, rename, delete, or commit
  `tools/teleop_mujoco_jaka_rh56.py`, `learned_policy/`, or concurrent user
  captures/models/calibration/experiments unless the user explicitly expands
  the scope.
- Never rewrite history, force-push, use `git clean`, reset another person's
  work, delete branches, or modify another linked worktree.
- Keep commits scoped. Inspect the staged diff, run `git diff --check`, exclude
  user work, fetch before push, and never overwrite remote work.
- Start a new Codex session for a separately authorized physical gate, after a
  major context-changing merge, or when the current session cannot retain the
  complete safety/evidence context.

## Layout and documentation

- `src/`: reusable Python packages and shared control contracts.
- `tools/`: Python entry points and diagnostics.
- `scripts/`: operator-facing wrappers.
- `native/`: JAKA diagnostic and EDG worker C++ sources.
- `configs/`: versioned examples and runtime policy.
- `data/sim_assets/`, `models/`: robot and simulation assets.
- `tests/`: offline tests; hardware is never required by the default suite.
- `docs/`: current architecture, operation, safety, development, reference,
  status, and indexed history.

Use one current page per topic and link to it from `docs/README.md`. Put dated
outcomes and superseded designs in `docs/history/`; do not edit raw evidence to
match later behavior. When moving evidence, keep report/raw-log relationships
and update the history index. New documentation must use repository-relative
paths, verified command names, and explicit validation levels.

## Setup, build, and validation

From the repository root:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e ".[dev]"

cmake -S native/jaka_servo_worker -B build/jaka_servo_worker
cmake --build build/jaka_servo_worker -j

PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m compileall -q src tools tests
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m pytest --collect-only -q -p no:cacheprovider
.venv/bin/python -m pytest -q
git diff --check
```

Critical Quest/JAKA checks:

```bash
.venv/bin/python -m pytest -q \
  tests/test_quest_jaka_shared_pipeline.py \
  tests/test_quest_jaka_output_feasibility.py \
  tests/test_quest_jaka_singularity_liveness.py \
  tests/test_jaka_edg_resampler.py \
  tests/test_native_jaka_servo_worker.py \
  tests/test_quest_jaka_hardware_cli.py
```

Run `bash -n` on changed shell scripts. The project currently configures no
separate formatter, linter, type checker, or CI workflow; do not invent a
passing claim for tools that are not configured.

Tests should name the current contract, use deterministic fake/offline
backends, and add regression coverage for fixed safety defects. Do not delete a
test because it is old or slow; first prove the behavior is obsolete or fully
duplicated. Physical probes remain separately gated and outside default pytest.

## Current constraints

The sole JAKA SDK session performs lightweight status polling in the command
worker; the earlier second-session monitor was rejected after a physical
no-motion failure. The latest shared output-acceleration correction is offline
tested but has not received a bounded post-fix physical validation. The earlier
J4 collision cause is unresolved. TCP calibration and Quest-driven physical
RH56 teleoperation are not complete. See the validation matrix and current
status before planning the next phase.
