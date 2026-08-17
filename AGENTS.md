# Repository Agent Guide

## Scope

- Follow the user's current request first. Treat this file as stable repository-wide guidance, not as a source of current experiment status.
- Inspect the relevant code and current documentation before changing behavior. Do not infer present requirements from stale reports, old experiments, or Git history alone.
- Keep repository-wide instructions here. Put genuinely subsystem-specific rules in a nested `AGENTS.md` only when that subsystem needs durable instructions of its own.
- Prefer the existing canonical implementation and configuration. Do not create a parallel path when the maintained path can be extended cleanly.

## Authorization and safety

- Default to offline, simulation, and read-only work.
- Do not enable, command, move, calibrate, or reconfigure physical hardware unless the user explicitly authorizes that physical action in the current task.
- Never weaken safety limits, watchdogs, freshness checks, workspace limits, legality checks, or cleanup behavior to make an experiment or test pass.
- Treat raw/master datasets and irreplaceable captures as immutable unless the task explicitly requests changing them. Create derived views or outputs instead.
- External writes and destructive operations require explicit task scope. Reading, analysis, local edits, and non-destructive tests do not.

## Engineering restraint

- Make the smallest change that solves the observed requirement.
- Prefer deletion, consolidation, or extension of an existing implementation over adding another implementation of the same responsibility.
- Do not add fallback paths, retries, compatibility layers, recovery states, configuration flags, wrappers, caches, state machines, or abstractions without a current caller, observed failure, or explicit requirement.
- Do not add code only for hypothetical future use.
- Validate untrusted input at the boundary that owns it. Do not repeatedly revalidate trusted internal data at consecutive layers.
- Prefer an explicit, observable failure over silent fallback that hides a programming, configuration, or integration error.
- Do not catch and rethrow errors through multiple layers unless the added layer contributes actionable context or owns recovery.
- Do not duplicate safety or validation logic across layers. Keep one authoritative check at the appropriate boundary.
- Do not compute or persist checksums, SHA hashes, file fingerprints, or whole-tree integrity manifests as routine validation. Use them only when integrity/provenance is the subject of the task, an external artifact must be verified, or an existing interface explicitly requires them.
- Do not introduce defensive complexity merely because something could theoretically fail.

## Temporary work and durable artifacts

- Investigation artifacts are temporary by default.
- Put ad-hoc probes, experiments, parameter sweeps, scratch analysis, debug output, and one-time migration helpers in an untracked or ignored temporary location whenever practical, not directly in maintained `src/`, `tools/`, or `tests/`.
- Before finishing a task, review every file created by that task and either:
  1. promote it because it has durable value, or
  2. delete it.
- A new tracked tool has durable value only if it supports a current documented workflow, is expected to be reused, or provides a maintained operator/developer capability.
- A file being referenced only by its own test is not sufficient reason to keep it.
- A one-off analysis script does not become permanent merely because it produced an important result. Preserve the result or concise evidence, not necessarily the generator.
- Generated logs, plots, dumps, replay outputs, and large analysis artifacts should not be committed unless they contain intentionally retained, non-reconstructible evidence.
- Research narratives and experiment-specific artifacts should stay out of the production `main` tree unless explicitly promoted into maintained documentation or code.

## Tests

- Add or retain tests for stable behavior, public interfaces, safety-critical behavior, or real regressions that could reasonably recur.
- Prefer extending an existing behavior-level test over adding a new test file.
- Do not add permanent tests for one-off probes, implementation details, private call order, mock call counts, temporary architecture, or historical constants.
- When deleting obsolete code, delete tests whose only purpose was to test that obsolete code.
- Test count and coverage percentage are not goals by themselves.
- Run the smallest relevant validation that gives confidence in the change.
- Run broader tests when the change crosses subsystem boundaries, affects shared infrastructure, changes safety-critical behavior, or the user explicitly requests broader validation.
- Do not create checksum comparisons, snapshot manifests, duplicate validators, or extra test harnesses merely to demonstrate that unrelated files were untouched.

## Git and worktree discipline

- Inspect the current branch, worktree, and `git status` before broad edits.
- Preserve unrelated user changes and intentionally untracked work.
- Keep commits scoped to the requested change.
- Do not rewrite history, force-push, hard-reset user work, run destructive `git clean`, or delete branches unless the task explicitly requires that operation and the affected history has been inspected.
- Before deleting or consolidating a branch, check its unique commits and whether those commits are reachable elsewhere.
- Do not commit datasets, checkpoints, transient logs, caches, generated build output, or large experiment artifacts unless they are intentionally versioned repository assets.

## Documentation

- Keep durable architecture, interfaces, setup, operation, and safety information in maintained documentation.
- Keep transient experiment status, dated investigations, benchmark outputs, checkpoint identifiers, commit SHAs, and research conclusions out of this file.
- Prefer one maintained source of truth per topic instead of copying the same rule or status into multiple documents.
- Update documentation only when the maintained behavior or interface changes; do not create a new report for every engineering task.

## Completion

Before finishing a change:

- review the final diff for unnecessary files and accidental complexity;
- remove temporary probes, debug code, unused compatibility paths, and generated artifacts;
- run the relevant validation;
- check for unrelated staged changes;
- report what changed, what was validated, and anything intentionally left unresolved.

Do not claim validation that was not actually performed.
