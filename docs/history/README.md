# History and evidence

This directory preserves dated results and superseded designs without allowing
them to compete with current instructions.

## Evidence classes

- `gates/`: physical validation reports and their raw measurements. A PASS is
  limited to the exact historical gate.
- `incidents/`: failures, diagnostics, corrections, replays, and associated raw
  evidence. Later fixes do not rewrite earlier outcomes.
- `archived_designs/`: handoffs, implementation audits, and status snapshots
  superseded by current architecture/status pages.

Current operational documentation lives outside this directory.

## Preserved sets

### JAKA foundation gates, 2026-07-16

`gates/jaka_foundation_20260716/` contains Gates 1/2, 3A read-only results, 3B
zero-motion/EDG timing stages, 3C minimal J6 plans/results, and their
`gate3b_measurements/` and `gate3c_measurements/` files. The report/raw-file
group is preserved together. Gate 3C physically passed its exact small J6
procedures; it is not proof of current Quest teleoperation.

### Quest/JAKA incident and correction sequence, 2026-07-22–23

`incidents/quest_jaka_20260722_23/` preserves:

- the initial simulation/physical parity audit;
- the resampler and shared-target correction follow-up;
- the J4 collision and payload/health-monitor diagnostic chronology;
- the singularity/liveness correction;
- the output velocity/acceleration feasibility correction;
- raw/replay/fake-worker measurements under `measurements/`.

Read the dated documents chronologically. The current status page is the
authoritative synthesis.

### Archived designs and handoffs

`archived_designs/teledex/` holds bounded TeleDex architecture, foundation
audits/reports, and old handoffs. `archived_designs/project_rebuild/` holds the
pre-Quest repository rebuild snapshot. `archived_designs/motion_input/` holds
the original input repository audit, streamer integration gate, offline
simulation gate, and dual-clutch design. They contain useful provenance and
parallel-project context but stale branch names, paths, or test totals must not
be used as current instructions.

## Retention policy

Preserve unique physical/incident evidence, meaningful hashes, and regression
fixtures. Generated local output stays ignored unless intentionally selected.
A future deletion requires proof of complete duplication, no active code/test
dependency, no unique reproducibility value, and an audit-manifest entry.
