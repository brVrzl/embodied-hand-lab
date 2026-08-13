# Physical bottle nominal52 expansion audit

## Scope and source preservation

This is an offline, repository-owned curation view over the immutable raw
episodes. No raw JSONL, metadata, or MP4 was rewritten. The audit scope is
source episodes 67--180 as explicitly listed in
`configs/training/physical_bottle_v4_nominal52.yaml`; episode 181 is an
unfinished `.partial` recording and is outside the view.

The human audit was supplied on 2026-08-13:

- 153 was reported missing and is excluded. Local files exist, so it is kept
  in the audit inventory as an explicit contradiction rather than silently
  discarded.
- 163--165, 167--171, and 175--180 are excluded as operator-marked unusable.
- All other recordings in the new range are accepted under the operator's
  instruction. Episode 161 is included, but its raw metadata warning
  (`invalid`, `aborted_robot_safety`, `control_source_timestamp_regression`) is
  preserved in the audit and should be reviewed before a safety-sensitive
  training run.

## Recovered logical segments

Offline row/action-state analysis and workspace video support these exact
source-frame ranges:

| derived segment | source | frames | decision |
|---|---:|---:|---|
| `172_a` | 172 | 0--503 | include |
| `172_b` | 172 | 802--1326 | include |
| `172_c` | 172 | 1664--2169 | include |
| `174_a` | 174 | 0--519 | include |
| `174_b` | 174 | 845--1488 | include |
| `174_c` | 174 | 1794--2091 | exclude, incomplete final task |

The excluded gaps are `172: 504--801` and `1327--1663`, and
`174: 520--844`; these are manual recovery/reset intervals. The final
`174: 1794--2091` interval is not used because the operator reported that
third task as missing/incomplete. No action chunk can cross these gaps because
each derived segment is a separate logical episode.

For ordinary new episodes, the crop keeps the complete approach through
grasp, lift/transport, placement, and release, then removes the recorded
stationary/manual-reset tail. Where a short initial hold was visible in the
action/video evidence, it was removed without removing approach motion.

## Result

The previous nominal33 view has 33 logical trajectories. The new accepted
cohort contributes 19: 14 standalone episodes, three segments from 172, and
two segments from 174. The resulting `physical_bottle_v4_nominal52` view has:

- **52 logical trajectories**;
- **33,111 matched rows** in ACT and ACT+Force;
- **1,102.100 seconds** of cropped task data;
- **37 train / 15 validation / 0 test** logical trajectories;
- identical frame ordering, source provenance, images, states, and actions in
  both views;
- state `[JAKA measured 6, RH56 measured 6]`, action `[JAKA target 6, RH56
  target 6]`, and six raw RH56 FORCE_ACT channels;
- no future source timestamp, invalid canonical row, or raw hash change in the
  final validation.

The new materializer reports 23,802 force-valid training rows; invalid/stale
force rows remain matched rather than being silently removed from only
ACT+Force. Force remains raw RH56 counts and is not converted or interpolated.

## Commands and validation

```bash
PYTHONPATH=src .venv/bin/python -m episode_dataset.cli audit-physical-bottle \
  --config configs/training/physical_bottle_v4_nominal52.yaml
PYTHONPATH=src .venv/bin/python -m episode_dataset.cli materialize-physical-bottle \
  --config configs/training/physical_bottle_v4_nominal52.yaml
PYTHONPATH=src .venv/bin/python -m episode_dataset.cli validate-physical-bottle \
  data/training/physical_bottle_v4_nominal52
```

The last command passed: schema/finite checks, causal timestamps, camera
frame-count checks, matched ACT/ACT+Force samples, episode boundaries, loader
smoke checks, and raw-source hash preservation.

No training or physical rollout was started by this audit.
