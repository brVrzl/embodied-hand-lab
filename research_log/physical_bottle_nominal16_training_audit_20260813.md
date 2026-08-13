# Physical bottle nominal16 curation and ACT comparison

Date: 2026-08-13 (Asia/Shanghai)

Worktree base inspected: `research/thread-b-force-interaction` at
`e491d3e873eafcb76712ba6a3194ec08e9d3a037`. The human decisions are from the
operator audit on 2026-08-13. All work was offline. No robot, camera, Quest, or
serial device was opened. `data/raw_episodes/` and the previous
`physical_bottle_v2` view were not modified.

## Decision

The old 25-episode view is a mixed-quality diagnostic dataset, not a clean
expert baseline. The new primary view is `physical_bottle_v2_nominal16`: 16
human-approved, correctly segmented, task-trimmed trajectories. It contains
12,177 rows (405.43 s), with 12 train trajectories / 9,377 rows and 4 held-out
trajectories / 2,800 rows.

The controlled 2k clean scratch run still recalled **0/56** held-out
approach-to-grasp transitions. Cleaning therefore did not by itself remove the
state-persistence failure. ImageNet initialization produced **12/56** recall at
2k, but 11 hits came from `ep109`, none from `ep088` or `ep108`, and its
all-horizon error remained worse than a copy-current-state baseline. This is a
useful representation signal, not a rollout-ready checkpoint.

Current gate: **KILL command-enabled rollout; INVESTIGATE model/temporal
objective; GO for further offline controlled work only.** The previously
planned pretrained experiment on the dirty val4 view was not started and its
launcher mode is retired.

## Human-audit manifest and exact provenance

The machine-readable authority is
`configs/training/physical_bottle_v2_nominal16.yaml`. The reviewed source cohort
is frozen explicitly so future collection cannot silently enter this view.
Unknown episode-specific failure reasons remain the generic
`manual_audit_non_nominal`.

| derived trajectory | source | task source frames | duration s | split | audit status |
|---|---:|---:|---:|---|---|
| ep067 | 67 | 62–856 | 26.47 | train | nominal full task |
| ep070 | 70 | 14–698 | 22.80 | train | nominal full task |
| ep081 | 81 | 0–946 | 31.53 | train | nominal full task |
| ep087 | 87 | 0–788 | 26.27 | val | nominal full task |
| ep088 | 88 | 61–743 | 22.73 | val | nominal full task |
| ep095 | 95 | 0–674 | 22.47 | train | nominal full task |
| ep096 | 96 | 0–733 | 24.43 | train | nominal full task |
| ep097 | 97 | 29–868 | 27.97 | train | nominal full task |
| ep098 | 98 | 22–760 | 24.63 | train | nominal full task |
| ep099_a | 99 | 0–965 | 32.17 | train | nominal full task |
| ep099_b | 99 | 1335–2348 | 33.80 | train | nominal full task |
| ep102_b | 102 | 1209–1934 | 24.17 | train | nominal full task |
| ep108 | 108 | 0–706 | 23.53 | val | nominal full task |
| ep109 | 109 | 0–620 | 20.67 | val | nominal full task |
| ep116 | 116 | 0–556 | 18.53 | train | nominal full task |
| ep117 | 117 | 0–698 | 23.27 | train | nominal full task |

Source 99 is `ep099_a` (0–965), excluded reset/recovery (966–1334), then
`ep099_b` (1335–2348), followed by excluded tail (2349–2497). Source 102 is
excluded non-nominal `ep102_a` (0–877), excluded reset/recovery (878–1208),
then nominal `ep102_b` (1209–1934), followed by excluded tail (1935–2084).
Boundaries were selected from synchronized action/clutch state and inspected
workspace/wrist imagery. They are explicit historical curation decisions, not
an automatic velocity detector.

The other excluded trajectories are `ep071`, `ep074`, `ep083`, `ep089`,
`ep090`, `ep091`, `ep092`, `ep102_a`, `ep105`, `ep113`, and `ep114`. They are
preserved for later failure/correction research and are absent from nominal
behavior cloning.

## Task-tail audit

Normal task ends use the persisted task-release row. Five recordings had
reviewed pre-task holds removed: source 67 (2.07 s), 70 (0.47 s), 88 (2.03 s),
97 (0.97 s), and 98 (0.73 s). Their first retained row precedes the first
accepted target change, preserving approach. Ordinary post-task tails were
4.97–5.00 s. `ep099_a` had a 12.30 s completion-to-next-task reset interval.
Per-trajectory post-task row-equivalent durations are: `ep067` 4.97 s;
`ep070`, `ep081`, `ep087`, `ep088`, `ep095`, `ep097`, `ep098`, `ep102_b`,
`ep108`, and `ep109` 5.00 s each; `ep096`, `ep099_b`, `ep116`, and `ep117`
4.97 s each; and `ep099_a` 12.30 s.

| view | trajectories | rows | duration s | approach rows | transition chunks | post-grasp rows | stationary-action fraction |
|---|---:|---:|---:|---:|---:|---:|---:|
| old mixed materialized | 25 | 20,744 | 690.70 | 8,647 | 350 | 12,097 | 23.84% |
| nominal16 before task trim | 16 | 14,979 | 498.83 | 5,383 | 224 | 9,596 | 32.89% |
| nominal16 task-trimmed | 16 | 12,177 | 405.43 | 5,195 | 224 | 6,982 | 17.93% |

Task trimming removed 2,802 rows (18.71%). Post-task tails account for 2,614
rows (17.45% of the pre-trim nominal view); the rest are pre-task holds. Of all
removed rows, 97.79% have action-change L2 <= `1e-4`; removed action-change L2
has mean 0.000361, p95 0, and max 0.02885. Thus the reset/tail hypothesis is
quantitatively real: those rows strongly over-represent “keep state.”

The old mixed materialization had already cropped ordinary persisted-release
tails. Its directly proven malformed-reset lower bound is 331 rows (1.60%):
source 99 frames 966–1169 and source 102 frames 878–1004. No unsupported
completion label was assigned to other non-nominal sources, so this is a lower
bound rather than a claim that the old view had no other idle content. These
331 rows are 66.77% stationary by the same threshold; action-change L2 has
mean 0.00533, p95 0.02176, and max 0.03257.

## Split and data integrity

The split unit is acquisition session, inferred from persisted workspace-camera
process identity and chronology. Session groups are sorted chronologically.
The integer SHA-256 of seed `physical_bottle_v2_nominal16_v1` has offset 2
modulo stride 3, so every third group selects sessions 87/88 and 108/109 (four
validation trajectories). The seed digest is
`9d74f8846471b87eaea1928e997931defd81e346b7b0566055c561750ee183ab`;
the selected payload digest is
`fbc11d4f4536a1db68588126d41265eba8a1ce793287e6e474a8090907d123f4`.
Both 99 segments remain together in train. No source/session leaks across
splits, and no visual difficulty label participates in selection.

The materializer validated two cameras, 12-D measured state, 12-D native
absolute action, six raw RH56 force channels, causal timestamps, identical ACT
and ACT+Force rows, and immutable raw hashes. The real pinned LeRobot 0.6.2
loader opened all 12,177 rows; action chunks are `[16,12]`, and end masks stop
at every logical segment/task/reset boundary. Force validity is 100% in this
view, but force remains excluded from the ACT models in this comparison.

## Clean transition distribution

The audit heuristic is used only for diagnostics: first sustained first-five
RH56 closure-vector L2 above `max(0.10, 25% of trajectory peak)`, relative to
the lowest-closure 15-row window in the first 60% of that trajectory. It is not
a semantic label or deployment threshold. All 16 trajectories expose 14
transition-containing 16-step chunks, for 224 total. Grasp onset ranges from
5.73 s (`ep102_b`) to 22.80 s (`ep099_a`); peak closure magnitude ranges
0.539–1.003. The variation remains substantial after cleaning, so transition
examples exist but are sparse relative to full trajectories.

| segment | rows | grasp onset (task s / source frame) | peak RH56 closure L2 | transition chunks |
|---|---:|---:|---:|---:|
| ep067 | 795 | 9.33 / 342 | 0.837 | 14 |
| ep070 | 685 | 10.50 / 329 | 0.793 | 14 |
| ep081 | 947 | 16.70 / 501 | 1.003 | 14 |
| ep087 | 789 | 11.00 / 330 | 0.727 | 14 |
| ep088 | 683 | 7.73 / 293 | 0.722 | 14 |
| ep095 | 675 | 8.23 / 247 | 0.659 | 14 |
| ep096 | 734 | 9.40 / 282 | 0.785 | 14 |
| ep097 | 840 | 8.67 / 289 | 0.880 | 14 |
| ep098 | 739 | 9.23 / 299 | 0.902 | 14 |
| ep099_a | 966 | 22.80 / 684 | 0.539 | 14 |
| ep099_b | 1,014 | 21.13 / 1968 | 0.805 | 14 |
| ep102_b | 726 | 5.73 / 1381 | 0.992 | 14 |
| ep108 | 707 | 9.50 / 285 | 0.773 | 14 |
| ep109 | 621 | 7.47 / 224 | 0.693 | 14 |
| ep116 | 557 | 6.20 / 186 | 0.800 | 14 |
| ep117 | 699 | 9.57 / 287 | 0.793 | 14 |

## Controlled checkpoint comparison on clean val4

All models use the same two cameras, 12-D state, 12-D absolute action, chunk
16, transformer 256/1024, batch 16, KL weight 1, no augmentation, seed 1000,
and 2,000 optimizer steps. The pretrained run differs only by cached,
checksum-pinned ImageNet ResNet18 initialization and backbone LR `1e-5` versus
`1e-4`. At 2k, 2,000 steps correspond to about 3.41 train-row passes, not
2,000 epochs.

| checkpoint diagnostic | transition recall | RH56 chunk peak-to-peak mean | policy RH56 all-horizon MAE | copy-state RH56 MAE |
|---|---:|---:|---:|---:|
| old mixed scratch 2k, clean val | 0/56 | 0.000173 | 0.01703 | 0.01572 |
| clean scratch 1k | 0/56 | 0.000293 | 0.03605 | 0.01572 |
| clean scratch 2k | 0/56 | 0.000740 | 0.04191 | 0.01572 |
| clean pretrained 1k | 2/56 | 0.002720 | 0.03932 | 0.01572 |
| clean pretrained 2k | 12/56 | 0.017745 | 0.04028 | 0.01572 |

Clean scratch validation loss was best near step 800 (0.2196) and worsened to
0.2586 at 2k. Clean pretrained was best near step 600 (0.2379) and ended at
0.2517. Scalar loss and transition behavior disagree: pretrained creates more
within-chunk dynamics despite no scalar-loss advantage. At pretrained 2k,
recall is `ep087=1/14`, `ep088=0/14`, `ep108=0/14`, `ep109=11/14`; recalled
timing is generally late (predicted minus recorded horizon mean +4.25, median
+4, range -1..+11). This concentration fails the offline physical gate.

The copy-current-RH56 baseline has 0 transition recall. Clean scratch also has
0 recall and nearly constant chunks, so the state-persistence shortcut remains
after curation. Pretraining weakens the literal persistence pattern but does
not yet yield robust phase-conditioned prediction.

## Answers to the audit questions

1. **Demonstration quality:** the original cohort changes from 25 purported
   clean episodes to 27 logical trajectories of which only 16 are nominal.
   This invalidates the old scientific comparison. However, clean scratch
   remains 0/56, so quality curation alone is not sufficient to explain or fix
   the rollout failure. A causal percentage cannot be identified from one
   stochastic controlled run.
2. **Malformed segmentation:** it introduced two cross-demonstration source
   records and at least 331 reset rows in the old materialized view. Correcting
   it is necessary for valid chunks, but clean scratch still fails; it is not
   the sole cause.
3. **Does clean scratch learn approach-to-grasp?** No at 1k or 2k under the
   held-out transition diagnostic (0/56 both).
4. **Is visual pretraining necessary?** Not proven necessary, but currently
   useful: it raises transition recall to 12/56. The result is too concentrated
   and error remains too high to establish a robust advantage or authorize a
   rollout.
5. **Does state persistence remain?** Yes for clean scratch. Its mean RH56
   chunk dynamic range is only 0.000740 at 2k, and it does not beat the
   copy-state baseline.
6. **Reuse excluded trajectories?** Yes, later as explicitly non-nominal data
   for failure-state detection, corrective demonstrations, or mixed-quality
   imitation. They must remain excluded from the nominal behavior-cloning
   baseline unless a separately designed experiment says otherwise.

## Reproduction and next step

```bash
PYTHONPATH=src .venv/bin/python -m episode_dataset.cli materialize-physical-bottle \
  --config configs/training/physical_bottle_v2_nominal16.yaml
PYTHONPATH=src .venv/bin/python -m episode_dataset.cli validate-physical-bottle \
  data/training/physical_bottle_v2_nominal16
scripts/train_physical_bottle_lerobot.sh clean-scratch
# only after scratch checkpoint + transition report exist:
scripts/train_physical_bottle_lerobot.sh clean-pretrained
```

Do not spend robot time on either 2k clean checkpoint. The next work should be
offline: inspect phase-balanced sampling/action-target ambiguity and test
whether a controlled temporal objective or action representation improves
transition recall across all four held-out sessions. A physical command should
be prepared only after a checkpoint shows distributed held-out transition
recall, finite/legal outputs, and command-disabled deployment validation.

## Validation record

- Focused curation/checkpoint/training-entry tests: 15 passed.
- Repository compileall and pytest collection: 724 tests collected.
- Full suite final run: 720 passed, 4 skipped, 2 pre-existing multiprocessing
  deprecation warnings. An initial full run had one timing-sensitive fake
  `jaka_zero_motion_probe` failure; the unchanged test passed in isolation and
  the complete suite then passed. No control code was changed for it.
- Local nominal16 validation: PASS, zero errors, ACT and ACT+Force each 12,177
  rows, raw hashes unchanged.
- Pinned LeRobot 0.6.2, network-disabled container: ACT and ACT+Force views both
  PASS, including representative decode and logical-boundary action chunks.
- `bash -n scripts/train_physical_bottle_lerobot.sh` and `git diff --check`:
  PASS.
