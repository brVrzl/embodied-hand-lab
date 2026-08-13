# Physical bottle nominal33 expansion audit

Date: 2026-08-13

Commit audited: `be46b798bbef1403644fa39b033d6727f79ddaa1` plus the lightweight files in this change

Validation level: offline dataset/video audit only; no hardware was accessed

## Decision

The versioned `physical_bottle_v3_nominal33` view adds 17 reviewed full-task
trajectories to the existing immutable nominal16 view. It contains **33 clean
logical trajectories, 22,653 rows, and 754.10 s** of task motion. ACT and
ACT+Force expose exactly the same rows, source frames, images, and absolute
12-D action targets; ACT+Force alone consumes the retained six raw FORCE_ACT
channels and validity/age provenance.

The new cohort decision is:

- include: 121, 131, 133, 134, 137--149;
- exclude by operator audit: 122--129 and 132;
- exclude as incomplete payload: 130 (partial metadata, no complete rows/videos);
- retain as review-required: 135 and 136. A person is visible during the
  actual task interval, so removing only an idle/reset tail cannot clean these
  trajectories without deleting demonstrated behavior.

This is deliberately conservative. Episodes 135/136 remain immutable and can
later support a robustness/nuisance study, but they are not used to train the
clean nominal-success baseline.

## Boundary and quality audit

Workspace video and synchronized action/state rows were inspected together.
Every included new crop retains complete approach, grasp, lift/transport,
placement, release, and final task motion. The boundary ends before stationary
rotation debounce or manual scene recovery. Significant pre-task holds were
removed without deleting approach.

| source | crop frames | rows | duration s | rows removed | force valid |
|---:|---:|---:|---:|---:|---:|
| 121 | 60--672 | 613 | 20.40 | 210 | 100% |
| 131 | 0--516 | 517 | 17.20 | 152 | 100% |
| 133 | 44--733 | 690 | 22.97 | 217 | 100% |
| 134 | 28--769 | 742 | 24.70 | 191 | 100% |
| 137 | 0--605 | 606 | 20.17 | 179 | 100% |
| 138 | 0--605 | 606 | 20.17 | 150 | 100% |
| 139 | 0--747 | 748 | 24.90 | 150 | 100% |
| 140 | 0--684 | 685 | 22.80 | 150 | 100% |
| 141 | 0--555 | 556 | 18.50 | 154 | 100% |
| 142 | 0--587 | 588 | 19.57 | 150 | 100% |
| 143 | 0--567 | 568 | 18.90 | 150 | 100% |
| 144 | 0--761 | 762 | 25.37 | 250 | 100% |
| 145 | 0--780 | 781 | 26.00 | 213 | 100% |
| 146 | 0--524 | 525 | 17.47 | 179 | 100% |
| 147 | 0--452 | 453 | 15.10 | 176 | 100% |
| 148 | 0--575 | 576 | 19.17 | 191 | 100% |
| 149 | 1--460 | 460 | 15.30 | 270 | 100% |

The 17 source recordings contain 13,608 raw rows; 10,476 rows remain after
task trimming. Thus 3,132 rows (23.0%, 104.4 s at 30 Hz) were excluded. In the
removed intervals, at most 0.67% of action transitions per episode exceeded
`1e-4`, with zero at the 95th percentile in every episode. This supports the
video finding that the removed portions are overwhelmingly stationary setup,
debounce, or reset tails rather than task action.

All 10,476 added rows passed finite shape checks for 12-D state, 12-D action,
and 6-D force. Canonical timestamps are strictly increasing; JAKA, RH56 angle,
workspace, and wrist selections are valid and causal; no stored source
timestamp exceeds its canonical timestamp. Both camera videos decode with a
frame count matching the selected rows.

## Dataset and split

The old nominal16 remains unchanged for reproducibility. Nominal33 uses
acquisition-session grouping, with no source/session leakage:

- train: 24 trajectories, 16,670 rows;
- validation: 9 trajectories, 5,983 rows;
- test: empty for this pilot expansion.

Historical validation sessions 87/88 and 108/109 remain fixed for comparison.
The complete new acquisition session 138--142 is held out; it was selected by
the declared chronological session rule, not task difficulty or visual result.

## Validation evidence

- materialization: 33 ACT and 33 ACT+Force logical episodes, 22,653 matched rows;
- deep validation: PASS, zero errors, raw source hashes unchanged;
- local ACT adapter: PASS, `[3,480,640]` workspace/wrist, state `[12]`, action
  chunk `[16,12]`, first/middle/last and batch size two checked;
- local ACT+Force adapter: PASS on identical rows, with force `[6]`;
- focused manifest/split tests: 5 passed;
- host LeRobot import: unavailable in `.venv`; the repository's pinned Docker
  LeRobot integration was not needed to establish the local derived-view
  contract and was not run in this audit.

Authoritative metadata is
`configs/training/physical_bottle_v3_nominal33.yaml`. Generated output is
`data/training/physical_bottle_v3_nominal33/` and remains ignored by Git.

## Collection recommendation

Thirty-three clean demonstrations are enough to run the next controlled
offline ACT/ACT+Force comparison and grasp-transition diagnostics; collecting
more before examining those results is not the highest-value next step. It is
still a small behavior-cloning dataset, so this is not a claim that 33 is
sufficient for robust physical generalization. If the clean model still fails
offline phase-transition tests, collect targeted demonstrations that broaden
bottle pose and robot start-pose coverage while preserving full-task quality,
rather than simply adding more near-duplicate trials. A practical next target
would be roughly 50--80 clean full-task demonstrations, added in balanced
acquisition sessions, after the current 33-demo offline comparison identifies
which initial-state or transition regions are underrepresented.
