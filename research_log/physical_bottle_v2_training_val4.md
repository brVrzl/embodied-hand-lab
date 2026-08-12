# Physical bottle v2 episode-level validation training

Date: 2026-08-12

## Result

The repository-owned entrypoint completed both offline training jobs without
hardware access:

```bash
scripts/train_physical_bottle_lerobot.sh val4
```

This is the first v2 run with a real validation split. The earlier v2 run used
all 25 logical episodes for training and therefore did not provide an
independent validation loss.

| job | train episodes/rows | validation episodes/rows | steps | final train loss | final eval loss | checkpoint |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| ACT | 21 / 18,064 | 4 / 2,680 | 2,000 | 0.144 | 0.1527 | `outputs/training/physical_bottle_v2/act_val4_run/checkpoints/002000/pretrained_model` |
| ACT+Force | 21 / 18,064 | 4 / 2,680 | 2,000 | 0.138 | 0.1513 | `outputs/training/physical_bottle_v2/act_force_val4_run/checkpoints/002000/pretrained_model` |

The final losses are optimization diagnostics, not physical task-success
metrics. No rollout was run in this round.

## Split

The held-out validation source episodes were selected by the operator as the
relatively good demonstrations:

```text
validation: 89, 98, 114, 116
```

The 21 training logical segments are:

```text
71, 74, 81, 83, 87, 88, 90, 91, 92, 95, 96, 97,
99_seg0, 99_seg1, 102_seg0, 102_seg1, 105, 108, 109, 113, 117
```

The materialized views contain all 25 episodes in deterministic order, with
training episodes first and validation episodes last. LeRobot 0.6.2 is then
configured with `eval_split: 0.16`; it reports `21 train, 4 eval`. This keeps
the split episode-level rather than randomly splitting frames.

Both model views contain the same 20,744 canonical rows and the same images,
state, action, and episode ordering. ACT+Force adds the separate raw
`observation.environment_state` force input; it does not change the 12-D
state or 12-D absolute/native action.

## Validation losses

| step | ACT eval loss | ACT+Force eval loss |
| ---: | ---: | ---: |
| 200 | 0.2220 | 0.2194 |
| 400 | 0.1693 | 0.1772 |
| 600 | 0.1598 | 0.1592 |
| 800 | 0.1821 | 0.1719 |
| 1,000 | 0.1579 | 0.1604 |
| 1,200 | 0.1560 | 0.1656 |
| 1,400 | 0.1518 | 0.1527 |
| 1,600 | 0.1469 | 0.1499 |
| 1,800 | 0.1523 | 0.1612 |
| 2,000 | 0.1527 | 0.1513 |

The curves are broadly comparable; this small four-episode validation set is
useful for detecting train-only configuration errors, but is not a reliable
model-selection benchmark.

## Unequal demonstration duration

The demonstrations range from roughly 18.5 s to 49.5 s. This changes the
number of training rows contributed by each episode, so longer demonstrations
have more frame samples under the current LeRobot sampler. It does not mix
episodes or create invalid action chunks: the loader resets chunk construction
at every episode boundary and repeat-last pads only the final short horizon.
For a larger benchmark, episode weighting or fixed segment sampling can be
considered separately; it was not introduced into this baseline.

## Evidence

Both derived views passed the repository loader validation:

```text
ACT:       PASS, 20,744 rows, 25 episodes, state [12], action [16, 12]
ACT+Force: PASS, 20,744 rows, 25 episodes, state [12], force [6], action [16, 12]
```

The validation reports also passed action-boundary checks. The generated
`meta/stats.json` files report count 18,064 for state/action/force statistics,
confirming that normalization statistics use training rows only. The run logs
are `outputs/training/physical_bottle_v2/logs/act_val4_training.log` and
`.../act_force_val4_training.log`.

The raw episodes and v2 master data were not modified.
