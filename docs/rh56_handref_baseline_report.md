# RH56 Hand-Ref Baseline Report

Date: 2026-05-04

## Scope

This report records the first complete MuJoCo baseline pass for the current RH56 hand-reference grasp planner.

The baseline is intentionally strict:

- Candidate generation uses the project-local RH56 collision model and existing `unifuc_pad_proxy`.
- Dataset quality is judged from exported candidate JSON, contact logs, MP4 videos, and keyframes.
- Success requires final lift above `0.05 m`, no initial penetration, no final table/self contact, valid family-specific opposing contact, and bounded XY displacement.
- The learned baseline uses an object-held-out split, not a random candidate split.

## Commands

Generate the three-object benchmark:

```bash
./scripts/run_rh56_handref_grasps.sh \
  --objects foam_block_40mm light_cylinder_36mm light_can_50mm \
  --max-candidates 40
```

Audit the generated candidates:

```bash
./scripts/audit_rh56_handref_grasps.sh \
  --benchmark-dir data/mujoco_handref_grasps \
  --out data/reports/rh56_handref_dataset_audit/report.json
```

Export visual evidence:

```bash
./scripts/export_rh56_handref_candidate_media.sh \
  --benchmark-dir data/mujoco_handref_grasps \
  --objects foam_block_40mm light_cylinder_36mm light_can_50mm \
  --ranks 0 \
  --out-dir data/replays/rh56_handref_candidates_audit \
  --fps 12 \
  --width 640 \
  --height 480
```

Train the GPU baseline:

```bash
./scripts/train_rh56_handref_ranker_baseline.sh \
  --benchmark-dir data/mujoco_handref_grasps \
  --out-dir data/baselines/rh56_handref_candidate_ranker \
  --val-objects light_can_50mm \
  --epochs 500 \
  --eval-every 25
```

## Data Audit

Strict audit output:

```text
data/reports/rh56_handref_dataset_audit/report.json
```

| Object | Candidates | Sim Success | Strict Pass | Best Strict Candidate | Best Lift |
| --- | ---: | ---: | ---: | --- | ---: |
| `foam_block_40mm` | 40 | 16 | 16 | `box_precision_pinch_p0_w1_z3_o2` | `0.100 m` |
| `light_cylinder_36mm` | 40 | 10 | 10 | `cylinder_power_envelope_p0_w0_z2_o2` | `0.112 m` |
| `light_can_50mm` | 40 | 10 | 10 | `cylinder_power_envelope_p0_w1_z1_o2` | `0.111 m` |

Total:

```text
strict_pass = 36 / 120 = 30.0%
strict_pass_objects = 3 / 3
```

Main reject causes:

- final lift below threshold.
- hand self-contact.
- invalid family contact pattern.
- excessive XY displacement / pushed-away object.
- object still touching table.

## Visual Audit

The first camera pass was rejected because the fixed camera was too close and mostly showed a dark hand silhouette. The media exporter now adds an `audit_wide_camera` per candidate scene.

Visual evidence paths:

```text
data/replays/rh56_handref_candidates_audit/<object>/<rank_candidate>/rollout.mp4
data/replays/rh56_handref_candidates_audit/<object>/<rank_candidate>/00_approach.png
data/replays/rh56_handref_candidates_audit/<object>/<rank_candidate>/01_preclose.png
data/replays/rh56_handref_candidates_audit/<object>/<rank_candidate>/02_closed.png
data/replays/rh56_handref_candidates_audit/<object>/<rank_candidate>/03_lift.png
```

Manual inspection of the `03_lift.png` keyframes confirms:

- the object is visibly off the table;
- the hand is still in contact with the object;
- no obvious table penetration is visible;
- foam block uses a precision pinch; cylinder and can use envelope/power grasps.

## Learned Baseline

The trained baseline is a small MLP candidate ranker:

```text
object features + candidate wrist/hand features + IK errors -> success probability
```

It trains on:

```text
foam_block_40mm + light_cylinder_36mm
```

and validates on held-out:

```text
light_can_50mm
```

Output:

```text
data/baselines/rh56_handref_candidate_ranker/model.pt
data/baselines/rh56_handref_candidate_ranker/metrics.json
data/baselines/rh56_handref_candidate_ranker/predictions.jsonl
```

Metrics:

| Metric | Value |
| --- | ---: |
| Device | `cuda` |
| Train candidates | 80 |
| Validation candidates | 40 |
| Train positive rate | `0.325` |
| Validation positive rate | `0.250` |
| Train AUC | `1.000` |
| Validation AUC | `0.790` |
| Validation top-1 success by object | `1.000` |
| Validation top-5 has success by object | `1.000` |

The validation top-1 candidate for held-out `light_can_50mm` is successful, with lift `0.100 m`.

## Interpretation

This is a baseline, not a final policy:

- It ranks generated candidates; it does not yet output continuous closed-loop robot actions from pixels.
- It is evaluated on one held-out object because the current generated set has three main objects.
- It is strong enough to serve as the first paper baseline for candidate selection and sim-to-real preset filtering.

Next required step before hardware claims:

1. Replay the best strict candidate for each object on the real RH56 with PC-direct feedback enabled.
2. Record actual angle/force/current traces.
3. Compare simulated expected contacts against RH56 angle residual and force/current changes.
4. Mark each replay as `real_success`, `weak_success`, or `failure`.
