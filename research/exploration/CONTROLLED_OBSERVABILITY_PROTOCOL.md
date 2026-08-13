# Controlled RH56 actuator-load observability sidecar

Status: **prepared only; not executed or physically validated**

Scope: RH56 native `FORCE_ACT` observability; no policy rollout

Hardware authority: none in this session

This protocol is a decision gate, not a request to collect a large dataset. A
future physical run requires separate, explicit operator authorization and the
repository's current hardware/safety procedure. It must not connect to or move
the JAKA automatically. The canonical teleoperation/data-collection stack is
not changed by this protocol.

## Measurement claim under test

`FORCE_ACT` is six signed raw RH56 actuator/push-rod load-register counts in
canonical order:

```text
index, middle, ring, pinky, thumb_close, thumb_lateral
```

The primary hypothesis is deliberately narrow:

> After conditioning on measured RH56 actuator position and motion direction,
> some controlled physical interaction conditions produce repeatable residual
> multi-channel load trajectories across independent setup blocks.

Positive evidence would establish condition observability in this setup. It
would not establish fingertip force, calibrated contact force, a tactile map,
or arbitrary contact localization.

## Sequential design

Stop after each gate unless its criterion is met. A setup block means a fresh
object/hand reset and baseline capture; blocks should be separated enough to
expose drift rather than treating adjacent cycles as independent trials.

### Gate 0 — timing and no-contact repeatability

Run three independent setup blocks. In each block record two repetitions of the
same slow open/close/open RH56 trajectory with no object or external contact
(six repetitions total).

- Expected comparison: within-block versus across-block raw load, per-update
  delta, initial baseline, q-conditioned residual, and closing/opening path.
- Positive evidence: timestamps remain causal; actual source rate and age are
  recorded; q-conditioned no-contact residual variation is materially smaller
  than the interaction effects tested at Gate 1.
- Negative evidence: source timing is invalid, baselines drift without a
  correctable block effect, or identical no-contact cycles are not repeatable.
- Decision: timing failure is **KILL** for learned history until fixed;
  uncontrolled drift is **HOLD** pending a better baseline protocol.

### Gate 1 — matched-trajectory interaction screen

Within each of the same three blocks, interleave one repetition of each
condition below, then repeat the randomized sequence once. This gives six
repetitions per condition and 30 short interaction cycles total. Use the exact
same bounded hand target trajectory for every condition.

| Condition | Ground truth / setup | Expected comparison | Positive evidence | Negative evidence |
| --- | --- | --- | --- | --- |
| `FREE_MOTION` | No object/contact, verified by RGB | Reference for q-conditioned load | Low residual spread | Spread overlaps every interaction condition |
| `FINGERTIP_REGION_CONTACT` | Fixture/object visibly contacts the distal/fingertip region; label comes from setup/RGB | Residual signature versus free and other regions | Cross-block separability after q conditioning | Separation exists only in raw q or one block |
| `MIDDLE_DISTAL_PHALANX_CONTACT` | Contact visibly placed on middle/distal phalanx region | Same | Repeatable redistribution distinct from free | Not distinguishable from free or fingertip-region condition |
| `SIDE_CONTACT` | Contact visibly placed on lateral/side surface | Same | Repeatable residual pattern | No cross-block repeatability |
| `BLOCKED_CLOSURE` | Padded fixture visibly prevents commanded closure | Residual magnitude, derivative, q/target/load inconsistency | Detectable before/while q progress stalls | Equivalent signal obtainable from q/target error alone |

The region names are experiment conditions supplied by the fixture/video. A
classifier distinguishing them may be called a *controlled-condition
classifier*, not a learned fingertip contact localizer.

Analyze with leave-one-setup-block-out evaluation. Start with linear/logistic
models and random forest. Report AUROC (one-vs-rest where applicable), balanced
accuracy, precision, recall, confusion matrix, and a label-permutation null.
Always compare `q`, load, `q + load`, short load history, and `q + history`.

Gate 1 passes only if load or q-conditioned residual load improves over q on a
held-out block and the effect is not carried by baseline identity alone. If
interaction versus free motion cannot be recovered reliably, **KILL** the
interaction-state supervisor and contact-condition branches.

### Gate 2 — stable grasp and bounded disturbance

Run only after Gate 1 passes. Use a bottle safety catch and an operator-approved
bounded disturbance procedure; do not target or require a drop.

| Condition | Minimum repeats | Metadata | Evidence sought | Kill evidence |
| --- | ---: | --- | --- | --- |
| `STABLE_BOTTLE_HOLD` | 3 blocks × 3 | bottle identity/fill mass, pose, hold duration, visible motion | A repeatable post-closure residual state beyond q | Stable holds vary as much as disturbed holds |
| `CONTROLLED_PULL_TENDENCY` | 3 blocks × 3 paired with stable holds | disturbance onset/end from independent instrument or synchronized operator event; direction; bounded magnitude if measured | A causal load-history change before independently visible motion | Signal begins only after visual motion, has unacceptable false positives, or q explains it |

Do not use the word *slip* unless synchronized RGB provides a locked, directly
visible slip-onset interval under the annotation protocol. Report causal lead
time relative to that interval and false alarms per minute. The internal screen
for pursuing a learned stable/unstable classifier is roughly AUROC >= 0.8 plus
useful lead time and acceptable false-positive behavior; it is not a benchmark
or paper claim.

### Gate 3 — load and friction transfer

Run only if Gate 2 yields a stable-grasp signal.

- Object load: two independently measured fill masses, three blocks × two
  repetitions per mass.
- Optional friction: one controlled surface change, paired within block, only
  if it can be specified and reproduced.
- Positive evidence: q-conditioned signature changes repeatably with the
  controlled factor and a model trained on one block transfers to another.
- Negative evidence: apparent differences follow baseline/session ordering or
  disappear under randomized, paired comparison.

## Required record for every cycle

- immutable trial id, setup-block id, randomized condition, repetition;
- operator-entered condition and contact-region ground truth, plus uncertainty;
- object id, geometry, measured mass where applicable, pose/fixture reference,
  and friction treatment;
- exact hand target-trajectory id and content hash;
- raw target and measured `ANGLE_ACT` in canonical order;
- raw `FORCE_ACT`, validity, request/source/response timestamps, and errors;
- canonical row timestamps and source ages without rewriting them to row time;
- synchronized RGB paths and camera timestamps;
- independent event timestamps for contact placement, hold start/end, and any
  disturbance; and
- abort/termination reason separated from scientific outcome.

Before analysis, hash every ignored raw sidecar file into a versioned identity
manifest. Never edit the capture in place. Derived tables and plots belong
under ignored `outputs/research/`; only lightweight config, code, tests, and the
decision log are committed.

## Analysis order and stopping rules

1. Verify channel order, validity, source causality, age, and actual polling.
2. Collapse zero-order-held rows to unique source updates.
3. Plot raw traces, q, targets, q error, residual load, and per-source deltas.
4. Quantify within-condition/across-block covariance, PCA, and directional
   hysteresis.
5. Fit q-only residual baselines on training blocks only.
6. Run simple grouped probes and label-permutation controls.
7. Consider a supervisor only if a semantic state passes its gate.

Stop and mark the direction **KILL** when q-only matches force-aware probes on
held-out blocks, when the effect is session identity, or when labels cannot be
independently established. Mark **HOLD** rather than inventing a semantic label
for an ambiguous physical event.
