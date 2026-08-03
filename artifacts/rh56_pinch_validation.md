# RH56DFX pinch retarget validation

Current physical outcome: index pinch contact confirmed by the operator; middle and tripod remain unverified. No tissue-grasp success is claimed.

## Quest hand-only revalidation stop (2026-07-31)

A 60-second Quest hand-only run was stopped fail-closed after RH56 feedback
reported canonical `ERROR=[0,0,0,0,0,4]` on `thumb_lateral`. There were 538
writes before the fault, with no serial/checksum/protocol failure. Immediately
beforehand the lateral feedback/load values were abnormal and the worker
raised `device_error_status`; the process closed the serial port and issued no
retry. A subsequent 5-second read-only probe made no writes and saw
`ERROR=[0,0,0,0,0,0]`, but `STATUS=[2,2,2,2,2,7]` persisted with thumb-lateral
ANGLE about 65. This is not a normal teleoperation state, so no further
command run is authorized until the operator inspects/resets that channel.

## Thumb-first physical probe (2026-07-31)

The latest bounded sequence used the canonical order `[index, middle, ring,
pinky, thumb_close, thumb_lateral]` and started each step from fresh measured
activation. It deliberately held index at `0.12` while thumb lateral moved to
`0.90`, then held index at `0.12` while thumb close moved to `0.40`, and only
then advanced index to `0.55` and `0.65`. All writes retained the 40 Hz
command-priority worker, latest-only mailbox, delta limit, measured feedback,
and contact-stop safety.

| stage | final measured normalized (index / thumb close / lateral) | force/current observation | result |
|---|---|---|---|
| lateral first | 0.122 / 0.218 / 0.911 | no current; force near baseline | lateral reached without index obstruction |
| thumb close | 0.122 / 0.396 / 0.914 | no current; force near baseline | thumb pre-position reached |
| index approach | 0.546 / 0.396 / 0.914 | index force about 232; no latch | no measured lateral retreat |
| contact probe | 0.551 / 0.395 / 0.917 | index force about 334; contact-stop latched at 0.555 | operator confirmed index/thumb fingertip contact |

The sequence supports the thumb-first ordering hypothesis and avoids the
earlier index-first lateral blockage. The operator confirmed the final pose as
real index/thumb fingertip contact; FORCE/CURRENT remains only a conservative
stop signal and cannot by itself distinguish contact from self-collision. The
hand was returned to measured all-open state after the probe. New Quest
operation uses the staged gate; the old simultaneous pose blend remains
disabled until a complete repeatable live-Quest capture is recorded.

## Middle lateral-endpoint probe (2026-07-31)

Because lateral `0.90` is aligned with the index opposition path, a bounded
endpoint probe used canonical target `[0, 0.75, 0, 0, 0.45, 1.0]`. The hand
reached `[0, 0.752, 0.002, 0, 0.448, 1.000]`; FORCE stayed near baseline,
contact-stop did not latch, and ERROR remained zero. This exhausts the
software/command lateral range. Middle pinch is therefore physically
unverified and may be mechanically unreachable on this hand; it is not a
remaining 0.8-ceiling problem.

The tripod probe `[0.50, 0.50, 0, 0, 0.40, 0.90]` reached
`[0.499, 0.500, 0.002, 0, 0.395, 0.909]` without contact and remains
physically unverified.

## Human intent features

All distances are divided by `distance(wrist, middle MCP)`.

| labelled Quest pose | thumb-index | thumb-middle | index-middle | detector result |
|---|---:|---:|---:|---|
| open | 1.229 | 1.451 | 0.225 | none |
| fist | 0.538 | 0.670 | 0.164 | none (power-grasp exclusion) |
| index pinch | 0.051 | 0.258 | 0.228 | index on all 72 stable-tail frames |
| middle pinch | 0.733 | 0.102 | 0.757 | middle on all 73 stable-tail frames |
| attempted tripod | 0.703 | 0.046 | 0.693 | middle, correctly not tripod |

The detector uses separate thumb-index and thumb-middle distances, both finger curls, index-middle distance for tripod, and mean ring/pinky curl to exclude a strong power grasp. Entry is 0.15 palm lengths and exit is 0.22; the separate thresholds provide hysteresis. Tracking invalid/nonfinite input resets immediately to `none`. A true tripod additionally requires both thumb distances below the entry/exit threshold and index-middle below 0.22/0.30.

The first tripod capture was a valid but middle-only human pose. A retry delivered a completely frozen old skeleton (`ptp=0` for every checked landmark-derived feature) and was rejected. Therefore tripod intent remains physically uncaptured even though the synthetic three-distance classifier path is offline tested.

## Bounded RH56 index-pinch search

No historical raw six-channel array was sent. The earlier A--D search used
only canonical index, thumb close, and thumb lateral; the newer staged probe
used the same channels with the normalized 0..1 production range. Every step
used the production 40 Hz worker, measured activation write, latest-only
mailbox, delta limit, and unchanged RH56 feedback/fault gates.

| label | requested index / thumb close / lateral | measured index / thumb close / lateral | final index force | final current | software/hardware fault |
|---|---|---|---:|---:|---|
| A | 0.35 / 0.35 / 0.35 | 0.345 / 0.343 / 0.361 | -16 | 0 | none |
| B | 0.45 / 0.40 / 0.45 | 0.449 / 0.395 / 0.459 | -19 | 0 | none |
| C | 0.50 / 0.45 / 0.55 | 0.499 / 0.445 / 0.562 | -15 | 0 | none |
| D | 0.55 / 0.50 / 0.65 | 0.548 / 0.494 / 0.659 | -15 | 0 | none |
| release | 0 / 0 / 0 | 0 / 0.007 / 0.019 | -2 | 0 | none |

None of the earlier A--D search poses produced a measurable contact load.
The later thumb-first probe is the first index pose confirmed by the operator
as fingertip contact; after it the hand was returned to measured all-open state.

The earlier independent-ceiling test at index about 0.52, thumb close about 0.74, and thumb lateral about 0.81 produced predictable index/thumb self-collision and high force. This demonstrates why pinch cannot be created by independently maximizing closure/opposition and why no further automatic sweep was performed.

## Pose blending status

The detector and event diagnostics are implemented. Index contact is now
confirmed, but the default remains staged thumb-first retargeting rather than
an unsequenced six-channel pose blend; middle and tripod still lack physical
contact validation.

Any future pose blend must occur after continuous relative retargeting and
remain behind confidence hysteresis, maximum blend-weight rate, the existing
channel delta/slew limits, full normalized 0..1 bounds, and the contact-stop
gate. Tracking loss must exit the assist and follow the existing clutch
hold/reacquisition contract.

## Quest live pinch oscillation diagnosis (2026-07-31)

During the post-calibration Quest run, the operator held a stable index pinch
while the hand repeated a small index/thumb-close motion and a thumb-lateral
retraction. The physical run was stopped at 144.4 s by operator request after
1539 RH56 writes; `ERROR`, serial, checksum, protocol, and worker-fault counts
remained zero. The telemetry contained 39 transitions between two requested
targets: the continuous base target held approximately
`[index=.12, thumb_close=.22, lateral=.90]`, while the index-pinch target
requested approximately `[index=.47, thumb_close=.38, lateral=.65]`.

The cause was the thumb-first sequencer, not serial timing or contact-force
feedback. After measured lateral reached the `.90-.04` gate, the
`index_approach` stage allowed continuous retargeting to lower lateral to
about `.65`; measured lateral then fell below the gate and the sequencer
returned to `thumb_preposition`, repeating the cycle. The revised policy does
not hold lateral throughout index approach. It passes targets through unless
they are already near the verified contact pose (index `.55`, thumb-close
`.40`, lateral `.90`) and measured lateral is still below the gate. Only that
specific blocked condition applies the temporary index/thumb-close guards and
requests lateral `.90`; once lateral is ready, the target is passed through
unchanged. The gate resets on pinch exit, tracking loss, or any target that
leaves the verified-pose neighborhood. Offline retarget tests pass with
regressions for early approach pass-through, blocked verified pose, release,
and obstruction re-entry. No new physical result has been claimed after this
fix.

## Physical grip re-alignment policy (2026-07-31)

The physical hand-only entry now enables `align_on_grip`. Each Grip press
still queues measured `ANGLE_ACT` as the activation target, preserving startup
continuity. It then uses the Quest pose captured on that same press as the
new absolute target reference, so an operator who is already making an index
pinch can drive RH56 toward the calibrated pinch pose instead of inheriting an
open or stale hand target. The RH56 worker remains authoritative for the
40-Hz delta limit, contact-stop, feedback-stale, STATUS, and ERROR gates.
Simulation retains relative-only behavior unless explicitly opted in. The
alignment path is covered offline; no physical pinch PASS is claimed here.

## Index-pinch triplet refinement (2026-07-31)

The validated index pose is no longer applied as a six-channel preset. During
an index-pinch detection, only the canonical triplet `(index, thumb_close,
thumb_lateral)` is assisted toward `(0.55, 0.40, 0.90)`; middle, ring, and
pinky remain the continuous Quest-mapped values. The physical run confirmed
this separation: the triplet target was `[.55, .40, .90]` while the other
three channels continued at nonzero mapped values rather than being reset to
zero. RH56 contact-stop remains independent: a force onset followed by a
stalled measured angle requires two samples before latching a hold target.
The run had no serial or RH56 fault; physical fingertip contact and thick
object retention still require operator confirmation.

The follow-up removed the old thumb-first preposition. When the validated
triplet is requested but measured lateral opposition is still catching up, the
session no longer rewrites index/thumb-close to `.12/.22`; it passes the
current measured-to-triplet transition directly. This keeps the index at or
below the confirmed `.55` contact value and lets lateral opposition proceed
without the visible retreat. The sequencer remains only a diagnostic state
label; it does not alter any channel.

## v2 combined physical observation (2026-08-03)

The combined entry loaded `quest_rh56dfx_real_20260803_v2` and ran for 234.32
seconds before the Quest input-recovery timeout hard stop. Native JAKA metrics
reported 10,035 accepted targets, zero rejected targets, zero controller
alarm/collision/E-stop events, and zero cleanup error. The RH56 contact gate
reported 11 detections at peak and preserved loaded activation five times; the
final latched state was released during cleanup. RH56 `ERROR` stayed all zero
with no worker, serial, checksum, or protocol fault. The operator had
previously reported an uncomplicated water-bottle grasp; this run did not
complete or prove the parcel-box or tissue grasp sequence, so no grasp PASS is
claimed.
