# RH56DFX real Quest calibration and bounded travel validation

Validation state: real Quest features calibrated; RH56 six-channel bounded free-space travel is physically validated through the configured normalized 1.0 command range. The staged thumb-first index probe and the 2026-07-31 anchor runs are physically recorded. Calibration v2 received a 60.325-second Quest hand-only physical run on 2026-08-03 with no JAKA session, RH56 ERROR, worker fault, or serial/protocol fault.

## Calibration identity and source

- selected physical hand-only calibration: `configs/hand/quest_rh56_real_retarget.yaml`
- calibration id: `quest_rh56dfx_real_20260803_v2`
- old physical-path calibration: `quest_rh56_sim_uncalibrated_v1`
- labelled source captures: open, fist, thumb open, thumb neutral, thumb opposed, index pinch, middle pinch, and attempted tripod
- each accepted statistic uses the final 1 s (about 72 frames) of an independently labelled Quest-only file
- raw labelled captures remain under `logs/rh56_cal_*_20260730_*.hts.jsonl`; the reproducible summary is `artifacts/rh56_real_calibration_capture_analysis.md`
- the v2 thumb pregrasp anchor comes from repeated 2026-08-03 hand-only HTS segments in `logs/quest_rh56_thumb_anchor_20260803.hts.jsonl`

The old calibration used generic 0/1 finger endpoints and thumb lateral extrema -0.60/0.25. It therefore exposed only the measured raw human feature fraction, and the latest old-calibration physical run reached only 0.437/0.443/0.468/0.569 on the four finger commands. It was also explicitly named simulation-uncalibrated while being loaded by the physical hand-only path.

## Human feature endpoints

The finger feature is the existing PIP+DIP curl plus the 0.15 deadbanded MCP contribution. The real calibration maps each measured open/closed endpoint to 0/1 before the relative clutch delta and output ceiling are applied.

| channel | open median | closed median | measured span | offline old feature at fist | offline real-calibrated feature at fist |
|---|---:|---:|---:|---:|---:|
| index | 0.051979 | 0.920303 | 0.868324 | 0.920 | 1.000 |
| middle | 0.055232 | 0.883532 | 0.828300 | 0.884 | 1.000 |
| ring | 0.035240 | 0.876213 | 0.840973 | 0.876 | 1.000 |
| pinky | 0.089340 | 0.898434 | 0.809094 | 0.898 | 1.000 |

All spans have the correct direction and exceed 0.8. The configured curve exponents are currently 1.0 because the capture establishes endpoints but does not justify a nonlinear mid-range distortion. The schema permits four independent positive exponents and validates finite, nonzero spans.

The new full-fist feature reaches 1.0 for all four fingers offline. The
PC-direct worker and bounded hand-test entry now both permit normalized 0..1;
there is no retained 0.8 software endpoint.

The bounded full-fist anchor reached measured index/middle/ring/pinky
`0.968/0.999/1.000/1.000` with thumb-close `0.844` and lateral `0.431`.
Index contact-stop latched at approximately `0.97`; this is a physical
contact/self-collision observation, not a software ceiling.

## Thumb opposition feature

The feature uses Quest landmarks wrist (0), thumb base/metacarpal (1), thumb tip (4), index MCP (5), middle MCP (9), and pinky MCP (17).

```text
across = normalize(pinky_MCP - index_MCP)
forward = orthogonal_component(middle_MCP - wrist, across)
normal = across x forward
raw = dot(thumb_tip - thumb_base, across) / distance(index_MCP, pinky_MCP)
```

This is a right-hand wrist/palm-local orthonormal frame. Division by MCP span makes the feature dimensionless and approximately invariant to hand scale; applying the same rigid wrist-frame rotation leaves the value unchanged in offline tests. Positive direction is index MCP toward pinky MCP and matches increasing RH56 `thumb_lateral` opposition.

Measured stable-tail medians were:

- fully open: -1.137268; p05/p95 -1.1582/-1.0676
- neutral: -0.817716; p05/p95 -0.8244/-0.8055
- fully opposed: 0.060326; p05/p95 -0.1415/0.0729

The opposed sample contains more motion/noise than neutral, but its median and direction are distinct. The v1 calibration mapped open/opposed linearly to 0/1. A repeated 2026-08-03 hand-only capture measured the comfortable straight-thumb pregrasp at raw median -0.339631 and index pinch at raw median about -0.211. V2 therefore uses a continuous monotonic three-point response:

```text
-1.137268 open -> 0.0
-0.339631 comfortable straight-thumb pregrasp -> 0.9
 0.060326 fully opposed -> 1.0
```

This keeps the original biological endpoints and full RH56 range while avoiding the over-opposition that a global gain increase would create during pinch. Grip engagement still begins with a forced write of fresh measured `ANGLE_ACT`; `align_on_grip` then transitions through the normal shaper toward the current absolute Quest feature, so reacquisition does not discard the calibrated span.

## Bounded RH56 channel travel

Every target below was generated in canonical order and encoded by the production protocol-order conversion. Activation began from fresh measured `ANGLE_ACT`; the forced activation write was observed before target submission and was not duplicate-suppressed. Transitions used the unchanged 40 Hz production worker and 0.05 delta limit.

| test | requested normalized | final measured normalized | result |
|---|---|---|---|
| recent old-teleop maximum | 0.437/0.443/0.468/0.569/0.750/0.800 | 0.432/0.443/0.463/0.566/0.744/0.810 | reproduced |
| index 0.8, thumb lateral high | index 0.800, lateral 0.800 | index 0.518, lateral 0.810 | expected index/thumb self-collision |
| thumb lateral retreat | lateral 0.000 | lateral 0.018 | collision load removed |
| index 0.8, thumb lateral open | index 0.800, lateral about 0.02 | index 0.798, lateral 0.033 | reached |
| middle 0.8, thumb lateral low | middle 0.800 | middle 0.799 | reached |
| ring 0.8, thumb lateral low | ring 0.800 | ring 0.797 | reached |
| pinky 0.8, thumb lateral low | pinky 0.800 | pinky 0.800 | reached |
| thumb close 0.8 | thumb close 0.800 | thumb close 0.802 | reached |
| thumb lateral open/middle/opposed | 0.000 / 0.400 / 0.800 | 0.019 / 0.410 / 0.813 | reached continuously |

The first index-ceiling run produced index `FORCE_ACT` about 1093 peak / 1054 final and `CURRENT` peak 2244 while the measured index stopped near 0.518. The operator identified this as acceptable, predictable index/thumb self-collision with no external object. Moving thumb lateral to minimum reduced index `FORCE_ACT` to about -1; repeating the same index target then reached 0.798 with low final force. No hardware force/current/speed register was modified, and no new software threshold was retained from this observation.

All bounded runs completed with ERROR `[0,0,0,0,0,0]`, normal observed STATUS `[2,2,2,2,2,2]`, no serial timeout/checksum/protocol fault, and no worker failure. The measured 0.8 endpoint was sufficient for free-space single-channel closure; the bounded full-fist anchor also reached the normalized 1.0 four-finger range.

## Raw 0 and current limitation

The normalized 1.0 bounded probes generated the protocol raw-0 endpoint for
the affected channels; no separate blind raw-array test was performed. The
persistent software configuration now allows 0..1 as explicitly requested and
does not modify hardware registers. Full tissue behavior remains unverified.

The first 2026-08-03 60-second capture received 4258 right-hand frames but the Touch grip remained released, so it deliberately performed zero RH56 writes and generated zero arm targets. The landmarks nevertheless supplied repeated open, comfortable pregrasp, and index-pinch labels for v2. The subsequent 60.325-second v2 hand-only run loaded the new calibration, performed 470 successful writes across four grip activations, and generated zero arm targets. Comfortable straight-thumb side-sweep samples mapped to feature median 0.892 and p90 0.919; requested/submitted lateral reached 0.933 and measured `ANGLE_ACT` reached 0.945. All RH56 ERROR values remained zero, with no worker, serial, checksum, or protocol fault.

## Mechanical and data limitations

- High thumb opposition can self-collide with a closing index; pinch blending must use physically validated contact poses rather than independently maximizing both actuators.
- Index and middle human pinch intent are separable in the labelled data: normalized contact distances were about 0.051 and 0.102 respectively.
- The first tripod file was middle-only (`thumb-index` about 0.703), and the retry was a completely frozen stale skeleton. Neither is accepted as tripod calibration evidence.
- The index/thumb fingertip contact pose is physically validated, and the operator reported a later water-bottle grasp without observed difficulty. Tissue hold/extraction/release and loaded parcel-box behavior remain incomplete; the 148.9-second combined run stopped during parcel-box grasp on the now-fixed measured-activation scheduling race.
- A lateral endpoint of normalized `1.0` was reached during the middle-pinch
  probe without fingertip contact; the current hand therefore has no remaining
  software lateral travel to expose for middle pinch.

## 2026-07-31 fault-reset/calibration attempt

After the Quest hand-only revalidation stopped on canonical `thumb_lateral`
with `ERROR=4`, a single official `CLEAR_ERROR` write (`1004=1`) was issued
under the dedicated fault-reset gate. The write count was exactly one and the
serial transport closed normally. During five seconds of verification,
`ERROR` remained all zero and `CURRENT` remained zero, but canonical `STATUS`
persisted as `[2,2,2,2,2,7]`; `ANGLE_ACT` stayed around
`[788,835,880,926,683,65]`. The official no-load force-sensor calibration was
therefore not started during that attempt, and no Quest teleoperation
immediately followed. This dated fault state was later cleared: a subsequent
no-load official force calibration completed with ERROR all zero, and the
2026-08-03 v2 hand-only run again ended with no RH56 hardware or transport
fault.
