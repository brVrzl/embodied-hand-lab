# RH56DFX real Quest calibration and bounded travel validation

Validation state: real Quest features calibrated; RH56 six-channel bounded free-space travel physically validated at the existing 0.8 ceiling. Normal Quest-driven operation with this calibration is not yet physically validated because the first post-change run received zero right-hand landmark frames.

## Calibration identity and source

- selected physical hand-only calibration: `configs/hand/quest_rh56_real_retarget.yaml`
- calibration id: `quest_rh56dfx_real_20260730_v1`
- old physical-path calibration: `quest_rh56_sim_uncalibrated_v1`
- labelled source captures: open, fist, thumb open, thumb neutral, thumb opposed, index pinch, middle pinch, and attempted tripod
- each accepted statistic uses the final 1 s (about 72 frames) of an independently labelled Quest-only file
- raw labelled captures remain under `logs/rh56_cal_*_20260730_*.hts.jsonl`; the reproducible summary is `artifacts/rh56_real_calibration_capture_analysis.md`

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

The new full-fist feature reaches 1.0 for all four fingers offline. The physical PC-direct worker still clips requested closure to 0.8, so a correctly referenced full fist can use the complete current software range without testing or enabling raw 0.

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

The opposed sample contains more motion/noise than neutral, but its median and direction are distinct. The real calibration maps open/opposed to 0/1. Relative reference capture is retained: grip engagement captures the current Quest feature and current measured RH56 state, avoiding a reacquisition jump. This also means engagement pose still matters; normal operation should engage from an open/neutral human pose with RH56 thumb lateral near its open endpoint.

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

All bounded runs completed with ERROR `[0,0,0,0,0,0]`, normal observed STATUS `[2,2,2,2,2,2]`, no serial timeout/checksum/protocol fault, and no worker failure. The 0.8 ceiling is therefore sufficient for free-space four-finger closure when thumb opposition does not create self-collision.

## Raw 0 and current limitation

Raw 0 / normalized 1.0 was not tested. These results do not authorize removing the 0.8 ceiling. They show that raw 200 is a usable software endpoint and that the earlier incomplete fist was primarily feature/reference utilization, not the mechanical endpoint.

The first 45 s post-change Quest hand-only run loaded `quest_rh56dfx_real_20260730_v1` but received zero right-hand frames. It performed zero RH56 writes, generated zero arm targets, and closed cleanly. Consequently command-range improvement under live Quest control remains physically pending even though the captured frames replay to full calibrated finger features offline.

## Mechanical and data limitations

- High thumb opposition can self-collide with a closing index; pinch blending must use physically validated contact poses rather than independently maximizing both actuators.
- Index and middle human pinch intent are separable in the labelled data: normalized contact distances were about 0.051 and 0.102 respectively.
- The first tripod file was middle-only (`thumb-index` about 0.703), and the retry was a completely frozen stale skeleton. Neither is accepted as tripod calibration evidence.
- No RH56 fingertip contact pose, tissue hold, extraction, or release has yet been validated by these endpoint tests.
