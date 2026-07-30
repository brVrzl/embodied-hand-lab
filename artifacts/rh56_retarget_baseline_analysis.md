# RH56 retarget baseline analysis

Validation level: offline analysis of previously recorded physical Quest/RH56 logs. This report does not create a new physical PASS.

## Inputs and conventions

- HTS: `logs/quest_jaka_rh56_combined_20260730_200422_3802773.hts.jsonl` (14150 valid right-landmark frames)
- retarget events: `logs/quest_jaka_rh56_combined_20260730_200422_3802773.events.jsonl` (10261 control ticks)
- RH56 telemetry: `logs/quest_jaka_rh56_combined_20260730_200422_3802773.rh56.jsonl` (4666 rows)
- loaded calibration identifier: `quest_rh56_sim_uncalibrated_v1`
- physical software closure ceiling: `0.8` (raw 200 with the current 1000-open/0-close encoding)
- canonical normalized order: index, middle, ring, pinky, thumb_close, thumb_lateral; 0=open and 1=close/opposed.
- The historical event fields ending in `_rad` carry normalized values on the physical output path. RH56 telemetry is used as the authoritative submitted/measured unit source.

## Quest feature coverage

`raw curl` is PIP+DIP bend in radians. MCP is the unclipped angle-to-palm-forward feature normalized by pi/2. Combined curl is the production distal feature plus the configured 0.15 deadbanded MCP contribution.

| finger / feature | min | p01 | p05 | p50 | p95 | p99 | max |
|---|---:|---:|---:|---:|---:|---:|---:|
| index / raw_curl_rad | 0.1135 | 0.1637 | 0.2702 | 0.8265 | 1.4417 | 1.5319 | 1.8437 |
| index / mcp_flexion | 0.0064 | 0.0153 | 0.0734 | 0.2418 | 0.4878 | 0.5954 | 0.6493 |
| index / pip_flexion_rad | 0.0421 | 0.0790 | 0.1975 | 0.6192 | 1.0070 | 1.0972 | 1.1916 |
| index / dip_flexion_rad | 0.0587 | 0.0633 | 0.0678 | 0.2071 | 0.4240 | 0.4999 | 0.6522 |
| index / combined_curl | 0.0374 | 0.0521 | 0.0860 | 0.2818 | 0.4900 | 0.5160 | 0.6101 |
| middle / raw_curl_rad | 0.1011 | 0.1577 | 0.2216 | 0.8455 | 1.4856 | 1.7481 | 2.2170 |
| middle / mcp_flexion | 0.0010 | 0.0581 | 0.1482 | 0.3131 | 0.5332 | 0.6495 | 0.6708 |
| middle / pip_flexion_rad | 0.0185 | 0.0480 | 0.1801 | 0.6320 | 1.0201 | 1.1546 | 1.3414 |
| middle / dip_flexion_rad | 0.0073 | 0.0139 | 0.0271 | 0.1953 | 0.4964 | 0.5930 | 0.8887 |
| middle / combined_curl | 0.0322 | 0.0503 | 0.0758 | 0.2966 | 0.4928 | 0.5766 | 0.7241 |
| ring / raw_curl_rad | 0.1150 | 0.1560 | 0.2877 | 0.8593 | 1.4490 | 1.7480 | 2.3708 |
| ring / mcp_flexion | 0.0706 | 0.0905 | 0.1245 | 0.2913 | 0.4821 | 0.6505 | 0.7319 |
| ring / pip_flexion_rad | 0.0724 | 0.0911 | 0.2310 | 0.6072 | 0.9579 | 1.1267 | 1.4447 |
| ring / dip_flexion_rad | 0.0420 | 0.0484 | 0.0604 | 0.2565 | 0.5078 | 0.6271 | 0.9359 |
| ring / combined_curl | 0.0385 | 0.0498 | 0.0949 | 0.3099 | 0.4751 | 0.5676 | 0.7642 |
| pinky / raw_curl_rad | 0.1891 | 0.2362 | 0.3956 | 0.8118 | 1.3476 | 1.7816 | 2.4137 |
| pinky / mcp_flexion | 0.1047 | 0.1386 | 0.1840 | 0.2673 | 0.4388 | 0.5619 | 0.6253 |
| pinky / pip_flexion_rad | 0.0836 | 0.0903 | 0.2492 | 0.5934 | 0.8828 | 1.1017 | 1.3620 |
| pinky / dip_flexion_rad | 0.0985 | 0.1067 | 0.1114 | 0.2241 | 0.4876 | 0.6865 | 1.0605 |
| pinky / combined_curl | 0.1007 | 0.1235 | 0.1503 | 0.2781 | 0.4381 | 0.5759 | 0.7734 |
| thumb_close / raw_curl_rad | 0.2758 | 0.3737 | 0.4100 | 0.5191 | 0.7611 | 1.4780 | 1.6242 |
| thumb_close / mcp_flexion | 0.4037 | 0.4193 | 0.4388 | 0.5101 | 0.6148 | 0.6573 | 0.6768 |
| thumb_close / pip_flexion_rad | 0.1368 | 0.1770 | 0.2263 | 0.3576 | 0.5644 | 1.1148 | 1.1194 |
| thumb_close / dip_flexion_rad | 0.0862 | 0.0915 | 0.0959 | 0.1642 | 0.3027 | 0.3919 | 0.5077 |
| thumb_close / combined_curl | 0.1388 | 0.1729 | 0.1838 | 0.2222 | 0.2836 | 0.4987 | 0.5439 |

## Thumb opposition input and mapping

The old feature uses Quest wrist-local landmarks 1 (thumb metacarpal/base), 4 (thumb tip), 5 (index MCP), 9 (middle MCP), and 17 (pinky MCP). `across = normalize(pinky_MCP - index_MCP)`; palm forward is the component of `middle_MCP - wrist` orthogonal to across; palm normal is `across × forward`. Raw opposition is `dot(thumb_tip - thumb_base, across) / distance(index_MCP, pinky_MCP)`. Thus it is wrist/palm-local and scale-normalized, not a world-frame coordinate. The configured old open/opposed extrema are -0.60/0.25.

The palm center requested for diagnosis is reported as the mean of wrist plus the four MCP landmarks, but the old production feature does not use that center. Relative grip capture stores the current feature and current measured RH56 target; subsequent mapping uses feature delta, gain 1.0, offset equal to the measured reference, dead zone 0.015, and clips to [0, 0.8]. Releasing grip clears the reference; a release-before-press cycle captures a new one. This avoids an activation jump but can discard available absolute travel and can recapture at a saturated/high thumb-lateral pose.

| thumb quantity | min | p01 | p05 | p50 | p95 | p99 | max |
|---|---:|---:|---:|---:|---:|---:|---:|
| raw across-palm (HTS) | -1.3117 | -1.2115 | -1.1432 | -0.5622 | -0.0793 | 0.1889 | 0.2440 |
| production raw across-palm (events) | -1.3117 | -1.2065 | -1.1532 | -0.8188 | -0.1198 | -0.0677 | 0.0563 |
| normalized feature (events) | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.5649 | 0.6262 | 0.7722 |
| captured reference | 0.0000 | 0.0000 | 0.0000 | 0.2963 | 0.2963 | 0.2963 | 0.2963 |
| feature delta after dead zone | -0.2963 | -0.2963 | -0.2138 | 0.1201 | 0.4913 | 0.6723 | 0.6851 |
| requested normalized | 0.4710 | 0.4710 | 0.5370 | 0.8000 | 0.8000 | 0.8000 | 0.8000 |
| clipped normalized | 0.4710 | 0.4710 | 0.5370 | 0.8000 | 0.8000 | 0.8000 | 0.8000 |
| submitted normalized | 0.4710 | 0.4710 | 0.7098 | 0.8000 | 0.8000 | 0.8000 | 0.8000 |
| measured ANGLE_ACT normalized | 0.4890 | 0.4930 | 0.7110 | 0.8110 | 0.8190 | 0.8200 | 0.8220 |

## Complete channel mapping and coverage

| channel | calibrated feature min..max | relative reference min..max | delta min..max | gain | requested min..max | clipped min..max | submitted min..max | protocol raw min..max | ANGLE_ACT norm min..max | lag ms* |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| index | 0.0374..0.6100 | 0.0640..0.3762 | -0.2935..0.3092 | 1.0 | -0.1229..0.4372 | 0.0000..0.4372 | 0.0000..0.4372 | 563..1000 | -0.0000..0.4310 | 70 |
| middle | 0.0322..0.7241 | 0.0627..0.4361 | -0.3706..0.2881 | 1.0 | -0.2091..0.4441 | 0.0000..0.4441 | 0.0000..0.4433 | 557..1000 | -0.0000..0.4390 | 65 |
| ring | 0.0399..0.7642 | 0.0915..0.4498 | -0.3776..0.3144 | 1.0 | -0.2236..0.4684 | 0.0000..0.4684 | 0.0000..0.4682 | 532..1000 | -0.0000..0.4610 | 65 |
| pinky | 0.1040..0.7734 | 0.1450..0.4528 | -0.2878..0.3903 | 1.0 | -0.0690..0.5693 | 0.0000..0.5693 | 0.0000..0.5693 | 431..1000 | -0.0000..0.5670 | 70 |
| thumb_close | 0.1053..0.5897 | 0.1456..0.2974 | -0.1921..0.3621 | 1.0 | 0.0896..0.7503 | 0.0896..0.7503 | 0.0909..0.7503 | 250..909 | 0.0970..0.7440 | 90 |
| thumb_lateral | 0.0000..0.7722 | 0.0000..0.2963 | -0.2963..0.6851 | 1.0 | 0.4710..0.8000 | 0.4710..0.8000 | 0.4710..0.8000 | 200..529 | 0.4890..0.8220 | 75 |

\* Lag is the 0--400 ms delay (5 ms grid) minimizing mean squared error between ANGLE samples and the interpolated submitted command. It is an observational command-to-feedback estimate, not a causal transport-only measurement.

## Event statistics

| channel | low saturation | high saturation | dead-zone occupancy | submitted unchanged occupancy |
|---|---:|---:|---:|---:|
| index | 114/2234 (0.051) | 0/2234 (0.000) | 152/2234 (0.068) | 240/1634 (0.147) |
| middle | 699/2234 (0.313) | 0/2234 (0.000) | 313/2234 (0.140) | 755/1634 (0.462) |
| ring | 606/2234 (0.271) | 0/2234 (0.000) | 256/2234 (0.115) | 695/1634 (0.425) |
| pinky | 566/2234 (0.253) | 0/2234 (0.000) | 163/2234 (0.073) | 560/1634 (0.343) |
| thumb_close | 0/2234 (0.000) | 0/2234 (0.000) | 208/2234 (0.093) | 197/1634 (0.121) |
| thumb_lateral | 0/2234 (0.000) | 1198/2234 (0.536) | 503/2234 (0.225) | 1391/1634 (0.851) |

- exact duplicate suppression: 124 rows
- all-channel unchanged command occupancy: 89/1634
- relative reference captures: 6
- grip reacquisition entries: 6
- tracking losses/recoveries: 2/3
- STATUS nonzero rows: 4584 (the observed normal status is 2; fault interpretation is owned by the runtime gate)
- ERROR fault rows: 0
- CURRENT absolute peaks by channel: index=344, middle=374, ring=383, pinky=377, thumb_close=355, thumb_lateral=304
- FORCE_ACT absolute peaks by channel: index=21, middle=16, ring=24, pinky=42, thumb_close=16, thumb_lateral=58

## Time distribution

Each cell is maximum submitted command / maximum measured ANGLE_ACT in that time bin.

| elapsed s | index | middle | ring | pinky | thumb_close | thumb_lateral |
|---|---:|---:|---:|---:|---:|---:|
| 0--10 | 0.428/0.426 | 0.443/0.439 | 0.468/0.461 | 0.569/0.567 | 0.484/0.482 | 0.800/0.822 |
| 10--20 | 0.366/0.362 | 0.251/0.247 | 0.173/0.171 | 0.079/0.075 | 0.500/0.497 | 0.800/0.814 |
| 20--30 | 0.317/0.328 | 0.238/0.235 | 0.239/0.236 | 0.270/0.267 | 0.493/0.492 | 0.800/0.821 |
| 30--40 | 0.437/0.426 | 0.326/0.323 | 0.272/0.274 | 0.245/0.245 | 0.750/0.744 | 0.800/0.821 |
| 40--50 | 0.428/0.431 | 0.318/0.323 | 0.273/0.267 | 0.286/0.283 | 0.722/0.742 | 0.800/0.820 |
| 50--60 | 0.152/0.152 | 0.022/0.021 | 0.074/0.073 | 0.133/0.132 | 0.330/0.326 | 0.751/0.765 |
| 60--70 | 0.139/0.136 | 0.010/0.007 | 0.017/0.014 | 0.054/0.061 | 0.330/0.326 | 0.765/0.779 |
| 70--80 | 0.063/0.061 | 0.000/0.006 | 0.015/0.014 | 0.016/0.015 | 0.317/0.318 | 0.765/0.779 |
| 80--90 | 0.063/0.061 | 0.000/0.006 | 0.015/0.014 | 0.016/0.015 | 0.317/0.318 | 0.765/0.783 |
| 90--100 | 0.063/0.061 | 0.000/0.006 | 0.015/0.014 | 0.016/0.015 | 0.317/0.318 | 0.765/0.784 |
| 100--110 | 0.308/0.301 | 0.293/0.287 | 0.267/0.266 | 0.201/0.194 | 0.318/0.318 | 0.800/0.814 |
| 110--120 | 0.304/0.301 | 0.293/0.287 | 0.266/0.266 | 0.199/0.194 | 0.292/0.292 | 0.800/0.814 |
| 120--130 | 0.301/0.301 | 0.287/0.287 | 0.266/0.266 | 0.194/0.194 | 0.312/0.308 | 0.800/0.814 |
| 130--140 | 0.281/0.286 | 0.258/0.262 | 0.201/0.201 | 0.153/0.162 | 0.311/0.308 | 0.800/0.813 |
| 140--150 | 0.281/0.286 | 0.258/0.262 | 0.201/0.201 | 0.153/0.162 | 0.311/0.308 | 0.800/0.811 |
| 150--160 | 0.281/0.286 | 0.258/0.262 | 0.201/0.201 | 0.153/0.162 | 0.311/0.308 | 0.800/0.811 |
| 160--170 | 0.281/0.286 | 0.258/0.262 | 0.201/0.201 | 0.153/0.162 | 0.311/0.308 | 0.800/0.811 |
| 170--180 | 0.281/0.286 | 0.258/0.262 | 0.201/0.201 | 0.153/0.162 | 0.311/0.308 | 0.800/0.812 |
| 180--190 | 0.281/0.286 | 0.258/0.262 | 0.201/0.201 | 0.153/0.162 | 0.311/0.308 | 0.800/0.812 |
| 190--200 | 0.281/0.286 | 0.258/0.262 | 0.201/0.201 | 0.153/0.162 | 0.311/0.308 | 0.800/0.812 |
| 200--202 | 0.281/0.286 | 0.258/0.262 | 0.201/0.201 | 0.153/0.162 | 0.311/0.308 | 0.800/0.812 |

## Major gesture segments

The capture has no operator gesture labels, so it is not valid to call arbitrary time windows fist, pinch, or grasp. The reproducible segments below are grip clutch cycles; maxima are canonical normalized clipped requests.

| clutch cycle | duration s | updated ticks | max index/middle/ring/pinky/thumb_close/thumb_lateral |
|---:|---:|---:|---|
| 1 | 21.533 | 1292 | 0.428/0.444/0.468/0.569/0.500/0.800 |
| 2 | 7.299 | 344 | 0.437/0.326/0.273/0.286/0.750/0.800 |
| 3 | 7.307 | 318 | 0.152/0.022/0.074/0.133/0.330/0.751 |
| 4 | 1.433 | 86 | 0.063/0.010/0.018/0.019/0.317/0.765 |
| 5 | 2.288 | 127 | 0.308/0.293/0.267/0.201/0.318/0.800 |
| 6 | 1.017 | 61 | 0.301/0.287/0.266/0.194/0.312/0.800 |

## Baseline findings

1. The real hand path loads a calibration explicitly named `sim_uncalibrated`; that identity and its generic extrema are unsuitable as the final hardware calibration.
2. The full chain is relative: a grip press captures both Quest features and measured ANGLE_ACT, then applies gain to feature delta. Consequently identical human poses can map differently after reacquisition, and captured high thumb-lateral state consumes remaining opposition travel.
3. Four-finger feature coverage, not the 0.8 ceiling, is the first limiting factor in the latest normal run: submitted maxima remain below the ceiling while raw 200 is still available.
4. Thumb lateral is moving and measured feedback follows it, but the old feature spends substantial time at a clipped feature endpoint and the relative offset repeatedly drives the output near/high saturation. This explains visually weak or stuck-looking motion without a serial-rate hypothesis.
5. The log contains no validated index, middle, tripod, or tissue-contact pose labels. It can quantify mapping and feedback, but cannot establish fingertip contact or tissue-grasp success.

## Reproduce

```bash
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python tools/analyze_rh56_retarget_log.py --hts logs/quest_jaka_rh56_combined_20260730_200422_3802773.hts.jsonl --events logs/quest_jaka_rh56_combined_20260730_200422_3802773.events.jsonl --telemetry logs/quest_jaka_rh56_combined_20260730_200422_3802773.rh56.jsonl --output artifacts/rh56_retarget_baseline_analysis.md --calibration-id quest_rh56_sim_uncalibrated_v1 --max-close 0.8
```
