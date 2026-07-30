# RH56 real Quest calibration capture analysis

Validation level: labelled Quest-only physical hand-tracking capture; no RH56 or JAKA command path was opened.

Each statistic is median [p05, p95] over the final 1 s of its independently labelled file.

| pose | stable / total frames | index curl | middle curl | ring curl | pinky curl | thumb close curl | thumb across-palm |
|---|---:|---:|---:|---:|---:|---:|---:|
| open | 72 / 361 | 0.0520 [0.0490, 0.0554] | 0.0552 [0.0537, 0.0572] | 0.0352 [0.0332, 0.0372] | 0.0893 [0.0795, 0.1091] | 0.1903 [0.1857, 0.1991] | -1.2004 [-1.2339, -1.1636] |
| fist | 72 / 357 | 0.9203 [0.8859, 0.9335] | 0.8835 [0.8661, 0.8872] | 0.8762 [0.8695, 0.8936] | 0.8984 [0.8711, 0.9242] | 0.3226 [0.1754, 0.4074] | -0.4288 [-1.0234, -0.2054] |
| thumb_open | 72 / 361 | 0.0593 [0.0556, 0.0640] | 0.0500 [0.0487, 0.0514] | 0.0307 [0.0273, 0.0335] | 0.0992 [0.0965, 0.1029] | 0.1999 [0.1979, 0.2086] | -1.1373 [-1.1582, -1.0676] |
| thumb_neutral | 73 / 358 | 0.1202 [0.1190, 0.1217] | 0.0910 [0.0887, 0.1041] | 0.1236 [0.1223, 0.1540] | 0.1574 [0.1538, 0.1928] | 0.1943 [0.1908, 0.1970] | -0.8177 [-0.8244, -0.8055] |
| thumb_opposed | 73 / 357 | 0.1159 [0.1128, 0.1189] | 0.1146 [0.1128, 0.1165] | 0.1615 [0.1588, 0.1640] | 0.1779 [0.1761, 0.1796] | 0.4107 [0.2799, 0.4400] | 0.0603 [-0.1415, 0.0729] |
| index_pinch | 72 / 357 | 0.4394 [0.4178, 0.4616] | 0.3553 [0.3417, 0.3816] | 0.3112 [0.3081, 0.3255] | 0.2288 [0.2256, 0.2345] | 0.2053 [0.2036, 0.2162] | -0.2167 [-0.2175, -0.1906] |
| middle_pinch | 73 / 356 | 0.1438 [0.1382, 0.1768] | 0.4608 [0.4009, 0.6965] | 0.4581 [0.2879, 0.6510] | 0.1297 [0.1245, 0.4536] | 0.2202 [0.2095, 0.3242] | -0.0059 [-0.0582, 0.0063] |
| tripod | 72 / 355 | 0.2525 [0.2489, 0.2594] | 0.4363 [0.4304, 0.4391] | 0.4706 [0.4294, 0.5029] | 0.4665 [0.3191, 0.5430] | 0.2265 [0.2220, 0.2330] | -0.0096 [-0.0133, 0.0005] |

## Pinch geometry

Distances are divided by `distance(wrist, middle MCP)`, matching the production fingertip-distance normalization.

| pose | thumb-index | thumb-middle | index-middle |
|---|---:|---:|---:|
| open | 1.2292 [1.1934, 1.2673] | 1.4513 [1.4150, 1.4904] | 0.2248 [0.2238, 0.2258] |
| fist | 0.5376 [0.5074, 0.6979] | 0.6702 [0.6098, 0.8490] | 0.1638 [0.1551, 0.1899] |
| thumb_open | 1.1820 [1.1431, 1.1888] | 1.4349 [1.3986, 1.4387] | 0.2683 [0.2622, 0.2746] |
| thumb_neutral | 0.8483 [0.8468, 0.8495] | 1.0974 [1.0705, 1.0988] | 0.2851 [0.2771, 0.2855] |
| thumb_opposed | 0.8206 [0.7659, 0.8304] | 0.8264 [0.7963, 0.8417] | 0.2658 [0.2626, 0.2672] |
| index_pinch | 0.0512 [0.0358, 0.0550] | 0.2582 [0.2210, 0.2641] | 0.2277 [0.2145, 0.2326] |
| middle_pinch | 0.7325 [0.6303, 0.8581] | 0.1019 [0.0177, 0.1719] | 0.7573 [0.6266, 0.8638] |
| tripod | 0.7026 [0.7002, 0.7052] | 0.0458 [0.0418, 0.0549] | 0.6933 [0.6851, 0.7009] |

## Measured endpoint medians

- finger open combined curl: `[0.051979, 0.055232, 0.035240, 0.089340]`
- finger closed combined curl: `[0.920303, 0.883532, 0.876213, 0.898434]`
- thumb open / neutral / opposed across-palm: `-1.137268 / -0.817716 / 0.060326`
- positive thumb feature direction is index-MCP toward pinky-MCP, matching increasing RH56 thumb_lateral opposition.

No RH56 pinch pose is inferred from these human features; actuator poses still require separate bounded physical contact validation.

## Inputs

- `open`: `logs/rh56_cal_open_20260730_211100.hts.jsonl`
- `fist`: `logs/rh56_cal_fist_20260730_211110.hts.jsonl`
- `thumb_open`: `logs/rh56_cal_thumb_open_20260730_211120.hts.jsonl`
- `thumb_neutral`: `logs/rh56_cal_thumb_neutral_20260730_211130.hts.jsonl`
- `thumb_opposed`: `logs/rh56_cal_thumb_opposed_20260730_211140.hts.jsonl`
- `index_pinch`: `logs/rh56_cal_index_pinch_20260730_211150.hts.jsonl`
- `middle_pinch`: `logs/rh56_cal_middle_pinch_20260730_211200.hts.jsonl`
- `tripod`: `logs/rh56_cal_tripod_20260730_211210.hts.jsonl`
