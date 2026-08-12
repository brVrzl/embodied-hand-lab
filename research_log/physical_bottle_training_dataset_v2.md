# Physical bottle training datasets v2

Raw episodes remain immutable. The two exported views use identical logical samples, frame ranges, ordering, images, and native absolute actions; ACT ignores the retained force provenance columns and ACT+Force exposes them.

- Clean logical trajectories: **25**
- ACT rows: **20744**
- ACT+Force rows: **20744**
- Matched samples: **True**
- Total cropped duration: **690.700 s**
- Train/val/test logical segments: `[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24]` / `[]` / `[]`

## Per logical trajectory

| segment | source | crop frames | crop duration | rows | excluded | force valid | classification | status |
|---|---:|---:|---:|---:|---:|---:|---|---|
| ep071_seg0 | 71 | 0–915 | 30.499999695 | 916 | 150 | 1.0 | CLEAN_FULL_TASK | included |
| ep074_seg0 | 74 | 0–1023 | 34.099999659 | 1024 | 149 | 1.0 | CLEAN_FULL_TASK | included |
| ep081_seg0 | 81 | 0–946 | 31.533333018 | 947 | 150 | 1.0 | CLEAN_FULL_TASK | included |
| ep083_seg0 | 83 | 0–1374 | 45.799999542 | 1375 | 150 | 1.0 | CLEAN_FULL_TASK | included |
| ep087_seg0 | 87 | 0–788 | 26.266666404 | 789 | 150 | 1.0 | CLEAN_FULL_TASK | included |
| ep088_seg0 | 88 | 0–743 | 24.766666419 | 744 | 150 | 1.0 | CLEAN_FULL_TASK | included |
| ep089_seg0 | 89 | 0–765 | 25.499999745 | 766 | 150 | 1.0 | CLEAN_FULL_TASK | included |
| ep090_seg0 | 90 | 0–775 | 25.833333075 | 776 | 150 | 1.0 | CLEAN_FULL_TASK | included |
| ep091_seg0 | 91 | 0–879 | 29.299999707 | 880 | 149 | 1.0 | CLEAN_FULL_TASK | included |
| ep092_seg0 | 92 | 0–1486 | 49.533332838 | 1487 | 149 | 1.0 | CLEAN_FULL_TASK | included |
| ep095_seg0 | 95 | 0–674 | 22.466666442 | 675 | 150 | 1.0 | CLEAN_FULL_TASK | included |
| ep096_seg0 | 96 | 0–733 | 24.433333089 | 734 | 149 | 1.0 | CLEAN_FULL_TASK | included |
| ep097_seg0 | 97 | 0–868 | 28.933333044 | 869 | 150 | 1.0 | CLEAN_FULL_TASK | included |
| ep098_seg0 | 98 | 0–760 | 25.366666413 | 761 | 150 | 1.0 | CLEAN_FULL_TASK | included |
| ep099_seg0 | 99 | 0–1169 | 38.966666277 | 1170 | 1328 | 1.0 | CLEAN_FULL_TASK | included |
| ep099_seg1 | 99 | 1620–2348 | 24.266666424 | 729 | 1769 | 1.0 | CLEAN_FULL_TASK | included |
| ep102_seg0 | 102 | 0–1004 | 33.466666332 | 1005 | 1080 | 1.0 | CLEAN_FULL_TASK | included |
| ep102_seg1 | 102 | 1320–1934 | 20.466666462 | 615 | 1470 | 1.0 | CLEAN_FULL_TASK | included |
| ep105_seg0 | 105 | 0–706 | 23.566666431 | 707 | 150 | 1.0 | CLEAN_FULL_TASK | included |
| ep108_seg0 | 108 | 0–706 | 23.533333098 | 707 | 150 | 1.0 | CLEAN_FULL_TASK | included |
| ep109_seg0 | 109 | 0–620 | 20.66666646 | 621 | 150 | 1.0 | CLEAN_FULL_TASK | included |
| ep113_seg0 | 113 | 0–594 | 19.799999802 | 595 | 150 | 1.0 | CLEAN_FULL_TASK | included |
| ep114_seg0 | 114 | 0–595 | 19.833333135 | 596 | 150 | 1.0 | CLEAN_FULL_TASK | included |
| ep116_seg0 | 116 | 0–556 | 18.533333148 | 557 | 149 | 1.0 | CLEAN_FULL_TASK | included |
| ep117_seg0 | 117 | 0–698 | 23.266666434 | 699 | 149 | 1.0 | CLEAN_FULL_TASK | included |

## Row filtering and semantics

Rows were excluded from both views only for invalid state/action/camera/synchronization data, malformed rows, non-causal source timestamps, or outside the reviewed logical crop. Force-invalid or stale rows are retained in both matched views with `force_valid=false`, `force_age_s`, and the causal source timestamp preserved. No force interpolation or Newton conversion is performed.

The stored state is 12-D `[JAKA measured joints 6, RH56 measured actuator positions 6]`. The stored action is 12-D `[JAKA accepted targets 6, RH56 targets 6]`, in the existing canonical RH56 order. Force is six raw RH56 FORCE_ACT counts. ACT action chunks are limited to one logical segment and use absolute/native targets with end masks.

Normalization statistics are under `stats/` and are computed only from the train logical-segment split; force statistics use only force-valid rows.
