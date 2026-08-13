# Physical bottle training datasets v2

Raw episodes remain immutable. The two exported views use identical logical samples, frame ranges, ordering, images, and native absolute actions; ACT ignores the retained force provenance columns and ACT+Force exposes them.

- Clean logical trajectories: **16**
- ACT rows: **12177**
- ACT+Force rows: **12177**
- Matched samples: **True**
- Total cropped duration: **405.433 s**
- Train/val/test logical segments: `[0, 1, 2, 5, 6, 7, 8, 9, 10, 11, 14, 15]` / `[3, 4, 12, 13]` / `[]`

## Per logical trajectory

| segment | source | crop frames | crop duration | rows | excluded | force valid | classification | status |
|---|---:|---:|---:|---:|---:|---:|---|---|
| ep067 | 67 | 62–856 | 26.466666402 | 795 | 211 | 1.0 | CLEAN_FULL_TASK | included |
| ep070 | 70 | 14–698 | 22.799999772 | 685 | 164 | 1.0 | CLEAN_FULL_TASK | included |
| ep081 | 81 | 0–946 | 31.533333018 | 947 | 150 | 1.0 | CLEAN_FULL_TASK | included |
| ep087 | 87 | 0–788 | 26.266666404 | 789 | 150 | 1.0 | CLEAN_FULL_TASK | included |
| ep088 | 88 | 61–743 | 22.733333106 | 683 | 211 | 1.0 | CLEAN_FULL_TASK | included |
| ep095 | 95 | 0–674 | 22.466666442 | 675 | 150 | 1.0 | CLEAN_FULL_TASK | included |
| ep096 | 96 | 0–733 | 24.433333089 | 734 | 149 | 1.0 | CLEAN_FULL_TASK | included |
| ep097 | 97 | 29–868 | 27.966666387 | 840 | 179 | 1.0 | CLEAN_FULL_TASK | included |
| ep098 | 98 | 22–760 | 24.633333087 | 739 | 172 | 1.0 | CLEAN_FULL_TASK | included |
| ep099_a | 99 | 0–965 | 32.166666345 | 966 | 1532 | 1.0 | CLEAN_FULL_TASK | included |
| ep099_b | 99 | 1335–2348 | 33.799999662 | 1014 | 1484 | 1.0 | CLEAN_FULL_TASK | included |
| ep102_b | 102 | 1209–1934 | 24.166666425 | 726 | 1359 | 1.0 | CLEAN_FULL_TASK | included |
| ep108 | 108 | 0–706 | 23.533333098 | 707 | 150 | 1.0 | CLEAN_FULL_TASK | included |
| ep109 | 109 | 0–620 | 20.66666646 | 621 | 150 | 1.0 | CLEAN_FULL_TASK | included |
| ep116 | 116 | 0–556 | 18.533333148 | 557 | 149 | 1.0 | CLEAN_FULL_TASK | included |
| ep117 | 117 | 0–698 | 23.266666434 | 699 | 149 | 1.0 | CLEAN_FULL_TASK | included |

## Row filtering and semantics

Rows were excluded from both views only for invalid state/action/camera/synchronization data, malformed rows, non-causal source timestamps, or outside the reviewed logical crop. Force-invalid or stale rows are retained in both matched views with `force_valid=false`, `force_age_s`, and the causal source timestamp preserved. No force interpolation or Newton conversion is performed.

The stored state is 12-D `[JAKA measured joints 6, RH56 measured actuator positions 6]`. The stored action is 12-D `[JAKA accepted targets 6, RH56 targets 6]`, in the existing canonical RH56 order. Force is six raw RH56 FORCE_ACT counts. ACT action chunks are limited to one logical segment and use absolute/native targets with end masks.

Normalization statistics are under `stats/` and are computed only from the train logical-segment split; force statistics use only force-valid rows.
