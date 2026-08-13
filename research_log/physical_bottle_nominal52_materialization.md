# Physical bottle training datasets v2

Raw episodes remain immutable. The two exported views use identical logical samples, frame ranges, ordering, images, and native absolute actions; ACT ignores the retained force provenance columns and ACT+Force exposes them.

- Clean logical trajectories: **52**
- ACT rows: **33111**
- ACT+Force rows: **33111**
- Matched samples: **True**
- Total cropped duration: **1102.100 s**
- Train/val/test logical segments: `[0, 1, 2, 5, 6, 7, 8, 9, 10, 11, 14, 15, 16, 17, 18, 19, 20, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45]` / `[3, 4, 12, 13, 21, 22, 23, 24, 25, 46, 47, 48, 49, 50, 51]` / `[]`

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
| ep121 | 121 | 60–672 | 20.399999796 | 613 | 210 | 1.0 | CLEAN_FULL_TASK | included |
| ep131 | 131 | 0–516 | 17.199999828 | 517 | 152 | 1.0 | CLEAN_FULL_TASK | included |
| ep133 | 133 | 44–733 | 22.966666437 | 690 | 217 | 1.0 | CLEAN_FULL_TASK | included |
| ep134 | 134 | 28–769 | 24.699999753 | 742 | 191 | 1.0 | CLEAN_FULL_TASK | included |
| ep137 | 137 | 0–605 | 20.166666465 | 606 | 179 | 1.0 | CLEAN_FULL_TASK | included |
| ep138 | 138 | 0–605 | 20.166666465 | 606 | 150 | 1.0 | CLEAN_FULL_TASK | included |
| ep139 | 139 | 0–747 | 24.899999751 | 748 | 150 | 1.0 | CLEAN_FULL_TASK | included |
| ep140 | 140 | 0–684 | 22.799999772 | 685 | 150 | 1.0 | CLEAN_FULL_TASK | included |
| ep141 | 141 | 0–555 | 18.499999815 | 556 | 154 | 1.0 | CLEAN_FULL_TASK | included |
| ep142 | 142 | 0–587 | 19.566666471 | 588 | 150 | 1.0 | CLEAN_FULL_TASK | included |
| ep143 | 143 | 0–567 | 18.899999811 | 568 | 150 | 1.0 | CLEAN_FULL_TASK | included |
| ep144 | 144 | 0–761 | 25.366666413 | 762 | 250 | 1.0 | CLEAN_FULL_TASK | included |
| ep145 | 145 | 0–780 | 25.99999974 | 781 | 213 | 1.0 | CLEAN_FULL_TASK | included |
| ep146 | 146 | 0–524 | 17.466666492 | 525 | 179 | 1.0 | CLEAN_FULL_TASK | included |
| ep147 | 147 | 0–452 | 15.099999849 | 453 | 176 | 1.0 | CLEAN_FULL_TASK | included |
| ep148 | 148 | 0–575 | 19.166666475 | 576 | 191 | 1.0 | CLEAN_FULL_TASK | included |
| ep149 | 149 | 1–460 | 15.299999847 | 460 | 270 | 1.0 | CLEAN_FULL_TASK | included |
| ep150 | 150 | 115–672 | 18.599999814 | 558 | 264 | 1.0 | CLEAN_FULL_TASK | included |
| ep151 | 151 | 1–564 | 18.766666479 | 564 | 149 | 1.0 | CLEAN_FULL_TASK | included |
| ep152 | 152 | 3–547 | 18.133333152 | 545 | 152 | 1.0 | CLEAN_FULL_TASK | included |
| ep154 | 154 | 1–484 | 16.099999839 | 484 | 150 | 1.0 | CLEAN_FULL_TASK | included |
| ep155 | 155 | 1–538 | 17.899999821 | 538 | 150 | 1.0 | CLEAN_FULL_TASK | included |
| ep156 | 156 | 19–559 | 17.99999982 | 541 | 168 | 1.0 | CLEAN_FULL_TASK | included |
| ep157 | 157 | 1–539 | 17.933333154 | 539 | 149 | 1.0 | CLEAN_FULL_TASK | included |
| ep158 | 158 | 1–485 | 16.133333172 | 485 | 150 | 1.0 | CLEAN_FULL_TASK | included |
| ep159 | 159 | 1–654 | 21.766666449 | 654 | 150 | 1.0 | CLEAN_FULL_TASK | included |
| ep160 | 160 | 1–707 | 23.533333098 | 707 | 149 | 1.0 | CLEAN_FULL_TASK | included |
| ep161 | 161 | 1–519 | 17.266666494 | 519 | 16 | 1.0 | CLEAN_FULL_TASK | included |
| ep162 | 162 | 42–521 | 15.966666507 | 480 | 191 | 1.0 | CLEAN_FULL_TASK | included |
| ep166 | 166 | 2–519 | 17.233333161 | 518 | 151 | 1.0 | CLEAN_FULL_TASK | included |
| ep172_a | 172 | 0–503 | 16.766666499 | 504 | 1898 | 1.0 | CLEAN_FULL_TASK | included |
| ep172_b | 172 | 802–1326 | 17.466666492 | 525 | 1877 | 1.0 | CLEAN_FULL_TASK | included |
| ep172_c | 172 | 1664–2169 | 16.833333165 | 506 | 1896 | 1.0 | CLEAN_FULL_TASK | included |
| ep173 | 173 | 1–627 | 20.866666458 | 627 | 150 | 1.0 | CLEAN_FULL_TASK | included |
| ep174_a | 174 | 0–519 | 17.299999827 | 520 | 1572 | 1.0 | CLEAN_FULL_TASK | included |
| ep174_b | 174 | 845–1488 | 21.433333119 | 644 | 1448 | 1.0 | CLEAN_FULL_TASK | included |

## Row filtering and semantics

Rows were excluded from both views only for invalid state/action/camera/synchronization data, malformed rows, non-causal source timestamps, or outside the reviewed logical crop. Force-invalid or stale rows are retained in both matched views with `force_valid=false`, `force_age_s`, and the causal source timestamp preserved. No force interpolation or Newton conversion is performed.

The stored state is 12-D `[JAKA measured joints 6, RH56 measured actuator positions 6]`. The stored action is 12-D `[JAKA accepted targets 6, RH56 targets 6]`, in the existing canonical RH56 order. Force is six raw RH56 FORCE_ACT counts. ACT action chunks are limited to one logical segment and use absolute/native targets with end masks.

Normalization statistics are under `stats/` and are computed only from the train logical-segment split; force statistics use only force-valid rows.
