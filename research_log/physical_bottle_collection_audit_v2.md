# Physical bottle collection audit v2

This report was generated from the immutable raw episode tree. Video readability and canonical row/timestamp checks were performed offline; trajectory classifications and merged boundaries are explicit reviewed manifest decisions.

- Raw metadata records discovered: **41**
- Complete payload episodes inspected: **32**
- Logical trajectories with payload: **34**
- Audit manifest entries (including excluded no-payload records): **43**
- Clean full-task logical segments: **25**
- Raw merged episodes split: **2**
- Incomplete-start trajectories: **9**
- Approaches recovered: **0**; impossible to recover: **9**
- Retained slip/drop trajectories with defensible raw evidence: **0**

## Source episode audit

| source episode | classification | payload | rows | canonical Hz | force valid | workspace | wrist | notes |
|---:|---|---|---:|---:|---:|---|---|---|
| 71 | CLEAN_FULL_TASK | payload_inspected | 1066 | 30.0000003 | 1.0 | 1066 @ 30.0 | 1066 @ 30.0 | approach |
| 74 | CLEAN_FULL_TASK | payload_inspected | 1173 | 30.000000300000004 | 1.0 | 1173 @ 30.0 | 1173 @ 30.0 | complete bottle task; reset tail cropped by release event |
| 81 | CLEAN_FULL_TASK | payload_inspected | 1097 | 30.0000003 | 1.0 | 1097 @ 30.0 | 1097 @ 30.0 | complete bottle task; reset tail cropped by release event |
| 82 | CORRUPT_OR_UNUSABLE | no_complete_payload | 0 | n/a | n/a | n/a @ n/a | n/a @ n/a | partial metadata only; no finalized row/video payload |
| 83 | CLEAN_FULL_TASK | payload_inspected | 1525 | 30.000000300000004 | 1.0 | 1525 @ 30.0 | 1525 @ 30.0 | complete bottle task; selected rows and both videos decode |
| 84 | CORRUPT_OR_UNUSABLE | no_complete_payload | 0 | n/a | n/a | n/a @ n/a | n/a @ n/a | metadata exists but finalized row/video payload is absent; rejected sidecar present |
| 85 | CORRUPT_OR_UNUSABLE | no_complete_payload | 0 | n/a | n/a | n/a @ n/a | n/a @ n/a | partial metadata only; no finalized row/video payload |
| 86 | CORRUPT_OR_UNUSABLE | no_complete_payload | 0 | n/a | n/a | n/a @ n/a | n/a @ n/a | metadata records control_source_timestamp_regression:jaka_command and no finalized payload |
| 87 | CLEAN_FULL_TASK | payload_inspected | 939 | 30.0000003 | 1.0 | 939 @ 30.0 | 939 @ 30.0 | complete bottle task; reset tail cropped by release event |
| 88 | CLEAN_FULL_TASK | payload_inspected | 894 | 30.000000300000004 | 1.0 | 894 @ 30.0 | 894 @ 30.0 | complete bottle task; reset tail cropped by release event |
| 89 | CLEAN_FULL_TASK | payload_inspected | 916 | 30.000000300000004 | 1.0 | 916 @ 30.0 | 916 @ 30.0 | complete bottle task; reset tail cropped by release event |
| 90 | CLEAN_FULL_TASK | payload_inspected | 926 | 30.0000003 | 1.0 | 926 @ 30.0 | 926 @ 30.0 | complete bottle task; reset tail cropped by release event |
| 91 | CLEAN_FULL_TASK | payload_inspected | 1029 | 30.000000300000004 | 1.0 | 1029 @ 30.0 | 1029 @ 30.0 | complete bottle task; reset tail cropped by release event |
| 92 | CLEAN_FULL_TASK | payload_inspected | 1636 | 30.000000300000004 | 1.0 | 1636 @ 30.0 | 1636 @ 30.0 | complete bottle task; reset tail cropped by release event |
| 93 | CORRUPT_OR_UNUSABLE | no_complete_payload | 0 | n/a | n/a | n/a @ n/a | n/a @ n/a | metadata exists but finalized row/video payload is absent; rejected sidecar present |
| 94 | CORRUPT_OR_UNUSABLE | no_complete_payload | 0 | n/a | n/a | n/a @ n/a | n/a @ n/a | metadata exists but finalized row/video payload is absent |
| 95 | CLEAN_FULL_TASK | payload_inspected | 825 | 30.000000300000004 | 1.0 | 825 @ 30.0 | 825 @ 30.0 | complete bottle task; reset tail cropped by release event |
| 96 | CLEAN_FULL_TASK | payload_inspected | 883 | 30.000000300000004 | 1.0 | 883 @ 30.0 | 883 @ 30.0 | complete bottle task; reset tail cropped by release event |
| 97 | CLEAN_FULL_TASK | payload_inspected | 1019 | 30.000000300000007 | 1.0 | 1019 @ 30.0 | 1019 @ 30.0 | complete bottle task; reset tail cropped by release event |
| 98 | CLEAN_FULL_TASK | payload_inspected | 911 | 29.967069454445667 | 1.0 | 911 @ 30.0 | 911 @ 30.0 | complete bottle task; reset tail cropped by release event |
| 99 | MERGED_MULTIPLE_EPISODES | payload_inspected | 2498 | 29.987990692193762 | 1.0 | 2498 @ 30.0 | 2498 @ 30.0 | two complete tasks separated by a clear human reset/idle interval around source frames 1170-1619 |
| 100 | CORRUPT_OR_UNUSABLE | no_complete_payload | 0 | n/a | n/a | n/a @ n/a | n/a @ n/a | metadata exists but finalized row/video payload is absent; rejected sidecar present |
| 101 | CORRUPT_OR_UNUSABLE | no_complete_payload | 0 | n/a | n/a | n/a @ n/a | n/a @ n/a | metadata exists but finalized row/video payload is absent |
| 102 | MERGED_MULTIPLE_EPISODES | payload_inspected | 2085 | 30.000000300000004 | 1.0 | 2085 @ 30.0 | 2085 @ 30.0 | two complete tasks separated by a clear human reset/idle interval around source frames 1005-1319 |
| 103 | INCOMPLETE_START | payload_inspected | 518 | 30.0000003 | 1.0 | 518 @ 30.0 | 518 @ 30.0 | first frame already holds bottle; missing approach is not present in source payload |
| 104 | INCOMPLETE_START | payload_inspected | 511 | 30.000000300000004 | 1.0 | 511 @ 30.0 | 511 @ 30.0 | first frame already holds bottle; missing approach is not present in source payload |
| 105 | CLEAN_FULL_TASK | payload_inspected | 857 | 29.964994465344226 | 1.0 | 857 @ 30.0 | 857 @ 30.0 | open gripper and bottle approach are present; complete task |
| 106 | INCOMPLETE_START | payload_inspected | 525 | 30.0000003 | 1.0 | 525 @ 30.0 | 525 @ 30.0 | first frame already holds bottle; missing approach is not present in source payload |
| 107 | INCOMPLETE_START | payload_inspected | 724 | 30.000000300000004 | 1.0 | 724 @ 30.0 | 724 @ 30.0 | first frame already holds bottle; missing approach is not present in source payload |
| 108 | CLEAN_FULL_TASK | payload_inspected | 857 | 30.000000300000004 | 1.0 | 857 @ 30.0 | 857 @ 30.0 | open gripper and bottle approach are present; complete task |
| 109 | CLEAN_FULL_TASK | payload_inspected | 771 | 30.0000003 | 1.0 | 771 @ 30.0 | 771 @ 30.0 | open gripper and bottle approach are present; complete task |
| 110 | INCOMPLETE_START | payload_inspected | 545 | 29.94495442788991 | 1.0 | 545 @ 30.0 | 545 @ 30.0 | first frame already holds bottle; missing approach is not present in source payload |
| 111 | INCOMPLETE_START | payload_inspected | 1021 | 30.000000300000004 | 1.0 | 1021 @ 30.0 | 1021 @ 30.0 | first frame already holds bottle; missing approach is not present in source payload |
| 112 | INCOMPLETE_START | payload_inspected | 586 | 30.0000003 | 1.0 | 586 @ 30.0 | 586 @ 30.0 | first frame already holds bottle; missing approach is not present in source payload |
| 113 | CLEAN_FULL_TASK | payload_inspected | 745 | 30.0000003 | 1.0 | 745 @ 30.0 | 745 @ 30.0 | open gripper and bottle approach are present; complete task |
| 114 | CLEAN_FULL_TASK | payload_inspected | 746 | 30.000000300000004 | 1.0 | 746 @ 30.0 | 746 @ 30.0 | open gripper and bottle approach are present; complete task |
| 115 | INCOMPLETE_START | payload_inspected | 493 | 30.000000300000004 | 1.0 | 493 @ 30.0 | 493 @ 30.0 | first frame already holds bottle; missing approach is not present in source payload |
| 116 | CLEAN_FULL_TASK | payload_inspected | 706 | 30.000000300000004 | 1.0 | 706 @ 30.0 | 706 @ 30.0 | open gripper and bottle approach are present; complete task |
| 117 | CLEAN_FULL_TASK | payload_inspected | 848 | 30.000000300000004 | 1.0 | 848 @ 30.0 | 848 @ 30.0 | open gripper and bottle approach are present; complete task |
| 118 | INCOMPLETE_START | payload_inspected | 669 | 30.000000300000004 | 1.0 | 669 @ 30.0 | 669 @ 30.0 | first frame already holds bottle; missing approach is not present in source payload |
| 119 | CORRUPT_OR_UNUSABLE | no_complete_payload | 0 | n/a | n/a | n/a @ n/a | n/a @ n/a | partial metadata only; no finalized row/video payload |

## Logical trajectory decisions

| logical segment | source | class | source frame range | include | decision |
|---|---:|---|---|---|---|
| ep071_seg0 | 71 | CLEAN_FULL_TASK | 0–release | True | authoritative release crop |
| ep074_seg0 | 74 | CLEAN_FULL_TASK | 0–release | True | authoritative release crop |
| ep081_seg0 | 81 | CLEAN_FULL_TASK | 0–release | True | authoritative release crop |
| ep083_seg0 | 83 | CLEAN_FULL_TASK | 0–release | True | authoritative release crop |
| ep087_seg0 | 87 | CLEAN_FULL_TASK | 0–release | True | authoritative release crop |
| ep088_seg0 | 88 | CLEAN_FULL_TASK | 0–release | True | authoritative release crop |
| ep089_seg0 | 89 | CLEAN_FULL_TASK | 0–release | True | authoritative release crop |
| ep090_seg0 | 90 | CLEAN_FULL_TASK | 0–release | True | authoritative release crop |
| ep091_seg0 | 91 | CLEAN_FULL_TASK | 0–release | True | authoritative release crop |
| ep092_seg0 | 92 | CLEAN_FULL_TASK | 0–release | True | authoritative release crop |
| ep095_seg0 | 95 | CLEAN_FULL_TASK | 0–release | True | authoritative release crop |
| ep096_seg0 | 96 | CLEAN_FULL_TASK | 0–release | True | authoritative release crop |
| ep097_seg0 | 97 | CLEAN_FULL_TASK | 0–release | True | authoritative release crop |
| ep098_seg0 | 98 | CLEAN_FULL_TASK | 0–release | True | authoritative release crop |
| ep099_seg0 | 99 | CLEAN_FULL_TASK | 0–1169 | True | reviewed first-task boundary before human reset |
| ep099_seg1 | 99 | CLEAN_FULL_TASK | 1620–release | True | reviewed post-reset approach; authoritative release crop |
| ep102_seg0 | 102 | CLEAN_FULL_TASK | 0–1004 | True | reviewed first-task boundary before human reset |
| ep102_seg1 | 102 | CLEAN_FULL_TASK | 1320–release | True | reviewed post-reset approach; authoritative release crop |
| ep105_seg0 | 105 | CLEAN_FULL_TASK | 0–release | True | authoritative release crop |
| ep108_seg0 | 108 | CLEAN_FULL_TASK | 0–release | True | authoritative release crop |
| ep109_seg0 | 109 | CLEAN_FULL_TASK | 0–release | True | authoritative release crop |
| ep113_seg0 | 113 | CLEAN_FULL_TASK | 0–release | True | authoritative release crop |
| ep114_seg0 | 114 | CLEAN_FULL_TASK | 0–release | True | authoritative release crop |
| ep116_seg0 | 116 | CLEAN_FULL_TASK | 0–release | True | authoritative release crop |
| ep117_seg0 | 117 | CLEAN_FULL_TASK | 0–release | True | authoritative release crop |
| ep103_seg0 | 103 | INCOMPLETE_START | 0–release | False | partial trajectory retained in excluded manifest |
| ep104_seg0 | 104 | INCOMPLETE_START | 0–release | False | partial trajectory retained in excluded manifest |
| ep106_seg0 | 106 | INCOMPLETE_START | 0–release | False | partial trajectory retained in excluded manifest |
| ep107_seg0 | 107 | INCOMPLETE_START | 0–release | False | partial trajectory retained in excluded manifest |
| ep110_seg0 | 110 | INCOMPLETE_START | 0–release | False | partial trajectory retained in excluded manifest |
| ep111_seg0 | 111 | INCOMPLETE_START | 0–release | False | partial trajectory retained in excluded manifest |
| ep112_seg0 | 112 | INCOMPLETE_START | 0–release | False | partial trajectory retained in excluded manifest |
| ep115_seg0 | 115 | INCOMPLETE_START | 0–release | False | partial trajectory retained in excluded manifest |
| ep118_seg0 | 118 | INCOMPLETE_START | 0–release | False | partial trajectory retained in excluded manifest |
| ep082_seg0 | 82 | CORRUPT_OR_UNUSABLE | 0–release | False | partial metadata only |
| ep084_seg0 | 84 | CORRUPT_OR_UNUSABLE | 0–release | False | source payload absent |
| ep085_seg0 | 85 | CORRUPT_OR_UNUSABLE | 0–release | False | partial metadata only |
| ep086_seg0 | 86 | CORRUPT_OR_UNUSABLE | 0–release | False | invalid timestamp regression and source payload absent |
| ep093_seg0 | 93 | CORRUPT_OR_UNUSABLE | 0–release | False | source payload absent |
| ep094_seg0 | 94 | CORRUPT_OR_UNUSABLE | 0–release | False | source payload absent |
| ep100_seg0 | 100 | CORRUPT_OR_UNUSABLE | 0–release | False | source payload absent |
| ep101_seg0 | 101 | CORRUPT_OR_UNUSABLE | 0–release | False | source payload absent |
| ep119_seg0 | 119 | CORRUPT_OR_UNUSABLE | 0–release | False | partial metadata only |

### Review conclusions

Episodes 99 and 102 have clear reset gaps and are represented as two logical segments each. Their first segments use the explicit visual/reset boundary in the manifest because the first task's release event was not separately persisted; the second segments end at the authoritative persisted release timestamp. No rows are synthesized, copied, or interpolated.

Episodes 103, 104, 106, 107, 110, 111, 112, 115, and 118 begin with the bottle already grasped. The missing approach is not present in a recoverable prefix or an adjacent continuous source, so these remain available in the excluded manifest but are not in the clean full-task set.

No retained payload in the current raw root provided defensible evidence for a slip/drop label. Missing/partial/invalid source records were not relabelled from metadata alone; the failure manifest is therefore empty and preserves this uncertainty.

All complete payload rows passed finite state/action/force checks, canonical timestamp monotonicity, causal source timestamp checks, and video decode checks. Force repetition is retained as native lower-rate zero-order hold data.
