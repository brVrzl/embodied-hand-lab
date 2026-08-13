# Physical bottle collection audit v2

This report was generated from the immutable raw episode tree. Video readability and canonical row/timestamp checks were performed offline; trajectory classifications and merged boundaries are explicit reviewed manifest decisions.

- Raw metadata records discovered: **25**
- Complete payload episodes inspected: **25**
- Logical trajectories with payload: **27**
- Audit manifest entries (including excluded no-payload records): **27**
- Clean full-task logical segments: **16**
- Raw merged episodes split: **2**
- Explicitly classified incomplete-start trajectories: **0** (generic non-nominal entries were not relabelled)
- Explicitly adjudicated approaches recovered: **0**; impossible to recover: **0**
- Specifically labelled slip/drop trajectories with defensible raw evidence: **0**

## Source episode audit

| source episode | classification | payload | rows | canonical Hz | force valid | workspace | wrist | notes |
|---:|---|---|---:|---:|---:|---|---|---|
| 67 | CLEAN_FULL_TASK | payload_inspected | 1006 | 30.000000300000007 | 1.0 | 1006 @ 30.0 | 1006 @ 30.0 | human-audited nominal full task |
| 70 | CLEAN_FULL_TASK | payload_inspected | 849 | 30.000000300000004 | 1.0 | 849 @ 30.0 | 849 @ 30.0 | human-audited nominal full task |
| 71 | MANUAL_AUDIT_NON_NOMINAL | payload_inspected | 1066 | 30.0000003 | 1.0 | 1066 @ 30.0 | 1066 @ 30.0 | manual_audit_non_nominal |
| 74 | MANUAL_AUDIT_NON_NOMINAL | payload_inspected | 1173 | 30.000000300000004 | 1.0 | 1173 @ 30.0 | 1173 @ 30.0 | manual_audit_non_nominal |
| 81 | CLEAN_FULL_TASK | payload_inspected | 1097 | 30.0000003 | 1.0 | 1097 @ 30.0 | 1097 @ 30.0 | human-audited nominal full task |
| 83 | MANUAL_AUDIT_NON_NOMINAL | payload_inspected | 1525 | 30.000000300000004 | 1.0 | 1525 @ 30.0 | 1525 @ 30.0 | manual_audit_non_nominal |
| 87 | CLEAN_FULL_TASK | payload_inspected | 939 | 30.0000003 | 1.0 | 939 @ 30.0 | 939 @ 30.0 | human-audited nominal full task |
| 88 | CLEAN_FULL_TASK | payload_inspected | 894 | 30.000000300000004 | 1.0 | 894 @ 30.0 | 894 @ 30.0 | human-audited nominal full task |
| 89 | MANUAL_AUDIT_NON_NOMINAL | payload_inspected | 916 | 30.000000300000004 | 1.0 | 916 @ 30.0 | 916 @ 30.0 | manual_audit_non_nominal |
| 90 | MANUAL_AUDIT_NON_NOMINAL | payload_inspected | 926 | 30.0000003 | 1.0 | 926 @ 30.0 | 926 @ 30.0 | manual_audit_non_nominal |
| 91 | MANUAL_AUDIT_NON_NOMINAL | payload_inspected | 1029 | 30.000000300000004 | 1.0 | 1029 @ 30.0 | 1029 @ 30.0 | manual_audit_non_nominal |
| 92 | MANUAL_AUDIT_NON_NOMINAL | payload_inspected | 1636 | 30.000000300000004 | 1.0 | 1636 @ 30.0 | 1636 @ 30.0 | manual_audit_non_nominal |
| 95 | CLEAN_FULL_TASK | payload_inspected | 825 | 30.000000300000004 | 1.0 | 825 @ 30.0 | 825 @ 30.0 | human-audited nominal full task |
| 96 | CLEAN_FULL_TASK | payload_inspected | 883 | 30.000000300000004 | 1.0 | 883 @ 30.0 | 883 @ 30.0 | human-audited nominal full task |
| 97 | CLEAN_FULL_TASK | payload_inspected | 1019 | 30.000000300000007 | 1.0 | 1019 @ 30.0 | 1019 @ 30.0 | human-audited nominal full task |
| 98 | CLEAN_FULL_TASK | payload_inspected | 911 | 29.967069454445667 | 1.0 | 911 @ 30.0 | 911 @ 30.0 | human-audited nominal full task |
| 99 | MERGED_MULTIPLE_EPISODES | payload_inspected | 2498 | 29.987990692193762 | 1.0 | 2498 @ 30.0 | 2498 @ 30.0 | two human-audited nominal demonstrations separated by a short reset interval |
| 102 | MERGED_MULTIPLE_EPISODES | payload_inspected | 2085 | 30.000000300000004 | 1.0 | 2085 @ 30.0 | 2085 @ 30.0 | first demonstration non-nominal; second human-audited nominal |
| 105 | MANUAL_AUDIT_NON_NOMINAL | payload_inspected | 857 | 29.964994465344226 | 1.0 | 857 @ 30.0 | 857 @ 30.0 | manual_audit_non_nominal |
| 108 | CLEAN_FULL_TASK | payload_inspected | 857 | 30.000000300000004 | 1.0 | 857 @ 30.0 | 857 @ 30.0 | human-audited nominal full task |
| 109 | CLEAN_FULL_TASK | payload_inspected | 771 | 30.0000003 | 1.0 | 771 @ 30.0 | 771 @ 30.0 | human-audited nominal full task |
| 113 | MANUAL_AUDIT_NON_NOMINAL | payload_inspected | 745 | 30.0000003 | 1.0 | 745 @ 30.0 | 745 @ 30.0 | manual_audit_non_nominal |
| 114 | MANUAL_AUDIT_NON_NOMINAL | payload_inspected | 746 | 30.000000300000004 | 1.0 | 746 @ 30.0 | 746 @ 30.0 | manual_audit_non_nominal |
| 116 | CLEAN_FULL_TASK | payload_inspected | 706 | 30.000000300000004 | 1.0 | 706 @ 30.0 | 706 @ 30.0 | human-audited nominal full task |
| 117 | CLEAN_FULL_TASK | payload_inspected | 848 | 30.000000300000004 | 1.0 | 848 @ 30.0 | 848 @ 30.0 | human-audited nominal full task |

## Logical trajectory decisions

| logical segment | source | class | source frame range | include | decision |
|---|---:|---|---|---|---|
| ep067 | 67 | CLEAN_FULL_TASK | 62–856 | True | task starts one causal row before first accepted target change after 2.07 s pre-task hold; ends at persisted task release |
| ep070 | 70 | CLEAN_FULL_TASK | 14–698 | True | task starts one causal row before first accepted target change after 0.47 s pre-task hold; ends at persisted task release |
| ep071 | 71 | MANUAL_AUDIT_NON_NOMINAL | 0–release | False | retained outside nominal baseline |
| ep074 | 74 | MANUAL_AUDIT_NON_NOMINAL | 0–release | False | retained outside nominal baseline |
| ep081 | 81 | CLEAN_FULL_TASK | 0–946 | True | task motion begins at recording start; ends at persisted task release |
| ep083 | 83 | MANUAL_AUDIT_NON_NOMINAL | 0–release | False | retained outside nominal baseline |
| ep087 | 87 | CLEAN_FULL_TASK | 0–788 | True | task motion begins at recording start; ends at persisted task release |
| ep088 | 88 | CLEAN_FULL_TASK | 61–743 | True | task starts one causal row before first accepted target change after 2.03 s pre-task hold; ends at persisted task release |
| ep089 | 89 | MANUAL_AUDIT_NON_NOMINAL | 0–release | False | retained outside nominal baseline |
| ep090 | 90 | MANUAL_AUDIT_NON_NOMINAL | 0–release | False | retained outside nominal baseline |
| ep091 | 91 | MANUAL_AUDIT_NON_NOMINAL | 0–release | False | retained outside nominal baseline |
| ep092 | 92 | MANUAL_AUDIT_NON_NOMINAL | 0–release | False | retained outside nominal baseline |
| ep095 | 95 | CLEAN_FULL_TASK | 0–674 | True | task motion begins at recording start; ends at persisted task release |
| ep096 | 96 | CLEAN_FULL_TASK | 0–733 | True | task begins at recording start; ends at persisted task release |
| ep097 | 97 | CLEAN_FULL_TASK | 29–868 | True | task starts one causal row before first accepted target change after 0.97 s pre-task hold; ends at persisted task release |
| ep098 | 98 | CLEAN_FULL_TASK | 22–760 | True | task starts one causal row before first accepted target change after 0.73 s pre-task hold; ends at persisted task release |
| ep099_a | 99 | CLEAN_FULL_TASK | 0–965 | True | first nominal demonstration ends at first post-release row; frames 966-1334 are reset/recovery and belong to neither task |
| ep099_b | 99 | CLEAN_FULL_TASK | 1335–2348 | True | second nominal demonstration starts at post-reset arm-reference press and ends at authoritative release row; frames 2349-2497 are post-task tail |
| ep102_a | 102 | MANUAL_AUDIT_NON_NOMINAL | 0–877 | False | first logical demonstration retained outside nominal baseline; frames 878-1208 are reset/recovery and belong to neither task |
| ep102_b | 102 | CLEAN_FULL_TASK | 1209–1934 | True | second nominal demonstration starts after excluded reset/recovery frames 878-1208 and ends at authoritative release row; frames 1935-2084 are post-task tail |
| ep105 | 105 | MANUAL_AUDIT_NON_NOMINAL | 0–release | False | retained outside nominal baseline |
| ep108 | 108 | CLEAN_FULL_TASK | 0–706 | True | task motion begins at recording start; ends at persisted task release |
| ep109 | 109 | CLEAN_FULL_TASK | 0–620 | True | task motion begins at recording start; ends at persisted task release |
| ep113 | 113 | MANUAL_AUDIT_NON_NOMINAL | 0–release | False | retained outside nominal baseline |
| ep114 | 114 | MANUAL_AUDIT_NON_NOMINAL | 0–release | False | retained outside nominal baseline |
| ep116 | 116 | CLEAN_FULL_TASK | 0–556 | True | task motion begins at recording start; ends at persisted task release |
| ep117 | 117 | CLEAN_FULL_TASK | 0–698 | True | task motion begins at recording start; ends at persisted task release |

### Review conclusions

The human audit supersedes the earlier description of this cohort as 25 clean expert demonstrations.

Episodes 99 and 102 each contain two demonstrations. Exact derived boundaries use synchronized trigger/action state and visual reset evidence; no raw row or video is rewritten.

All trajectories outside the 16 nominal segments are retained with the generic manual_audit_non_nominal reason unless direct evidence supports a more specific label.

The nominal baseline contains only approach, grasp, lift/transport, place, and release trajectories accepted by the operator audit.
