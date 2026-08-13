# Physical bottle collection audit v2

This report was generated from the immutable raw episode tree. Video readability and canonical row/timestamp checks were performed offline; trajectory classifications and merged boundaries are explicit reviewed manifest decisions.

- Raw metadata records discovered: **54**
- Complete payload episodes inspected: **53**
- Logical trajectories with payload: **55**
- Audit manifest entries (including excluded no-payload records): **56**
- Clean full-task logical segments: **33**
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
| 99 | MERGED_MULTIPLE_EPISODES | payload_inspected | 2498 | 29.987990692193762 | 1.0 | 2498 @ 30.0 | 2498 @ 30.0 | two human-audited nominal demonstrations separated by reset |
| 102 | MERGED_MULTIPLE_EPISODES | payload_inspected | 2085 | 30.000000300000004 | 1.0 | 2085 @ 30.0 | 2085 @ 30.0 | first demonstration non-nominal; second human-audited nominal |
| 105 | MANUAL_AUDIT_NON_NOMINAL | payload_inspected | 857 | 29.964994465344226 | 1.0 | 857 @ 30.0 | 857 @ 30.0 | manual_audit_non_nominal |
| 108 | CLEAN_FULL_TASK | payload_inspected | 857 | 30.000000300000004 | 1.0 | 857 @ 30.0 | 857 @ 30.0 | human-audited nominal full task |
| 109 | CLEAN_FULL_TASK | payload_inspected | 771 | 30.0000003 | 1.0 | 771 @ 30.0 | 771 @ 30.0 | human-audited nominal full task |
| 113 | MANUAL_AUDIT_NON_NOMINAL | payload_inspected | 745 | 30.0000003 | 1.0 | 745 @ 30.0 | 745 @ 30.0 | manual_audit_non_nominal |
| 114 | MANUAL_AUDIT_NON_NOMINAL | payload_inspected | 746 | 30.000000300000004 | 1.0 | 746 @ 30.0 | 746 @ 30.0 | manual_audit_non_nominal |
| 116 | CLEAN_FULL_TASK | payload_inspected | 706 | 30.000000300000004 | 1.0 | 706 @ 30.0 | 706 @ 30.0 | human-audited nominal full task |
| 117 | CLEAN_FULL_TASK | payload_inspected | 848 | 30.000000300000004 | 1.0 | 848 @ 30.0 | 848 @ 30.0 | human-audited nominal full task |
| 121 | CLEAN_FULL_TASK | payload_inspected | 823 | 30.000000300000004 | 1.0 | 823 @ 30.0 | 823 @ 30.0 | operator accepted; offline task-boundary review passed |
| 122 | MANUAL_AUDIT_NON_NOMINAL | payload_inspected | 865 | 30.0000003 | 1.0 | 865 @ 30.0 | 865 @ 30.0 | operator_review_unusable |
| 123 | MANUAL_AUDIT_NON_NOMINAL | payload_inspected | 567 | 30.000000300000004 | 1.0 | 567 @ 30.0 | 567 @ 30.0 | operator_review_unusable |
| 124 | MANUAL_AUDIT_NON_NOMINAL | payload_inspected | 269 | 30.000000300000004 | 1.0 | 269 @ 30.0 | 269 @ 30.0 | operator_review_unusable |
| 125 | MANUAL_AUDIT_NON_NOMINAL | payload_inspected | 282 | 29.89361732021277 | 1.0 | 282 @ 30.0 | 282 @ 30.0 | operator_review_unusable |
| 126 | MANUAL_AUDIT_NON_NOMINAL | payload_inspected | 58 | 30.000000300000004 | 1.0 | 58 @ 30.0 | 58 @ 30.0 | operator_review_unusable |
| 127 | MANUAL_AUDIT_NON_NOMINAL | payload_inspected | 644 | 30.000000300000004 | 1.0 | 644 @ 30.0 | 644 @ 30.0 | operator_review_unusable |
| 128 | MANUAL_AUDIT_NON_NOMINAL | payload_inspected | 287 | 30.0000003 | 1.0 | 287 @ 30.0 | 287 @ 30.0 | operator_review_unusable |
| 129 | MANUAL_AUDIT_NON_NOMINAL | payload_inspected | 906 | 30.000000300000004 | 1.0 | 906 @ 30.0 | 906 @ 30.0 | operator_review_unusable |
| 130 | CORRUPT_OR_UNUSABLE | no_complete_payload | 0 | n/a | n/a | n/a @ n/a | n/a @ n/a | partial metadata only; no complete row/video payload |
| 131 | CLEAN_FULL_TASK | payload_inspected | 669 | 30.000000300000004 | 1.0 | 669 @ 30.0 | 669 @ 30.0 | operator accepted; offline task-boundary review passed |
| 132 | MANUAL_AUDIT_NON_NOMINAL | payload_inspected | 169 | 30.0000003 | 1.0 | 169 @ 30.0 | 169 @ 30.0 | operator_review_unusable |
| 133 | CLEAN_FULL_TASK | payload_inspected | 907 | 30.000000300000004 | 1.0 | 907 @ 30.0 | 907 @ 30.0 | operator accepted; offline task-boundary review passed |
| 134 | CLEAN_FULL_TASK | payload_inspected | 933 | 30.000000300000004 | 1.0 | 933 @ 30.0 | 933 @ 30.0 | operator accepted; offline task-boundary review passed |
| 135 | REVIEW_REQUIRED | payload_inspected | 977 | 30.000000300000004 | 1.0 | 977 @ 30.0 | 977 @ 30.0 | person visible during task interval; excluded from clean baseline |
| 136 | REVIEW_REQUIRED | payload_inspected | 731 | 30.000000300000004 | 1.0 | 731 @ 30.0 | 731 @ 30.0 | person prominently visible during task interval; excluded from clean baseline |
| 137 | CLEAN_FULL_TASK | payload_inspected | 785 | 30.000000300000004 | 1.0 | 785 @ 30.0 | 785 @ 30.0 | operator accepted; offline task-boundary review passed |
| 138 | CLEAN_FULL_TASK | payload_inspected | 756 | 30.0000003 | 1.0 | 756 @ 30.0 | 756 @ 30.0 | operator accepted; offline task-boundary review passed |
| 139 | CLEAN_FULL_TASK | payload_inspected | 898 | 30.000000300000004 | 1.0 | 898 @ 30.0 | 898 @ 30.0 | operator accepted; offline task-boundary review passed |
| 140 | CLEAN_FULL_TASK | payload_inspected | 835 | 30.000000300000004 | 1.0 | 835 @ 30.0 | 835 @ 30.0 | operator accepted; offline task-boundary review passed |
| 141 | CLEAN_FULL_TASK | payload_inspected | 710 | 30.0000003 | 1.0 | 710 @ 30.0 | 710 @ 30.0 | operator accepted; offline task-boundary review passed |
| 142 | CLEAN_FULL_TASK | payload_inspected | 738 | 30.000000300000004 | 1.0 | 738 @ 30.0 | 738 @ 30.0 | operator accepted; offline task-boundary review passed |
| 143 | CLEAN_FULL_TASK | payload_inspected | 718 | 30.000000300000004 | 1.0 | 718 @ 30.0 | 718 @ 30.0 | operator accepted; offline task-boundary review passed |
| 144 | CLEAN_FULL_TASK | payload_inspected | 1012 | 30.000000300000004 | 1.0 | 1012 @ 30.0 | 1012 @ 30.0 | operator accepted; offline task-boundary review passed |
| 145 | CLEAN_FULL_TASK | payload_inspected | 994 | 30.000000300000004 | 1.0 | 994 @ 30.0 | 994 @ 30.0 | operator accepted; offline task-boundary review passed |
| 146 | CLEAN_FULL_TASK | payload_inspected | 704 | 30.000000300000004 | 1.0 | 704 @ 30.0 | 704 @ 30.0 | operator accepted; offline task-boundary review passed |
| 147 | CLEAN_FULL_TASK | payload_inspected | 629 | 29.95230554594595 | 1.0 | 629 @ 30.0 | 629 @ 30.0 | operator accepted; offline task-boundary review passed |
| 148 | CLEAN_FULL_TASK | payload_inspected | 767 | 30.000000300000004 | 1.0 | 767 @ 30.0 | 767 @ 30.0 | operator accepted; offline task-boundary review passed |
| 149 | CLEAN_FULL_TASK | payload_inspected | 730 | 30.000000300000004 | 1.0 | 730 @ 30.0 | 730 @ 30.0 | operator accepted; offline task-boundary review passed |

## Logical trajectory decisions

| logical segment | source | class | source frame range | include | decision |
|---|---:|---|---|---|---|
| ep067 | 67 | CLEAN_FULL_TASK | 62–856 | True | nominal16 reviewed crop |
| ep070 | 70 | CLEAN_FULL_TASK | 14–698 | True | nominal16 reviewed crop |
| ep071 | 71 | MANUAL_AUDIT_NON_NOMINAL | 0–release | False |  |
| ep074 | 74 | MANUAL_AUDIT_NON_NOMINAL | 0–release | False |  |
| ep081 | 81 | CLEAN_FULL_TASK | 0–946 | True | nominal16 reviewed crop |
| ep083 | 83 | MANUAL_AUDIT_NON_NOMINAL | 0–release | False |  |
| ep087 | 87 | CLEAN_FULL_TASK | 0–788 | True | nominal16 reviewed crop |
| ep088 | 88 | CLEAN_FULL_TASK | 61–743 | True | nominal16 reviewed crop |
| ep089 | 89 | MANUAL_AUDIT_NON_NOMINAL | 0–release | False |  |
| ep090 | 90 | MANUAL_AUDIT_NON_NOMINAL | 0–release | False |  |
| ep091 | 91 | MANUAL_AUDIT_NON_NOMINAL | 0–release | False |  |
| ep092 | 92 | MANUAL_AUDIT_NON_NOMINAL | 0–release | False |  |
| ep095 | 95 | CLEAN_FULL_TASK | 0–674 | True | nominal16 reviewed crop |
| ep096 | 96 | CLEAN_FULL_TASK | 0–733 | True | nominal16 reviewed crop |
| ep097 | 97 | CLEAN_FULL_TASK | 29–868 | True | nominal16 reviewed crop |
| ep098 | 98 | CLEAN_FULL_TASK | 22–760 | True | nominal16 reviewed crop |
| ep099_a | 99 | CLEAN_FULL_TASK | 0–965 | True | first nominal task; reset interval excluded |
| ep099_b | 99 | CLEAN_FULL_TASK | 1335–2348 | True | second nominal task; reset interval excluded |
| ep102_a | 102 | MANUAL_AUDIT_NON_NOMINAL | 0–877 | False |  |
| ep102_b | 102 | CLEAN_FULL_TASK | 1209–1934 | True | second nominal task; first task and reset excluded |
| ep105 | 105 | MANUAL_AUDIT_NON_NOMINAL | 0–release | False |  |
| ep108 | 108 | CLEAN_FULL_TASK | 0–706 | True | nominal16 reviewed crop |
| ep109 | 109 | CLEAN_FULL_TASK | 0–620 | True | nominal16 reviewed crop |
| ep113 | 113 | MANUAL_AUDIT_NON_NOMINAL | 0–release | False |  |
| ep114 | 114 | MANUAL_AUDIT_NON_NOMINAL | 0–release | False |  |
| ep116 | 116 | CLEAN_FULL_TASK | 0–556 | True | nominal16 reviewed crop |
| ep117 | 117 | CLEAN_FULL_TASK | 0–698 | True | nominal16 reviewed crop |
| ep121 | 121 | CLEAN_FULL_TASK | 60–672 | True | complete approach through final task motion; pre-task hold and post-task reset tail excluded |
| ep122 | 122 | MANUAL_AUDIT_NON_NOMINAL | 0–release | False |  |
| ep123 | 123 | MANUAL_AUDIT_NON_NOMINAL | 0–release | False |  |
| ep124 | 124 | MANUAL_AUDIT_NON_NOMINAL | 0–release | False |  |
| ep125 | 125 | MANUAL_AUDIT_NON_NOMINAL | 0–release | False |  |
| ep126 | 126 | MANUAL_AUDIT_NON_NOMINAL | 0–release | False |  |
| ep127 | 127 | MANUAL_AUDIT_NON_NOMINAL | 0–release | False |  |
| ep128 | 128 | MANUAL_AUDIT_NON_NOMINAL | 0–release | False |  |
| ep129 | 129 | MANUAL_AUDIT_NON_NOMINAL | 0–release | False |  |
| ep130 | 130 | CORRUPT_OR_UNUSABLE | 0–release | False |  |
| ep131 | 131 | CLEAN_FULL_TASK | 0–516 | True | complete task; stationary/reset tail excluded |
| ep132 | 132 | MANUAL_AUDIT_NON_NOMINAL | 0–release | False |  |
| ep133 | 133 | CLEAN_FULL_TASK | 44–733 | True | complete task; pre-task hold and stationary/reset tail excluded |
| ep134 | 134 | CLEAN_FULL_TASK | 28–769 | True | complete task; pre-task hold and stationary/reset tail excluded |
| ep135 | 135 | REVIEW_REQUIRED | 0–release | False |  |
| ep136 | 136 | REVIEW_REQUIRED | 0–release | False |  |
| ep137 | 137 | CLEAN_FULL_TASK | 0–605 | True | complete task; stationary/reset tail excluded |
| ep138 | 138 | CLEAN_FULL_TASK | 0–605 | True | complete task; stationary/reset tail excluded |
| ep139 | 139 | CLEAN_FULL_TASK | 0–747 | True | complete task; stationary/reset tail excluded |
| ep140 | 140 | CLEAN_FULL_TASK | 0–684 | True | complete task; stationary/reset tail excluded |
| ep141 | 141 | CLEAN_FULL_TASK | 0–555 | True | complete task; stationary/reset tail excluded |
| ep142 | 142 | CLEAN_FULL_TASK | 0–587 | True | complete task; stationary/reset tail excluded |
| ep143 | 143 | CLEAN_FULL_TASK | 0–567 | True | complete task; stationary/reset tail excluded |
| ep144 | 144 | CLEAN_FULL_TASK | 0–761 | True | complete task; 3.33 s stationary pre-rotation tail excluded |
| ep145 | 145 | CLEAN_FULL_TASK | 0–780 | True | complete task; stationary/reset tail excluded |
| ep146 | 146 | CLEAN_FULL_TASK | 0–524 | True | complete task; stationary/reset tail excluded |
| ep147 | 147 | CLEAN_FULL_TASK | 0–452 | True | complete task; stationary/reset tail excluded |
| ep148 | 148 | CLEAN_FULL_TASK | 0–575 | True | complete task; stationary/reset tail excluded |
| ep149 | 149 | CLEAN_FULL_TASK | 1–460 | True | complete task begins after one setup row; stationary/reset tail excluded |

### Review conclusions

The nominal16 source decisions and corrected 99/102 segmentation are preserved unchanged.

Episodes 122-129 and 132 are excluded by the operator audit; episode 130 has partial metadata but no complete training payload.

Episodes 135 and 136 remain review-required because a person is visible during the actual task interval, so tail cropping cannot remove that visual nuisance without deleting task behavior.

New nominal tasks retain complete approach, grasp, lift/transport, place, release, and final robot motion, while excluding pre-task hold and post-task stationary/manual-reset intervals.

Raw rows and videos are never rewritten; every included row keeps its source episode, frame, and timestamp provenance.
