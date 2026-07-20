# JAKA base fitting and provisional P registration

Date: 2026-07-15. This historical registration report records the accepted 01/02 evidence. The later integrated workspace uses P as the engineering/MuJoCo world and enables provisional primitive collisions; see `DIGITAL_TW.md` for current status.

## Evidence and provenance

- Video 02 board inventory is user-confirmed: `Board_A3_1`, `Board_A4_1`, and `Board_A4_2`. The A4 instances use identical IDs and are separated only by spatial clustering.
- Manufacturer dimensions transcribed from the supplied installation drawing: 124 mm fixed-base outer diameter, 110 mm four-hole PCD, four 6.6 mm holes, 5 mm × 5.5 mm auxiliary locating holes, shown 10±0.05 mm offset, and approximately 54±0.025 / 55±0.025 mm vertical offsets. The source image/PDF was not present in the repository, so unresolved drawing-label semantics remain explicitly null.
- User approximate installation values: raw 250±10 mm fixed-base outer edge to named table edge, conditionally derived 312±10 mm P centre to that edge, 145±7.5 mm to the named right edge, and 50±3.5 mm tabletop-to-P mounting plane.

## Formal P definition

P origin is the centre of the fixed 110 mm mounting-hole PCD on the lowest fixed JAKA mounting plane. +z is the mounting-plane normal upward. +x follows the two parallel aluminium rails from the robot toward the front transverse member. +y completes a right-handed frame. Rotating/upper shells, joint housing and cables are excluded datums.

The local fit uses the fixed 124 mm outer circle as a centre aid because only two bolt heads can be safely annotated. It is not a four-hole PCD fit.

## Model datum evidence and `T_B_P`

`jaka_Link_0` and `Link0.STL` both have identity MJCF transforms. The mesh bounds are approximately `[-61.981,+61.808] × [-61.930,+61.930] × [0,31.000]` mm: its 123.79×123.86 mm diameter and minimum z=0 strongly support coincident centre and mounting plane. This does not document the manufacturer/model +x/+y datum or installed yaw. Therefore:

- centre and z-plane relation: geometry-consistent;
- x/y orientation: unresolved;
- `T_B_P`: null/unresolved, not identity;
- `T_B_R`: null because B-frame composition is unresolved. This does not block the current P-world scene.

## Local primitive fit

The fit uses manual semantic ROIs and deterministic robust plane/circle/line fitting in reconstruction 01. The accepted provisional local scale used for thresholds is 0.105142887 m/R01-unit.

| Check | Result | Status |
|---|---:|---|
| tabletop plane | 15,826 inliers, 1.02 mm RMS | PASS provisional |
| fixed-base outer diameter | 122.862 mm vs 124 mm, −1.138 mm | PASS provisional/CAD-radius-gated |
| outer-circle radial fit | 115/325 inliers, 1.54 mm RMS | WARN manual ROI |
| mounting PCD | not fitted; two bolt centres only | MISSING |
| rail centreline spacing | 75.215 mm | PROVISIONAL |
| residual to 77.782 mm (`110/sqrt(2)`) | −2.566 mm | supports 45° hypothesis |
| residual to 110 mm | −34.785 mm | rejects direct-alignment hypothesis for current sparse fit |
| 50 mm profile width | not reliably fitted from groove-dominated points | MISSING |

The selected installed-hole interpretation is `bolt_columns_rotated_45deg_to_rails`, with moderate reconstruction support and required visual/physical verification. Annotated bolt centres in `frame_000000.jpg` are `(787,1061)` and `(792,1248)` pixels; no other two are claimed.

## 01 to 02 registration

Direct cross-video SIFT/3D matching without masks produced competing false transformations because all three ChArUco patterns repeat IDs/texture. Those failed results are retained. After expanding and masking every detected board polygon, three independent sampling seeds give scale 0.83238–0.83407 R02/R01. The selected provisional result is:

- `T_R02_R01` scale: 0.833052489;
- 177/897 RANSAC inliers;
- RMS: 4.13 mm using the earlier 02 ChArUco scale;
- maximum inlier error: 7.64 mm;
- status: accepted for local/global visualization and transferring base fits, not dense fusion.

## Metric scale reassessment

| Source | m/R02-unit | provenance |
|---|---:|---|
| A3 instance | 0.127115969 | confirmed board geometry; indirect sparse-corner association |
| A4 instance 1 | 0.123818798 | same pattern, spatial cluster |
| A4 instance 2 | 0.130486973 | same pattern, spatial cluster |
| 124 mm base diameter | 0.127383051 | manufacturer dimension plus radius-gated ROI fit |
| 77.782 mm rail candidate | 0.130520502 | diagnostic only; orientation not yet verified |
| 110 mm PCD | MISSING | four bolts unavailable |
| 50 mm profile width | MISSING | both profile faces unavailable |

The robust median of the three board instances plus the base source is **0.127249510 m/R02-unit**, with source-level robust sigma **0.002498926 (1.96%)**. The primary source span is 5.24%, exceeding the 1–2% guidance because the two A4 instances disagree. Status remains `PROVISIONAL_primary_sources_exceed_2_percent_span`; table dimensions are not primary scale inputs.

## Provisional `T_P_R`

Six saved correspondences use the fitted centre, four outer-circle cardinal constraints, and the tabletop plane. They are auditable but statistically correlated. RANSAC accepts all six:

- scale: 0.127242865921 m/R02-unit;
- translation (m): `[1.151801590, -0.424000326, 0.625415044]`;
- quaternion xyzw: `[-0.576801519, -0.630109946, 0.380216905, 0.354537119]`;
- numerical RMS / maximum: 0.162 / 0.335 mm.

The sub-millimetre residual is not independent calibration accuracy: five points share one circle fit, and the sixth uses the approximate 50 mm height. `T_P_R` is therefore stored as `provisional_correlated_primitive_fit_not_final_calibration`, with several-millimetre common-mode uncertainty.

## Table pose

The tabletop plane is known provisionally in P at z≈−50 mm; its centre plane is z≈−60 mm with the 20 mm thickness. Width is hypothesized along P x and length along P y. The 312/145 mm distances create four mirror candidates because “front/bottom” and “right” were image descriptions, not signed P axes. No candidate is selected and table collision remains disabled. See [table-frame convention](table_frame_convention.svg).

## Remaining future calibration evidence

1. Archive the actual manufacturer base drawing page and identify B +x/+y relative to the PCD/mounting plane.
2. Supply one annotated overhead installation photo labeling P +x, front transverse member, named front edge, named right edge, and the original measurement lines.
3. Measure rail centreline spacing to ±1 mm (or provide mounting-frame CAD).
4. Before final acceptance, measure tabletop-to-lowest-fixed-mounting-plane height to ±1 mm.

The current integrated scene uses only explicit provisional table/aluminium primitives for collision. No reconstruction-derived collision mesh is authorized.

## Verification executed

- Focused base/transform/scale/ChArUco tests: 21 passed.
- Full repository suite: 168 passed in 9.48 s.
- Digital-twin validator: `incomplete`, 30 PASS / 4 WARN / 17 MISSING. This is the expected state while `T_B_P`, `T_B_R`, PCD/profile checks, table mirror selection and camera calibration remain unresolved.
