# Digital-Twin Measurement Request — after base fitting

This revision does not ask for values already supplied. Manufacturer-defined, user-confirmed, user-approximate, reconstruction-fitted, and geometrically derived values remain separate.

## Recorded values — do not remeasure now

| Parameter | Value | Status / source | Uncertainty or caveat |
|---|---:|---|---|
| Fixed JAKA base outer diameter | 124 mm | manufacturer-defined, transcribed from supplied drawing | drawing file is not yet archived in the repository |
| Mounting-hole PCD | 110 mm | manufacturer-defined | four 6.6 mm holes |
| Aluminium profile | 50×50 mm | user-confirmed | exact rail spacing is separate |
| Video 02 board inventory | one A3, two A4 | user-confirmed | A4 sheets reuse IDs and are spatial instances |
| P to named front table edge | about 312 mm | derived from raw 250 mm outer-edge distance plus 62 mm radius | valid only if the measurement line is radial and perpendicular to the edge; ±10 mm |
| P to named right table edge | about 145 mm | user-approximate | ±5–10 mm |
| Tabletop to P mounting plane | about 50 mm | user-approximate | ±2–5 mm |
| Table | about 1380×730×20 mm, 750 mm above floor | user-approximate | dimensions remain validation priors |

The fitted local model gives 122.86 mm for the fixed-base outer diameter and 75.22 mm for rail centreline spacing. The latter is 2.57 mm from the `110/sqrt(2)=77.782 mm` candidate and 34.78 mm from 110 mm. This supports, but does not prove, a 45-degree mounting-pattern orientation.

## Future robot calibration — not an integrated-workspace blocker

The P-frame MuJoCo workspace now runs without a calibrated `T_B_P`. The following evidence is still required before claiming final robot/world registration or Manipulation Ready, but it is no longer P0 for visualization or simulator-side workspace engineering.

### P0-1: archive the manufacturer drawing and identify its base axes

1. **Priority:** future robot calibration; blocks only verified `T_B_P`/`T_B_R` and Manipulation Ready.
2. **Parameter:** JAKA model-frame datum relative to the fixed mounting plane and hole pattern.
3. **Symbol:** `T_B_P`.
4. **Start landmark:** centre of the 110 mm mounting PCD on the lowest fixed mounting plane (`P`).
5. **End landmark:** manufacturer/model `jaka_Link_0` origin and its +x/+y directions (`B`).
6. **Direction:** full xyz and especially yaw about +z.
7. **Tool:** original installation drawing/CAD/PDF; do not estimate with a tape.
8. **Unit:** mm and degrees.
9. **Required tolerance:** use the drawing tolerance; engineering target ±1 mm and ±0.2°.
10. **Why:** the MJCF mesh proves only that a centred 123.79×123.86 mm mesh begins at B z=0; it does not document the manufacturer x/y datum.
11. **Identification:** page must show the fixed mounting plane, PCD and base-axis arrows/datum.
12. **Photo/file required:** yes—the actual source page, not a cropped dimension transcription.
13. **Mandatory:** yes for any B-frame calibration claim; no for the current P-world scene.
14. **Without it:** `T_P_R` remains usable provisionally, while `T_B_P` and `T_B_R` remain null.
15. **Repository/CAD:** not found in the current repository.

### P0-2: one annotated overhead installation photo

1. **Priority:** P1 validation improvement; the current operational table pose remains explicitly provisional.
2. **Parameter:** installed sign of P +x/+y, bolt-pattern rotation, and names of the measured table edges.
3. **Symbol:** `sign(P_x)`, `sign(P_y)`, `theta_holes_rails`.
4. **Start landmark:** PCD centre / fixed base centre.
5. **End landmark:** front transverse rail, named front table edge, and named right table edge.
6. **Direction:** mark arrows `P +x` and `P +y`; draw the original 250 mm and 145 mm measurement lines.
7. **Tool:** phone photo plus simple markup; camera should be as close to normal to the tabletop as practical.
8. **Unit:** no new numeric value is required in the photo.
9. **Required tolerance:** bolt centres should be identifiable within about 2 image pixels if visible.
10. **Why:** selects one of four table-pose mirror candidates and verifies whether `110/sqrt(2)` is the installed rail hypothesis.
11. **Identification:** use fixed bolt heads/mounting plate, not the rotating shell, white upper housing, or cables.
12. **Photo required:** yes; label the front transverse member and both table edges.
13. **Mandatory:** no for the first integrated scene; yes before final robot/world registration.
14. **Without it:** operational table collision remains provisional and mounting-pattern orientation remains provisional.
15. **Repository/CAD:** video 01 shows only two bolt heads safely enough for annotation.

### P0-3: rail centreline spacing (one measurement)

1. **Priority:** P1 collision-validation improvement for accepting the 45-degree installation hypothesis.
2. **Parameter:** perpendicular centreline distance of the two long 50 mm profiles.
3. **Symbol:** `d_rail_cc`.
4. **Start landmark:** centreline of rail 1 at an unobstructed common cross-section.
5. **End landmark:** centreline of rail 2 at the same cross-section.
6. **Direction:** perpendicular to the rail long axes.
7. **Tool:** calliper; alternatively measure the same-side outer faces because equal 50 mm profiles preserve centre spacing.
8. **Unit:** mm.
9. **Required tolerance:** ±1 mm.
10. **Why:** discriminates 77.782 mm from 110 mm and validates the sparse 75.22 mm fit.
11. **Identification:** use the two profiles directly below/supporting the fixed base, not the transverse member.
12. **Photo required:** include the tool/endpoints in P0-2 if practical.
13. **Mandatory:** no for visualization; yes before accepting the aluminium collision proxy for manipulation.
14. **Without it:** rail orientation remains `derived_candidate_requires_visual_verification`.
15. **Repository/CAD:** not found.

### P0-4: precise mounting-plane height, only for final acceptance

1. **Priority:** P1 for final vertical registration; the approximate value is sufficient for the current operational scene.
2. **Parameter:** tabletop top plane to the lowest fixed JAKA mounting plane.
3. **Symbol:** `h_table_P`.
4. **Start landmark:** tabletop top surface beside the mount.
5. **End landmark:** lowest fixed mounting plane that defines P.
6. **Direction:** tabletop normal.
7. **Tool:** calliper/height gauge and straightedge.
8. **Unit:** mm.
9. **Required tolerance:** ±1 mm.
10. **Why:** removes the current ±2–5 mm common-mode z uncertainty.
11. **Identification:** do not measure to the top of the 50 mm rail or rotating shell unless it is demonstrably the P plane.
12. **Photo required:** yes if the plane is hidden.
13. **Mandatory:** only before declaring final `T_P_R`.
14. **Without it:** provisional P alignment remains, but the sub-5 mm manipulation-zone target is not demonstrated.
15. **Repository/CAD:** the supplied approximate value is 50 mm.

See [table-frame convention](table_frame_convention.svg). No broad landmark survey or immediate table remeasurement is requested.

## P1 — recommended, not blocking this pass

- Identify the previously measured approximately 500 mm aluminium member in P0-2.
- If possible, provide a close-up where all four fixed mounting bolts are visible. This would permit a real 110 mm PCD residual instead of the current `MISSING` result.
- Remeasure table dimensions only if the final registered plane/edges fall outside the recorded ±10 mm dimension uncertainties.
- Continue to treat the repository `T_F_H` as physically unverified until adapter CAD or a flange/hand survey is supplied.
