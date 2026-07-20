# 01/02 Capture, ChArUco and Sparse-Reconstruction Report

Date: 2026-07-15. This report records the first real reconstruction attempt; it does not claim B-frame registration or final collision geometry.

## Capture metadata and audit decisions

| Field | 01 | 02 |
|---|---:|---:|
| Exact file | `01.MOV` | `02.MOV` |
| Size | 64,549,569 bytes | 135,458,497 bytes |
| Duration | 22.333 s | 47.002 s |
| Stored dimensions | 2160×3840 | 3840×2160 |
| Rotation metadata | 90° | 0° |
| Nominal frame rate | 30.0 fps | 29.9989 fps |
| Codec | HEVC | HEVC |
| Audit samples | 45 at 2 fps | 94 at 2 fps |
| Audit accepted | 45 | 92 |
| Audit sharpness median | 1128.84 | 1331.34 |
| Reconstruction frames | 66/67 accepted | 136/141 accepted |

`ffprobe` remains unavailable. Average frame rate, VFR, audio, pixel/color metadata, device/creation metadata and location-metadata presence therefore remain null. No GPS coordinate was printed or stored, and derived JPEGs contain no copied QuickTime/EXIF metadata.

01 is the close-range base/rail model. 02 is the primary global workspace model. The robot and boards appear stationary within both sampled sequences, but this is visual evidence, not an instrumented motion measurement. The boards appear to retain their table placement across captures; exact inter-video rigidity has not been metrically verified. Cables remain a transient/occluding layer and must not become collision geometry.

The original `IMG_6607.MOV` has useful additional background coverage but no board and a changed scene setup. Do not include it in the primary metric workspace solve. Use it only as a separate supplementary rendering source after registration.

## Board verification

- Actual-image detection proves `squaresX=5`, `squaresY=7`; this gives 24 internal ChArUco corners and 17 markers. The reverse 7×5 configuration detects markers but interpolates zero ChArUco corners.
- Both sizes use `DICT_4X4_50` and the same marker layout/IDs 0–16. IDs do not distinguish physical boards.
- A3 is configured at 50 mm square / 37 mm marker. A4 is 35 mm / 26 mm. Both 100 mm print bars were user-verified at 100 mm.
- Video 02 contains three simultaneous complete patterns, now user-confirmed as one A3 and two A4 physical sheets. The A4 sheets remain distinguishable only by spatial/temporal clustering because their marker IDs are identical.
- No matching source PDF exists in the repository.

| Detection metric | 01 | 02 |
|---|---:|---:|
| Sampled frames at 3 fps | 67 | 141 |
| Frames with accepted detection | 53 | 131 |
| Accepted board observations | 72 | 320 |
| Full 24-corner observations | 24 | 145 |
| Observed marker IDs | 0–16 | 0–16 |
| Foreshortening proxy range | 0.600–0.998 | 0.436–0.975 |

Annotated frames and contact sheets are under `artifacts/digital_twin/charuco/{01,02}/`. No board pose was fabricated because independent iPhone intrinsics were unavailable; COLMAP self-calibration was used for reconstruction instead.

## COLMAP execution and acceptance

Native ARM64 COLMAP 3.9.1 was downloaded and extracted under `artifacts/digital_twin/runtime/colmap_apt/` without system installation or venv changes. CPU SIFT, one camera per video, `OPENCV`, sequential overlap 15, focal/extra-parameter refinement, fixed principal point and moderate 1920 px images were used. 02 additionally ran exhaustive matching for loop coverage.

The first 02 wrapper process received external SIGTERM immediately after exhaustive matching completed. An interactive mapper was then stopped at 68 images to avoid another session timeout, and the same database was mapped in a monitored detached process. The final mapper completed normally. Logs and execution-state records are preserved; this interruption is not hidden.

| Sparse metric | 01 selected model 0 | 02 selected model 1 |
|---|---:|---:|
| Extracted / registered | 66 / 66 | 136 / 136 |
| Registration ratio | 100% | 100% |
| Models produced | 1 | 2 (one selected; one rejected tiny component) |
| Sparse 3D points | 42,745 | 56,642 |
| Mean track length | 7.577 | 15.494 |
| Mean observations/registered image | 4,907.38 | 6,452.88 |
| Mean reprojection error | 0.921 px | 0.978 px |
| Median reprojection error | 0.742 px | 0.851 px |
| First/last-window shared points | 1,113 | 560 |
| Camera model/resolution | OPENCV 1080×1920 | OPENCV 1920×1080 |
| Estimated `fx, fy` | 1606.42, 1606.91 | 1582.87, 1585.59 |

The 01/02 focal estimates differ by about 1.3–1.5% at the common downscale, which is broadly consistent, but they remain separate camera objects because orientation/crop/stabilization metadata is incomplete. 02 model 0 has only 2 images and 205 points with implausible intrinsics; it is rejected. Main-model camera trajectories are continuous over every accepted extraction frame and contain first/last sparse overlap, providing loop-closure evidence.

Sparse visual review shows the tabletop, table edges/frame, parallel/transverse aluminium members, ChArUco regions and JAKA base region. It does not prove physical dimensions or semantic segmentation. Debug 3D/frustum/PCA top/side images are under each reconstruction's `visualizations/`; PCA views are explicitly not B-frame top/side views. Board overlays provide the reliable board-region visualization. No dense reconstruction was run.

## Metric-scale attempt

The scale extractor associates ChArUco corners to nearby triangulated SIFT observations, removes catastrophic associations, clusters reconstructed square sizes, and estimates A3/A4 independently. For 01, 19 board observations produce a reconstructed large/small ratio 1.204 rather than expected 1.429; no scale reference is emitted and 01 scale is rejected.

For 02 at the default 3 px association threshold:

- 196 board observations passed minimum sparse-association checks;
- 17 catastrophic association outliers were removed;
- reconstructed A3/A4 square cluster ratio: 1.4242; expected: 1.4286;
- A3 scale: **0.127116 m/R-unit** from 74 observations (71 robust inliers);
- A4 scale: **0.125684 m/R-unit** from 105 observations (104 robust inliers);
- relative A3/A4 difference: **1.133%**, within the preliminary 2% agreement threshold;
- combined candidate: **0.126214 m/R-unit**;
- square-length residual RMS: **3.23 mm**; maximum: **7.60 mm**;
- scale-dispersion indicator: **0.01269 m/R-unit** (about 10% of the candidate).

Sensitivity is not yet strong: a tighter 2.5 px association produces A3 0.129552 and A4 0.122257 m/R-unit, a 5.79% material disagreement. Thresholds of 1–2 px do not retain enough adjacent sparse-point pairs. The subsequent base-registration pass spatially separated both A4 instances and added CAD/base checks; see `BASE_REGISTRATION_REPORT.md` for the newer provisional scale and `T_P_R`. `T_B_R` remains null.

## Reconstruction grouping decision

- Keep 02 as the global reconstruction and 01 as a higher-detail local base reconstruction.
- Do not run a joint COLMAP solve in this pass. Both separate models register 100%, while repeated marker IDs already create a rejected tiny 02 component and make exhaustive cross-video pairing riskier. A joint solve would also require separate camera objects and adds no current acceptance benefit.
- Later align/merge the metric 01 local model to 02 using fixed rail/base/board geometry, with residual reporting. Do not merge arbitrary R coordinates by eye.
- Keep the original video separate and rendering-only unless stable cross-session registration is demonstrated.

## Frame P and transform status

P is now formally defined at the fixed 110 mm mounting-PCD centre on the lowest fixed mounting plane; +z is upward, +x follows the rails toward the front transverse member, and +y is right-handed. The 124 mm outer circle is only a fitted centre aid.

- `T_B_P`: unresolved/null; no CAD/manufacturer datum proves P≡B.
- `T_P_R`: numerical but provisional/correlated; see `artifacts/digital_twin/calibration/T_P_R.json`.
- `T_B_R`: unresolved derived transform, `T_B_R = T_B_P × T_P_R`.
- Scale: a weak 02 candidate exists as above, but no transform field was silently populated.

See `MEASUREMENT_REQUEST.md` and its P-frame sketch for the minimal remaining measurements.
