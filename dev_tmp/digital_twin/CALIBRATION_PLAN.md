# Digital-Twin Camera Calibration Plan

All outputs use column vectors and `T_A_B` maps B into A. OpenCV/RealSense optical axes are +x right, +y down, +z forward. No calibration result is claimed in this document.

## Common target and acceptance protocol

The current captures use two verified print specifications from `digital_twin/configs/charuco_boards.yaml`: A3 with `squaresX=5`, `squaresY=7`, 50 mm squares and 37 mm markers; A4 with the same grid, 35 mm squares and 26 mm markers. Both use `DICT_4X4_50`, were printed at actual size, and their 100 mm verification bars were physically measured as 100 mm. Detection on the actual images confirms the 5-across/7-down orientation; reversing it to 7×5 produces no interpolated ChArUco corners. The 5×7 square grid has 24 internal corners, not 35 corners.

The two patterns reuse marker IDs 0–16, and video 02 contains three physical board-pattern instances. Marker IDs therefore do not identify A3 versus A4. Preserve paper size, physical placement context and reconstructed square-size consistency. Confirm whether the third instance is a second A4 copy. Keep every board flat; a later camera-calibration target should preferably be bonded to a rigid matte backing.

For intrinsic calibration, capture 30–50 sharp views spanning roughly 0.3–1.5 m (adjust so the board remains resolvable), yaw/pitch angles from approximately -45° to +45°, several roll angles, and board corners reaching every image quadrant and image edge. At least 20 accepted views after rejection should remain. Avoid many near-duplicate frontal views. Report per-view and overall error; prefer RMS reprojection below 1 px and investigate any view above 2 px. A low RMS alone does not validate metric scale.

Validate on 10+ held-out views and by measuring known 3D baselines not used in fitting. Repeat the complete capture after remount/power cycle where relevant. Report translation/rotation repeatability instead of silently averaging inconsistent calibrations.

## iPhone reconstruction camera I

- Use one shared `OPENCV` camera **within each video**, enabling focal length and distortion refinement while initially holding principal point refinement off. Do not force 01 and 02 to share a camera because orientation/crop differ and zoom/stabilization metadata is incomplete.
- EXIF/QuickTime focal/device metadata may initialize the solve if `ffprobe` later exposes it, but it is not a metric scale source. The current host cannot inspect those fields.
- Primary captures are `01.MOV` (about 22.33 s, raw 2160×3840, 90° rotation metadata) and `02.MOV` (about 47.00 s, raw 3840×2160). Both are HEVC at nominal ~30 fps. The working extraction is 3 fps and 1920 px maximum dimension with source timestamps. In 02, audit samples at 30.000 s and 31.500 s were rejected for low sharpness. The original video remains a separate, board-free supplementary visual source.
- Treat the phone as a likely rolling-shutter camera. COLMAP's standard model is global-shutter; reject rapid motion and inspect reprojection residuals/spatial bias. Re-capture slowly if residuals correlate with image rows or motion.
- Assess registered-image ratio, feature tracks, sparse-point reprojection error, coverage, camera trajectory continuity, focal/distortion plausibility, and visual ghosting. Do not proceed to GS/NeRF if poses are fragmented or unstable.
- Recover metric scale independently from A3 and A4 square geometry and require their residuals to agree before selecting a scale. Approximate table dimensions are validation priors, not the primary scale source.
- Register metric R to the visible physical frame P first. Resolve `T_B_P` from CAD/physical datums later, then compose `T_B_R = T_B_P × T_P_R`; do not estimate the invisible B origin by eye.
- I is an acquisition camera only. No final `T_B_I` is required unless individual phone poses are needed for rendering/debug; per-frame COLMAP poses can be composed with `T_B_R` when registered.

## Fixed external camera C_ext

The repository lists two D435 units: `side` serial 346222072985 and `top` serial 346522072675. Select one or explicitly calibrate both; never merge their intrinsics.

1. Lock the deployed resolution, stream, alignment mode, exposure policy and focus/lens configuration.
2. Calibrate color and depth intrinsics separately as needed; when depth is aligned to color, preserve the color optical-frame convention used by `vision_interface`.
3. For eye-to-hand calibration, rigidly fix the camera and board, then acquire 20–30 robot/target observations with broad translation, wrist orientation and depth diversity. Board normals should cover at least about ±35° yaw/pitch and multiple roll angles; avoid all poses on one plane or one wrist orientation.
4. The robot-side pose must refer to the verified flange/tool datum, not an arbitrary current TCP. Store the solved output specifically as `T_B_C_ext`.
5. Validate by projecting independently surveyed B-frame landmarks into held-out images and by transforming held-out RGB-D board clouds into B. Report actual reprojection, plane and 3D point errors.
6. Repeat the solution from another data subset and after a mount disturbance test. Prefer <1 px reprojection, ≤2 mm translation and ≤0.2° orientation repeatability; report actual values even if worse.
7. Import translation in metres and quaternion xyzw into `digital_twin/configs/transforms.yaml`; the simulator adapter must convert to MuJoCo wxyz.

## Wrist-mounted camera C_wrist

No wrist camera model or rigid mount was found. Perform this section only if hardware exists.

1. Verify that the mount is rigid and repeatable; identify whether F or H is the parent datum. Prefer F unless the camera is mechanically attached and surveyed to H.
2. Calibrate intrinsics with 30–50 diverse board views before hand-eye capture.
3. For eye-in-hand calibration, fix the board in B and acquire 25–40 robot poses spanning at least three distances, ±35° or more board-normal diversity, multiple roll angles, and broad image corner coverage. Avoid near-singular or purely rotational motion sets.
4. Solve and store `T_F_C_wrist` (or explicitly add `T_H_C_wrist`, never both without a consistency check).
5. Validate on held-out poses by board reprojection and 3D target position in B. Repeat after removing/remounting the camera.
6. Prefer <1 px reprojection and ≤1 mm/0.2° remount repeatability, but always report measured performance.
7. Import only after quaternion order, optical-axis convention and transform direction pass `validate_scene_alignment.py`.
