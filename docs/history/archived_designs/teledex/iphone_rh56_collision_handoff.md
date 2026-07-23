# RH56DFX Collision Project Handoff

## Quick Start for New Codex Session

Start here:

1. `cd /home/thor/projects/embodied_lab`
2. Confirm the branch and dirty state:
   ```bash
   git branch --show-current
   git status --short
   ```
   Integration branch at handoff: `integration/rh56-visual-coacd-default`. The worktree also contains unrelated camera, vision, recorder, documentation, and configuration work. Do not revert unrelated changes.
3. Run the focused collision tests:
   ```bash
   .venv/bin/python -m pytest tests/test_rh56_visual_coacd_default_asset.py tests/test_correll_rh56dfx_assets.py tests/test_mujoco_rh56_collision_modes.py tests/test_rh56_visual_coacd_stage2.py
   ```
   Current result at handoff: `33 passed`.
4. Reproduce the Stage 1 audit:
   ```bash
   .venv/bin/python tools/audit_rh56_visual_coacd_stage1.py \
     --out-dir /tmp/rh56_visual_coacd_stage1 \
     --independent-samples 32 \
     --actuator-samples 64 \
     --max-contact-rows 80
   ```
5. Do not regenerate, retune, or modify `data/sim_assets/meshes/rh56_collision_visual_coacd/`; the current 148 hulls are the selected default baseline.
6. Do not add collision exclusions just to hide thumb/index contacts.
7. Stage 2 should implement dynamic trajectory validation using actuator commands and `mujoco.mj_step`; teleport/qpos sampling is diagnostic only.

## Project Overview

`embodied_lab` is a rebuilding workspace for a `JAKA mini2 + Inspire RH56` manipulation stack. It contains real robot bring-up, teleoperation, data recording, MuJoCo/ManiSkill assets, Correll RH56DFX reference assets, and RH56 pregrasp tooling.

The RH56 collision validation project selected the current 148-hull `visual_coacd` set as the default MuJoCo collision baseline. The completed collision path is:

```text
vendor RH56 visual STL per rigid link
  -> CoACD convex decomposition per link
  -> 148 separate convex STL collision components
  -> runtime MuJoCo injection as visual_coacd collision mode
  -> Stage 1 static classification and audit
  -> Stage 2A dynamic trajectory validation and manual root-cause review
  -> selected fixed default runtime baseline
```

Collision geometry work is closed for the current baseline. Do not regenerate CoACD, edit hulls, change actuator mappings, or change the seven exclusions. Stage 2A evidence remains available for diagnostics; it is not a Stage 2B grasp-success CI gate.

## Current Repository Status

Current integration branch: `integration/rh56-visual-coacd-default`.

Default runtime assets:

- `data/sim_assets/jaka_rh56_visual_coacd.xml`: committed, directly compilable runtime model.
- `data/sim_assets/jaka_rh56_visual_coacd.manifest.json`: baseline identity, geometry policy, exclusions, and SHA-256 checksums.
- `tools/build_rh56_visual_coacd_runtime_asset.py`: deterministic XML/manifest derivation and `--check`; it does not run CoACD.
- `data/sim_assets/jaka_rh56.xml`: preserved derivation source for isolated comparison modes, not the ordinary runtime default.

Ordinary ManiSkill, IK shadow, teleop shadow, pregrasp, benchmark, readiness, and debug defaults now select `visual_coacd`. Explicit `correll_mesh`, `unifuc_pad_proxy`, and legacy diagnostic modes remain available and are derived from `jaka_rh56.xml`.

Stage 2A files changed in the current session:

- `PROJECT_HANDOFF.md`
- `src/sim_maniskill/rh56_collision.py`
- `src/sim_maniskill/rh56_collision_validation.py`
- `src/sim_maniskill/rh56_collision_diagnostics.py` (new)
- `tools/mujoco_rh56_grasp_benchmark.py`
- `tools/view_mujoco_rh56_pose_contact.py`
- `tools/validate_rh56_visual_coacd_stage2.py`
- `tools/investigate_rh56_visual_coacd_stage2a.py` (new)
- `tests/test_mujoco_rh56_collision_modes.py`
- `tests/test_rh56_visual_coacd_stage2.py`

The worktree also contains unrelated camera/vision/recorder/configuration changes that appeared during this session. Treat them as user or concurrent work and do not revert them. No collision geometry, vendor visual mesh, exclusion, actuator mapping, joint limit, or `data/sim_assets/jaka_rh56.xml` change was made during the focused investigation.

## Current Architecture

Key modules:

- `src/sim_maniskill/rh56_collision.py`
  - Runtime patching for RH56 collision modes.
  - Contains Correll mesh injection, analytic proxy injection, `visual_coacd` injection, and reviewed internal exclusions.
- `tools/generate_rh56_visual_coacd_collision.py`
  - Offline generator from visual STL to CoACD convex parts.
  - Do not rerun during Stage 2 unless explicitly directed.
- `tools/audit_rh56_visual_coacd_stage1.py`
  - Stage 1 audit for transform alignment and self-contact classification.
- `tools/mujoco_rh56_grasp_benchmark.py`
  - MuJoCo grasp benchmark.
  - Now uses semantic contact groups instead of requiring `thumb_pad_proxy`.
- `tools/check_mujoco_rh56_collision_modes.py`
  - Static pose/contact comparison across collision modes.
- `tools/view_mujoco_rh56_pose_contact.py`
  - Viewer/derived XML builder for RH56 pose/contact inspection.
- `src/sim_maniskill/rh56_collision_diagnostics.py`
  - Static same-qpos mode comparison, CoACD manifest resolution, and transformed vendor visual-mesh intersection/proximity diagnostics.
- `tools/investigate_rh56_visual_coacd_stage2a.py`
  - Focused deterministic root-cause investigation for `sim_best_pinch` + `iterative_incremental`.
- `data/sim_assets/jaka_rh56.xml`
  - Mounted JAKA+RH56 derivation source and comparison-mode integration anchor.
- `data/sim_assets/jaka_rh56_visual_coacd.xml`
  - Default mounted runtime model with only 13 disabled vendor visual geoms and 148 active CoACD hulls on RH56 bodies.
- `data/sim_assets/correll_rh56dfx/`
  - Imported Correll RH56DFX reference assets, including floating FK and force/torque scenes.

There is no RH56 URDF/xacro found in the repository at this handoff. The active integration target is MuJoCo XML.

## Current Collision Pipeline

Source visual meshes:

- `data/sim_assets/meshes/rh56/R_hand_base_link.STL`
- `data/sim_assets/meshes/rh56/R_thumb_proximal_base.STL`
- `data/sim_assets/meshes/rh56/R_thumb_proximal.STL`
- `data/sim_assets/meshes/rh56/R_thumb_intermediate.STL`
- `data/sim_assets/meshes/rh56/R_thumb_distal.STL`
- `data/sim_assets/meshes/rh56/R_index_proximal.STL`
- `data/sim_assets/meshes/rh56/R_index_distal.STL`
- `data/sim_assets/meshes/rh56/R_middle_proximal.STL`
- `data/sim_assets/meshes/rh56/R_middle_distal.STL`
- `data/sim_assets/meshes/rh56/R_ring_proximal.STL`
- `data/sim_assets/meshes/rh56/R_ring_distal.STL`
- `data/sim_assets/meshes/rh56/R_pinky_proximal.STL`
- `data/sim_assets/meshes/rh56/R_pinky_distal.STL`

Generated collision meshes:

- Directory: `data/sim_assets/meshes/rh56_collision_visual_coacd/`
- Manifest: `data/sim_assets/meshes/rh56_collision_visual_coacd/manifest.json`
- Hull files: `148`
- Body count: `13`
- Total collision mesh faces: `81898`
- Directory size at handoff: about `4.3M`

Runtime mode:

- Collision mode name: `visual_coacd`
- Implemented by `patch_rh56_visual_coacd_collision_model` in `src/sim_maniskill/rh56_collision.py`.
- Selected default RH56 collision baseline: the current unchanged set of 148 CoACD hulls, subject only to later human-reviewed local hull refinement.
- The dedicated runtime derivation contains exactly 13 collision-disabled vendor visual geoms and 148 active CoACD geoms on RH56 bodies. Legacy analytic/proxy and Correll geoms are removed rather than retained disabled, and unreferenced Correll mesh declarations are pruned from that derived XML.
- Review-only legacy/Correll overlays are opt-in in `tools/view_mujoco_rh56_pose_contact.py`; they are renamed `review_disabled_*`, assigned to group 4, and remain collision-disabled.
- Available collision modes in `tools/mujoco_rh56_grasp_benchmark.py`:
  - `correll_mesh`
  - `visual_coacd`
  - `proxy`
  - `mesh`
  - `mesh_proxy`
  - `unifuc_pad_proxy`

## Completed Milestones

Stage 0 completed:

- Imported/identified current RH56 visual STL assets.
- Imported Correll RH56DFX reference assets under `data/sim_assets/correll_rh56dfx/`.
- Added Correll asset tests and reference adapter code under `src/pregrasp/correll_rh56dfx.py`.

Stage 1 completed:

- Generated one CoACD collision set from existing visual STL files.
- Added `visual_coacd` collision mode.
- Fixed body-local transform inheritance for `visual_coacd` geoms.
- Verified `rh56_R_hand_base_link` CoACD geoms inherit the non-identity visual `pos/quat`.
- Verified legacy analytic/proxy and Correll collision geoms are absent when `visual_coacd` is active.
- Added reviewed internal structural exclusions.
- Avoided blanket parent-child exclusion.
- Rewrote benchmark contact anchoring to semantic contact groups, removing hard dependency on `thumb_pad_proxy`.
- Added Stage 1 audit script.
- Added regression tests for exact runtime geometry inventory, transform inheritance, Correll asset pruning, unchanged hand mass/inertia, review-overlay opt-in behavior, and reviewed exclusions.

## Important Implementation Decisions

- Do not regenerate or retune the 148 CoACD hulls during Stage 2.
- Do not treat successful CoACD generation as completion.
- Do not hide contacts by broad exclusions.
- Keep thumb/index, finger/finger, and finger/palm contacts active.
- Exclude only explicitly reviewed internal structural contacts.
- Separate independent XML joint-limit sampling from actuator/synergy-reachable sampling.
- Treat teleport/qpos sampling as diagnostic only.
- Stage 2 must validate dynamic trajectories with actuator commands and MuJoCo stepping.
- `data/sim_assets/jaka_rh56.xml` remains the mounted integration anchor; do not replace it casually.
- Correll assets remain reference/planning assets and a comparison collision mode, not the final mounted hand truth.
- The current 148-hull set is the default runtime collision baseline. Do not alter individual hulls without a later local refinement backed by human review and the Stage 2 diagnostics.

## Current Reviewed Exclusion Policy

Reviewed internal structural exclusions currently in `src/sim_maniskill/rh56_collision.py`:

- `rh56_R_thumb_proximal_base` <-> `rh56_R_thumb_proximal`
- `rh56_R_thumb_proximal` <-> `rh56_R_thumb_intermediate`
- `rh56_R_thumb_intermediate` <-> `rh56_R_thumb_distal`
- `rh56_R_index_proximal` <-> `rh56_R_index_distal`
- `rh56_R_middle_proximal` <-> `rh56_R_middle_distal`
- `rh56_R_ring_proximal` <-> `rh56_R_ring_distal`
- `rh56_R_pinky_proximal` <-> `rh56_R_pinky_distal`

Policy:

- Do not exclude thumb/index contacts.
- Do not exclude finger/finger contacts.
- Do not exclude finger/palm contacts.
- Do not exclude palm/proximal finger parent-child pairs.
- Any new exclusion must be justified as hidden/internal structural geometry and documented here.

Current derived `visual_coacd` model has `nexclude = 7`.

## Current Validation Status

Stage 2A infrastructure status as of 2026-07-13:

- Added `src/sim_maniskill/rh56_collision_validation.py`.
  - Provides reusable MuJoCo contact classification by body/geom names, hand semantic regions, reviewed internal pairs, and severity.
  - Distinguishes fingertip/pad contact, proximal/dorsal structural contact, internal/joint-region contact, finger/palm, hand/object, hand/table, arm contact, object/table, reviewed excluded internal pairs, and unknown/unreviewed pairs.
  - Does not classify all thumb/index contact as forbidden.
  - Drives RH56 position actuators over time with repeated `mujoco.mj_step`; direct `qpos` writes are limited to reset/initial-state setup.
  - Records controls, measured actuator joint qpos, qpos/qvel, target error, contact bodies/geoms, semantic regions, contact distance/normal/position, contact force, constraint force, actuator force, command sequence, events, and representative states.
  - Implements `slow_validation`, `nominal`, `hybrid`, and `stress` command profiles. The repository exposes RH56 raw hardware speed units and defaults, but no conversion from raw speed units to radians/sec; the `nominal` profile is explicitly inferred from the teleop software policy of `delta_limit=0.05` at `command_hz=15`.
  - `hybrid` now uses nominal free-space speed and slows near the target or after relevant RH56 contact; the known JAKA Link0/Link1 contact no longer triggers hand slowdown.
  - Iterative stages now reach and hold their own stage target before advancing. The earlier runner incorrectly tested intermediate-stage completion against the final target.
  - Contact persistence is consecutive and resets on separation; multiple geom contacts for one body pair no longer multiply persistence time.
  - Saved dynamic contacts are synchronized to the saved post-step qpos with `mj_forward` after each required `mj_step`.
  - Reports `slow_progress`, timeout without blockage, stable blockage, reached, and numerical instability separately.
- Added `src/sim_maniskill/rh56_collision_diagnostics.py`.
  - Replays saved qpos independently in all three modes for static collision-representation comparison only.
  - Resolves generated geom names to exact manifest IDs and convex-part STL files.
  - Loads the original vendor STL meshes and applies mesh scale, geom-local transform, and saved body-world transform.
  - Uses a deterministic BVH triangle-SAT/distance fallback because `python-fcl` and `rtree` are unavailable; every result reports this limitation.
  - Confirms transformed raw STL vertices align with MuJoCo's compiled vendor mesh reconstruction within about `9.5e-10 m` maximum.
- Added `tools/validate_rh56_visual_coacd_stage2.py`.
  - Builds identical dynamic trajectory runs for `visual_coacd`, `correll_mesh`, and `unifuc_pad_proxy`.
  - Supports simultaneous, thumb-first, finger-first, and iterative incremental command orders.
  - Emits per-run `summary.json`, `samples.csv`, `contacts.csv`, and `plots.svg`, plus an aggregate `stage2a_summary.json`.
  - Emits aggregate `mode_comparisons` with candidate classifications for CoACD-only premature/outward contact, collision tunnelling/missed collision, shared path/kinematic interpenetration, transient grazing, and inconclusive cases.
  - Supports an object-present smoke path, but the deterministic object placement is preliminary and not a grasp-success conclusion.
- Added `tests/test_rh56_visual_coacd_stage2.py`.
  - Also covers same-qpos cross-evaluation, exact manifest-part identification, conservative focused classification, and visual-mesh transform composition.

Focused tests after Stage 2A infrastructure:

```bash
.venv/bin/python -m pytest tests/test_correll_rh56dfx_assets.py tests/test_mujoco_rh56_collision_modes.py tests/test_rh56_visual_coacd_stage2.py
```

Current result:

- `21 passed`

### Focused `sim_best_pinch` Root-Cause Result

Focused command:

```bash
.venv/bin/python tools/investigate_rh56_visual_coacd_stage2a.py \
  --out-dir /tmp/rh56_visual_coacd_stage2a_focused_final \
  --profiles slow_validation nominal hybrid \
  --timeout-scale 2 \
  --seed 0
```

Conservative classification:

- `shared_visual_or_kinematic_intersection`
- Do **not** promote the earlier `coacd_only_premature_contact_or_outward_approximation_candidate` to `confirmed_coacd_outward_approximation`.

Dynamic outcomes:

| Profile | `visual_coacd` | `correll_mesh` | `unifuc_pad_proxy` |
|---|---|---|---|
| `slow_validation` | blocked at `7.168 s` | reached at `6.520 s` | reached at `6.488 s` |
| `nominal` | blocked at `4.232 s` | reached at `3.168 s` | reached at `3.150 s` |
| `hybrid` | blocked at `4.778 s` | reached at `3.504 s` | reached at `3.602 s` |

No focused run timed out, was classified as slow progress, or reported numerical instability. The nominal `visual_coacd` trajectory was repeated independently and matched exactly: identical sample count, outcome, contact/blockage times, aligned qpos, and controls.

First behavioral divergence:

- All three profiles remain dynamically identical or materially aligned until `visual_coacd` first reports thumb distal/index distal contact during `iterative_3`.
- First contact times are `4.586 s` slow, `2.144 s` nominal, and `2.374 s` hybrid.
- At those exact qpos, both reference collision modes remain thumb/index contact-free.
- The original vendor visual meshes are separated only `0.0052-0.0204 mm` one `2 ms` step before CoACD contact, then intersect at the CoACD first-contact state in every profile.
- Therefore the differential is not explained by insufficient timeout, hybrid slowdown, numerical instability, or dynamic divergence before contact. The reference collision modes permit continued motion after the original vendor visuals begin intersecting.

Exact CoACD components:

- First contact in every profile: `rh56_R_thumb_distal:000` (`R_thumb_distal_part000.stl`) with `rh56_R_index_distal:001` (`R_index_distal_part001.stl`).
- Maximum penetration in every profile: `rh56_R_thumb_distal:001` (`R_thumb_distal_part001.stl`) with `rh56_R_index_distal:001`.
- Maximum thumb/index CoACD penetration is about `1.3251 mm`; final target error is about `0.200769` actuator units.
- The CoACD first-contact point is locally about `0.15-0.17 mm` from the original thumb visual surface and `0.002-0.015 mm` from the index visual surface, so local outward approximation may affect the reported point/timing. It is not the confirmed root cause because the original meshes already intersect at the same saved qpos.

Focused artifacts:

- Markdown report: `/tmp/rh56_visual_coacd_stage2a_focused_final/root_cause_report.md`
- Focused JSON: `/tmp/rh56_visual_coacd_stage2a_focused_final/focused_root_cause_report.json`
- Same-qpos replay: `/tmp/rh56_visual_coacd_stage2a_focused_final/same_qpos_cross_evaluation.json`
- Visual diagnostics: `/tmp/rh56_visual_coacd_stage2a_focused_final/visual_mesh_diagnostics.json`
- Component aggregation: `/tmp/rh56_visual_coacd_stage2a_focused_final/coacd_component_contacts.json`
- Per-mode CSV/JSON/SVG: `/tmp/rh56_visual_coacd_stage2a_focused_final/trajectories/`
- Aligned plots: `/tmp/rh56_visual_coacd_stage2a_focused_final/plots/`

### Stage 2A Controlled Closeout and Object-Mediated Investigation

Closeout command:

```bash
.venv/bin/python tools/investigate_rh56_visual_coacd_stage2a.py \
  --out-dir /tmp/rh56_visual_coacd_stage2a_closeout_final \
  --profiles slow_validation nominal hybrid \
  --timeout-scale 2 \
  --seed 0 \
  --contact-hold-seconds 1.0
```

Comparison semantics:

- Root cause remains `shared_visual_or_kinematic_intersection`; this is not a confirmed CoACD defect.
- Representation-level interpretation is `visual_consistent_blocking_reference_modes_permissive` for all three profiles.
- General comparisons without original visual evidence now remain `inconclusive`; a coarse reference mode reaching is not treated as ground truth.
- `sim_best_pinch` is `diagnostic_only` in `configs/sim/rh56_stage2_trajectories.yaml`. Repository usage does not establish whether the full command is a free-space or object-required target.
- Empty-hand distal contact is `terminal_contact_candidate_unresolved_target_semantics`, never successful free-space reach.

Controlled hold-after-contact results:

| Profile | Continuous max | Hold max | Hold settled after 1 s | Settled normal force |
|---|---:|---:|---:|---:|
| `slow_validation` | 1.3251 mm | 0.1644 mm | 0.0822 mm | 3.7817 |
| `nominal` | 1.3251 mm | 0.3088 mm | 0.0901 mm | 3.7496 |
| `hybrid` | 1.3251 mm | 0.2530 mm | 0.0901 mm | 3.7496 |

In every profile, penetration increased while the controller continued pushing. Holding the actuator target at first persistent distal contact reduced maximum penetration by more than 75% and settled near 0.09 mm. The approximately 1.3251 mm maximum is therefore primarily post-contact continued actuator loading plus soft contact response, not initial collision overlap alone. This is diagnostic; production control was not changed.

Object-mediated `foam_cube` scenario:

- Uses the repository's 36 mm, 18 g foam-cube specification and recorded `pinch_grasp_box_v2` arm pose.
- The free cube rests on a table. Its deterministic center is the midpoint of transformed original vendor thumb/index closest points at the saved pre-contact hand state.
- All nine runs established bilateral thumb/object and index/object contact before thumb/index self-contact; none prevented later self-contact.
- No forbidden structural contact or numerical instability occurred.
- `visual_coacd` blocked in all profiles with about 1.3252 mm maximum self-penetration.
- `correll_mesh` reached in all profiles while allowing about 0.5584-0.6366 mm self-penetration.
- `unifuc_pad_proxy` reached in all profiles while allowing about 3.9770-4.0137 mm self-penetration.
- Bilateral-contact duration ranged from 0.056-0.214 s for `visual_coacd`, 0.112-0.168 s for `correll_mesh`, and 0.752-1.432 s for `unifuc_pad_proxy`.
- Cube displacement was about 36.8-55.6 mm. Stable grasp or lift retention is not established.
- Every object-mediated result is `bilateral_object_contact_then_thumb_index_self_contact`; there is currently no successful object-mediated closeout case.

Stage 2A gate readiness:

- Candidate later Stage 2B gates: manifest schema validity, deterministic replay/state validity, prominent unknown/unreviewed pair reporting, and forbidden structural-contact detection.
- Must remain report-only: canonical target reachability, empty-hand distal terminal-contact expectations, object retention/grasp success, penetration and force thresholds, and representation-comparison outcomes.
- No broad Stage 2B thresholds or grasp-success gates were added.

Closeout artifacts:

- Markdown: `/tmp/rh56_visual_coacd_stage2a_closeout_final/stage2a_closeout_report.md`
- JSON: `/tmp/rh56_visual_coacd_stage2a_closeout_final/stage2a_closeout_report.json`
- Post-contact response: `/tmp/rh56_visual_coacd_stage2a_closeout_final/post_contact_response.json`
- Object outcomes: `/tmp/rh56_visual_coacd_stage2a_closeout_final/object_mediated_outcomes.json`
- Object visual diagnostics: `/tmp/rh56_visual_coacd_stage2a_closeout_final/object_visual_diagnostics.json`
- Empty-hand CSV/JSON/SVG: `/tmp/rh56_visual_coacd_stage2a_closeout_final/trajectories/`
- Hold-variant CSV/JSON/SVG: `/tmp/rh56_visual_coacd_stage2a_closeout_final/contact_response_hold/`
- Object CSV/JSON/SVG: `/tmp/rh56_visual_coacd_stage2a_closeout_final/object_trajectories/`
- Aligned plots: `/tmp/rh56_visual_coacd_stage2a_closeout_final/plots/`

Focused collision tests after closeout:

```bash
.venv/bin/python -m pytest -q \
  tests/test_rh56_visual_coacd_stage2.py \
  tests/test_correll_rh56dfx_assets.py \
  tests/test_mujoco_rh56_collision_modes.py
```

Current result: `26 passed`.

The shortened smoke below predates the stage-local completion and contact-state synchronization fixes. Keep it as historical context, not as the authoritative focused result.

Stage 2A deterministic object-free smoke command:

```bash
.venv/bin/python tools/validate_rh56_visual_coacd_stage2.py \
  --out-dir /tmp/rh56_visual_coacd_stage2a_codex_smoke \
  --collision-modes visual_coacd correll_mesh unifuc_pad_proxy \
  --targets sim_best_pinch power_close \
  --strategies simultaneous thumb_first finger_first iterative_incremental \
  --profile nominal \
  --timeout-scale 0.5 \
  --sample-stride 20
```

Observed smoke behavior:

- `visual_coacd` did not reach `sim_best_pinch` or `power_close` in this shortened nominal smoke.
  - `sim_best_pinch`: simultaneous/thumb-first/finger-first timed out without stable blockage classification; iterative incremental blocked on thumb distal/index distal and is an expected path-obstruction candidate.
  - `power_close`: simultaneous/thumb-first timed out; finger-first blocked on thumb proximal/index distal and is currently classified as persistent mechanical blockage; iterative incremental blocked on thumb intermediate/index distal and is an expected path-obstruction candidate.
  - RH56 self-penetration in this smoke was about `0.00031 m` to `0.00193 m` depending on target/order.
- `correll_mesh` reached `sim_best_pinch` under all four command orders in this smoke, but blocked on `power_close` with thumb intermediate/index distal path-obstruction candidates.
- `unifuc_pad_proxy` reached both `sim_best_pinch` and `power_close` under all four command orders in this smoke.
- The new aggregate mode comparison classified `visual_coacd` `sim_best_pinch` + `iterative_incremental` as a `coacd_only_premature_contact_or_outward_approximation_candidate` because both reference modes reached the identical command sequence while `visual_coacd` blocked. Other shortened-smoke comparisons remained inconclusive or path-obstruction candidates.
- The global max penetration column still includes the known mounted JAKA Link0/Link1 contact, so Stage 2A reports `max_rh56_self_penetration_m` separately.
- No conclusion is made yet that `visual_coacd` is defective. The current smoke shows thumb/index behavior is path- and collision-mode-dependent and requires deeper review with full-duration slow validation, object-present validation, and visual-mesh intersection diagnostics.

Stage 2A deterministic object-present smoke command:

```bash
.venv/bin/python tools/validate_rh56_visual_coacd_stage2.py \
  --out-dir /tmp/rh56_visual_coacd_stage2a_object_smoke \
  --collision-modes visual_coacd correll_mesh unifuc_pad_proxy \
  --targets sim_best_pinch \
  --strategies simultaneous \
  --profile nominal \
  --timeout-scale 0.25 \
  --sample-stride 25 \
  --include-object
```

Object-present smoke result:

- `6` runs completed: object-free and object-present scenes for all three collision modes.
- All runs timed out due to the intentionally short timeout; this is only an infrastructure smoke, not a reachability or grasp-success conclusion.
- Artifacts were emitted under `/tmp/rh56_visual_coacd_stage2a_object_smoke`.

Focused tests:

```bash
.venv/bin/python -m pytest tests/test_correll_rh56dfx_assets.py tests/test_mujoco_rh56_collision_modes.py
```

Current result at handoff:

- `11 passed`

Mesh validity:

- CoACD hull files: `148`
- Watertight hulls: `148/148`
- Convex hulls: `148/148`
- Requires `trimesh` and `scipy` for `trimesh.is_convex`.

Stage 1 alignment audit command:

```bash
.venv/bin/python tools/audit_rh56_visual_coacd_stage1.py \
  --out-dir /tmp/rh56_visual_coacd_stage1 \
  --independent-samples 32 \
  --actuator-samples 64 \
  --max-contact-rows 80
```

Stage 1 alignment summary:

- All collision transforms match visual transforms.
- `rh56_R_hand_base_link` centroid error: about `5.987 mm`.
- Other link centroid errors: about `0.528 mm` to `4.539 mm`.
- AABB surface coverage: approximately `1.0` for all links, with two links around `0.9995-0.9998`.

Stage 1 self-contact classification:

- Independent joint-limit samples: `35`
  - Contact rows: `386`
  - Unique body pairs: thumb/index only
  - Max penetration: about `0.008875 m`
- Actuator-reachable samples: `67`
  - Contact rows: `242`
  - Unique body pairs: thumb/index only
  - Max penetration: about `0.008921 m`

Important: these contacts are not hidden by exclusions. They remain the primary Stage 2 issue.

## Current Collision Statistics

Canonical pose comparison from `tools/check_mujoco_rh56_collision_modes.py` after Stage 1:

| Mode | Pose | Total Contacts | RH56 Hand Self Contacts |
|---|---|---:|---:|
| `visual_coacd` | `open` | 4 | 0 |
| `visual_coacd` | `thumb_rotate` | 4 | 0 |
| `visual_coacd` | `real_pinch_v4` | 4 | 0 |
| `visual_coacd` | `sim_best_pinch` | 16 | 12 |
| `visual_coacd` | `power_close` | 13 | 9 |
| `correll_mesh` | `sim_best_pinch` | 5 | 1 |
| `correll_mesh` | `power_close` | 5 | 1 |
| `unifuc_pad_proxy` | `sim_best_pinch` | 2 | 0 |
| `unifuc_pad_proxy` | `power_close` | 0 | 0 |

The 4 contacts in open/thumb_rotate/real_pinch for mesh modes are JAKA Link0/Link1 contacts already present in the mounted model, not RH56 hand self-collision.

## Current Performance Numbers

Measured with short MuJoCo stepping in the current environment:

| Mode/XML | Geoms | Meshes | Excludes | Contacts at End | Steps/s |
|---|---:|---:|---:|---:|---:|
| `rh56_visual_coacd.xml` | 169 | 168 | 7 | 4 | not remeasured after reference-geometry removal |
| `pose_collision_correll_mesh.xml` | 46 | 32 | 0 | 4 | about `24.7k` |
| `pose_collision_unifuc_pad_proxy.xml` | 56 | 32 | 0 | 0 | about `50.2k` |

These are local smoke numbers, not CI thresholds yet.

## Existing Scripts

Collision and RH56 simulation:

- `tools/generate_rh56_visual_coacd_collision.py`
- `tools/audit_rh56_visual_coacd_stage1.py`
- `tools/audit_mujoco_rh56_collision_readiness.py`
- `tools/check_mujoco_rh56_collision_modes.py`
- `tools/check_mujoco_rh56_codebook_contacts.py`
- `tools/mujoco_rh56_grasp_benchmark.py`
- `tools/view_mujoco_rh56_pose_contact.py`
- `tools/debug_mujoco_jaka_rh56_viewer.py`
- `tools/preview_mujoco_tennis_ball_lift.py`

Pregrasp:

- `tools/predict_rh56_pregrasp.py`
- `tools/generate_rh56_pregrasp_dataset.py`

Hardware/bring-up examples:

- `tools/check_rh56_connection.py`
- `tools/check_rh56_via_jaka.py`
- `tools/rh56_pc_direct_bringup.py`
- `tools/run_rh56_ros2_json_bridge.py`
- `tools/run_jaka_rh56_rviz_joint_state_bridge.py`
- `tools/check_jaka_connection.py`
- `tools/check_jaka_zero_motion.py`
- `tools/check_jaka_small_joint_motion.py`
- `tools/check_jaka_small_tcp_motion.py`

Teleop/data examples:

- `tools/teleop_mujoco_jaka_rh56.py`
- `tools/iphone_mediapipe_hand_teleop.py`
- `tools/iphone_rh56_safety_gate.py`
- `tools/collect_mujoco_tennis_ball_data.py`

## Existing Tests

Collision and Correll asset tests:

- `tests/test_correll_rh56dfx_assets.py`
- `tests/test_mujoco_rh56_collision_modes.py`
- `tests/test_rh56_visual_coacd_stage2.py`

RH56/pregrasp tests:

- `tests/test_rh56_hand_schema.py`
- `tests/test_rh56_ros2_bridge.py`
- `tests/test_rh56_serial_backend.py`
- `tests/test_rh56_jaka_tool_backend.py`
- `tests/test_mock_rh56.py`
- `tests/test_pregrasp_prediction.py`
- `tests/test_pregrasp_hardware_constraints.py`
- `tests/test_rh56_pregrasp_dataset_generator.py`

Broader project tests:

- `tests/test_configs.py`
- `tests/test_episode_recorder.py`
- `tests/test_jaka_preset_config.py`
- `tests/test_jaka_servo_jog.py`
- `tests/test_jaka_tio_signal_client.py`
- `tests/test_mock_jaka.py`
- `tests/test_realsense_adapter.py`
- `tests/test_relative_pose_lag_follow.py`
- `tests/test_robot_bringup_ros2_bridge.py`
- `tests/test_rviz_joint_state_bridge.py`
- `tests/test_rviz_shadow_sync.py`
- `tests/test_xbox_ros2_teleop.py`
- `tests/test_xbox_rviz_shadow.py`

## Existing Benchmark Tools

- `tools/mujoco_rh56_grasp_benchmark.py`
  - Supports `--collision-mode visual_coacd`.
  - Uses semantic contact groups for thumb/index/middle/ring_pinky instead of requiring pad proxy names.
  - Current smoke run with `visual_coacd` executes but does not yet succeed on `foam_cube`.
- `tools/check_mujoco_rh56_collision_modes.py`
  - Static canonical pose comparison across collision modes.
- `tools/check_mujoco_rh56_codebook_contacts.py`
  - Existing codebook/contact diagnostics.
- `tools/audit_mujoco_rh56_collision_readiness.py`
  - Still has legacy expectations such as pad proxies; not yet Stage 2-ready.
- `tools/view_mujoco_rh56_pose_contact.py`
  - Visual inspection and derived XML generation.

## How to Reproduce Stage 1

Do not regenerate hulls for Stage 2. Use this only to reproduce the current asset set if explicitly needed:

```bash
.venv/bin/python tools/generate_rh56_visual_coacd_collision.py --overwrite
```

Current generated assets should already exist at:

```text
data/sim_assets/meshes/rh56_collision_visual_coacd/
```

Run Stage 1 audit:

```bash
.venv/bin/python tools/audit_rh56_visual_coacd_stage1.py \
  --out-dir /tmp/rh56_visual_coacd_stage1 \
  --independent-samples 32 \
  --actuator-samples 64 \
  --max-contact-rows 80
```

Run focused tests:

```bash
.venv/bin/python -m pytest tests/test_correll_rh56dfx_assets.py tests/test_mujoco_rh56_collision_modes.py
```

Run canonical collision-mode comparison:

```bash
.venv/bin/python tools/check_mujoco_rh56_collision_modes.py \
  --collision-modes visual_coacd correll_mesh unifuc_pad_proxy \
  --out-dir /tmp/rh56_collision_modes_stage1 \
  --max-pairs 120
```

## How to Reproduce Current Benchmark

Smoke benchmark for `visual_coacd`:

```bash
.venv/bin/python tools/mujoco_rh56_grasp_benchmark.py \
  --collision-mode visual_coacd \
  --objects foam_cube \
  --duration 0.4 \
  --max-candidates 1 \
  --point-count 24 \
  --out-dir /tmp/rh56_visual_coacd_grasp_smoke
```

Current smoke result:

- Runs without `thumb_pad_proxy`.
- `num_success`: `0`
- `best_success`: `false`
- `best_lift_m`: about `-0.0024`
- `best_contacts` included object/table contacts and no hand self contacts in the best row.

This is not a grasp-performance conclusion. It only verifies that `visual_coacd` can be exercised by the benchmark without old proxy dependencies.

## Dynamic-Validation Design Decisions

Stage 2 validation should:

- Use actuator commands and `mujoco.mj_step`, not only direct qpos teleport.
- Separate independent joint-limit tests from actuator/synergy-reachable trajectories.
- Record contact timelines, not only final contact state.
- Classify contacts into forbidden structural collision, legitimate fingertip/object contact, finger/finger contact, finger/palm contact, hand/table contact, arm contact, and object/table contact.
- Treat only forbidden self-collision as CI failure.
- Keep thumb/index interactions visible until classified.
- Detect solver warnings, contact-force spikes, persistent penetrations, and blocked controller motion.
- Report both body pairs and geom pairs.
- Include original visual-mesh intersection checks as diagnostic evidence.

## Future CI Policy

Stage 2 CI should add repeatable tests for:

- Mesh validity: watertight and convex generated collision components.
- Body-local visual-to-collision alignment: transformed vertices, centroid error, AABB error, and surface coverage.
- Canonical-pose forbidden-contact checks.
- Actuator-reachable trajectory checks with `mj_step`.
- Pinch and power-grasp stability checks.
- Contact-force stability and penetration thresholds.
- Solver warning or instability detection.
- Hull statistics and budget checks.
- Simulation speed smoke thresholds.

CI must not fail all thumb/index or object-contact cases blindly. It should fail forbidden/unintended structural collision, and classify legitimate grasp-surface contacts separately.

## Known Issues

- TODO [HIGH]: Stage 2A infrastructure and the focused pinch classification are reliable enough for investigation, but broader contact/blockage thresholds and final policy are not yet ready for CI gates.
- TODO [BLOCKER]: Actuator-reachable thumb/index contacts remain. Stage 1 found `242` contact rows in `67` actuator-reachable samples, max penetration about `8.921 mm`.
- TODO [BLOCKER]: Independent joint-limit thumb/index contacts remain. Stage 1 found `386` contact rows in `35` independent samples, max penetration about `8.875 mm`.
- TODO [HIGH]: `tools/audit_mujoco_rh56_collision_readiness.py` is not aligned with `visual_coacd`; it still reports legacy `missing_pad_proxies`.
- TODO [HIGH]: Grasp benchmark has only a smoke run for `visual_coacd`; it is not yet a Stage 2 validation gate.
- TODO [HIGH]: Contact classification is centralized and the focused full-duration pinch case has visual diagnostics, but `power_close`, other paths, and object-present cases still need equivalent review.
- TODO [HIGH]: The mounted model still has JAKA Link0/Link1 contacts in mesh modes; these are known non-hand contacts but should be isolated in validation.
- TODO [MEDIUM]: `data/sim_assets/README.md` and some docs still describe Correll mesh as default runtime collision; update docs after Stage 2 policy is settled.
- TODO [MEDIUM]: Performance thresholds are not codified in tests.
- TODO [MEDIUM]: Visual-to-collision alignment thresholds are not codified in tests.
- TODO [LOW]: Stage 3 geometry optimization has not started.

## Open Questions

- PARTIAL [BLOCKER]: For `sim_best_pinch` + `iterative_incremental`, original thumb/index vendor visuals intersect at CoACD contact, so this case is not confirmed CoACD outward approximation. Whether the commanded pose or current kinematics/coupling match physical RH56DFX behavior remains unresolved.
- TODO [HIGH]: What exact trajectory set should define actuator-reachable RH56 validation?
- TODO [HIGH]: What penetration/contact-force thresholds should fail CI for forbidden self-collision?
- TODO [HIGH]: Should palm/base CoACD centroid error around `5.987 mm` be acceptable, or should it become a Stage 2 failure threshold?
- TODO [MEDIUM]: Should Stage 2 use real RH56 command logs or only synthetic actuator sweeps first?
- TODO [MEDIUM]: How should contact-force stability be normalized across object masses and solver settings?
- TODO [LOW]: Should URDF integration be added later if a canonical RH56 URDF becomes available?

## Remaining Roadmap

Stage 2:

- DONE [INFRA]: Implemented first Stage 2A dynamic trajectory validator using actuator commands and `mj_step`.
- TODO [HIGH]: Review and tune Stage 2A contact/blockage classification on full-duration slow trajectories before making CI gates.
- PARTIAL [HIGH]: Exact runtime inventory, exclusions, and hand inertials now have regression tests; broader Stage 1 mesh-quality/alignment audit thresholds are not yet CI gates.
- DONE [INFRA]: Built reusable contact classifier in `src/sim_maniskill/rh56_collision_validation.py`.
- DONE [INFRA]: Added trajectory-level JSON/CSV/SVG reports with body pair, geom pair, penetration depth, contact force, timestep, qpos/qvel/ctrl, and contact classification.
- DONE [FOCUSED]: Classified `sim_best_pinch` + `iterative_incremental` as `shared_visual_or_kinematic_intersection` with full slow/nominal/hybrid trajectories, same-qpos replay, original visual-mesh diagnostics, exact CoACD-part IDs, and deterministic repeat evidence.
- TODO [HIGH]: Add pinch and power-grasp stability checks that use semantic contact groups.
- TODO [MEDIUM]: Add simulation speed and hull-statistics CI smoke tests.

Stage 3:

- TODO [LOW]: Optimize geometry after Stage 2 baseline is stable.
- TODO [LOW]: Preserve high fidelity at fingertips, finger pads, distal palmar surfaces, and useful palm grasp surfaces.
- TODO [LOW]: Simplify dorsal shells and non-contact surfaces.
- TODO [LOW]: Remove or aggressively simplify hidden joint-internal geometry, screw holes, grooves, decorative details, and other non-contact regions.
- TODO [LOW]: Compare every optimized version against the validated Stage 2 baseline so geometry changes cannot hide kinematic or transform errors.

## Files That Should Not Be Modified Lightly

- `data/sim_assets/jaka_rh56.xml`
  - Mounted JAKA+RH56 integration anchor used by IK, previews, benchmark, and ManiSkill paths.
- `data/sim_assets/meshes/rh56_collision_visual_coacd/`
  - Current 148-hull Stage 1 baseline. Do not regenerate during Stage 2.
- `data/sim_assets/meshes/rh56/`
  - Vendor visual STL source meshes.
- `data/sim_assets/correll_rh56dfx/`
  - Third-party reference assets with license.
- `src/rh56_driver/hand_schema.py`
  - Canonical RH56 hand order and command mapping.
- `src/pregrasp/correll_rh56dfx.py`
  - Correll asset adapter and planner mapping.
- `src/sim_maniskill/rh56_collision.py`
  - Central collision-mode runtime patching.
- `tools/generate_rh56_visual_coacd_collision.py`
  - Generator for the baseline hulls. Avoid changing generator parameters until Stage 2 has baseline CI.

## Files Expected to Change During Stage 2

- `tools/audit_rh56_visual_coacd_stage1.py`
  - May be split or promoted into reusable validation code.
- `tools/audit_mujoco_rh56_collision_readiness.py`
  - Should be updated to understand `visual_coacd` and semantic contact categories.
- `tools/check_mujoco_rh56_collision_modes.py`
  - Should classify forbidden vs legitimate contacts.
- `tools/mujoco_rh56_grasp_benchmark.py`
  - Should emit richer dynamic contact/force reports.
- `src/sim_maniskill/rh56_collision.py`
  - May need contact classification helpers or carefully reviewed exclusions only.
- `tests/test_mujoco_rh56_collision_modes.py`
  - Should grow Stage 2 CI tests.
- Stage 2 files now present:
  - `src/sim_maniskill/rh56_collision_validation.py`
  - `src/sim_maniskill/rh56_collision_diagnostics.py`
  - `tools/validate_rh56_visual_coacd_stage2.py`
  - `tools/investigate_rh56_visual_coacd_stage2a.py`
  - `tests/test_rh56_visual_coacd_stage2.py`

## Important Assumptions

- The current RH56 visual STL meshes are assumed to match the physical hand closely enough to be the collision source.
- Existing 148 CoACD hulls are a Stage 1 baseline, not final geometry.
- MuJoCo XML is the active integration target; no RH56 URDF is currently present in this repository.
- `data/sim_assets/jaka_rh56.xml` remains a recovery/integration anchor, not final ground truth.
- Correll RH56DFX assets are useful references but are geometrically coarser than the visual-derived CoACD baseline.
- Real hardware behavior may constrain thumb/index postures beyond what current synthetic commands represent.

## Things Intentionally NOT Solved Yet

- TODO [HIGH]: Stage 2A dynamic trajectory validator exists, but final contact/blockage policy is not reliable enough for CI gates.
- PARTIAL [BLOCKER]: The focused `sim_best_pinch` iterative case is classified as shared visual/kinematic intersection. Other command orders, `power_close`, object-present arrest, and physical-hand fidelity remain unresolved.
- TODO [HIGH]: No CI pass/fail policy for forbidden contact vs legitimate grasp contact is implemented.
- TODO [HIGH]: Contact-force stability is recorded but not yet validated against reviewed thresholds.
- TODO [HIGH]: Solver warning detection is recorded in trajectory samples but not yet turned into a reviewed validation gate.
- TODO [MEDIUM]: Stage 1 audit JSON is not yet consumed by automated tests.
- TODO [MEDIUM]: Documentation outside this handoff may be stale about default collision mode.
- TODO [LOW]: No Stage 3 geometry simplification has been attempted.
- TODO [LOW]: No URDF export/integration has been implemented because no RH56 URDF is present.

## Next Immediate Task

Continue Stage 2A without changing collision geometry:

1. Determine whether the `sim_best_pinch` command or mounted XML kinematics/coupling should permit the vendor thumb/index visuals to intersect; compare against physical RH56DFX behavior or a trusted kinematic source.
2. Apply the focused diagnostic workflow to `power_close` before expanding the full scenario matrix.
3. Improve object-present target/object placement so object arrest can be distinguished from empty-hand thumb/index self-contact.
4. Review whether the reference modes' continued motion after original visual intersection represents intentional coarseness or missed collision; do not treat their reachability as physical ground truth.
5. Only after broader contact and blockage classification is reliable, decide which Stage 2B checks should become CI gates.
6. Do not regenerate CoACD hulls, retune hull parameters, or add new exclusions based on this result.
