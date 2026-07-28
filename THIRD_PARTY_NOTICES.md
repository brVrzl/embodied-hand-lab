# Third-party notices

This file records the provenance and license evidence available in this
repository. It does not grant rights beyond the corresponding upstream
license, vendor agreement, or applicable law.

## Correll Robotics Lab RH56DFX assets

- Location: `data/sim_assets/correll_rh56dfx/`
- Contents: reference MuJoCo XML and visual/collision meshes.
- Copyright: Correll Robotics Lab, University of Colorado Boulder.
- License: MIT; the complete upstream notice is retained at
  `data/sim_assets/correll_rh56dfx/LICENSE`.
- Runtime role: reference only. The mounted runtime model uses the separately
  derived project asset documented in `data/sim_assets/README.md`.

## JAKA SDK 2.2.7 snapshot

- Location: `third_party/jaka_sdk/v2.2.7/`
- Contents: vendor headers, shared libraries, and English/Chinese release notes.
- Runtime role: the native JAKA workers may link this locally supplied SDK;
  default tests and simulation do not connect to a controller.
- License evidence: no redistributable license text is present in this
  repository. Treat the files as vendor-supplied material subject to the
  applicable JAKA SDK agreement. Do not redistribute them independently until
  those terms are confirmed.

## Inspire RH56 reference snapshot

- Location: `third_party/inspire_hand/rh56/`
- Contents: protocol examples, headers, Python examples, and a driver design
  note.
- Runtime role: reference/vendor examples. Current project backends live in
  `src/rh56_driver/`; the Quest/MuJoCo integration does not import this
  snapshot.
- License evidence: no license text or reliable upstream URL is recorded in
  this repository. Preserve source attribution and do not redistribute this
  snapshot independently until ownership and license terms are confirmed.

## External sibling dependency

The inference-only π0.5-DROID shadow workflow uses a separate OpenPI checkout
at the commit recorded in `learned_policy/pi05_shadow/VALIDATION_REPORT.md`.
OpenPI is not vendored here; its own checkout and license remain authoritative.
