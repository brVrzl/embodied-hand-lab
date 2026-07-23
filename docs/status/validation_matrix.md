# Validation matrix

Status is scoped to the current implementation and preserved evidence. “Offline
tested” and “simulation validated” do not mean physically passed.

| Capability | Implementation/offline | Simulation | Physical | Latest evidence / limitation |
|---|---|---|---|---|
| Quest packet/input parsing | implemented; tested | validated | input previously observed | HTS/CTRL provider tests; deployed APK still external |
| Wrist pose | implemented; tested | validated | partially observed | bounded historical Quest/JAKA runs |
| Controller clutch | implemented; tested | validated | partially validated | release-before-press and bounded runs |
| Reference capture/startup continuity | implemented; tested | validated | partially validated | post-EDG `q_hold`; no-jump contract |
| Coordinate mapping | implemented; tested | validated | partially validated | full envelope not proven |
| Translation/quaternion filters | implemented; tested | validated | partially exercised | current YAML and filter tests |
| Shared IK | implemented; tested | validated | partially exercised | MuJoCo model used plant-free |
| Continuation/branch policy | implemented; tested | validated | partially exercised | max 5 backtracks, min 1/32 |
| Jacobian singularity policy | implemented; tested | validated | not fully validated | J5 15° is warning only |
| Output velocity feasibility | implemented; tested | validated | partially exercised | checked pre-acceptance |
| Output acceleration feasibility | implemented; tested/replayed | validated offline | **not yet validated** | latest 4π rad/s² correction |
| `HOLD_REJECTED` recovery | implemented; tested | validated | not yet validated for acceleration fix | holds with fresh heartbeat |
| Native resampler | implemented; tested/fake worker | replay validated | partially validated | 125 Hz, 8 ms, latest destination |
| Native joint worker / zero native IK | implemented; tested | fake-worker validated | partially validated | joint mode `kine_inverse` count zero |
| Controller-health monitor | implemented; tested | fake-worker validated | timing path passed bounded run | sole-session lightweight polling |
| MuJoCo arm | implemented; tested | validated | n/a | shared accepted-target adapter |
| RH56 simulation | implemented; tested | validated | n/a | left-grip retarget path |
| Physical JAKA translation | implemented; tested offline | validated | partial | larger run ended in J4 collision |
| Physical JAKA orientation | implemented; tested offline | validated | partial | do not infer full envelope |
| Clutch release/cleanup | implemented; tested | validated | partial | historical bounded use |
| Collision-event propagation | implemented; tested offline | fake-worker validated | not intentionally validated | collision remains hard stop |
| Payload-corrected post-fix path | implemented through acceleration fix | replay validated | incomplete | polling timing passed; acceleration fix pending |
| TCP calibration | interfaces exist | model frames tested | not validated | TCP1–TCP10 recorded zero |
| RH56 physical teleoperation | separate drivers tested offline | simulation hand validated | not validated in Quest path | separately gated legacy paths |
| Foundation J6 gates | historical implementation | n/a | passed for exact +0.25°/+5° gates | July 16 evidence; not full teleop |
| Digital-twin workspace | implemented; tested | integrated workspace | not applicable | 3 failed trajectories; calibration pending |
