# Motion-input repository audit

Audit date: 2026-07-16. Scope: current worktree, all locally available branches,
and complete local Git history through `be4fcf3`.

## Finding

There was no Quest, Oculus/Meta XR, OpenXR, Unity, Vision Pro, DexUMI, UMI, or XR
motion-input implementation in the available repository or its local history.
No Unity `Assets`, `Packages/manifest.json`, `.unity`, `.asmdef`, Android XR
project, OpenXR loader, or Meta SDK package existed. Consequently, no repository
Quest SDK, OpenXR runtime, Unity, or Meta XR SDK version could be determined.

The new `integrations/quest_unity` package is a fresh isolated integration, not
a reconstruction of historical code.

## Existing input experiments

| Input | Existing locations | Character | Reuse decision |
|---|---|---|---|
| iPhone camera / MediaPipe | `src/teleop_tools/iphone_hand.py`, `hand_depth.py`, `tools/iphone_*`, `scripts/*iphone*` | Camera hand landmarks coupled to RH56 experiments | Do not modify; a future provider may translate observations to UMIP. |
| HEBI Mobile I/O / iPhone | `src/teleop_tools/hebi_mobile_io.py`, `relative_pose_lag_follow.py`, HEBI tools/configs | Phone pose tied to existing arm experiments | Do not modify; provider extraction is a later migration. |
| TeleDex | evolving untracked `src/teleop_tools/teledex_*` and `src/teleoperation/` work | Actively owned by another development session | Read-only; propose UMIP adapter only after review. |
| Xbox | `src/teleop_tools/xbox_ros2.py`, `xbox_rviz_shadow.py` | Controller path coupled to ROS/RViz and robot semantics | Do not modify. |
| RealSense / vision | `src/vision_interface` | Camera frames, not operator motion protocol | Remains separate; a future vision-tracking provider may consume it. |
| Episode recorder | `src/data_recorder` | Robot/episode records, not source motion provenance | Not reused or modified. |

The repository is Python-first (`src`, `tools`, `tests`) with native code only
for the JAKA worker and vendor SDKs. Existing visualization is robot/RViz or
MuJoCo oriented, so it is intentionally not reused for input visualization.

## Concurrent-work protection

The audit found a dirty worktree with active changes in teleoperation, JAKA,
EDG-adjacent, RealSense, digital-twin, and robot files. This implementation does
not edit those areas. Its only existing-file change is the additive
`motion-input-viz` optional dependency in `pyproject.toml`.

No code under these paths was changed:

- `src/teleoperation`, `src/teleop_tools`;
- `src/jaka_driver_adapter`, `native/jaka_servo_worker`;
- `src/rh56_driver`, `src/robot_bringup`;
- robot, teleoperation, trajectory, IK, safety, or SDK configuration.

## New ownership boundary

`src/motion_input`, `integrations/quest_unity`, `tools/umip_motion_input.py`,
`tests/test_motion_input_*`, `tests/test_quest_motion_provider.py`, and this
documentation are input-platform-owned. Their dependency arrow ends at UMIP;
there is no arrow into robot code in this stage.
