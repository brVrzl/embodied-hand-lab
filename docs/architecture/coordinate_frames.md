# Quest-to-JAKA coordinate frames

The incoming Hand Tracking Streamer data uses Unity/OpenXR conventions and is
canonicalized by `motion_input.hts_canonical` before robot mapping. Do not add
another implicit handedness or quaternion-order conversion downstream.

At clutch capture the session stores:

- the current wrist transform;
- a gravity-aligned head-yaw basis;
- the authoritative robot TCP pose associated with the accepted/held joints.

For each current wrist sample, the local relative transform is:

```text
T_relative = inverse(T_wrist_reference) * T_wrist_current
```

Translation is expressed in the latched horizontal head-yaw frame and mapped
to robot-base coordinates with the current basis rows `[-X, +Z, +Y]`.
Configured translational gains are 1.0. Later head motion does not drag an
engaged arm reference.

Orientation remains the local/body wrist-relative rotation and is conjugated
by `diag(-1, -1, +1)` for the robot mapping. Translation and rotation then pass
through One Euro filters plus current 1 mm and 2° deadbands before continuation
IK.

Reference capture is release-before-press. A fresh controller press captures a
fresh Quest reference; stale data, stream loss, or a completed physical stop
requires release and another fresh press. Do not document SPACE-key clutch or
world-frame head-follow behavior for the current live 6-DoF path.

The generic UMIP frame contract remains documented in
`docs/motion_input/COORDINATE_FRAMES.md`; this page describes the additional
Quest/JAKA mapping policy.
