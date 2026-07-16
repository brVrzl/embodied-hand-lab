# UMIP coordinate frames

UMIP poses are relationships between a child frame (wrist, palm, joint) and the
single `coordinate_frame` named by the sample. They are not presumed to be in a
robot, lab, or universal world frame.

## Numeric convention

- Cartesian, right-handed after provider translation.
- meters only;
- quaternion `x,y,z,w`, unit norm;
- rotation applies before translation;
- no implicit degree/radian, millimeter/meter, handedness, or quaternion-order
  conversion.

These match OpenXR's base numeric convention. Source: [OpenXR coordinate
system](https://registry.khronos.org/OpenXR/specs/1.1/html/xrspec.html#coordinate-system).

## Frame table

| Frame | Origin | Axes and handedness | Motion |
|---|---|---|---|
| Quest/OpenXR `VIEW` (head space) | primary viewer/view centroid | right-handed, +X right, +Y up, -Z forward; not gravity aligned | follows head |
| Quest/OpenXR `LOCAL` | runtime startup/calibrated zero | right-handed, +X right, +Y up, -Z forward; gravity aligned | world locked, runtime may refine |
| Quest/OpenXR `LOCAL_FLOOR` | local origin projected to estimated floor | right-handed, +X right, +Y up, -Z forward | world locked, runtime may refine |
| Quest/OpenXR `STAGE` | floor center of runtime play rectangle | right-handed, +Y up; X/Z align to rectangle, conventional -Z forward | world locked; optional/possibly unavailable |
| Unity tracking space (wire only) | XR Origin/tracking origin | left-handed, +X right, +Y up, +Z forward | follows selected origin mode |
| World space | application-defined registered frame | must explicitly declare axes/origin/handedness; UMIP output must be right-handed meters | provider/application defined |
| Hand/root space | Unity XR Hand root at wrist | raw Unity child axes; Unity documents forward toward fingers | follows hand |
| Wrist space | wrist joint/root | after conversion, -Z follows Unity root forward toward fingers; remaining axes are the converted SDK orientation | follows wrist |
| Palm space | OpenXR/Unity palm joint centroid | child axes are exactly the returned joint orientation converted to right-handed basis; do not substitute grip/aim axes | follows palm |
| Joint local pose | named joint relative to sample frame | child axes are exactly provider-returned orientation | follows joint |

OpenXR reference-space definitions: [OpenXR spaces](https://registry.khronos.org/OpenXR/specs/1.1/html/xrspec.html#spaces).
Unity root definition: [XRHand root pose](https://docs.unity3d.com/Packages/com.unity.xr.hands%401.5/api/UnityEngine.XR.Hands.XRHand.html).

Palm joint, `palm_ext`, grip, grip-surface, aim, and wrist are different semantic
poses. A provider must label what it emits and must not silently substitute one.
The current Quest bridge emits the XR Hands palm joint and wrist root, not grip
or aim. OpenXR grip/grip-surface axes are separately standardized in the
[semantic path specification](https://registry.khronos.org/OpenXR/specs/1.1/html/xrspec.html#semantic-path-standard-pose-identifiers).

## Quest frame identity

The provider constructs:

```text
quest/<device_id>/<session_id>/<reference_space>:openxr
```

The session ID is part of the frame because a restarted runtime may choose a
new local origin. Equal numeric coordinates from different session frame IDs
must not be treated as the same physical frame without registration.

## Centralized Unity conversion

`motion_input.frames.unity_to_openxr_pose` is the only basis conversion:

```text
position:  ( x,  y,  z) -> ( x,  y, -z)
quaternion (x, y, z, w) -> (-x, -y, z, w)
```

This is the matrix reflection `S R S`, `S=diag(1,1,-1)`, expressed as a
quaternion. The inverse is identical. Tests verify round-trip pose consistency.
It is not calibration, scaling, filtering, registration, or control.

## Transform ownership

Providers may perform only a documented source-basis/unit conversion necessary
to state their observation in UMIP. Static/dynamic frame registration belongs
in a future shared transform service proposed to the teleoperation team. Until
that interface is reviewed, no Quest-to-lab/world/robot transform exists here.

Transforms must never be scattered through providers, tools, or controllers.
The `FrameRegistry` rejects conflicting definitions for the same ID; future
dynamic transforms should be timestamped data with explicit parent/child IDs,
not anonymous matrices in metadata.
