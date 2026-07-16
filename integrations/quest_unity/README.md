# Quest Unity bridge

This package is the only headset-side component of the Motion Input Platform. It
reads Unity XR Hands and sends versioned `quest-hand-frame` UDP datagrams to the
Python `QuestMotionProvider`. It does not contain robot or teleoperation logic.

Verified package baseline selected for the bridge API:

- Unity 6000.0 LTS;
- `com.unity.xr.hands` 1.7.3;
- `com.unity.xr.openxr` 1.16.1 in the host project;
- optional `com.unity.xr.meta-openxr` 2.4.1 only if the host project uses
  additional Meta-specific extensions.

Install this directory as a local Unity package. In XR Plug-in Management,
enable OpenXR for Android, select the Meta Quest feature group, and enable Hand
Tracking. Add `QuestUmipPublisher` to an otherwise empty GameObject, set the
receiver IP/port, and ensure `referenceSpace` matches the XR Origin selection.

The publisher deliberately uses raw tracking-origin poses, not scene/world
transforms. It emits Unity's left-handed basis and the Python provider performs
the one centralized conversion to OpenXR's right-handed convention. Unity XR
Hands does not expose the underlying `XrTime` or Meta's binary hand
confidence, so `device_timestamp` and `tracking_confidence` are `null`. They must
not be synthesized. A future native OpenXR bridge can populate them without a
UMIP schema change.

The bridge has not been compiled or device-tested in this Python repository;
that requires a Unity Android project and Quest 3 hardware. The wire parser,
provider, recording, replay, diagnostics, and disconnect/recovery behavior are
covered by hardware-free Python tests.
