# Quest SDK and OpenXR review

Review date: 2026-07-16. Claims below distinguish repository facts, selected
build baselines, standardized capability, and device measurements still needed.

## Version inventory

| Item | Repository before this work | Selected/reviewed baseline | Meaning |
|---|---|---|---|
| Unity | absent | Unity 6000.0 LTS | Host required to compile the bridge; not installed here. |
| Unity XR Hands | absent | 1.7.3 | Pinned bridge dependency and current released Unity 6 package. |
| Unity OpenXR plug-in | absent | 1.16.1 | Required in the consuming Unity project. |
| Unity OpenXR Meta | absent | 2.4.1 reviewed | Optional for additional Meta-specific extensions; not required by the core XR Hands bridge. |
| Meta XR SDK | absent | not required by this bridge | Do not claim a repository version. Add only if a Meta-only feature is justified. |
| OpenXR specification | absent | 1.1.61 reviewed | Specification revision, not the unknown headset runtime version. |
| Quest/Horizon OS | absent | unknown | Must be recorded from the test headset. |

Unity publishes XR Hands 1.7.3 for Unity 6000.0, OpenXR 1.16.1, and Meta OpenXR
2.4.1 in its current package manuals. The official Meta Movement sample states
that its current body/eye/face package requires Meta XR SDK v81 or newer, but
that does not establish a version in this repository and does not make Movement
necessary for hand joints.

Sources: [Unity XR Hands](https://docs.unity3d.com/6000.0/Documentation/Manual/com.unity.xr.hands.html),
[Unity OpenXR](https://docs.unity3d.com/6000.0/Documentation/Manual/com.unity.xr.openxr.html),
[Unity OpenXR Meta](https://docs.unity3d.com/6000.0/Documentation/Manual/com.unity.xr.meta-openxr.html),
[Meta Unity Movement sample](https://github.com/oculus-samples/Unity-Movement),
[OpenXR 1.1 registry](https://registry.khronos.org/OpenXR/).

## Recommended API

Use `XR_EXT_hand_tracking` through Unity XR Hands for the first bridge. This is
the narrowest cross-vendor API and exposes separate left/right trackers,
activity, 26 ordered joints, pose-valid/tracked flags, radius, and optional
velocity. Unity recommends reading `XRHandSubsystem.updatedHands`; Dynamic is
appropriate for application/input work, while BeforeRender is lower render
latency but is called separately and late in the frame.

Sources: [XR Hands API](https://docs.unity3d.com/Packages/com.unity.xr.hands%401.7/manual/index.html),
[XRHandSubsystem](https://docs.unity3d.com/Packages/com.unity.xr.hands%401.5/api/UnityEngine.XR.Hands.XRHandSubsystem.html),
[`XR_EXT_hand_tracking`](https://registry.khronos.org/OpenXR/specs/1.1/man/html/XR_EXT_hand_tracking.html).

Do not use Interaction SDK gesture objects as the base protocol. If enabled,
pinch/aim or Meta microgestures are translated into optional UMIP gesture fields
inside the provider. SDK classes never cross the boundary.

## Capability matrix

| Capability | Standard/API fact | Bridge behavior |
|---|---|---|
| Hand tracking | Separate left and right trackers; `isActive`/`isTracked` indicates availability. | One UMIP sample per side per Dynamic update. |
| Wrist | Joint index 1; Unity hand root pose is located at the wrist and points forward toward the fingers. | Required `wrist_pose` while tracking. |
| Palm | Joint index 0; `XR_EXT_palm_pose` also defines a semantic palm action pose. | Optional `palm_pose` from the palm joint. |
| Joints | OpenXR default set has 26 entries (palm, wrist, 24 finger points). | Optional typed joint array; UMIP is not limited to 21. |
| Tracking state | OpenXR separates valid and tracked pose flags and an active tracker state. | `tracking`, `limited`, `not_tracking`, or `disconnected`; no stale pose on loss. |
| Hand confidence | Core hand tracking has flags, not a normalized confidence scalar. Meta `OVRHand` can expose a binary high/low confidence in Meta-specific APIs. | `null` through XR Hands; never invent a number. Provider-specific scales must be named in metadata. |
| Pinch/gesture | Meta Aim Hand / `XR_FB_hand_tracking_aim`, hand interaction, and Meta microgesture extensions can expose gesture values. | Schema reserved now; current bridge leaves values null/empty. |
| Frequency | `XR_META_hand_tracking_frequency_hint` defines runtime hints, not a portable guaranteed Hz. Unity callback cadence also depends on app update cadence. | Diagnostics measure actual received/capture cadence. No fixed Quest 3 rate is claimed. |
| SDK timestamp | Native `xrLocateHandJointsEXT` takes an `XrTime`; OpenXR defines it as monotonic nanoseconds with a runtime-chosen epoch. | Current Unity abstraction does not surface that `XrTime`, so `device_timestamp` is null. |
| Capture time | Application callback time is distinct from requested SDK location time. | Unity realtime nanoseconds in a session-specific clock. |

Sources: [OpenXR joint list](https://registry.khronos.org/OpenXR/specs/1.1/man/html/XrHandJointEXT.html),
[joint locations](https://registry.khronos.org/OpenXR/specs/1.1/man/html/XrHandJointLocationsEXT.html),
[`XrTime`](https://registry.khronos.org/OpenXR/specs/1.1/man/html/XrTime.html),
[frequency-hint extension index](https://registry.khronos.org/OpenXR/specs/1.1/man/html/openxr.html),
[Unity hand root pose](https://docs.unity3d.com/Packages/com.unity.xr.hands%401.5/api/UnityEngine.XR.Hands.XRHand.html).

## Latency

There is no single standards-defined Quest hand-tracking latency. OpenXR poses
may be requested for past or predicted times, and Unity offers Dynamic and
BeforeRender observations. Network, serialization, scheduling, prediction, and
clock synchronization all alter end-to-end numbers.

UMIP therefore records capture, SDK/device, receive, and processing times
separately. Diagnostics calculate latency only when clock IDs match. For the
current Quest bridge, capture uses a headset-local monotonic clock and receive
uses the host monotonic clock, so end-to-end latency is correctly reported as
unavailable rather than a false number. Hardware acceptance must add a measured
clock offset/uncertainty or a native OpenXR time-conversion/synchronization path.

Because the Unity publisher emits both sides on every Dynamic update, the Quest
provider treats one second without any datagram as a disconnect and emits one
disconnect event per side. The timeout is configurable and recovery is the next
valid source frame; no pose is generated during the gap.

## Quest hardware acceptance checklist

Before calling the provider device-validated, record:

1. Unity, package, Horizon OS, headset firmware, and Android build identifiers.
2. Supported OpenXR version and enabled extensions from runtime logs.
3. 10-minute and 2-hour capture rates for each hand, including p50/p95 jitter.
4. Loss/recovery under occlusion, hands leaving view, controller switching, app
   focus loss, sleep/wake, Wi-Fi loss, and publisher restart.
5. Timestamp offset/uncertainty and p50/p95/p99 capture-to-receive latency after
   clock synchronization.
6. CPU, thermal behavior, UDP loss, and sequence gaps at all intended rates.

These measurements are intentionally not replaced by display refresh rate or
camera latency figures; neither is hand-pose delivery latency.
