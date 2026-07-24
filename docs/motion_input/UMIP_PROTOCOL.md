# Unified Motion Input Protocol (UMIP) 1.0

UMIP is the only contract visible to a future teleoperation framework. It
describes observations, not commands, targets, trajectories, control policy,
filtering, safety, IK, or scaling.

## Architecture and dependency rule

```text
device SDK -> provider-private value -> MotionInputSample
                                      -> recorder
                                      -> replay provider
                                      -> input visualization / diagnostics
```

Consumers import `motion_input.model` and `motion_input.provider`. Providers may
import device SDKs. The reverse dependency is forbidden. A future input provider,
Vision Pro, DexUMI, UMI, leader arms, SpaceMouse, vision, or mocap implements
`MotionInputProvider`; it does not add a downstream device branch.

## Provider lifecycle

`MotionInputProvider` exposes `descriptor`, `open()`, `read(timeout_s)`, and
`close()`. `read` returns one immutable `MotionInputSample`, or `None` when the
timeout expires. Replay implements the identical interface. A finite replay
ends in `EXHAUSTED`; live providers remain open until closed or failed.

This pull interface supplies backpressure and deterministic ordering without
forcing a threading/event-loop dependency. An asynchronous adapter can wrap it
later without changing samples.

## MotionInputSample

| Field | Type | Rule |
|---|---|---|
| `protocol_version` | `MAJOR.MINOR` | Current `1.0`; reject unknown major, tolerate same-major optional additions. |
| `sample_id` | string | Globally unique observation identity. Quest uses deterministic UUIDv5 for normal frames. |
| `stream_id` | string | Stable logical stream identity for ordering and diagnostics. |
| `sequence_number` | non-negative integer | Monotonic per stream; never silently renumber received data. |
| `capture_timestamp` | `Timestamp` | Provider callback/capture instant, not host receipt. |
| `device_timestamp` | optional `Timestamp` | Native SDK/device observation or requested pose time when exposed. |
| `receive_timestamp` | `Timestamp` | Host ingress instant. |
| `processing_timestamp` | optional `Timestamp` | Instant UMIP construction finished. |
| `tracking_state` | enum | `tracking`, `limited`, `not_tracking`, `disconnected`. |
| `tracking_confidence` | optional float | `[0,1]`; provider scale must be documented. Null is better than invented confidence. |
| `coordinate_frame` | string | Registered frame ID; pose is relative to this frame. |
| `device` | `DeviceDescriptor` | Stable ID, type, maker, model, versions, and JSON metadata. |
| `side` | enum | `left`, `right`, or `none` for non-sided future inputs. |
| `wrist_pose` | optional `Pose6D` | Mandatory for `tracking`; forbidden for loss/disconnect events. |
| `palm_pose` | optional `Pose6D` | Same frame and units as wrist. |
| `motion_kind` | enum | Absolute pose now; relative 6-DoF reserved for devices such as SpaceMouse. |
| `articulation` | optional typed object | Any joint set, gestures, pinch/grasp, and joint confidence. |
| `metadata` | JSON object | Observation provenance; no control semantics. |
| `extensions` | namespaced JSON object | Forward-compatible experiments such as `vendor.feature`. |

`Pose6D` is exactly three finite meters and a finite unit quaternion in `x,y,z,w`
order. No Euler angles cross the protocol. UMIP rejects non-unit quaternions; it
does not normalize or otherwise filter them.

`Timestamp` contains signed-domain-independent non-negative nanoseconds, a
required `clock_id`, and optional uncertainty. Two values may be subtracted only
when their clock IDs match. Capture and device timestamps may legitimately be
the same value with different semantics, but fields are never collapsed.

## Tracking and ordering semantics

- `tracking` requires a real wrist pose.
- `not_tracking` and `disconnected` must contain no wrist or palm pose. This
  prevents downstream consumers from mistaking stale values for observations.
- `limited` may carry a valid pose whose provider quality is degraded.
- Recovery is a normal subsequent tracking sample; sequence numbers continue.
- Providers expose source order. Diagnostics report gaps and out-of-order data;
  the input platform does not reorder, smooth, extrapolate, or generate poses.

## Articulation and future devices

`HandArticulation` uses semantic joint names rather than a fixed SDK enum. Quest
can carry OpenXR's 26 points; a MediaPipe provider can carry its 21; mocap or a
glove can carry another named set. Each joint has pose, tracking state, optional
radius, and optional confidence. Gesture records have name, active state,
optional confidence, and optional scalar value. Pinch and grasp strengths are
reserved `[0,1]` fields.

Relative devices set `motion_kind=relative_pose_delta` and identify the logical
motion semantic in provider metadata. Integrating such input into an absolute
target is a downstream policy and is not performed here. This preserves device
truth and avoids hiding trajectory generation in a provider.

## Compatibility policy

- Major versions change required meaning and require a new reader.
- Minor versions may add optional fields, enum-independent names, or recording
  record types. Same-major readers ignore unknown top-level fields/record types.
- Required fields are never silently reinterpreted.
- Extensions must be namespaced and JSON-compatible. Promoted extensions get a
  typed optional field in a future minor revision.
- Device SDK objects, numeric enums, native handles, and engine transforms never
  appear in serialized UMIP.

## Recording format

`.umip.jsonl` is UTF-8 newline-delimited JSON:

1. one `header` with recording format `1.0`, UMIP version, recording ID, UTC
   creation time, device descriptor, and metadata;
2. zero or more `sample` records containing canonical UMIP JSON;
3. an optional `footer` with count and clean-close time.

Every complete line is recoverable if capture is interrupted. Creation uses
exclusive mode to prevent accidental overwrite. Unknown same-major record types
are skipped. A missing footer means interrupted/unfinalized, not corrupt.

Replay preserves recorded sample identity and timestamps and changes only
delivery timing: as-recorded, fixed rate, or immediate. It never edits poses.

## Diagnostics definitions

- frequency: reciprocal of mean positive capture interval, falling back to
  receive interval;
- frame drops: positive sequence gaps;
- timestamp jitter: RMS interval deviation from the median;
- latency: receive minus capture only for identical clock IDs;
- processing latency: processing minus receive only for identical clocks;
- CPU: process CPU time divided by diagnostic wall interval;
- confidence: min/mean/p95/max of reported values;
- interruptions/recoveries: transitions between tracking/limited and other
  states, with duration when receive timestamps are comparable.

Distribution samples use a bounded 100,000-observation rolling window, while
counts remain lifetime totals. Diagnostics therefore have bounded memory during
long-running capture.

## Proposed future teleoperation interface

After review, the other team can accept a `MotionInputProvider` dependency or a
callback that receives `MotionInputSample`. It should validate protocol major
version and required semantic/frame configuration once at session startup. Any
world registration, calibration, filtering, scaling, safety, IK, target
generation, or command behavior stays on its side of the boundary.

No teleoperation code is modified by this proposal.
