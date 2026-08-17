# Hand Tracking Streamer input integration

> **Status: historical snapshot, 2026-07-23.** This document preserves the
> integration evidence and workspace state recorded at that time. It is not the
> current operating guide. See
> [`docs/motion_input/README.md`](../../../motion_input/README.md) and
> [`docs/operation/quest_setup.md`](../../../operation/quest_setup.md).

Status: implementation, live input validation, and offline replay validation
complete. The required right-hand stream loss-and-recovery safety gate passes.
Selective per-hand loss is not supported or not demonstrated by the streamer;
it is not a production requirement. Nothing in this path can command JAKA or
Inspire.

## Checkpoint and ownership

- repository: `/home/thor/projects/embodied_lab`
- selected base: `8ecd70b0ceb6249cc75877c315d8a7d7afc9a472`
- base branch: `feature/quest-motion-input-platform`
- child branch: `feature/quest-hand-tracking-streamer-integration`
- isolated worktree: `/home/thor/projects/embodied_lab_quest_input`

`8ecd70b` is the correct base because it is the dedicated Quest/UMIP commit,
directly based on `main` at `d45875a`; it contains no later JAKA control
foundation or TeleDex live-control commits. The shared primary checkout was
dirty with camera, digital-twin, TeleDex, JAKA, and other workstream changes.
No dirty file was copied, reset, stashed, or edited.

Owned files for this task:

- `src/motion_input/hts_protocol.py`
- `src/motion_input/hts_transport.py`
- `src/motion_input/hts_canonical.py`
- `src/motion_input/hts_operator.py`
- `src/motion_input/hts_gate.py`
- `src/motion_input/hts_telemetry.py`
- `src/motion_input/__init__.py` (exports only)
- `configs/motion_input/quest_hts_right_hand.yaml`
- `tools/quest_hand_tracking_streamer.py`
- `tests/test_hts_protocol.py`
- `tests/test_hts_canonical.py`
- `tests/test_hts_recording_replay.py`
- `tests/test_hts_operator.py`
- `docs/motion_input/README.md`
- this document

## Protocol findings and evidence

The store app is the open-source `wengmister/hand-tracking-streamer`. Findings
were checked against the tagged v1.1.0 source at commit
`5ff7c1cfea0ead1bb8a0e233bc7770d94d31feb5` and the official Python SDK v1.1.0
at `6ca05d1d5b00dfd89535aef74f698d6765bf1229`:

- <https://github.com/wengmister/hand-tracking-streamer/blob/v1.1.0/CONNECTIONS.md>
- <https://github.com/wengmister/hand-tracking-streamer/blob/v1.1.0/hand_tracking_streamer/Assets/Scripts/HandLandmarkStreamer.cs>
- <https://github.com/wengmister/hand-tracking-streamer/blob/v1.1.0/hand_tracking_streamer/Assets/Scripts/HeadPoseStreamer.cs>
- <https://github.com/wengmister/hand-tracking-sdk/tree/v1.1.0>

The app supports wireless UDP, wireless TCP, and ADB-reversed wired TCP. This
gate selects wireless UDP on configurable port `9000`, the documented default.
The receiver binds `0.0.0.0:9000`; the Quest target must be the host's physical
Wi-Fi address, not the bind wildcard.

The wire is unversioned UTF-8 CSV. Each valid hand update is one UDP datagram
with two newline-delimited records:

```text
Right wrist:, x, y, z, qx, qy, qz, qw
Right landmarks:, x0, y0, z0, ... x20, y20, z20
```

When the app's debug-info option is enabled, labels include paired source
metadata:

```text
Right wrist | f = 123 | t = 987654321:, ...
Right landmarks | f = 123 | t = 987654321:, ...
Head pose | f = 45 | t = 987654999:, ...
```

Head pose, when enabled, is a separate one-line datagram. Hands target 100 Hz
in source (`0.01 s`); head targets 30 Hz. The live UDP capture delivered each
enabled stream at about 71 Hz, with batching. The source documents wired TCP at
roughly 70 Hz, but TCP is outside this gate.

Field availability:

| Field | HTS v1.1 availability |
|---|---|
| head position/orientation | optional app setting; Unity world, metres, XYZW |
| left/right wrist position/orientation | explicit side; Unity world, metres, XYZW |
| 21 hand landmark positions | wrist-local, metres, fixed order below |
| landmark orientations | unavailable |
| confidence | unavailable |
| explicit tracking-valid/loss record | unavailable; app stops that hand's packets |
| source timestamp | only with debug-info headers; Quest monotonic clock |
| source sequence | only with debug-info headers; separate counter per stream |

Landmark order is wrist; thumb metacarpal/proximal/distal/tip; then
proximal/intermediate/distal/tip for index, middle, ring, and little fingers.
The parser requires exactly 7 pose floats or 63 landmark floats and rejects
truncation, extra values, invalid UTF-8, NaN/infinity, malformed headers, and
quaternions outside a bounded norm tolerance. Rounded source quaternions are
checked and then normalized for the canonical state.

HTS sends no tracking-loss packet. `tracking_valid` is therefore an explicit
host inference: a hand is valid only while a complete paired wrist/landmark
datagram is no older than the configured stale threshold (default 250 ms).
When stale, the canonical contract removes the pose and joints rather than
reusing old data. Recovery requires a new complete datagram.

Live occlusion evidence indicates that the app publishes its enabled hand
streams as a shared group: attempting to occlude only the right hand stopped
both left and right datagrams while head datagrams continued. Independent
per-hand loss publication is therefore unsupported or unverified at the app
level. Repeating the same selective-occlusion experiment is not required
without new evidence that the app can publish the two hands independently.

Source and host monotonic clocks do not share an epoch. Absolute one-way
latency/packet age cannot be claimed from their numeric difference. The tool
reports host stream age and source/host interval behavior; source sequence gaps
are reported only when debug headers exist.

## Frames and conversion

Raw HTS convention is Unity left-handed: +X right, +Y up, +Z forward. Wrist and
head are in the Unity world/tracking-origin frame. Landmark positions are local
to their respective wrist. Units are metres and quaternion order is XYZW.

This repository's existing UMIP target uses an OpenXR-style right-handed basis:
+X right, +Y up, -Z forward. The single numeric basis conversion is:

```text
position:   ( x,  y,  z) -> ( x,  y, -z)
quaternion: (qx, qy, qz, qw) -> (-qx, -qy, qz, qw)
```

The official SDK also offers other right-handed/robotics bases, including a
generic Y reflection and RFU/FLU mappings. Those are not silently mixed into
this contract. The chosen conversion preserves the repository's documented
OpenXR-style +Y-up frame. The live operator sequence verified canonical X for
lateral motion, sign-reversed canonical Z for forward/back motion, and smooth
normalized orientation changes during wrist rotation.

Frame meanings:

| Frame | Meaning and current status |
|---|---|
| raw HTS Unity world | source tracking origin; left-handed; never exposed as robot coordinates |
| `quest_world` | basis-converted source tracking origin; session origin is not calibrated |
| `quest_head` | tracked head child pose in `quest_world`, when enabled |
| `left_wrist`, `right_wrist` | wrist-local frames; landmarks are expressed here |
| `canonical_operator` | reference-local, bounded relative operator transform; offline only |
| `future_robot_base` | reserved name only; no Quest-to-robot registration exists |

No absolute Quest position is mapped to a JAKA TCP target.

## Network checkpoint

Observed before the live gate:

- physical Wi-Fi interface: `wlP1p1s0`, `10.24.1.68/16`, link up
- default route: `10.24.0.1` via `wlP1p1s0`, source `10.24.1.68`
- robot-side Ethernet: `enP2p1s0`, `192.168.71.19/24` (must not be entered)
- loopback, Docker, and L4T bridge addresses are not candidates
- UDP port 9000 was unused before starting the receiver
- `ufw` and `firewall-cmd` are not installed; nftables/iptables rules require
  root and could not be read, so successful Quest reception is the firewall gate

Selected values:

```text
PROJECT_IP=10.24.1.68
PORT=9000
TRANSPORT=UDP
```

Start a bounded 120-second validated capture from the isolated worktree:

```bash
cd /home/thor/projects/embodied_lab_quest_input
PYTHONPATH=src python3 tools/quest_hand_tracking_streamer.py live \
  --project-ip 10.24.1.68 --port 9000 --duration-sec 120 \
  --stale-ms 250 --frozen-sec 2
```

The tool prints the exact IP/port, writes a timestamped raw JSONL file under
ignored `logs/quest_input/`, displays a terminal wrist table, and writes a JSON
report. Raw replay uses the same parser and canonical assembler:

```bash
PYTHONPATH=src python3 tools/quest_hand_tracking_streamer.py replay \
  logs/quest_input/<capture>.hts.jsonl
```

## Diagnostic live motion and schema gate

Configure UDP, both hands, port 9000, head pose if available, and debug info if
available. Then perform, slowly:

1. hold both hands still in front of the headset;
2. move only the right hand left/right;
3. move only the right hand forward/backward;
4. rotate the right wrist;
5. open and close the right hand;
6. repeat briefly with the left hand;
7. move one hand outside tracking view, then return it.

Diagnostic acceptance requires correct side identity, axis changes matching the documented
basis, loss followed by recovery, plausible rates, 21 joints per valid hand,
unit-near wrist quaternion norms, no unexplained discontinuities, and no parser
rejections. A repeated-value warning is only a potential frozen-stream signal;
intentional stillness must be distinguished from a transport freeze using the
source sequence and subsequent motion.

## Observed live results

All captures are ignored runtime evidence under `logs/quest_input/`; they are
not part of the source diff.

The first 180-second attempt received zero datagrams because the receiver closed
before headset setup completed. The successful motion capture was:

```text
logs/quest_input/quest_live_retry_20260717T1704+0800.hts.jsonl
logs/quest_input/quest_live_retry_20260717T1704+0800.hts.report.json
```

The first Quest packet arrived 411.327 seconds after the bounded receiver
started. The captured source was `10.24.0.78`, with separate UDP source ports
for head, left, and right streams. Over 85.596 seconds the receiver observed:

- 18,238 datagrams / 7,878,325 bytes / 30,366 parsed lines;
- 6,110 one-line head datagrams, about 71.38 Hz;
- 6,064 two-line left-hand datagrams, about 70.85 Hz;
- 6,064 two-line right-hand datagrams, about 70.85 Hz;
- exactly 21 landmarks in every parsed hand frame; zero missing joints;
- zero malformed datagrams;
- raw quaternion norms: left 0.999282–1.000677 (mean 1.000121), right
  0.999245–1.000767 (mean 0.999920); canonical quaternions were normalized;
- UDP batching: per-hand median interarrival about 7.4–7.6 ms and p95 about
  46.8–47.1 ms;
- one simultaneous startup hand-stream gap of about 0.755 seconds while head
  continued; both hands recovered;
- no source timestamps or source sequences because the app used legacy labels,
  so one-way latency and source sequence loss are not measurable. Empty gap
  dictionaries must not be interpreted as proof of zero packet loss.

Motion segmentation preserved labels and side identity during the commanded
sequence. In the right-hand lateral phase, one-second X ranges were 0.122 m and
0.089 m while Y/Z ranges stayed at or below 0.019 m. In the following
forward/back phase, source Unity Z ranges were 0.095 m and 0.060 m; after the
documented conversion this is canonical Z with reversed sign (+Unity Z forward
becomes -canonical Z forward). During right-wrist rotation, orientation changed
about 129–169 degrees per one-second window while each positional-axis range
stayed at or below about 0.020 m. The later left-hand phase changed left pose
and orientation while the right hand was nearly still, supporting preservation
of the explicit Left/Right labels.

Potential frozen-pose counters were left=4 and right=3. Raw inspection showed
these were long runs of source packets with exactly repeated, rounded wrist
poses, primarily during intentional setup/still periods and while the opposite
hand moved. They are retained as warnings because HTS supplies no confidence or
tracking-valid bit; the receiver cannot prove that every repeated run is
intentional stillness.

### Selective per-hand loss gate: NOT SUPPORTED / NOT VERIFIED

The isolated capture was:

```text
logs/quest_input/quest_occlusion_gate_20260717T1720+0800.hts.jsonl
logs/quest_input/quest_occlusion_gate_20260717T1720+0800.hts.report.json
```

It contained 11,959 valid datagrams over 59.611 seconds: 4,117 head frames and
3,921 frames for each hand, with 21 joints per hand and zero malformed data.
The intended condition was left continuously visible while only the right hand
was hidden. Instead, left and right packet gaps were aligned:

| Gap | Left | Right | Head |
|---|---:|---:|---:|
| 1 | 2.520 s | 2.520 s | continued (no >250 ms gap) |
| 2 | 0.652 s | 0.650 s | two batched gaps of 0.334/0.251 s |
| 3 | 0.788 s | 0.788 s | 0.788 s gap |

For gap 1, both hands crossed the 250 ms threshold and became `NOT_TRACKING`
while head remained live; both then recovered to `TRACK` with 21 joints. Thus:

- stale detection and recovery work correctly;
- right-hand recovery with 21 joints is verified;
- the visible left hand did **not** remain `TRACK` during isolated right-hand
  occlusion;
- selective per-hand tracking-loss behavior is not supported or verified by
  this streamer behavior;
- this is not a receiver failure and does not block right-hand-only production
  input semantics.

The isolated capture also contained 777 paired samples where left and right
wrist positions were identical to HTS's four-decimal precision while their
orientations differed. This does not prove the labels were swapped, but it is a
cross-hand diagnostic ambiguity. The production interface consumes only the
explicit `Right` stream and has no left-hand dependency.

### Required right-hand stream loss-and-recovery safety gate: PASS

The production chain requires the Quest right hand only. The left hand is an
optional diagnostic stream and head pose is an optional diagnostic/reference
stream. Neither can keep a stale right hand valid or independently disengage a
right-hand mapping that does not explicitly require them. The repository YAML
configuration is `configs/motion_input/quest_hts_right_hand.yaml`:

```yaml
required_hand: right
left_hand_required: false
head_pose_required: false
required_joint_count: 21
stale_after_ms: 250
```

The existing occlusion recording was replayed through the same parser,
canonical assembler, and offline safety state machine. It provides sufficient
evidence; another live run was not necessary:

```bash
PYTHONPATH=src python3 tools/quest_hand_tracking_streamer.py replay \
  logs/quest_input/quest_occlusion_gate_20260717T1720+0800.hts.jsonl \
  --pose-table-hz 2 \
  --report logs/quest_input/quest_occlusion_gate_right_hand_replay.report.json
```

Replay results:

- 3,921 distinct right-hand frames were valid with exactly 21 joints;
- zero invalid right-hand states retained a wrist pose or joints;
- zero malformed datagrams were present;
- the primary publication gap was 2.519528 seconds;
- canonical validity crossed to stale at 0.250000001 seconds after the last
  right-hand sample; head remained valid at that boundary;
- an offline `ENGAGED` pipeline transitioned to `DISENGAGED` with reason
  `right_hand_stale`, invalidated its reference, and emitted emergency-neutral;
- recovery contained 21 joints but remained `DISENGAGED` and invalid for
  mapping;
- subsequent recovery also did not auto-engage; explicit engagement and a new
  reference capture are required;
- jump and workspace-envelope tests prove discontinuities are rejected rather
  than exposed as valid relative targets.

The revised gate therefore passes all required-right-hand interruption and
recovery semantics. It does not claim independent left/right publication.

## Offline tests

```bash
python3 -m pytest -q \
  tests/test_hts_protocol.py \
  tests/test_hts_canonical.py \
  tests/test_hts_recording_replay.py \
  tests/test_hts_operator.py
```

Coverage includes parser schema, malformed/truncated/extra data, invalid
floats, quaternion checking/normalization, axis and 90-degree rotation
conversion, units, side preservation, stale loss/recovery, sequence gaps,
replay equivalence, raw inspection, and static absence of robot imports.

The operator tests additionally cover the right-only YAML configuration,
explicit engage/reference capture, stale and tracking-loss disengagement,
malformed/frozen-stream faults, reference invalidation, no automatic recovery,
optional left/head loss, translation and orientation deltas/scaling, low-pass
filtering, jump rejection, operator workspace checks, transition logs, and
capture-based gate replay.

## Safety and next gates

`RightHandOperatorPipeline` is an offline data boundary. Its state machine is:

```text
DISENGAGED
  -> explicit engage with a fresh tracked 21-joint right hand
ARMED_REFERENCE_CAPTURE
  -> explicit valid reference capture
ENGAGED
  -> stale / loss / malformed / frozen / jump / envelope violation
DISENGAGED
```

Every state transition records a host monotonic timestamp and reason. Loss
invalidates the previous reference. Recovery alone never restores engagement;
a new explicit engage and reference capture are mandatory. A neutral output
has zero translation, identity orientation, `valid_for_mapping=false`, and
`emergency_neutral=true`. `prepare_inactive_future_input` remains the older
always-disengaged compatibility boundary. Neither class has a command backend.

The offline relative transform is current right wrist relative to the captured
right-wrist reference in `canonical_operator`. It supports per-axis translation
scaling, relative or disabled orientation mapping, orientation scaling,
low-pass filtering, input-jump rejection, a relative operator-space envelope,
stale rejection, and neutral disengaged output. These are not JAKA base or TCP
coordinates; no robot-base transform exists.

The exact next arm-only offline gate is to calibrate a provisional
`canonical_operator -> future_robot_base` transform and replay the bounded
relative targets through offline JAKA workspace, IK, singularity, velocity,
acceleration, and collision checks. That gate must not connect to the robot and
must retain explicit clutch/reference semantics, visible enable state, stale
stop, and physical emergency-stop readiness before any later hardware gate.

Before Inspire retargeting, a separate offline gate must verify the 21-point
order live, choose joint-angle or fingertip retargeting, enforce Inspire limits
and coupled/tendon behavior, filter observations, calibrate per finger, and
define a safe open-hand fallback.

No JAKA or Inspire connection is made by this implementation. No robot command
has been sent during repository inspection, implementation, or offline tests.
