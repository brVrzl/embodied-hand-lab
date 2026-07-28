# Quest CTRL host transport gate

Status: current input-only transport reference. The original local reference
clone used for the audit has been removed; the fixed upstream commit below is
the recoverable source record.

This gate exercises only the host input boundary for a Quest left controller.
It does not start MuJoCo, build a robot target, import a JAKA SDK, or connect to
an Inspire/RH56 device.

## Audited Quest source

The reviewed Quest source is
[`brVrzl/hand-tracking-streamer` commit `5b8eac7e`](https://github.com/brVrzl/hand-tracking-streamer/tree/5b8eac7e30ce12481b89b123099693bc658bc578),
branch `feature/mixed-input-log-probe`. The Unity scene contains an enabled
`LeftControllerPacketSender` on the active `MixedInputLogProbe` object. Its
configured period is 1/60 s and it reuses the existing UDP destination and port.
The right-hand source remains attached to the mixed-input probe. This is direct
source/scene evidence that the build can publish right bare-hand tracking and
left-controller facts simultaneously; the remote `main` commit `5ff7c1c` does
not contain this extension.

The Quest sender keeps the original wrist, 21-landmark, and head lines
unchanged. It adds one independent line:

```text
CTRL,v=1,session=987654321,seq=123,t_ns=123456789012345,connected=1,active=1,tracked=1,index=0.123456,grip=0.654321
```

The host requires the exact field set and order. Integers are unsigned 64-bit
ASCII decimal values, booleans are exactly `0` or `1`, and trigger values must
be finite and in `[0,1]`. A CTRL datagram contains exactly one non-empty UTF-8
line. The parser rejects malformed UTF-8, missing, duplicate, unknown or
reordered fields, unsupported versions, invalid integers/booleans, NaN/Inf,
out-of-range analog values, and non-empty trailing content. It does not perform
hysteresis or any robot operation.

The legacy HTS parser ignores exact `CTRL,` lines and preserves all prior
hand/head validation. The new transport dispatcher sends a CTRL datagram to the
strict controller parser and every other datagram through the original HTS
parser. Both therefore share the single UDP socket on port 9000 without parser
cross-coupling.

## Host time, session, and sequence policy

`session`, `seq`, and Quest `t_ns` are retained. Staleness is computed only as

```text
host_monotonic_now - host_receive_monotonic
```

Quest `t_ns` is never subtracted from a host clock. Within one session it is
used only for source interval, pause, and timestamp-reorder diagnostics.

- `seq + 1` and forward gaps are accepted; gaps are counted.
- A duplicate or decreasing sequence is counted but cannot replace or refresh
  the latest accepted sample.
- A previously unseen session is a sender restart and may reset `seq`.
- The previous session is retired. Its delayed packets cannot replace the
  current session or create a clutch edge.
- The same session may recover after a host-side stale interval. Recovery is
  not an automatic re-engagement.

The three Quest facts remain separately visible. Initial controller validity is
`connected && active && tracked && fresh`. A false fact, a malformed CTRL, no
sample, or staleness makes both clutch inputs invalid immediately.

## Dual-clutch boundary

The canonical provider-independent clutch implementation now lives in
`motion_input.clutch`; `quest_jaka_sim.clutch` is a compatibility re-export.
This lets the input-only gate use the exact existing `AnalogClutchSample` and
`AnalogHoldToRun` types without importing the simulation package.

The mapping is fixed:

| CTRL fact | Existing channel | Press | Release |
|---|---|---:|---:|
| `index` | arm | `>= 0.75` | `<= 0.55` |
| `grip` | hand | `>= 0.75` | `<= 0.55` |

The channels are independent and are not combined into a mode enum. After
startup, sender restart, invalid facts, malformed CTRL, or stale input, the
adapter blocks both samples until one valid packet observes both index and grip
in the released range. The existing per-channel hysteresis then requires a
later press edge. Left-controller pose is absent and cannot influence a target.

The live gate has exactly one source, `live_udp_only`. It exposes no fake,
replay, keyboard, mode, MuJoCo, or hardware option. Deterministic values remain
available only by directly constructing packets in tests, so they cannot run
alongside its UDP receiver.

## Bounded transport-only gate

From the repository root:

```bash
PYTHONPATH=src .venv/bin/python \
  tools/quest_controller_transport_gate.py \
  --bind 0.0.0.0 --port 9000 --project-ip <HOST_IPV4> \
  --print-hz 5 --required-data-timeout-sec 20 --duration-sec 180
```

The terminal emits a 5 Hz JSON summary. The timestamped directory
`logs/quest_transport_gate/` receives a full raw datagram recording, exception
and summary log, and final JSON report. If either CTRL or right-hand data has
not appeared within 20 seconds, the gate stops with exit status 3. No listener
is created merely by importing any module.
