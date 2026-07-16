# Motion Input Platform

Status: input-only implementation complete; review is required before any
teleoperation integration.

The platform turns device observations into the Unified Motion Input Protocol
(UMIP), records and replays the same values, visualizes hands and frames, and
reports stream quality. It imports no robot package and issues no commands.

```text
Quest / future device
        |
device-isolated provider
        |
      UMIP 1.0
     /   |    \
record replay diagnostics/visualization
        |
future teleoperation adapter (not implemented here)
```

## Components

- `src/motion_input/model.py`: immutable UMIP values and invariants.
- `src/motion_input/provider.py`: common live/replay provider lifecycle.
- `src/motion_input/quest.py`: Quest wire parser, UDP source, UMIP translator.
- `integrations/quest_unity/`: input-only Unity XR Hands publisher.
- `src/motion_input/recording.py` and `replay.py`: versioned recording and live-equivalent replay.
- `src/motion_input/visualization.py`: text and optional 3-D hand-frame view.
- `src/motion_input/diagnostics.py`: rate, drops, ordering, jitter, latency,
  confidence, interruption, and process CPU statistics.
- `tools/umip_motion_input.py`: operator CLI.

Detailed documents:

- [Repository audit](REPOSITORY_AUDIT.md)
- [Quest SDK review](QUEST_SDK_REVIEW.md)
- [UMIP protocol](UMIP_PROTOCOL.md)
- [Coordinate frames](COORDINATE_FRAMES.md)

## Usage

Record Quest datagrams without contacting any robot:

```bash
.venv/bin/python tools/umip_motion_input.py record data/quest/session.umip.jsonl \
  --allowed-sender 192.168.1.50 --duration-sec 60
```

Replay, visualize, and report:

```bash
.venv/bin/python tools/umip_motion_input.py replay data/quest/session.umip.jsonl
.venv/bin/python tools/umip_motion_input.py visualize data/quest/session.umip.jsonl
.venv/bin/python tools/umip_motion_input.py diagnose data/quest/session.umip.jsonl \
  --output data/quest/session.diagnostics.json
```

Install `.[motion-input-viz]` and add `--matplotlib` for the optional 3-D
tracking-origin, wrist, and palm triads. There is no robot model.

## Validation

```bash
.venv/bin/python -m pytest -q \
  tests/test_motion_input_protocol.py \
  tests/test_motion_input_recording_replay.py \
  tests/test_quest_motion_provider.py \
  tests/test_motion_input_diagnostics.py
```

Hardware-free coverage includes disconnect, loss/recovery, sequence and time
ordering, serialization, crash-recoverable recording, replay timing, coordinate
conversion, diagnostics, visualization state, and 20,000 samples of sustained
streaming. Unity compilation, Quest 3 runtime behavior, and device rate/latency
remain explicit hardware validation items.

## Integration gate

The future teleoperation session may depend only on `MotionInputProvider` and
`MotionInputSample`. It must not import `motion_input.quest`, Unity, OpenXR, or
Meta types. No such adapter is added in this change because that framework is
owned by another session.
