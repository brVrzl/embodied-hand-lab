# Motion Input Platform

Status: the device-neutral input platform and Quest HTS/CTRL providers are
implemented and integrated into the current Quest/JAKA simulation and shared
target pipeline. This package describes observations, never robot commands.

```text
Quest / future device
        |
device-isolated provider
        |
      UMIP 1.0
     /   |    \
record replay diagnostics/visualization
        |
explicit teleoperation consumer
```

## Components

- `src/motion_input/model.py`: immutable UMIP values and invariants.
- `src/motion_input/provider.py`: common live/replay lifecycle.
- `src/motion_input/quest.py`: Quest wire parser, UDP source, UMIP translator.
- `src/motion_input/hts_protocol.py`: strict HTS v1.1 CSV schema.
- `src/motion_input/hts_transport.py`: input-only UDP transport and raw replay.
- `src/motion_input/hts_canonical.py`: Unity/OpenXR canonical conversion.
- `src/motion_input/controller_provider.py`: CTRL v1 validation and freshness.
- `src/motion_input/recording.py`, `replay.py`, `diagnostics.py`, and
  `visualization.py`: device-neutral support.
- `integrations/quest_unity/`: input-only Unity publisher sources.

Current references:

- [UMIP observation contract](UMIP_PROTOCOL.md)
- [coordinate-frame contract](COORDINATE_FRAMES.md)
- [Quest controller host transport](QUEST_CONTROLLER_TRANSPORT_HOST.md)
- [Quest SDK/OpenXR review](QUEST_SDK_REVIEW.md)
- [current Quest host setup](../operation/quest_setup.md)

Dated repository audits, streamer integration gates, offline simulation gates,
and the initial dual-clutch design are preserved under
`docs/history/archived_designs/motion_input/`. They no longer define current
branches, local paths, test totals, or integration status.

## Safe usage

Inspect the input-only tools:

```bash
.venv/bin/python tools/umip_motion_input.py --help
.venv/bin/python tools/quest_hand_tracking_streamer.py --help
```

Recordings default under ignored `data/` paths and may contain personal motion
data. Select any sender address and output path for the current trusted network;
do not copy old example addresses as operational truth.

## Focused validation

```bash
.venv/bin/python -m pytest -q \
  tests/test_motion_input_protocol.py \
  tests/test_motion_input_recording_replay.py \
  tests/test_quest_motion_provider.py \
  tests/test_hand_tracking_streamer_provider.py \
  tests/test_quest_controller_transport.py \
  tests/test_motion_input_diagnostics.py
```
