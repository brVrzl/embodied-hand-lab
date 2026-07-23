# Repository layout

| Path | Role | Status |
|---|---|---|
| `src/motion_input` | device-neutral input model, HTS/CTRL providers, replay | current |
| `src/quest_jaka_sim` | clutch, mapping, filters, continuation/IK, simulation | current primary |
| `src/teleoperation` | accepted-target, output feasibility, JAKA adapter | current primary |
| `native/jaka_servo_worker` | physical transport/safety worker | current, gated |
| `src/rh56_driver` | RH56 schema and backends | current parallel path |
| `src/jaka_driver_adapter`, `src/robot_bringup` | bring-up/legacy adapters | current or compatibility, not shared Quest authority |
| `src/teleop_tools` | Xbox/HEBI/iPhone/TeleDex experiments | active parallel/legacy |
| `src/pregrasp`, `src/vision_interface`, `src/data_recorder` | research/data areas | active parallel |
| `configs` | versioned examples and policy | current; local device facts must be verified |
| `data/sim_assets`, `models` | simulation assets | current; see their READMEs |
| `tests` | default offline test suite | current |
| `docs/history` | evidence and superseded narrative | historical, immutable outcomes |
| `third_party` | vendor/reference snapshots | do not treat as project style or edit casually |

The repository intentionally contains several parallel research paths. A file
outside the primary Quest/JAKA pipeline is not obsolete merely because another
path is newer.
