# Repository layout

| Path | Role | Status |
|---|---|---|
| `src/motion_input` | device-neutral input model, HTS/CTRL providers, replay | current |
| `src/quest_jaka_sim` | clutch, mapping, filters, continuation/IK, simulation | current primary |
| `src/teleoperation` | accepted-target, output feasibility, JAKA adapter | current primary |
| `native/jaka_servo_worker` | physical transport/safety worker | current, gated |
| `src/rh56_driver` | RH56 schema and backends | current parallel path |
| `src/jaka_driver_adapter`, `src/robot_bringup` | bring-up/legacy adapters | current or compatibility, not shared Quest authority |
| `src/teleop_tools` | HEBI/iPhone experiments | active parallel/legacy |
| `src/vision_interface` | perception and RealSense calibration | active parallel |
| `configs` | versioned examples and policy | current; local device facts must be verified |
| `data/sim_assets`, `models` | simulation assets | current; see their READMEs |
| `tests` | default offline test suite | current |
| `docs/history` | evidence and superseded narrative | historical, immutable outcomes |
| `third_party` | vendor/reference snapshots | do not treat as project style or edit casually |

The repository retains only parallel paths that still have an identified use.
They do not override the primary Quest/JAKA contracts.
