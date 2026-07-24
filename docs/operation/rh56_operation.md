# RH56 operation

The repository has three distinct RH56 roles:

1. Mounted MuJoCo RH56 in the Quest/JAKA simulation, with left grip retargeting.
2. A PC-direct USB-RS485 driver/ROS2 bridge in `src/rh56_driver`.
3. Experimental iPhone, HEBI, and JAKA-tool-RS485 workflows.

Canonical command order is:

```text
[index, middle, ring, pinky, thumb_close, thumb_lateral]
```

Do not infer normalized values versus raw counts. Messages and logs should
state schema/version, source, timestamp, unit, and canonical order.

Safe local inspection:

```bash
./scripts/check_rh56_connection.sh --help
./scripts/rh56_pc_direct_bringup.sh --help
.venv/bin/python tools/iphone_mediapipe_hand_teleop.py --help
```

Those commands are help only. A serial port, physical hand, or JAKA tool bus
must not be opened without explicit authorization and a clear stop policy.
Quest grip control is simulation-validated; it is not a validated
Quest-to-physical-RH56 path.

Current model roles and limitations are documented in
`data/sim_assets/README.md`.
