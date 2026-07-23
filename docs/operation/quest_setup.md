# Quest host setup

The current live path consumes two datagram families on one UDP port:

- Hand Tracking Streamer hand/head packets.
- CTRL v1 packets from `LeftControllerPacketSender`.

The host receiver timestamps each datagram, validates syntax/finiteness/order,
and keeps a bounded FIFO of 256 observations. Controller freshness defaults to
150 ms; wrist/head freshness defaults to 250 ms; input interpolation delay is
20 ms.

## Safe host checks

These commands only inspect local help:

```bash
.venv/bin/python tools/quest_hand_tracking_streamer.py --help
.venv/bin/python tools/quest_jaka_mujoco_sim.py live-6dof --help
```

Choose the host address explicitly; do not copy an old report's LAN address or
username. Permit the selected UDP port through the local firewall only on the
intended trusted interface. Packet capture and recording can contain personal
motion data and should follow the local data-retention policy.

## Clutch behavior

The arm uses left index trigger. The operator must release before the first
press. A rising press captures wrist, head yaw, and robot TCP references. Hold
to run; release disengages. After stale/lost input or a completed hardware
session, release and press again to capture a fresh reference.

The left grip controls RH56 only in the current simulation integration. Current
Quest-driven physical RH56 teleoperation is not validated.

The audited Unity source/build history is retained in
`docs/motion_input/QUEST_CONTROLLER_TRANSPORT_HOST.md`; it is integration
evidence, not a promise about whichever APK is currently installed.
