# Glossary

- **AcceptedArmTarget** — immutable six-joint target created only after shared
  mapping, IK, continuity, collision, singularity, and output feasibility pass.
- **CTRL v1** — Quest left-controller sidecar packet used for clutch/grip facts.
- **EDG** — JAKA external guidance mode used by the native joint worker.
- **HOLD_REJECTED** — fresh-heartbeat state that holds the last safe target
  after a recoverable candidate rejection.
- **HTS** — Hand Tracking Streamer packet source for Quest hand/head poses.
- **latest destination** — newest accepted target toward which the native
  resampler moves without replaying a queued backlog.
- **plant-free hardware path** — MuJoCo supplies kinematics/collision checks but
  is not stepped as a simulated plant and its `qpos` is not followed.
- **q_hold** — measured joint state after entering EDG; startup continuity
  authority.
- **shared pipeline** — all input, mapping, filter, IK, and feasibility work
  before the simulation/hardware adapter split.
- **UMIP** — device-neutral observation contract in `src/motion_input`.
