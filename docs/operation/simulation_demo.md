# Quest/JAKA MuJoCo simulation

This is the recommended first operational workflow. It receives Quest network
input and drives only MuJoCo. It does not import, initialize, log in to, or
command the JAKA or RH56 physical SDKs.

## Prerequisites

```bash
.venv/bin/python -m pip install -e ".[dev]"
./scripts/run_quest_jaka_sim_demo.sh --help
```

Use a Quest Hand Tracking Streamer build with the CTRL v1 left-controller
sidecar. The ordinary upstream hand-only streamer cannot engage the current
controller clutch. Configure the Quest sender to the host IPv4 address and UDP
port 9000 (or the value explicitly selected on both ends).

## Live simulation

From a graphical host:

```bash
./scripts/run_quest_jaka_sim_demo.sh \
  --config configs/sim/quest_hts_jaka_mini2_live_demo.yaml \
  --bind 0.0.0.0 \
  --port 9000 \
  --project-ip <HOST_IPV4> \
  --duration-sec 600 \
  --telemetry-hz 2 \
  --viewer
```

Left index is release-before-press arm clutch/reference capture and hold-to-run.
Left grip independently controls the simulated RH56 hand. Release the trigger
before the first engagement. Keep the first capture still; the arm must not
jump. Candidate rejection displays/records `HOLD_REJECTED` while preserving
the last safe target.

The wrapper can discover a local graphical session when invoked over SSH.
Prefer explicit `--display` and `--xauthority` if discovery is ambiguous.

## Offline and replay modes

Inspect exact options:

```bash
.venv/bin/python tools/quest_jaka_mujoco_sim.py --help
.venv/bin/python tools/quest_jaka_mujoco_sim.py replay --help
.venv/bin/python tools/quest_jaka_mujoco_sim.py replay-6dof --help
.venv/bin/python tools/quest_jaka_mujoco_sim.py live-6dof --help
```

Use committed regression fixtures or an explicitly selected local recording.
Recordings are not automatically committed. Add `--ik-debug` only when detailed
joint, TCP, singularity, continuation, and rejection diagnostics are needed.

## Acceptance checks

- No physical SDK import or connection occurs.
- First engagement is stationary and jump-free.
- Translation and orientation follow the documented frame mapping.
- Release holds/stops the session as documented.
- Recoverable infeasibility holds safely and retreat can recover.
- Input loss or timeout stops rather than replaying stale motion.

See [troubleshooting](troubleshooting.md) for packet, viewer, and rejection
diagnostics.
