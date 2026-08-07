# RH56 H0 MuJoCo self-test

## Purpose and safety boundary

H0 is a low-amplitude, simulation-only check of the six semantic Inspire
RH56DFX actuator channels in the mounted JAKA/RH56 MuJoCo model. It verifies
model loading, channel-to-actuator binding, command limits, finite state, log
output, and an unchanged arm target.

The current H0 call chain is:

```text
tools/rh56_h0_self_test.py
  -> rh56_sim.Rh56H0SelfTest
  -> MuJoCo model/data
  -> six RH56 MuJoCo position actuators
```

It has no Quest receiver, network transport, serial transport, robot SDK, or
physical actuator adapter. Running H0 does not authorize or validate JAKA or
RH56 hardware. Its result must be described as offline simulation evidence.

## Maintained model contract

H0 defaults to:

```text
model:      assets/jaka_rh56_visual_coacd.xml
arm config: configs/sim/quest_hts_jaka_mini2_live_demo.yaml
```

The model contains:

- six range-limited JAKA position actuators;
- 12 RH56 joints and six range-limited RH56 position actuators;
- 148 reviewed active convex RH56 collision hulls;
- collision-disabled vendor visual geometry;
- seven reviewed adjacent-body RH56 contact exclusions, plus the current
  JAKA Link 0/Link 1 exclusion;
- six equality constraints for passive joint following.

The six equality constraints are only an underactuated-hand approximation:

- thumb PIP and DIP follow the directly actuated thumb-close joint using cubic
  polynomials fitted to the local command/angle table;
- each non-thumb distal joint follows its directly actuated MCP joint.

This deterministic six-input model does not reproduce tendon compliance,
backlash, load-dependent coupling, calibrated current/force limits, passive
joint state, or contact sensing. The RH56 feedback names `ANGLE_ACT`,
`CURRENT`, `FORCE_ACT`, `ERROR`, and `STATUS` are not produced by H0, and the
model must not be presented as measuring them.

The fixed hand mount transform is inherited from the maintained combined
model. No independent physical mount calibration in this repository proves
that transform, so H0 validates the model as committed rather than physical
mount accuracy.

## Semantic channels

Upper layers use canonical names and must not infer semantics from MuJoCo
actuator order.

| Canonical channel | Direct actuator | Direct joint | Joint/control range (rad) | Positive motion |
|---|---|---|---:|---|
| `index` | `rh56_R_index_MCP_joint_act` | `rh56_R_index_MCP_joint` | 0--1.70 | close |
| `middle` | `rh56_R_middle_MCP_joint_act` | `rh56_R_middle_MCP_joint` | 0--1.68 | close |
| `ring` | `rh56_R_ring_MCP_joint_act` | `rh56_R_ring_MCP_joint` | 0--1.70 | close |
| `pinky` | `rh56_R_pinky_MCP_joint_act` | `rh56_R_pinky_MCP_joint` | 0--1.70 | close |
| `thumb_close` | `rh56_R_thumb_MCP_joint2_act` | `rh56_R_thumb_MCP_joint2` | 0--0.698132 | bend/close |
| `thumb_lateral` | `rh56_R_thumb_MCP_joint1_act` | `rh56_R_thumb_MCP_joint1` | 0--1.396263 | lateral/opposition |

The canonical execution order is:

```text
[index, middle, ring, pinky, thumb_close, thumb_lateral]
```

H0 resolves each actuator and joint by name, verifies that every direct joint
and actuator is limited, and commands only the intersection of those ranges.

## Initial contact boundary

The canonical runtime model has zero contacts after a direct MuJoCo
load/forward at model `qpos0`. H0 then applies the six configured initial arm
joints and open-hand targets before forwarding the model; the maintained
configuration also reports zero penetrating contacts.

This is distinct from the committed integrated workspace derivative
`models/digital_twin/workspace_scene.xml`, which currently reports four
duplicate approximately -3 mm Link 0/Link 1 self-contact records at its model
`qpos0`. H0 does not load that derivative. See
`docs/digital_twin/README.md` for the regeneration and validation boundary.

Zero initial contact does not prove that every combined thumb-close and
thumb-lateral target is collision-free. Extreme commands can bring the
conservative thumb and index convex hulls into contact. H0 deliberately uses a
bounded low-amplitude, one-channel-at-a-time motion and is not a full hand
collision certification.

## Test sequence

For each channel, H0:

1. starts from the open/neutral hand target;
2. moves through a smoothstep trajectory toward a positive excursion;
3. returns to neutral;
4. attempts a negative excursion only when legal range exists;
5. returns to neutral before moving to the next channel.

The default excursion is 15% of each channel's effective range.
`--amplitude-scale` is rejected unless it is in `(0, 0.20]`. At the current
open lower-limit pose, a negative excursion is logged as
`negative_skipped_illegal` instead of commanding beyond the limit.

Throughout the sequence, the six JAKA actuator targets remain equal to their
configured initial targets. On normal completion, viewer close, or Ctrl-C, H0
restores the initial hand and arm targets in a `finally` path.

## Run and verify

Run these commands from the repository root. They are all offline.

Check that the generated runtime asset is current:

```bash
.venv/bin/python tools/build_rh56_visual_coacd_runtime_asset.py --check
```

Run the focused regression tests:

```bash
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m pytest -q validation/rh56/test_h0_self_test.py
```

Run a short headless smoke test:

```bash
PYTHONPATH=src .venv/bin/python tools/rh56_h0_self_test.py \
  --headless \
  --cycle-seconds 0.05 \
  --amplitude-scale 0.01 \
  --repeat 1 \
  --log-path artifacts/rh56_h0/smoke.jsonl
```

For optional visual inspection on a graphical workstation:

```bash
PYTHONPATH=src .venv/bin/python tools/rh56_h0_self_test.py --viewer
```

The default headless run uses `--cycle-seconds 2.0`,
`--amplitude-scale 0.15`, and `--repeat 1`. If `--log-path` is omitted, output
is written to timestamped JSONL under `logs/rh56_h0/`.

## Result interpretation

The process exits with status 0 only when:

- all six channel sequences complete;
- no non-finite command or state is observed; and
- the arm actuator target is unchanged.

Each JSONL record includes simulation and host monotonic time, repeat and phase,
canonical channel, actuator and joint names, requested and clipped control,
actual direct-joint position, joint/control ranges, saturation, finite-value
status, and phase progress.

A successful run supports only these claims:

- the maintained MuJoCo model loads;
- its six semantic direct-actuator mappings and limits are internally
  consistent;
- the bounded H0 sequence remains finite;
- the arm target is not changed by the H0 runner.

It does not validate physical direction, physical mount alignment, passive
joint motion, current or force feedback, collision safety over the full command
space, teleoperation retargeting, or any real-device behavior.
