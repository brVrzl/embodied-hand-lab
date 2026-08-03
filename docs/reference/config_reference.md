# Configuration reference

This is a compact inventory. The authoritative loading, units, environment
variables, and local-copy rules are in
[Configuration](../configuration/CONFIGURATION.md).

Configuration never authorizes hardware. There is no repository-wide implicit
merge and no general `.env` loader. For a field supported by a maintained
entry point, precedence is:

```text
code schema/default < explicitly selected YAML < explicit CLI override
```

Older standalone tools may implement only part of this pattern; use the
owning loader and current `--help`.

## Maintained YAML inventory

| Configuration | Current role and boundary |
| --- | --- |
| `configs/sim/quest_hts_jaka_mini2_live_demo.yaml` | Authoritative shared live Quest/JAKA target-generation policy before either output adapter, including the capped 10-second no-motion Quest input recovery window; also owns the maintained MuJoCo live-scene settings |
| `configs/sim/quest_hts_jaka_mini2_offline.yaml` | Recorded-input and headless/offline simulation policy; deliberately smaller and different from live policy |
| `configs/sim/quest_rh56_retarget.yaml` | Simulation-only Quest-to-RH56 feature calibration |
| `configs/sim/jaka_collision_sweep_poses.yaml` | Offline digital-twin collision-sweep pose samples |
| `configs/motion_input/quest_hts_right_hand.yaml` | HTS receiver and canonical-operator preparation values; not physical robot bounds |
| `configs/benchmark/smoke.yaml` | Deterministic offline JAKA joint-reach/RH56 actuator pre-shape smoke benchmark |
| `configs/hand/rh56_pc_direct_teleop.yaml` | Maintained PC-direct transport, scheduler, channel order, feedback, command bounds, and safety policy; actual stable serial device remains a CLI choice |
| `configs/hand/quest_rh56_real_retarget.yaml` | Maintained live Quest hand-feature calibration used by hand-only and combined physical RH56 entries and the live simulation default; does not own RH56 protocol travel or authorize writes |
| `configs/camera/default_rgbd.yaml` | Small mock RGB-D fixture, not a physical-camera default |
| `configs/camera/realsense_thor.yaml` | Site-specific dual-D435 snapshot with recorded serials; not portable and not end-to-end validated |
| `configs/perception/d435_tabletop.yaml` | Offline tabletop processing parameters; camera-to-JAKA transform and workspace remain explicitly uncalibrated |
| `configs/data_collection/dual_d435_episode.example.yaml` | Copyable dual-D435 settings shared by simulation capture and the separately gated physical v2 collector; local serials/calibration are required |
| `configs/training/distributed.example.yaml` | Proposed future trainer contract; explicitly not consumed by a current ACT, Diffusion Policy, or other trainer |

`digital_twin/configs/` is a separate set of calibration evidence, provisional
scene geometry, transforms, collision classifications, and examples owned by
specific digital-twin tools. It is not loaded globally. Follow the
[digital-twin guide](../digital_twin/README.md).

## Shared arm and scene boundary

The live Quest YAML defines freshness, clutch semantics, frames, provisional
mapping, filters, continuation IK, singularity/joint/collision checks, output
velocity and acceleration feasibility, transport period, and timeouts.
Physical and MuJoCo adapters are identical only through immutable
`AcceptedArmTarget`.

The configured live viewer can load
`digital_twin/configs/workspace.yaml`, including a provisional table.
`SharedJakaTargetGenerator` still uses the base MJCF, so injected scene
geometry is not shared pre-acceptance collision authority and cannot establish
physical clearance.

## Device values are not controller truth

- JAKA IP addresses are explicit command-line gate inputs. Versioned examples
  are not a statement of the current network.
- Actual RH56 serial identity is selected explicitly, preferably through a
  stable `/dev/serial/by-id/...` path. The placeholder in YAML must not be used
  as a device.
- Camera roles are selected by verified serial, never `/dev/video*` ordering.
- Payload, COM, installation, TCP, collision, and controller safety settings
  must be verified at the controller. Software must not silently apply
  recorded operator values.
- A syntactically valid physical YAML is neither a no-motion guarantee nor
  permission to connect.

For RH56, the canonical software order is:

```text
[index, middle, ring, pinky, thumb_close, thumb_lateral]
```

Raw `ANGLE_ACT`, `CURRENT`, `FORCE_ACT`, `ERROR`, and `STATUS` are register
feedback, not a tactile array or complete passive-joint state. MuJoCo hand
actuator radians and physical RH56 register counts are different units.

## Validation

The following offline checks are the applicable repository commands:

```bash
.venv/bin/python -m pytest -q -p no:cacheprovider tests/test_configs.py
.venv/bin/python - <<'PY'
from pathlib import Path
import yaml

files = sorted(Path("configs").rglob("*.yaml"))
for file in files:
    value = yaml.safe_load(file.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise TypeError(f"{file}: YAML root must be a mapping")
print(f"parsed {len(files)} YAML mappings")
PY
```

These checks prove syntax and selected loader contracts only. Hardware
profiles, physical calibration, and device identity require their separately
gated procedures. The historical foundation policy is bounded by its dated
[minimal-joint validation](../history/gates/jaka_foundation_20260716/jaka_gate3c_minimal_joint_validation_20260716.md);
it does not override current configuration.
