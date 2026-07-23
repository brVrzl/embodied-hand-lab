# Configuration

Versioned YAML is loaded from `configs/`; command-line arguments select
environment-specific values such as bind address, duration, and physical
authorization. Precedence for a current entry point is:

```text
code schema/defaults < selected YAML < explicit CLI override
```

Do not assume every older tool uses that complete hierarchy; verify its
`--help` and loader.

The authoritative Quest live policy is
`configs/sim/quest_hts_jaka_mini2_live_demo.yaml`. Related files:

- `configs/motion_input/quest_hts_right_hand.yaml`: HTS provider/input facts;
- `configs/sim/quest_hts_jaka_mini2_offline.yaml`: recorded/offline simulation;
- `configs/sim/quest_rh56_retarget.yaml`: simulated hand retargeting;
- `configs/robot/jaka_mini2_real.yaml`: physical connection example, not
  authorization or controller truth.

Local IPs, serial devices, display paths, and camera URLs belong on the command
line or in uncommitted local configuration. Example addresses are not a
statement of the current network. Payload/TCP/controller settings are not
owned by these files.
