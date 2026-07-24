# π0.5-DROID shadow integration

This directory is deliberately inference-only. It opens the two RealSense RGB
streams, accepts an already-produced state snapshot from a JSON file, sends an
OpenPI DROID observation to a websocket policy server, validates the response,
and logs it. It does not import JAKA or RH56 libraries and has no command,
target, servo, publisher, or actuator path.

Run it with the independent OpenPI environment:

```bash
cd /home/thor/projects/embodied_lab/learned_policy/pi05_shadow
/home/thor/projects/openpi/.venv/bin/python camera_probe.py --duration-s 5

# Schema/camera smoke test only; the synthetic state is explicit in artifacts.
/home/thor/projects/openpi/.venv/bin/python shadow_client.py \
  --synthetic-state --no-query

# Inference-only query after a compatible server and a real DROID-format,
# read-only state snapshot are available.
/home/thor/projects/openpi/.venv/bin/python shadow_client.py \
  --state-json /path/to/read_only_droid_state.json \
  --host 127.0.0.1 --port 8000
```

The accepted state file is intentionally strict:

```json
{
  "schema": "openpi.pi05_droid_state.v1",
  "timestamp_ns": 0,
  "source": "name-of-existing-read-only-interface",
  "joint_position": [0, 0, 0, 0, 0, 0, 0],
  "gripper_position": [0]
}
```

At OpenPI commit `15a9616a00943ada6c20a0f158e3adb39df2ccac`,
`DroidInputs` requires two HWC RGB images, seven DROID/Franka joint positions,
one gripper position, and a prompt. The `pi05_droid` config has a 15-step
horizon, and `DroidOutputs` returns eight values per step: seven DROID/Franka
joint-velocity actions and one gripper-position action.

JAKA mini2 has six arm joints. This adapter therefore rejects a six-element
state and never pads it, drops an action dimension, or maps a prediction to the
JAKA/RH56 stack. A validated embodiment adapter or fine-tuned checkpoint is a
separate future task and must remain downstream of the existing accepted-target
and safety gates.
