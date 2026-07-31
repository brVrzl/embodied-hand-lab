# embodiment_core

`embodiment_core` contains small shared contracts that do not own a hardware
connection:

- YAML loading and path resolution;
- common observation/action types and logging helpers;
- the project-selected conservative JAKA joint-limit constants;
- the read-only `doctor` inventory;
- the unified offline `embodied-lab` CLI.

Importing this package does not open a robot, hand, camera, headset socket, or
serial port. Physical control remains in separately gated adapters and native
workers.
