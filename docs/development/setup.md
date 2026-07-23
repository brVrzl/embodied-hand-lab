# Development setup

Python 3.10 or newer is required:

```bash
cd "$(git rev-parse --show-toplevel)"
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -e ".[dev]"
```

Optional extras are declared in `pyproject.toml`: `data`, `vision-teleop`,
`realsense`, `phone-teleop`, `motion-input-viz`, `teledex-teleop`, `gamepad`,
`sim`, and `asset-tools`. Install only the extras needed for the current
offline task.

ROS2 Humble and vendor SDK dependencies are system/environment concerns and are
not fully installed by pip. The default pytest suite uses fake/offline
backends; it must not need a robot, hand, headset, camera, or vendor login.

Do not commit `.venv`, local SDK builds, captures, calibration data, credentials,
or runtime logs.
