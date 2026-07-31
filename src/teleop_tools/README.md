# Experimental phone hand input

`teleop_tools` contains an experimental iPhone/IP-camera plus MediaPipe
landmark-to-RH56 target mapper. It is outside the authoritative Quest/JAKA
pipeline and is not a maintained physical RH56 transport.

The current wrappers are:

```bash
./scripts/check_iphone_camera_stream.sh --help
./scripts/run_iphone_mediapipe_hand_teleop.sh --help
```

The mapper can record landmark and six-axis normalized target JSONL. Its
optional ROS 2 publisher exposes a generic `/rh56/command_angles` message for
external experiments; this repository has no maintained ROS 2 RH56 command
consumer. It therefore must not be described as a physical hand safety gate or
as proof that RH56DFX was commanded.

Opening a camera is a physical-device action and needs separate authorization.
Any downstream actuator consumer needs its own independently reviewed and
authorized safety boundary. The current physical RH56 path is the PC-direct
worker documented in `docs/operation/rh56_operation.md`. This experimental
mapper currently has no dedicated behavior-level regression test.
