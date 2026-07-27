# Teleoperation rearchitecture research sources

This branch vendors no third-party source or Git history.  The following are
research sources only; the recorded revisions were observed with `git
ls-remote` on 2026-07-27.  No code was copied into this repository.

| Source | Revision | License status | Reuse decision |
| --- | --- | --- | --- |
| [Unitree xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate) | `7dc9aa1a6edbf4a9f4f887d8ab6fc449ea5135f6` | Apache-2.0; dependency licenses remain separate | Architecture and data-recording ideas only; Unitree DDS and robot-specific code are not reusable here. |
| [SpesRobotics teleop](https://github.com/SpesRobotics/teleop) | `c5d808155a87b584d6147a5943d4b87c34c92db0` | Apache-2.0 | Its PoseStamped/WebXR boundary is compatible in principle; no code copied. |
| [JAKA jaka_ros2](https://github.com/JAKARobotics/jaka_ros2) | `aadcf50ad7954a9cd694d910b6c9a1f06be3ee22` | No top-level license declared in the source tree/API at audit time | Reference only. Do not copy or vendor until JAKA gives an explicit license. |
| [MoveIt 2](https://github.com/moveit/moveit2) | `f737f202666802e13390081778e4fea25b8bc552` | BSD-3-Clause | Candidate dependency only, never copied. |
| [UM-ARM-Lab vr_teleop](https://github.com/UM-ARM-Lab/vr_teleop) | `8eba4f00b3c3dfc514e74f393cf542d05a569243` | No top-level license observed; archived | Historical architecture reference only. |
| [UM-ARM-Lab vr_ros2_bridge](https://github.com/UM-ARM-Lab/vr_ros2_bridge) | `07ea3b9ae57d4ab1bba43a84c03ef92e34080597` | No top-level license observed | Reference only pending license confirmation. |
| [OpenTeleVision](https://github.com/OpenTeleVision/TeleVision) | `e6e25afdb16c1b326b5bf37bd0ae79919bf79f26` | Apache-2.0 | Quest/WebXR visual-feedback and recording reference only. |
| [OpenTeach](https://github.com/OpenTeach/OpenTeach) | `9c0c3165d8ad57473b6b0d008b071461191c16f4` | License not reconfirmed in this audit | Research pointer only; no reuse. |
| [robosuite](https://github.com/ARISE-Initiative/robosuite) | `5ce6643f3092639d08f7b0f90ed1c6a84f50552c` | MIT | Simulation/data-collection reference only. |

The installed local JAKA SDK (`third_party/jaka_sdk/v2.2.7`) remains subject to
its existing vendor terms; this document does not grant any additional rights.
