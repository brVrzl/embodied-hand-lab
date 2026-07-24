# Architecture overview

## Current primary stack

The current primary stack combines a JAKA Mini2 six-joint arm, Inspire RH56DFX
hand, Meta Quest 3 hand/head tracking, a left Touch controller clutch, MuJoCo,
and an optional physical JAKA ServoJ/EDG adapter. The physical adapter is
separately authorized and is not used by simulation or the default tests.

The live policy is configured by
`configs/sim/quest_hts_jaka_mini2_live_demo.yaml`. The main entry points are
`tools/quest_jaka_mujoco_sim.py` for simulation and
`tools/quest_jaka_hardware.py` for deliberately gated physical stages.

## Control flow

```text
Quest HTS hand/head datagrams + CTRL v1 sidecar
        |
QuestDatagramReceiverWorker (bounded FIFO, receive timestamp)
        |
HTS/controller validation, freshness, ordering
        |
release-before-press clutch and reference capture
        |
relative wrist transform, latched head yaw, robot basis mapping
        |
translation/quaternion filters and deadbands
        |
bounded SE(3) continuation + shared MuJoCo IK
        |
limits, collision, Jacobian, branch, pose, output feasibility
        |
immutable AcceptedArmTarget
       / \
MuJoCo    JAKA joint adapter -> native 125 Hz EDG worker
```

The MuJoCo model is both the simulation plant and, on the hardware path, the
plant-free kinematic/collision model. The physical process does not step the
MuJoCo plant or copy its `qpos`.

## Authoritative modules

| Area | Source | Configuration | Principal tests |
|---|---|---|---|
| HTS/CTRL transport | `src/motion_input`, `src/quest_jaka_sim/live_input.py` | `configs/motion_input/quest_hts_right_hand.yaml` | `test_hand_tracking_streamer_provider.py`, `test_quest_controller_transport.py` |
| clutch/mapping/filter/IK | `src/quest_jaka_sim` | `configs/sim/quest_hts_jaka_mini2_live_demo.yaml` | `test_quest_jaka_shared_pipeline.py`, mapping/filter/IK tests |
| common target contract | `src/teleoperation/accepted_target.py` | live simulation config | shared-pipeline tests |
| output feasibility | `src/teleoperation/output_feasibility.py` | live simulation config | `test_quest_jaka_output_feasibility.py` |
| MuJoCo adapter | `src/quest_jaka_sim/output.py` | simulation configs | simulation/shared-pipeline tests |
| JAKA adapter | `src/teleoperation/jaka/quest_adapter.py` | live simulation config plus gated CLI args | hardware CLI/shared-pipeline tests |
| native transport | `native/jaka_servo_worker/main.cpp` | CLI plus shared-memory protocol | native worker/resampler tests |
| RH56 | `src/rh56_driver`, simulation retargeter | `configs/hand`, `configs/sim/quest_rh56_retarget.yaml` | RH56 schema/backend/simulation tests |

Other current project areas—digital twin, vision, HEBI phone teleoperation,
iPhone RH56 experiments, and ROS2/RViz bring-up—remain outside the primary
Quest/JAKA shared pipeline.
