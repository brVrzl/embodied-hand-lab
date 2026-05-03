# RH56 URDF Inventory

Date: 2026-04-28

## Conclusion

The matching model for the current JAKA mini2 + RH56 MuJoCo setup is:

```text
URDF/三代手标准版/inspire_hand_3_standard/src/inspire_hand_r2024.11.5/
```

The current MuJoCo RH56 meshes under:

```text
/home/w/projects/RoboTwin/robot_sim/assets/rh56/meshes/
```

are byte-identical to the meshes in this standard right-hand package. The current base MuJoCo XML therefore does not appear to use the wrong RH56 geometry version.

## Available Packages

| Path | Usefulness | Notes |
| --- | --- | --- |
| `URDF/三代手标准版/inspire_hand_3_standard/src/inspire_hand_r2024.11.5/` | High | Best match for current right-hand RH56 model. |
| `URDF/三代手标准版/inspire_hand_3_standard/src/inspire_hand_l2024.11.5/` | Low for current setup | Left hand; useful only for reference. |
| `URDF/三代手短手腕/inspire_hand_r2024.11.5/` | Medium | Same hand family, but palm/finger base offsets differ from the standard version. Do not swap into current setup without verifying the physical wrist/adapter. |
| `URDF/三代手+双自由度手腕/inspire_hand_r2024.11.5/` | Medium | Includes two wrist joints and different base chain. Useful if the real mounting has that wrist, otherwise not a direct replacement. |
| `URDF/4B4C夹爪/` | Low | Different gripper, not RH56. |
| `URDF/四代手/urdf_right_with_force_sensor.zip` | Low for current RH56 | Fourth-generation tactile/force-sensor hand package, not a drop-in RH56 model. |

## Standard Right-Hand Contents

Useful files:

```text
URDF/三代手标准版/inspire_hand_3_standard/src/inspire_hand_r2024.11.5/urdf/inspire_hand_r.urdf
URDF/三代手标准版/inspire_hand_3_standard/src/inspire_hand_r2024.11.5/urdf/inspire_hand_r.xacro
URDF/三代手标准版/inspire_hand_3_standard/src/inspire_hand_r2024.11.5/meshes/*.STL
URDF/三代手标准版/inspire_hand_3_standard/src/inspire_hand_r2024.11.5/config/joint_trajectory_controller.yaml
URDF/三代手标准版/inspire_hand_3_standard/src/inspire_hand_r2024.11.5/config/gazebo_controller.yaml
URDF/三代手标准版/inspire_hand_3_standard/src/inspire_hand_r2024.11.5/right_hand_control.py
URDF/三代手标准版/inspire_hand_3_standard/src/inspire_hand_r2024.11.5/right_hand使用手册.txt
```

The ROS/Gazebo controller controls six active joints:

```text
R_thumb_MCP_joint1
R_thumb_MCP_joint2
R_index_MCP_joint
R_middle_MCP_joint
R_ring_MCP_joint
R_pinky_MCP_joint
```

This corresponds to the project-level physical DOF order:

```text
[pinky, ring, middle, index, thumb_bend, thumb_rotate]
```

and the MuJoCo actuator order:

```text
[thumb_rotate, thumb_bend, index, middle, ring, pinky]
```

## Joint Limits And Mimic Couplings

The standard right-hand URDF defines:

| Joint | Limit |
| --- | ---: |
| `R_thumb_MCP_joint1` | `[0, 1.1]` |
| `R_thumb_MCP_joint2` | `[0, 0.5]` |
| `R_thumb_PIP_joint` | `[0, 1.0]`, mimic `R_thumb_MCP_joint2` |
| `R_thumb_DIP_joint` | `[0, 1.2]`, mimic `R_thumb_MCP_joint2` |
| `R_index_MCP_joint` | `[0, 1.7]` |
| `R_index_DIP_joint` | `[0, 1.6]`, mimic `R_index_MCP_joint` |
| `R_middle_MCP_joint` | `[0, 1.68]` |
| `R_middle_DIP_joint` | `[0, 1.6]`, mimic `R_middle_MCP_joint` |
| `R_ring_MCP_joint` | `[0, 1.7]` |
| `R_ring_DIP_joint` | `[0, 1.6]`, mimic `R_ring_MCP_joint` |
| `R_pinky_MCP_joint` | `[0, 1.7]` |
| `R_pinky_DIP_joint` | `[0, 1.6]`, mimic `R_pinky_MCP_joint` |

Important discrepancy:

- `inspire_hand_r.urdf`: thumb PIP/DIP mimic multipliers are `0.6` and `0.8`.
- `inspire_hand_r.xacro`: thumb PIP/DIP joint mimic tags use `0.8` and `1.2`.
- The Gazebo mimic plugin block in the same xacro uses `1.0` and `1.0` for both thumb mimic plugins.

The current MuJoCo XML uses the URDF-style equality:

```text
thumb_PIP = 0.6 * thumb_MCP_joint2
thumb_DIP = 0.8 * thumb_MCP_joint2
other_DIP = 1.0 * other_MCP
```

This should be treated as a calibration parameter, not as proven ground truth.

## What Is Usable Now

Use immediately:

- Standard right-hand URDF and meshes as the authoritative geometry/kinematic chain for the current model.
- Joint limits from the standard URDF for action clipping.
- Six active joint list from `joint_trajectory_controller.yaml`.
- `right_hand_control.py` as a reference for ROS/Gazebo command order.
- The URDF/xacro mimic discrepancy as a concrete thumb-coupling ablation.

Do not use directly yet:

- Full mesh collision for grasp success. The URDF collision geometry is the same detailed STL mesh as the visual geometry, which is too heavy and too brittle for contact-rich MuJoCo grasp validation.
- The short-wrist or two-DOF-wrist packages as direct replacements unless the physical hand mount is confirmed to match them.
- Fourth-generation force-sensor hand assets for the RH56 setup.

## Next Calibration Step

Run the pose/contact viewer and compare three things against the real hand or the saved photos:

1. Current URDF coupling: thumb PIP/DIP = `0.6/0.8`.
2. Xacro joint mimic coupling: thumb PIP/DIP = `0.8/1.2`.
3. Gazebo plugin coupling: thumb PIP/DIP = `1.0/1.0`.

The best coupling is the one whose thumb pad path matches the real thumb during `thumb_rotate -> pinch close`, especially near the thumb-index blocking region.

Commands:

```bash
scripts/view_mujoco_rh56_pose_contact.sh --mode poses --thumb-coupling urdf
scripts/view_mujoco_rh56_pose_contact.sh --mode poses --thumb-coupling xacro
scripts/view_mujoco_rh56_pose_contact.sh --mode poses --thumb-coupling gazebo_plugin
```

## Original STL Collision Test

The viewer and benchmark can now switch collision models:

```bash
# Original RH56 STL mesh collision only.
scripts/view_mujoco_rh56_pose_contact.sh --mode poses --collision-mode mesh

# Current simplified fingertip proxy collision.
scripts/view_mujoco_rh56_pose_contact.sh --mode poses --collision-mode proxy

# Original STL collision plus visible proxy markers.
scripts/view_mujoco_rh56_pose_contact.sh --mode poses --collision-mode mesh_proxy
```

For a small benchmark smoke test:

```bash
.venv/bin/python tools/mujoco_rh56_grasp_benchmark.py \
  --objects foam_cube \
  --max-candidates 4 \
  --duration 1.5 \
  --collision-mode mesh \
  --out-dir data/mujoco_grasp_benchmark_mesh_test
```

Initial observation: the original STL mesh collision loads and runs, but it already produces hand self-contacts in several poses. For example, `sim_best_pinch` and `power_close` report thumb-index contacts, which matches the observed physical blocking behavior. It also creates hand-table contacts during the current grasp benchmark, so it is useful for checking self-collision realism but is not yet a stable grasp-validation collision model.
