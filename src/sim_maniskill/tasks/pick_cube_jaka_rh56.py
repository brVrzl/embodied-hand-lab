from __future__ import annotations

from typing import Any

import numpy as np
import sapien
import torch
from mani_skill.envs.sapien_env import BaseEnv
from mani_skill.envs.tasks.tabletop.pick_cube import PickCubeEnv
from mani_skill.sensors.camera import CameraConfig
from mani_skill.utils import sapien_utils
from mani_skill.utils.building import actors
from mani_skill.utils.building.ground import build_ground
from mani_skill.utils.registration import register_env
from mani_skill.utils.structs.pose import Pose

from sim_maniskill.agents.jaka_rh56 import JAKA_RH56_BASE_POSE

JAKA_RH56_TABLE_LENGTH = 1.20
JAKA_RH56_TABLE_WIDTH = 0.60
JAKA_RH56_TABLE_THICKNESS = 0.04
JAKA_RH56_BASE_OFFSET_FROM_RIGHT_EDGE = 0.25
JAKA_RH56_BASE_OFFSET_FROM_FRONT_EDGE = 0.30
JAKA_RH56_TABLE_RIGHT_EDGE_X = float(JAKA_RH56_BASE_POSE.p[0] - JAKA_RH56_BASE_OFFSET_FROM_RIGHT_EDGE)
JAKA_RH56_TABLE_FRONT_EDGE_Y = float(JAKA_RH56_BASE_POSE.p[1] - JAKA_RH56_BASE_OFFSET_FROM_FRONT_EDGE)
JAKA_RH56_TABLE_CENTER = (
    JAKA_RH56_TABLE_RIGHT_EDGE_X + JAKA_RH56_TABLE_LENGTH / 2,
    JAKA_RH56_TABLE_FRONT_EDGE_Y + JAKA_RH56_TABLE_WIDTH / 2,
)
JAKA_RH56_CUBE_HALF_SIZE = 0.02
JAKA_RH56_GOAL_THRESH = 0.025
JAKA_RH56_CUBE_SPAWN_HALF_SIZE = 0.07
JAKA_RH56_CUBE_SPAWN_CENTER = (-0.10, 0.0)
JAKA_RH56_MAX_GOAL_HEIGHT = 0.15
JAKA_RH56_LIFT_SUCCESS_HEIGHT = 0.075
JAKA_RH56_LIFT_STATIC_QVEL = 0.35
JAKA_RH56_CAMERA_TARGET = [-0.12, 0.0, 0.05]
JAKA_RH56_SENSOR_CAM_EYE_POS = [0.24, -0.16, 0.78]
JAKA_RH56_SENSOR_CAM_TARGET_POS = JAKA_RH56_CAMERA_TARGET
JAKA_RH56_CLOSE_SENSOR_CAM_EYE_POS = [0.02, -0.30, 0.26]
JAKA_RH56_CLOSE_SENSOR_CAM_TARGET_POS = [-0.13, -0.01, 0.055]
JAKA_RH56_HUMAN_CAM_EYE_POS = [0.34, -0.24, 0.90]
JAKA_RH56_HUMAN_CAM_TARGET_POS = [-0.16, 0.0, 0.08]
JAKA_RH56_PICK_CUBE_PREGRASP_QPOS = np.array(
    [
        0.123,
        0.429,
        1.496,
        -1.447,
        -0.019,
        -2.164,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ],
    dtype=np.float32,
)


@register_env("PickCubeJakaRH56-v1", max_episode_steps=50)
class PickCubeJakaRH56Env(PickCubeEnv):
    SUPPORTED_ROBOTS = ["jaka_rh56"]
    goal_thresh = JAKA_RH56_GOAL_THRESH
    cube_spawn_half_size = JAKA_RH56_CUBE_SPAWN_HALF_SIZE
    cube_spawn_center = JAKA_RH56_CUBE_SPAWN_CENTER

    def __init__(
        self,
        *args,
        robot_uids: str = "jaka_rh56",
        robot_init_qpos_noise: float = 0.0,
        start_pose: str = "zero",
        sensor_camera_preset: str = "default",
        sensor_camera_fov: float | None = None,
        **kwargs,
    ) -> None:
        if start_pose not in {"zero", "pregrasp"}:
            raise ValueError(f"Unsupported start_pose={start_pose!r}; expected 'zero' or 'pregrasp'.")
        if sensor_camera_preset not in {"default", "close"}:
            raise ValueError(f"Unsupported sensor_camera_preset={sensor_camera_preset!r}; expected 'default' or 'close'.")
        self.robot_init_qpos_noise = robot_init_qpos_noise
        self.start_pose = start_pose
        self.cube_half_size = JAKA_RH56_CUBE_HALF_SIZE
        self.goal_thresh = JAKA_RH56_GOAL_THRESH
        self.cube_spawn_half_size = JAKA_RH56_CUBE_SPAWN_HALF_SIZE
        self.cube_spawn_center = JAKA_RH56_CUBE_SPAWN_CENTER
        self.max_goal_height = JAKA_RH56_MAX_GOAL_HEIGHT
        self.sensor_camera_preset = sensor_camera_preset
        if sensor_camera_preset == "close":
            self.sensor_cam_eye_pos = JAKA_RH56_CLOSE_SENSOR_CAM_EYE_POS
            self.sensor_cam_target_pos = JAKA_RH56_CLOSE_SENSOR_CAM_TARGET_POS
            self.sensor_camera_fov = float(sensor_camera_fov) if sensor_camera_fov is not None else 0.65
        else:
            self.sensor_cam_eye_pos = JAKA_RH56_SENSOR_CAM_EYE_POS
            self.sensor_cam_target_pos = JAKA_RH56_SENSOR_CAM_TARGET_POS
            self.sensor_camera_fov = float(sensor_camera_fov) if sensor_camera_fov is not None else np.pi / 2
        self.human_cam_eye_pos = JAKA_RH56_HUMAN_CAM_EYE_POS
        self.human_cam_target_pos = JAKA_RH56_HUMAN_CAM_TARGET_POS
        BaseEnv.__init__(self, *args, robot_uids=robot_uids, **kwargs)

    @property
    def _default_sensor_configs(self):
        pose = sapien_utils.look_at(
            eye=self.sensor_cam_eye_pos,
            target=self.sensor_cam_target_pos,
        )
        return [CameraConfig("base_camera", pose, 128, 128, self.sensor_camera_fov, 0.01, 100)]

    @property
    def _default_human_render_camera_configs(self):
        pose = sapien_utils.look_at(
            eye=self.human_cam_eye_pos,
            target=self.human_cam_target_pos,
        )
        return CameraConfig("render_camera", pose, 768, 768, 1, 0.01, 100)

    def _load_agent(self, options: dict):
        BaseEnv._load_agent(self, options, JAKA_RH56_BASE_POSE)

    def _load_scene(self, options: dict):
        builder = self.scene.create_actor_builder()
        builder.add_box_collision(
            pose=sapien.Pose(p=[0, 0, -JAKA_RH56_TABLE_THICKNESS / 2]),
            half_size=[
                JAKA_RH56_TABLE_LENGTH / 2,
                JAKA_RH56_TABLE_WIDTH / 2,
                JAKA_RH56_TABLE_THICKNESS / 2,
            ],
        )
        builder.add_box_visual(
            pose=sapien.Pose(p=[0, 0, -JAKA_RH56_TABLE_THICKNESS / 2]),
            half_size=[
                JAKA_RH56_TABLE_LENGTH / 2,
                JAKA_RH56_TABLE_WIDTH / 2,
                JAKA_RH56_TABLE_THICKNESS / 2,
            ],
            material=sapien.render.RenderMaterial(
                base_color=[0.82, 0.76, 0.67, 1.0],
            ),
        )
        builder.initial_pose = sapien.Pose(
            p=[JAKA_RH56_TABLE_CENTER[0], JAKA_RH56_TABLE_CENTER[1], 0.0]
        )
        self.table = builder.build_kinematic(name="jaka-rh56-worktable")
        self.ground = build_ground(self.scene, floor_width=5, altitude=-0.75)
        self.scene_objects = [self.table, self.ground]

        self.cube = actors.build_cube(
            self.scene,
            half_size=self.cube_half_size,
            color=[1, 0, 0, 1],
            name="cube",
            initial_pose=sapien.Pose(p=[0, 0, self.cube_half_size]),
        )
        self.goal_site = actors.build_sphere(
            self.scene,
            radius=self.goal_thresh,
            color=[0, 1, 0, 1],
            name="goal_site",
            body_type="kinematic",
            add_collision=False,
            initial_pose=sapien.Pose(),
        )
        self._hidden_objects.append(self.goal_site)

    def _initialize_episode(self, env_idx: torch.Tensor, options: dict):
        with torch.device(self.device):
            b = len(env_idx)
            self.table.set_pose(
                sapien.Pose(p=[JAKA_RH56_TABLE_CENTER[0], JAKA_RH56_TABLE_CENTER[1], 0.0])
            )
            self._initialize_agent_pose(env_idx)

            xyz = torch.zeros((b, 3))
            xyz[:, :2] = (
                torch.rand((b, 2), device=self.device) * self.cube_spawn_half_size * 2
                - self.cube_spawn_half_size
            )
            xyz[:, 0] += self.cube_spawn_center[0]
            xyz[:, 1] += self.cube_spawn_center[1]
            xyz[:, 2] = self.cube_half_size
            qs = torch.from_numpy(
                np.asarray(
                    [
                        [1.0, 0.0, 0.0, 0.0],
                    ]
                    * b,
                    dtype=np.float32,
                )
            ).to(self.device)
            self.cube.set_pose(Pose.create_from_pq(xyz, qs))

            goal_xyz = torch.zeros((b, 3))
            goal_xyz[:, :2] = (
                torch.rand((b, 2), device=self.device) * self.cube_spawn_half_size * 2
                - self.cube_spawn_half_size
            )
            goal_xyz[:, 0] += self.cube_spawn_center[0]
            goal_xyz[:, 1] += self.cube_spawn_center[1]
            goal_xyz[:, 2] = torch.rand((b), device=self.device) * self.max_goal_height + xyz[:, 2]
            self.goal_site.set_pose(Pose.create_from_pq(goal_xyz))

    def _initialize_agent_pose(self, env_idx: torch.Tensor) -> None:
        b = len(env_idx)
        if self.start_pose == "pregrasp":
            qpos = np.repeat(JAKA_RH56_PICK_CUBE_PREGRASP_QPOS[None, :], b, axis=0)
        else:
            qpos = np.repeat(self.agent.keyframes["rest"].qpos[None, :], b, axis=0)
        if self.robot_init_qpos_noise > 0:
            qpos[:, : len(self.agent.arm_joint_names)] += self._episode_rng.normal(
                0,
                self.robot_init_qpos_noise,
                (b, len(self.agent.arm_joint_names)),
            )
        self.agent.reset(qpos)
        self.agent.robot.set_pose(JAKA_RH56_BASE_POSE)

    def get_scene_summary(self) -> dict[str, Any]:
        return {
            "robot_uid": self.robot_uids,
            "robot_base_pose": self.agent.robot.pose.raw_pose.tolist(),
            "tcp_pose": self.agent.tcp_pose.raw_pose.tolist(),
            "cube_pose": self.cube.pose.raw_pose.tolist(),
            "goal_pose": self.goal_site.pose.raw_pose.tolist(),
            "qpos": self.agent.robot.get_qpos().tolist(),
            "full_joint_names": list(self.agent.full_joint_names),
            "arm_joint_names": list(self.agent.arm_joint_names),
            "hand_joint_names": list(self.agent.hand_joint_names),
            "start_pose": self.start_pose,
            "cube_spawn_center": list(self.cube_spawn_center),
            "cube_spawn_half_size": float(self.cube_spawn_half_size),
            "table": {
                "length_m": JAKA_RH56_TABLE_LENGTH,
                "width_m": JAKA_RH56_TABLE_WIDTH,
                "center_xy_m": list(JAKA_RH56_TABLE_CENTER),
                "right_edge_x_m": JAKA_RH56_TABLE_RIGHT_EDGE_X,
                "front_edge_y_m": JAKA_RH56_TABLE_FRONT_EDGE_Y,
                "robot_mount_offset_from_right_edge_m": JAKA_RH56_BASE_OFFSET_FROM_RIGHT_EDGE,
                "robot_mount_offset_from_front_edge_m": JAKA_RH56_BASE_OFFSET_FROM_FRONT_EDGE,
            },
            "camera_recommendation": {
                "strategy": "single fixed camera on the front-left upper side of the table",
                "sensor_camera_preset": self.sensor_camera_preset,
                "sensor_fov_rad": float(self.sensor_camera_fov),
                "sensor_eye_xyz_m": list(self.sensor_cam_eye_pos),
                "sensor_target_xyz_m": list(self.sensor_cam_target_pos),
                "human_eye_xyz_m": list(self.human_cam_eye_pos),
                "human_target_xyz_m": list(self.human_cam_target_pos),
                "relative_to_table": {
                    "sensor_from_left_edge_m": JAKA_RH56_TABLE_RIGHT_EDGE_X + JAKA_RH56_TABLE_LENGTH - self.sensor_cam_eye_pos[0],
                    "sensor_from_front_edge_m": self.sensor_cam_eye_pos[1] - JAKA_RH56_TABLE_FRONT_EDGE_Y,
                    "sensor_height_above_table_m": self.sensor_cam_eye_pos[2],
                },
            },
        }


@register_env("LiftCubeJakaRH56-v1", max_episode_steps=80)
class LiftCubeJakaRH56Env(PickCubeJakaRH56Env):
    """Contact-only lift/hold evaluation task for the JAKA+RH56 embodiment.

    Unlike the privileged PickCube oracle used for data-pipeline smoke tests,
    this task's success condition depends on simulated contact: the cube must
    be above the lift threshold while grasped and the arm must be reasonably
    static. It is intentionally stricter and may fail until the RH56 contact
    model is calibrated.
    """

    lift_success_height = JAKA_RH56_LIFT_SUCCESS_HEIGHT
    lift_static_qvel = JAKA_RH56_LIFT_STATIC_QVEL

    def evaluate(self):
        object_height = self.cube.pose.p[:, 2]
        is_lifted = object_height >= self.lift_success_height
        is_grasped = self.agent.is_grasping(self.cube)
        is_robot_static = self.agent.is_static(self.lift_static_qvel)
        return {
            "success": is_lifted & is_grasped & is_robot_static,
            "is_lifted": is_lifted,
            "is_grasped": is_grasped,
            "is_robot_static": is_robot_static,
            "object_height": object_height,
            "lift_success_height": torch.full_like(object_height, float(self.lift_success_height)),
        }

    def compute_dense_reward(self, obs: Any, action: torch.Tensor, info: dict):
        tcp_to_obj_dist = torch.linalg.norm(self.cube.pose.p - self.agent.tcp_pose.p, axis=1)
        reaching_reward = 1 - torch.tanh(5 * tcp_to_obj_dist)
        lift_progress = torch.clamp(
            (self.cube.pose.p[:, 2] - self.cube_half_size) / max(self.lift_success_height - self.cube_half_size, 1e-6),
            0.0,
            1.0,
        )
        reward = reaching_reward + info["is_grasped"].float() + lift_progress
        reward += info["is_robot_static"].float() * info["is_lifted"].float()
        reward[info["success"]] = 5.0
        return reward
