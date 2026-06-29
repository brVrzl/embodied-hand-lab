from __future__ import annotations

from copy import deepcopy
from pathlib import Path

import numpy as np
import sapien
import torch
from mani_skill.agents.base_agent import BaseAgent, Keyframe
from mani_skill.agents.controllers import PDJointPosControllerConfig, deepcopy_dict
from mani_skill.agents.registration import register_agent
from mani_skill.utils import sapien_utils
from mani_skill.utils.structs.actor import Actor
from transforms3d.euler import euler2quat

from sim_maniskill.rh56_collision import patch_rh56_collision_text

LOCAL_ASSET_ROOT = Path(__file__).resolve().parents[3] / "data" / "sim_assets"
LOCAL_XML_PATH = LOCAL_ASSET_ROOT / "jaka_rh56.xml"
DESKTOP_ASSET_PREFIX = "/home/w/Desktop/robot_sim/assets"
HAND_MOUNT_TOKEN = 'pos="0 0 0.009"'
HAND_MOUNT_WITH_USER_FLANGE = 'pos="0 0 0.009"'
JAKA_RH56_BASE_POSE = sapien.Pose(p=[-0.615, 0, 0], q=euler2quat(0, 0, np.pi))
JAKA_RH56_FULL_REST_QPOS = np.array(
    [
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
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ],
    dtype=np.float32,
)


def _rewrite_mjcf_text(xml_text: str) -> str:
    xml_text = xml_text.replace(HAND_MOUNT_TOKEN, HAND_MOUNT_WITH_USER_FLANGE, 1)
    return patch_rh56_collision_text(xml_text)


def ensure_local_mjcf() -> Path:
    if not LOCAL_XML_PATH.exists():
        raise FileNotFoundError(
            f"Missing local JAKA+RH56 MJCF at {LOCAL_XML_PATH}. "
            "Expected data/sim_assets/jaka_rh56.xml and its local meshes to be present."
        )
    return LOCAL_XML_PATH


@register_agent()
class JakaRH56(BaseAgent):
    uid = "jaka_rh56"
    mjcf_path = str(ensure_local_mjcf())
    urdf_config = dict()

    arm_joint_names = [
        "jaka_joint_1",
        "jaka_joint_2",
        "jaka_joint_3",
        "jaka_joint_4",
        "jaka_joint_5",
        "jaka_joint_6",
    ]
    hand_joint_names = [
        "rh56_R_thumb_MCP_joint1",
        "rh56_R_thumb_MCP_joint2",
        "rh56_R_index_MCP_joint",
        "rh56_R_middle_MCP_joint",
        "rh56_R_ring_MCP_joint",
        "rh56_R_pinky_MCP_joint",
    ]
    full_joint_names = [
        "jaka_joint_1",
        "jaka_joint_2",
        "jaka_joint_3",
        "jaka_joint_4",
        "jaka_joint_5",
        "jaka_joint_6",
        "rh56_R_thumb_MCP_joint1",
        "rh56_R_index_MCP_joint",
        "rh56_R_middle_MCP_joint",
        "rh56_R_ring_MCP_joint",
        "rh56_R_pinky_MCP_joint",
        "rh56_R_thumb_MCP_joint2",
        "rh56_R_index_DIP_joint",
        "rh56_R_middle_DIP_joint",
        "rh56_R_ring_DIP_joint",
        "rh56_R_pinky_DIP_joint",
        "rh56_R_thumb_PIP_joint",
        "rh56_R_thumb_DIP_joint",
    ]
    ee_link_name = "rh56_R_hand_base_link"

    arm_stiffness = 1e3
    arm_damping = 1e2
    arm_force_limit = 200

    hand_stiffness = 60
    hand_damping = 10
    hand_force_limit = 30

    keyframes = dict(
        rest=Keyframe(
            pose=JAKA_RH56_BASE_POSE,
            qpos=JAKA_RH56_FULL_REST_QPOS.copy(),
        )
    )

    @property
    def _controller_configs(self):
        arm_pd_joint_pos = PDJointPosControllerConfig(
            self.arm_joint_names,
            lower=None,
            upper=None,
            stiffness=self.arm_stiffness,
            damping=self.arm_damping,
            force_limit=self.arm_force_limit,
            normalize_action=False,
        )
        arm_pd_joint_delta_pos = PDJointPosControllerConfig(
            self.arm_joint_names,
            lower=-0.1,
            upper=0.1,
            stiffness=self.arm_stiffness,
            damping=self.arm_damping,
            force_limit=self.arm_force_limit,
            use_delta=True,
        )
        arm_pd_joint_target_delta_pos = deepcopy(arm_pd_joint_delta_pos)
        arm_pd_joint_target_delta_pos.use_target = True

        hand_pd_joint_pos = PDJointPosControllerConfig(
            self.hand_joint_names,
            lower=None,
            upper=None,
            stiffness=self.hand_stiffness,
            damping=self.hand_damping,
            force_limit=self.hand_force_limit,
            normalize_action=False,
        )
        hand_pd_joint_delta_pos = PDJointPosControllerConfig(
            self.hand_joint_names,
            lower=-0.15,
            upper=0.15,
            stiffness=self.hand_stiffness,
            damping=self.hand_damping,
            force_limit=self.hand_force_limit,
            use_delta=True,
        )
        hand_pd_joint_target_delta_pos = deepcopy(hand_pd_joint_delta_pos)
        hand_pd_joint_target_delta_pos.use_target = True

        return deepcopy_dict(
            dict(
                pd_joint_pos=dict(arm=arm_pd_joint_pos, gripper=hand_pd_joint_pos),
                pd_joint_delta_pos=dict(
                    arm=arm_pd_joint_delta_pos,
                    gripper=hand_pd_joint_delta_pos,
                ),
                pd_joint_target_delta_pos=dict(
                    arm=arm_pd_joint_target_delta_pos,
                    gripper=hand_pd_joint_target_delta_pos,
                ),
            )
        )

    def _after_init(self):
        self.tcp = sapien_utils.get_obj_by_name(
            self.robot.get_links(),
            self.ee_link_name,
        )
        self.thumb_tip_link = sapien_utils.get_obj_by_name(
            self.robot.get_links(),
            "rh56_R_thumb_distal",
        )
        self.finger_tip_links = sapien_utils.get_objs_by_names(
            self.robot.get_links(),
            [
                "rh56_R_index_distal",
                "rh56_R_middle_distal",
                "rh56_R_ring_distal",
                "rh56_R_pinky_distal",
            ],
        )

    @property
    def tcp_pose(self):
        return self.tcp.pose

    def is_grasping(self, object: Actor, min_force: float = 0.25):
        thumb_force = torch.linalg.norm(
            self.scene.get_pairwise_contact_forces(self.thumb_tip_link, object),
            axis=1,
        )
        other_forces = []
        for link in self.finger_tip_links:
            other_forces.append(
                torch.linalg.norm(
                    self.scene.get_pairwise_contact_forces(link, object),
                    axis=1,
                )
            )
        stacked_other_forces = torch.stack(other_forces, dim=1)
        any_finger_contact = torch.any(stacked_other_forces >= min_force, dim=1)
        return torch.logical_and(thumb_force >= min_force, any_finger_contact)

    def is_static(self, threshold: float = 0.2):
        qvel = self.robot.get_qvel()[..., : len(self.arm_joint_names)]
        return torch.max(torch.abs(qvel), dim=1).values <= threshold
