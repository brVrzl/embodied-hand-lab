"""Explicit semantic-to-MuJoCo RH56DFX mapping for simulation-only tools."""

from __future__ import annotations

from dataclasses import dataclass


CANONICAL_CHANNEL_ORDER = (
    "index",
    "middle",
    "ring",
    "pinky",
    "thumb_close",
    "thumb_lateral",
)


@dataclass(frozen=True, slots=True)
class Rh56SimChannel:
    canonical: str
    actuator: str
    joint: str
    protocol_index: int
    raw_index: int
    mujoco_positive_motion: str
    hardware_raw_direction: int


# Hardware protocol order is [pinky, ring, middle, index, thumb_close,
# thumb_lateral].  The retained legacy raw/debug order is [thumb_close,
# thumb_lateral, index, middle, ring, pinky].  Both were re-verified against
# src/rh56_driver/hand_schema.py; H0 never imports that hardware package.
RH56_CHANNELS = (
    Rh56SimChannel(
        "index",
        "rh56_R_index_MCP_joint_act",
        "rh56_R_index_MCP_joint",
        3,
        2,
        "positive closes index",
        -1,
    ),
    Rh56SimChannel(
        "middle",
        "rh56_R_middle_MCP_joint_act",
        "rh56_R_middle_MCP_joint",
        2,
        3,
        "positive closes middle",
        -1,
    ),
    Rh56SimChannel(
        "ring",
        "rh56_R_ring_MCP_joint_act",
        "rh56_R_ring_MCP_joint",
        1,
        4,
        "positive closes ring",
        -1,
    ),
    Rh56SimChannel(
        "pinky",
        "rh56_R_pinky_MCP_joint_act",
        "rh56_R_pinky_MCP_joint",
        0,
        5,
        "positive closes pinky",
        -1,
    ),
    Rh56SimChannel(
        "thumb_close",
        "rh56_R_thumb_MCP_joint2_act",
        "rh56_R_thumb_MCP_joint2",
        4,
        0,
        "positive bends thumb",
        -1,
    ),
    Rh56SimChannel(
        "thumb_lateral",
        "rh56_R_thumb_MCP_joint1_act",
        "rh56_R_thumb_MCP_joint1",
        5,
        1,
        "positive moves thumb laterally toward opposition",
        -1,
    ),
)


if tuple(channel.canonical for channel in RH56_CHANNELS) != CANONICAL_CHANNEL_ORDER:
    raise RuntimeError("RH56 simulation mapping is not in canonical order")
