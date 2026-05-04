from __future__ import annotations

import xml.etree.ElementTree as ET
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from mujoco_rh56_grasp_benchmark import _configure_collision_model


def _minimal_hand_xml() -> ET.Element:
    return ET.fromstring(
        """
        <mujoco model="hand">
          <worldbody>
            <body name="rh56_R_hand_base_link">
              <geom name="rh56_R_hand_base_link_geom_0" type="mesh" mesh="rh56_R_hand_base_link"/>
              <body name="rh56_R_thumb_distal"/>
              <body name="rh56_R_index_proximal"/>
              <body name="rh56_R_index_distal"/>
              <body name="rh56_R_middle_proximal"/>
              <body name="rh56_R_middle_distal"/>
              <body name="rh56_R_ring_proximal"/>
              <body name="rh56_R_ring_distal"/>
              <body name="rh56_R_pinky_proximal"/>
              <body name="rh56_R_pinky_distal"/>
            </body>
          </worldbody>
        </mujoco>
        """
    )


def test_unifuc_pad_proxy_uses_rectangular_phalanx_pads() -> None:
    root = _minimal_hand_xml()

    _configure_collision_model(root, collision_mode="unifuc_pad_proxy", include_calibration_markers=True)

    geoms = {geom.get("name"): geom for geom in root.iter("geom")}
    assert geoms["index_pad_proxy"].get("type") == "box"
    assert geoms["index_proximal_pad_proxy"].get("type") == "box"
    assert geoms["middle_tip_pad_proxy"].get("type") == "box"
    assert geoms["pinky_pad_proxy"].get("pos", "").split()[2].startswith("-0.006")
    assert geoms["index_unifuc_center"].get("contype") == "0"
