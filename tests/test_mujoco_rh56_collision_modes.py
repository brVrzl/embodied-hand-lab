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
              <body name="rh56_R_index_distal"/>
              <body name="rh56_R_middle_distal"/>
              <body name="rh56_R_ring_distal"/>
              <body name="rh56_R_pinky_distal"/>
            </body>
          </worldbody>
        </mujoco>
        """
    )


def test_unifuc_pad_proxy_uses_existing_distal_rectangular_pads() -> None:
    root = _minimal_hand_xml()

    _configure_collision_model(root, collision_mode="unifuc_pad_proxy", include_calibration_markers=True)

    geoms = {geom.get("name"): geom for geom in root.iter("geom")}
    assert geoms["index_pad_proxy"].get("type") == "box"
    assert geoms["index_pad_proxy"].get("pos") == "0.0083 0.0250 0.0015"
    assert geoms["middle_pad_proxy"].get("pos") == "0.0064 0.0260 0.0015"
    assert "index_proximal_pad_proxy" not in geoms
    assert "middle_tip_pad_proxy" not in geoms
    assert geoms["index_unifuc_center"].get("contype") == "0"
