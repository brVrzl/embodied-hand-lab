from __future__ import annotations

import xml.etree.ElementTree as ET

from sim_maniskill.agents.jaka_rh56 import _rewrite_mjcf_text


def test_rewrite_mjcf_text_updates_asset_root_and_mount_offset() -> None:
    xml = """
    <mesh file="/home/w/Desktop/robot_sim/assets/rh56/meshes/foo.STL"/>
    <body name="rh56_R_hand_base_link" pos="0 0 0.009"/>
    """

    rewritten = _rewrite_mjcf_text(xml)

    assert "/home/w/projects/RoboTwin/robot_sim/assets/rh56/meshes/foo.STL" in rewritten
    assert 'pos="0 0 0.009"' in rewritten


def test_rewrite_mjcf_text_adds_rh56_collision_proxies() -> None:
    xml = """
    <mujoco model="hand">
      <worldbody>
        <body name="rh56_R_hand_base_link">
          <geom name="rh56_R_hand_base_link_geom_0" type="mesh" mesh="rh56_R_hand_base_link"/>
          <body name="rh56_R_index_distal">
            <geom name="rh56_R_index_distal_geom_0" type="mesh" mesh="rh56_R_index_distal"/>
          </body>
        </body>
      </worldbody>
    </mujoco>
    """

    rewritten = _rewrite_mjcf_text(xml)

    assert 'name="rh56_R_palm_collision"' in rewritten
    assert 'name="rh56_R_index_distal_collision"' in rewritten
    assert 'name="rh56_R_index_distal_geom_0" type="mesh" mesh="rh56_R_index_distal" contype="0" conaffinity="0"' in rewritten
    index_collision = next(
        geom for geom in ET.fromstring(rewritten).iter("geom") if geom.get("name") == "rh56_R_index_distal_collision"
    )
    assert index_collision.get("contype") == "2"
    assert index_collision.get("conaffinity") == "3"
