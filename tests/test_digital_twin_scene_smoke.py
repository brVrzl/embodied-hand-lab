from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "models/digital_twin/workspace_scene.xml"


def _id(model: mujoco.MjModel, kind: mujoco.mjtObj, name: str) -> int:
    return mujoco.mj_name2id(model, kind, name)


def test_maintained_workspace_scene_loads_robot_hand_and_workspace() -> None:
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    data.qpos[:] = model.qpos0
    mujoco.mj_forward(model, data)
    base = _id(model, mujoco.mjtObj.mjOBJ_BODY, "jaka_Link_0")
    hand = _id(model, mujoco.mjtObj.mjOBJ_BODY, "rh56_R_hand_base_link")
    assert base >= 0 and hand >= 0
    assert np.allclose(data.xpos[base], [0.0, 0.0, 0.0])
    assert _id(model, mujoco.mjtObj.mjOBJ_GEOM, "workspace_tabletop") >= 0
    assert _id(model, mujoco.mjtObj.mjOBJ_GEOM, "workspace_rail_positive_y") >= 0


def test_maintained_workspace_scene_preserves_zero_reference() -> None:
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    assert np.allclose(model.qpos0, 0.0)
    joints = [
        _id(model, mujoco.mjtObj.mjOBJ_JOINT, f"jaka_joint_{index}")
        for index in range(1, 7)
    ]
    assert all(index >= 0 for index in joints)
    assert [model.qpos0[model.jnt_qposadr[index]] for index in joints] == [0.0] * 6
