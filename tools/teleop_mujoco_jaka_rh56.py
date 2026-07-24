from __future__ import annotations

import argparse
import importlib
import json
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from debug_mujoco_jaka_rh56_viewer import (
    BASE_XML,
    COLLISION_MODES,
    DEBUG_DIR,
    HAND_ACTUATOR_NAMES,
    HAND_OPEN_CTRL,
    PREGRASP_QPOS,
    _actuator_ids,
    _contact_summary,
    _set_initial_state,
    build_debug_xml,
)
from rh56_driver.hand_schema import CANONICAL_HAND_ORDER, RH56_INTERNAL_ORDER, canonical_to_raw


TELEOP_XML = DEBUG_DIR / "jaka_rh56_teleop.xml"
TELEOP_SNAPSHOT = DEBUG_DIR / "teleop_last_pose.json"
EE_SITE_NAME = "teleop_ee_site"
TARGET_BODY_NAME = "teleop_ee_target"
TARGET_GEOM_NAME = "teleop_ee_target_geom"
ARM_ACTUATOR_NAMES = [f"jaka_joint_{idx}_act" for idx in range(1, 7)]
ARM_JOINT_NAMES = [f"jaka_joint_{idx}" for idx in range(1, 7)]
HAND_CANONICAL_CLOSE_CTRL = np.asarray([1.25, 1.25, 1.25, 1.25, 0.45, 0.75], dtype=np.float64)
PINCH_PRESET = np.asarray([0.95, 0.95, 0.30, 0.20, 1.00, 0.80], dtype=np.float64)


def _find_body(root: ET.Element, name: str) -> ET.Element:
    for body in root.iter("body"):
        if body.get("name") == name:
            return body
    raise RuntimeError(f"Missing body {name!r} in MJCF.")


def _add_teleop_target(xml_path: str | Path, out_xml: str | Path) -> None:
    tree = ET.parse(xml_path)
    root = tree.getroot()
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError("MJCF does not contain <worldbody>.")

    hand_body = _find_body(root, "rh56_R_hand_base_link")
    if not any(site.get("name") == EE_SITE_NAME for site in hand_body.iter("site")):
        ET.SubElement(
            hand_body,
            "site",
            {
                "name": EE_SITE_NAME,
                "pos": "0 0 0",
                "type": "sphere",
                "size": "0.010",
                "rgba": "0 1 0 0.85",
            },
        )

    if not any(body.get("name") == TARGET_BODY_NAME for body in worldbody.iter("body")):
        target = ET.SubElement(
            worldbody,
            "body",
            {
                "name": TARGET_BODY_NAME,
                "mocap": "true",
                "pos": "-0.20 0 0.12",
            },
        )
        ET.SubElement(
            target,
            "geom",
            {
                "name": TARGET_GEOM_NAME,
                "type": "sphere",
                "size": "0.018",
                "rgba": "0.0 0.55 1.0 0.70",
                "contype": "0",
                "conaffinity": "0",
            },
        )
        ET.SubElement(
            target,
            "site",
            {
                "name": "teleop_ee_target_site",
                "type": "sphere",
                "size": "0.008",
                "rgba": "0.0 1.0 1.0 1.0",
            },
        )

    Path(out_xml).parent.mkdir(parents=True, exist_ok=True)
    tree.write(out_xml, encoding="utf-8", xml_declaration=False)


def build_teleop_xml(
    base_xml: str | Path,
    out_xml: str | Path,
    *,
    scenario: str,
    collision_mode: str = "visual_coacd",
) -> dict[str, object]:
    tmp_xml = Path(out_xml).with_name(Path(out_xml).stem + "_debug_base.xml")
    summary = build_debug_xml(
        base_xml,
        tmp_xml,
        scenario=scenario,
        collision_mode=collision_mode,
    )
    _add_teleop_target(tmp_xml, out_xml)
    summary["teleop_xml"] = str(Path(out_xml).resolve())
    return summary


@dataclass
class TeleopState:
    arm_q_target: np.ndarray
    hand_norm: np.ndarray
    selected_hand_dof: int = 0
    paused: bool = False
    quit_requested: bool = False
    print_requested: bool = True
    save_requested: bool = False


class JakaRh56Teleop:
    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        *,
        scenario: str,
        target_step: float,
        hand_step: float,
        ik_gain: float,
        ik_damping: float,
        ik_max_step: float,
    ) -> None:
        self.model = model
        self.data = data
        self.scenario = scenario
        self.target_step = float(target_step)
        self.hand_step = float(hand_step)
        self.ik_gain = float(ik_gain)
        self.ik_damping = float(ik_damping)
        self.ik_max_step = float(ik_max_step)

        self.arm_actuator_ids = _actuator_ids(model, ARM_ACTUATOR_NAMES)
        self.hand_actuator_ids = _actuator_ids(model, HAND_ACTUATOR_NAMES)
        self.arm_qpos_ids = np.asarray(
            [
                model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)]
                for name in ARM_JOINT_NAMES
            ],
            dtype=np.int32,
        )
        self.arm_dof_ids = np.asarray(
            [
                model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)]
                for name in ARM_JOINT_NAMES
            ],
            dtype=np.int32,
        )
        self.ee_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, EE_SITE_NAME)
        if self.ee_site_id < 0:
            raise KeyError(f"Missing site {EE_SITE_NAME}")
        target_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, TARGET_BODY_NAME)
        if target_body_id < 0:
            raise KeyError(f"Missing body {TARGET_BODY_NAME}")
        self.target_mocap_id = int(model.body_mocapid[target_body_id])
        if self.target_mocap_id < 0:
            raise RuntimeError(f"Body {TARGET_BODY_NAME} is not a mocap body.")

        _set_initial_state(model, data, scenario=scenario)
        data.ctrl[self.arm_actuator_ids] = PREGRASP_QPOS[:6]
        data.ctrl[self.hand_actuator_ids] = HAND_OPEN_CTRL
        mujoco.mj_forward(model, data)
        self.state = TeleopState(
            arm_q_target=data.qpos[self.arm_qpos_ids].copy(),
            hand_norm=np.zeros(len(CANONICAL_HAND_ORDER), dtype=np.float64),
        )
        self.reset_target_to_ee()

    def reset_target_to_ee(self) -> None:
        mujoco.mj_forward(self.model, self.data)
        self.data.mocap_pos[self.target_mocap_id] = self.data.site_xpos[self.ee_site_id].copy()
        quat = np.zeros(4, dtype=np.float64)
        mujoco.mju_mat2Quat(quat, self.data.site_xmat[self.ee_site_id])
        self.data.mocap_quat[self.target_mocap_id] = quat

    def _hand_ctrl_from_norm(self) -> np.ndarray:
        canonical_ctrl = np.clip(self.state.hand_norm, 0.0, 1.0) * HAND_CANONICAL_CLOSE_CTRL
        return np.asarray(canonical_to_raw(canonical_ctrl, raw_order=RH56_INTERNAL_ORDER), dtype=np.float64)

    def _apply_controls(self) -> None:
        self.data.ctrl[self.arm_actuator_ids] = self.state.arm_q_target
        self.data.ctrl[self.hand_actuator_ids] = self._hand_ctrl_from_norm()

    def _solve_position_ik(self) -> None:
        mujoco.mj_forward(self.model, self.data)
        target = self.data.mocap_pos[self.target_mocap_id]
        current = self.data.site_xpos[self.ee_site_id]
        err = target - current
        if np.linalg.norm(err) < 1e-4:
            return
        jacp = np.zeros((3, self.model.nv), dtype=np.float64)
        jacr = np.zeros((3, self.model.nv), dtype=np.float64)
        mujoco.mj_jacSite(self.model, self.data, jacp, jacr, self.ee_site_id)
        j_arm = jacp[:, self.arm_dof_ids]
        lhs = j_arm @ j_arm.T + (self.ik_damping**2) * np.eye(3)
        dq = j_arm.T @ np.linalg.solve(lhs, self.ik_gain * err)
        dq = np.clip(dq, -self.ik_max_step, self.ik_max_step)
        self.state.arm_q_target = self.state.arm_q_target + dq
        for idx, qpos_id in enumerate(self.arm_qpos_ids):
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, ARM_JOINT_NAMES[idx])
            if bool(self.model.jnt_limited[joint_id]):
                low, high = self.model.jnt_range[joint_id]
                self.state.arm_q_target[idx] = np.clip(self.state.arm_q_target[idx], low, high)

    def step(self) -> None:
        self._solve_position_ik()
        self._apply_controls()
        if not self.state.paused:
            mujoco.mj_step(self.model, self.data)
        else:
            mujoco.mj_forward(self.model, self.data)

    def handle_key(self, key: int) -> None:
        try:
            char = chr(key).lower()
        except ValueError:
            return
        if char == "h":
            self.print_help()
        elif char == "o":
            self.state.hand_norm[:] = 0.0
        elif char == "c":
            self.state.hand_norm[:] = 1.0
        elif char == "p":
            self.state.hand_norm[:] = PINCH_PRESET
        elif char in "123456":
            self.state.selected_hand_dof = int(char) - 1
        elif char == "[":
            self.state.selected_hand_dof = (self.state.selected_hand_dof - 1) % len(CANONICAL_HAND_ORDER)
        elif char == "]":
            self.state.selected_hand_dof = (self.state.selected_hand_dof + 1) % len(CANONICAL_HAND_ORDER)
        elif char in "-_":
            idx = self.state.selected_hand_dof
            self.state.hand_norm[idx] = max(0.0, self.state.hand_norm[idx] - self.hand_step)
        elif char in "=+":
            idx = self.state.selected_hand_dof
            self.state.hand_norm[idx] = min(1.0, self.state.hand_norm[idx] + self.hand_step)
        elif char == "w":
            self.data.mocap_pos[self.target_mocap_id, 0] += self.target_step
        elif char == "s":
            self.data.mocap_pos[self.target_mocap_id, 0] -= self.target_step
        elif char == "a":
            self.data.mocap_pos[self.target_mocap_id, 1] += self.target_step
        elif char == "d":
            self.data.mocap_pos[self.target_mocap_id, 1] -= self.target_step
        elif char == "q":
            self.data.mocap_pos[self.target_mocap_id, 2] += self.target_step
        elif char == "e":
            self.data.mocap_pos[self.target_mocap_id, 2] -= self.target_step
        elif char == "r":
            _set_initial_state(self.model, self.data, scenario=self.scenario)
            self.state.arm_q_target = self.data.qpos[self.arm_qpos_ids].copy()
            self.state.hand_norm[:] = 0.0
            self.reset_target_to_ee()
        elif char == " ":
            self.state.paused = not self.state.paused
        elif char == "t":
            self.state.print_requested = True
        elif char == "x":
            self.state.save_requested = True
        elif char == "\x1b":
            self.state.quit_requested = True

    def status(self) -> dict[str, object]:
        mujoco.mj_forward(self.model, self.data)
        ee_pos = self.data.site_xpos[self.ee_site_id].copy()
        target_pos = self.data.mocap_pos[self.target_mocap_id].copy()
        cube_body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "debug_cube_body")
        cube_pos = self.data.xpos[cube_body].copy() if cube_body >= 0 else np.zeros(3)
        return {
            "time": float(self.data.time),
            "scenario": self.scenario,
            "ee_pos": ee_pos.round(5).tolist(),
            "target_pos": target_pos.round(5).tolist(),
            "target_error_m": float(np.linalg.norm(target_pos - ee_pos)),
            "arm_q_target": self.state.arm_q_target.round(5).tolist(),
            "arm_q_current": self.data.qpos[self.arm_qpos_ids].round(5).tolist(),
            "hand_canonical_order": list(CANONICAL_HAND_ORDER),
            "hand_norm": self.state.hand_norm.round(4).tolist(),
            "selected_hand_dof": CANONICAL_HAND_ORDER[self.state.selected_hand_dof],
            "cube_pos": cube_pos.round(5).tolist(),
            "contacts": _contact_summary(self.model, self.data),
        }

    def print_status(self) -> None:
        info = self.status()
        print(json.dumps(info, indent=2), flush=True)

    def print_help(self) -> None:
        print(
            "\nMuJoCo JAKA+RH56 teleop keys:\n"
            "  Mouse: select the blue target sphere, then use MuJoCo's perturb controls to drag it.\n"
            "  W/S A/D Q/E: nudge target in world x/y/z when mouse dragging is awkward.\n"
            "  1..6 or [/]: select hand DOF in canonical order.\n"
            "  -/=: decrease/increase selected hand DOF.\n"
            "  O/C/P: open / close / pinch preset.\n"
            "  Space: pause/resume physics. R: reset. T: print status. X: save snapshot. H: help.\n",
            flush=True,
        )

    def save_snapshot(self, path: str | Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self.status(), indent=2), encoding="utf-8")
        print(f"Saved teleop snapshot to {path}", flush=True)


def run_teleop(
    xml_path: str | Path,
    *,
    scenario: str,
    viewer: bool,
    duration: float,
    target_step: float,
    hand_step: float,
    ik_gain: float,
    ik_damping: float,
    ik_max_step: float,
    snapshot_path: str | Path,
) -> None:
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    teleop = JakaRh56Teleop(
        model,
        data,
        scenario=scenario,
        target_step=target_step,
        hand_step=hand_step,
        ik_gain=ik_gain,
        ik_damping=ik_damping,
        ik_max_step=ik_max_step,
    )
    teleop.print_help()

    last_print = -1.0
    if viewer:
        mujoco_viewer = importlib.import_module("mujoco.viewer")

        with mujoco_viewer.launch_passive(model, data, key_callback=teleop.handle_key) as handle:
            handle.cam.azimuth = -120
            handle.cam.elevation = -20
            handle.cam.distance = 0.55
            handle.cam.lookat[:] = [-0.42, -0.05, 0.16]
            while handle.is_running() and data.time < duration and not teleop.state.quit_requested:
                teleop.step()
                if data.time - last_print >= 1.0 or teleop.state.print_requested:
                    teleop.print_status()
                    teleop.state.print_requested = False
                    last_print = data.time
                if teleop.state.save_requested:
                    teleop.save_snapshot(snapshot_path)
                    teleop.state.save_requested = False
                handle.set_texts(
                    (
                        None,
                        None,
                        "JAKA RH56 teleop",
                        f"selected={CANONICAL_HAND_ORDER[teleop.state.selected_hand_dof]} "
                        f"hand={np.round(teleop.state.hand_norm, 2).tolist()} "
                        f"err={teleop.status()['target_error_m']:.4f}m",
                    )
                )
                handle.sync()
                time.sleep(model.opt.timestep)
    else:
        while data.time < duration:
            teleop.step()
            if data.time - last_print >= 1.0:
                teleop.print_status()
                last_print = data.time
        teleop.save_snapshot(snapshot_path)


def main() -> None:
    parser = argparse.ArgumentParser(description="Mouse/keyboard teleop tool for JAKA mini2 + RH56 in MuJoCo.")
    parser.add_argument("--base-xml", default=str(BASE_XML))
    parser.add_argument("--out-xml", default=str(TELEOP_XML))
    parser.add_argument("--scenario", choices=["hand_close", "cube_in_hand", "table_cube"], default="cube_in_hand")
    parser.add_argument("--collision-mode", choices=COLLISION_MODES, default="visual_coacd")
    parser.add_argument("--duration", type=float, default=600.0)
    parser.add_argument("--no-viewer", action="store_true")
    parser.add_argument("--target-step", type=float, default=0.005)
    parser.add_argument("--hand-step", type=float, default=0.03)
    parser.add_argument("--ik-gain", type=float, default=0.65)
    parser.add_argument("--ik-damping", type=float, default=0.04)
    parser.add_argument("--ik-max-step", type=float, default=0.015)
    parser.add_argument("--snapshot-path", default=str(TELEOP_SNAPSHOT))
    args = parser.parse_args()

    summary = build_teleop_xml(
        args.base_xml,
        args.out_xml,
        scenario=args.scenario,
        collision_mode=args.collision_mode,
    )
    print(json.dumps(summary, indent=2), flush=True)
    run_teleop(
        args.out_xml,
        scenario=args.scenario,
        viewer=not args.no_viewer,
        duration=args.duration,
        target_step=args.target_step,
        hand_step=args.hand_step,
        ik_gain=args.ik_gain,
        ik_damping=args.ik_damping,
        ik_max_step=args.ik_max_step,
        snapshot_path=args.snapshot_path,
    )


if __name__ == "__main__":
    main()
