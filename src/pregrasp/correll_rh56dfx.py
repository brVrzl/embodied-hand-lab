from __future__ import annotations

from dataclasses import asdict, dataclass
from functools import lru_cache
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np

from rh56_driver.hand_schema import CANONICAL_HAND_ORDER

PROJECT_ROOT = Path(__file__).resolve().parents[2]
CORRELL_ASSET_ROOT = PROJECT_ROOT / "data" / "sim_assets" / "correll_rh56dfx"

XML_FILES: Mapping[str, str] = {
    "fixed": "inspire_right.xml",
    "floating_grasp": "inspire_grasp_scene.xml",
    "floating_force": "inspire_force_scene.xml",
    "fixed_force": "inspire_scene.xml",
}

CORRELL_ACTUATOR_ORDER: tuple[str, ...] = (
    "pinky",
    "ring",
    "middle",
    "index",
    "thumb_proximal",
    "thumb_yaw",
)

TIP_SITE_NAMES: tuple[str, ...] = (
    "right_thumb_tip",
    "right_index_tip",
    "right_middle_tip",
    "right_ring_tip",
    "right_pinky_tip",
)

FORCE_SENSOR_NAMES: tuple[str, ...] = (
    "thumb_tip_force",
    "index_tip_force",
    "middle_tip_force",
    "ring_tip_force",
    "pinky_tip_force",
)

TORQUE_SENSOR_NAMES: tuple[str, ...] = (
    "thumb_tip_torque",
    "index_tip_torque",
    "middle_tip_torque",
    "ring_tip_torque",
    "pinky_tip_torque",
)

DEFAULT_CTRL_RANGES = {
    "pinky": (0.0, 1.57),
    "ring": (0.0, 1.57),
    "middle": (0.0, 1.50),
    "index": (0.0, 1.50),
    "thumb_proximal": (0.1, 0.57),
    "thumb_yaw": (0.0, 1.308),
}

LINE_THUMB_YAW_RAD = 1.16


@dataclass(frozen=True, slots=True)
class CorrellAssetValidation:
    xml_models: dict[str, dict[str, int]]
    missing_files: list[str]
    missing_actuators: dict[str, list[str]]
    missing_sites: dict[str, list[str]]
    missing_sensors: dict[str, list[str]]

    @property
    def valid(self) -> bool:
        return not (
            self.missing_files
            or any(self.missing_actuators.values())
            or any(self.missing_sites.values())
            or any(self.missing_sensors.values())
        )

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


@dataclass(frozen=True, slots=True)
class CorrellLinePregraspPlan:
    target_width_m: float
    achieved_width_m: float
    width_error_m: float
    canonical_command: list[float]
    correll_ctrl: list[float]
    tilt_y_rad: float
    midpoint_base_m: list[float]
    thumb_tip_base_m: list[float]
    index_tip_base_m: list[float]

    def to_dict(self) -> dict[str, object]:
        result = asdict(self)
        result["canonical_hand_order"] = list(CANONICAL_HAND_ORDER)
        result["correll_actuator_order"] = list(CORRELL_ACTUATOR_ORDER)
        return result


def asset_xml_path(kind: str) -> Path:
    try:
        filename = XML_FILES[kind]
    except KeyError as exc:
        raise ValueError(f"Unknown Correll RH56DFX XML kind {kind!r}.") from exc
    return CORRELL_ASSET_ROOT / filename


def validate_correll_assets() -> CorrellAssetValidation:
    mujoco = _mujoco()
    missing_files: list[str] = []
    xml_models: dict[str, dict[str, int]] = {}
    missing_actuators: dict[str, list[str]] = {}
    missing_sites: dict[str, list[str]] = {}
    missing_sensors: dict[str, list[str]] = {}

    for kind in XML_FILES:
        path = asset_xml_path(kind)
        if not path.exists():
            missing_files.append(str(path))
            continue
        model = mujoco.MjModel.from_xml_path(str(path))
        xml_models[kind] = {
            "nq": int(model.nq),
            "nu": int(model.nu),
            "nbody": int(model.nbody),
            "ngeom": int(model.ngeom),
            "nsite": int(model.nsite),
            "nsensor": int(model.nsensor),
        }
        missing_actuators[kind] = _missing_names(model, mujoco.mjtObj.mjOBJ_ACTUATOR, CORRELL_ACTUATOR_ORDER)
        expected_sites = TIP_SITE_NAMES if kind != "floating_force" else TIP_SITE_NAMES[:5]
        missing_sites[kind] = _missing_names(model, mujoco.mjtObj.mjOBJ_SITE, expected_sites)
        expected_sensors = (
            FORCE_SENSOR_NAMES + TORQUE_SENSOR_NAMES
            if kind in {"fixed_force", "floating_force"}
            else ()
        )
        missing_sensors[kind] = _missing_names(model, mujoco.mjtObj.mjOBJ_SENSOR, expected_sensors)

    return CorrellAssetValidation(
        xml_models=xml_models,
        missing_files=missing_files,
        missing_actuators=missing_actuators,
        missing_sites=missing_sites,
        missing_sensors=missing_sensors,
    )


def canonical_norm_to_correll_ctrl(
    canonical_command: Sequence[float],
    *,
    ctrl_ranges: Mapping[str, tuple[float, float]] | None = None,
) -> list[float]:
    command = _as_array(canonical_command, expected=len(CANONICAL_HAND_ORDER))
    ranges = ctrl_ranges or DEFAULT_CTRL_RANGES
    by_name = dict(zip(CANONICAL_HAND_ORDER, np.clip(command, 0.0, 1.0), strict=True))
    normalized_by_actuator = {
        "pinky": by_name["pinky"],
        "ring": by_name["ring"],
        "middle": by_name["middle"],
        "index": by_name["index"],
        "thumb_proximal": by_name["thumb_close"],
        "thumb_yaw": by_name["thumb_lateral"],
    }
    ctrl: list[float] = []
    for name in CORRELL_ACTUATOR_ORDER:
        lo, hi = ranges[name]
        ctrl.append(float(lo + normalized_by_actuator[name] * (hi - lo)))
    return ctrl


def correll_ctrl_to_canonical_norm(
    correll_ctrl: Sequence[float],
    *,
    ctrl_ranges: Mapping[str, tuple[float, float]] | None = None,
) -> list[float]:
    ctrl = _as_array(correll_ctrl, expected=len(CORRELL_ACTUATOR_ORDER))
    ranges = ctrl_ranges or DEFAULT_CTRL_RANGES
    normalized: dict[str, float] = {}
    for name, value in zip(CORRELL_ACTUATOR_ORDER, ctrl, strict=True):
        lo, hi = ranges[name]
        normalized[name] = float(np.clip((value - lo) / max(hi - lo, 1e-9), 0.0, 1.0))
    return [
        normalized["index"],
        normalized["middle"],
        normalized["ring"],
        normalized["pinky"],
        normalized["thumb_proximal"],
        normalized["thumb_yaw"],
    ]


class CorrellLineGraspPlanner:
    """Width-to-command planner backed by Correll's floating RH56DFX MuJoCo model."""

    def __init__(self, xml_path: str | Path | None = None) -> None:
        mujoco = _mujoco()
        self._mujoco = mujoco
        self.xml_path = Path(xml_path) if xml_path is not None else asset_xml_path("floating_grasp")
        self.model = mujoco.MjModel.from_xml_path(str(self.xml_path))
        self.data = mujoco.MjData(self.model)
        self.ctrl_ranges = self._read_ctrl_ranges()
        self._qpos_addr = {
            name: self._joint_qpos_addr(name)
            for name in (
                "thumb_proximal_yaw_joint",
                "thumb_proximal_pitch_joint",
                "thumb_intermediate_joint",
                "thumb_distal_joint",
                "index_proximal_joint",
                "index_intermediate_joint",
            )
        }
        self._thumb_site_id = self._site_id("right_thumb_tip")
        self._index_site_id = self._site_id("right_index_tip")

    def plan_line_width(self, width_m: float, *, samples: int = 240) -> CorrellLinePregraspPlan:
        if width_m <= 0.0:
            raise ValueError("width_m must be positive.")
        if samples < 16:
            raise ValueError("samples must be at least 16.")

        best: tuple[float, float, np.ndarray, np.ndarray, np.ndarray] | None = None
        for closure in np.linspace(0.0, 1.0, samples):
            ctrl = np.asarray(
                canonical_norm_to_correll_ctrl(
                    [closure, 0.0, 0.0, 0.0, closure, _line_thumb_yaw_norm(self.ctrl_ranges)],
                    ctrl_ranges=self.ctrl_ranges,
                ),
                dtype=np.float64,
            )
            thumb_tip, index_tip = self._finger_tips_for_ctrl(ctrl)
            delta = thumb_tip - index_tip
            achieved_width = float(np.hypot(delta[0], delta[2]))
            error = abs(achieved_width - width_m)
            if best is None or error < best[0]:
                best = (error, achieved_width, ctrl.copy(), thumb_tip.copy(), index_tip.copy())

        assert best is not None
        error, achieved_width, ctrl, thumb_tip, index_tip = best
        delta = thumb_tip - index_tip
        tilt_y = float(np.clip(np.arctan2(-delta[2], delta[0]), -np.pi / 2.0, np.pi / 2.0))
        midpoint = (thumb_tip + index_tip) * 0.5
        return CorrellLinePregraspPlan(
            target_width_m=float(width_m),
            achieved_width_m=achieved_width,
            width_error_m=float(error),
            canonical_command=correll_ctrl_to_canonical_norm(ctrl, ctrl_ranges=self.ctrl_ranges),
            correll_ctrl=ctrl.astype(float).tolist(),
            tilt_y_rad=tilt_y,
            midpoint_base_m=midpoint.astype(float).tolist(),
            thumb_tip_base_m=thumb_tip.astype(float).tolist(),
            index_tip_base_m=index_tip.astype(float).tolist(),
        )

    def _read_ctrl_ranges(self) -> dict[str, tuple[float, float]]:
        ranges: dict[str, tuple[float, float]] = {}
        for name in CORRELL_ACTUATOR_ORDER:
            actuator_id = self._required_id(self._mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            lo, hi = self.model.actuator_ctrlrange[actuator_id]
            ranges[name] = (float(lo), float(hi))
        return ranges

    def _finger_tips_for_ctrl(self, ctrl: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        self.data.qpos[:] = 0.0
        ctrl_by_name = dict(zip(CORRELL_ACTUATOR_ORDER, ctrl.tolist(), strict=True))
        thumb_pitch = float(ctrl_by_name["thumb_proximal"])
        thumb_yaw = float(ctrl_by_name["thumb_yaw"])
        index = float(ctrl_by_name["index"])
        q = self.data.qpos
        a = self._qpos_addr
        q[a["thumb_proximal_yaw_joint"]] = thumb_yaw
        q[a["thumb_proximal_pitch_joint"]] = thumb_pitch
        q[a["thumb_intermediate_joint"]] = 0.15 + 1.33 * thumb_pitch
        q[a["thumb_distal_joint"]] = 0.15 + 0.66 * thumb_pitch
        q[a["index_proximal_joint"]] = index
        q[a["index_intermediate_joint"]] = -0.05 + 1.1169 * index
        self._mujoco.mj_kinematics(self.model, self.data)
        return (
            self.data.site_xpos[self._thumb_site_id].copy(),
            self.data.site_xpos[self._index_site_id].copy(),
        )

    def _joint_qpos_addr(self, name: str) -> int:
        joint_id = self._required_id(self._mujoco.mjtObj.mjOBJ_JOINT, name)
        return int(self.model.jnt_qposadr[joint_id])

    def _site_id(self, name: str) -> int:
        return self._required_id(self._mujoco.mjtObj.mjOBJ_SITE, name)

    def _required_id(self, object_type: object, name: str) -> int:
        object_id = self._mujoco.mj_name2id(self.model, object_type, name)
        if object_id < 0:
            raise KeyError(f"{self.xml_path} is missing {name!r}.")
        return int(object_id)


def _missing_names(model: object, object_type: object, names: Sequence[str]) -> list[str]:
    mujoco = _mujoco()
    return [name for name in names if mujoco.mj_name2id(model, object_type, name) < 0]


def _line_thumb_yaw_norm(ctrl_ranges: Mapping[str, tuple[float, float]]) -> float:
    lo, hi = ctrl_ranges["thumb_yaw"]
    return float(np.clip((LINE_THUMB_YAW_RAD - lo) / max(hi - lo, 1e-9), 0.0, 1.0))


def _as_array(values: Sequence[float] | np.ndarray, *, expected: int) -> np.ndarray:
    array = np.asarray(values, dtype=np.float64).reshape(-1)
    if array.size != expected:
        raise ValueError(f"Expected {expected} values, got {array.size}.")
    if not np.isfinite(array).all():
        raise ValueError("Values contain NaN or infinite values.")
    return array


@lru_cache(maxsize=1)
def _mujoco():
    import mujoco

    return mujoco
