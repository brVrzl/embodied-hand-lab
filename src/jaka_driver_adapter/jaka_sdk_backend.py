from __future__ import annotations

import ctypes
import importlib
import math
from pathlib import Path
import sys
from types import ModuleType
from typing import Any

from embodiment_core.logger import get_logger
from embodiment_core.types import JointState, Pose

from .interfaces import JakaBackend


class JakaSDKBackend(JakaBackend):
    """Adapter for the official JAKA Python SDK.

    The callable names used here are grounded in the local SDK demo/docs found under:
    `/home/w/projects/RoboTwin/机械臂资料/机械臂/SDK V2.2.7/`.
    """

    def __init__(self, config: dict[str, Any]) -> None:
        self.config = config
        self.logger = get_logger("JakaSDKBackend")
        self.speed_scale = float(config.get("default_speed_scale", 0.2))
        self._sdk_module: ModuleType | None = None
        self._robot: Any | None = None
        self._joint_names = config.get("joint_names", [f"joint_{i+1}" for i in range(6)])

    def connect(self) -> bool:
        self._sdk_module = self._load_sdk_module()
        robot_cls = getattr(self._sdk_module, "RC", None)
        if robot_cls is None:
            raise RuntimeError("Official JAKA Python SDK module does not export RC.")
        self._robot = robot_cls(self.config["ip"])
        self._login()
        self.logger.info("Connected to JAKA controller at %s", self.config["ip"])

        sdk_cfg = self.config.get("sdk", {})
        if sdk_cfg.get("auto_power_on", False):
            self._require_success(self._invoke("power_on"), "power_on")
        if sdk_cfg.get("auto_enable_robot", False):
            self._require_success(self._invoke("enable_robot"), "enable_robot")

        if self.speed_scale:
            try:
                self.set_speed_scale(self.speed_scale)
            except Exception as exc:
                self.logger.warning("set_speed_scale skipped during connect: %s", exc)
        return True

    def get_joint_state(self) -> JointState:
        result = self._invoke("get_joint_position")
        joints = self._extract_joint_values(result)
        return JointState(names=list(self._joint_names), positions=joints)

    def move_joints(self, joints: list[float], blocking: bool = True) -> bool:
        result = self._invoke(
            "joint_move",
            list(joints),
            0,
            bool(blocking),
            self._scaled_joint_speed(),
        )
        self._require_success(result, "joint_move")
        return True

    def move_pose(self, pose: Pose, blocking: bool = True) -> bool:
        sdk_pose = self._encode_pose_for_sdk(pose)
        try:
            result = self._invoke(
                "linear_move",
                sdk_pose,
                0,
                bool(blocking),
                self._scaled_linear_speed_mm_s(),
            )
            self._require_success(result, "linear_move")
        except TypeError as exc:
            if "argument" not in str(exc):
                raise
            result = self._invoke(
                "linear_move_extend",
                sdk_pose,
                0,
                bool(blocking),
                self._scaled_linear_speed_mm_s(),
                self._scaled_linear_acc_mm_s2(),
                1,
                None,
            )
            self._require_success(result, "linear_move_extend")
        return True

    def stop(self) -> None:
        self._require_success(self._invoke("motion_abort"), "motion_abort")

    def set_speed_scale(self, scale: float) -> None:
        if scale <= 0.0 or scale > 1.0:
            raise ValueError("Speed scale must be within (0.0, 1.0].")
        self.speed_scale = scale
        if self._robot is None:
            return
        if hasattr(self._robot, "set_rapidrate"):
            try:
                self._require_success(self._invoke("set_rapidrate", int(scale * 100)), "set_rapidrate")
            except RuntimeError as exc:
                if "set_rapidrate" in str(exc) and "code -61" in str(exc):
                    self.logger.warning(
                        "set_rapidrate unsupported on this controller/SDK combination; using local speed scaling only."
                    )
                    return
                raise
        else:
            self.logger.warning(
                "SDK method set_rapidrate() is unavailable; speed scale will only be applied locally."
            )

    def disconnect(self) -> None:
        if self._robot is None:
            return
        for method_name in ("logout", "log_out", "login_out"):
            method = getattr(self._robot, method_name, None)
            if method is None:
                continue
            try:
                result = method()
            except TypeError:
                continue
            self._require_success(result, method_name)
            self.logger.info("Disconnected from JAKA controller.")
            return
        self.logger.warning("No supported JAKA SDK logout method found; leaving session as-is.")

    def call_sdk_method(self, method_name: str, *args: Any) -> Any:
        return self._invoke(method_name, *args)

    @staticmethod
    def ensure_success(result: Any, method_name: str) -> None:
        JakaSDKBackend._require_success(result, method_name)

    def _load_sdk_module(self) -> ModuleType:
        module_name = self.config.get("sdk", {}).get("python_module", "jkrc")
        search_paths = self.config.get("sdk", {}).get("python_search_paths", [])
        for candidate in search_paths:
            if candidate and candidate not in sys.path:
                sys.path.insert(0, candidate)
            self._preload_sdk_dependencies(candidate)
        try:
            return importlib.import_module(module_name)
        except Exception as exc:
            raise RuntimeError(
                f"Failed to import official JAKA Python SDK module {module_name!r}. "
                "Check configs/robot/jaka_mini2.yaml sdk.python_search_paths."
            ) from exc

    def _preload_sdk_dependencies(self, candidate: str) -> None:
        if not candidate:
            return
        sdk_dir = Path(candidate)
        lib_path = sdk_dir / "libjakaAPI.so"
        if not lib_path.exists():
            return
        try:
            ctypes.CDLL(str(lib_path), mode=ctypes.RTLD_GLOBAL)
        except OSError as exc:
            raise RuntimeError(f"Failed to preload JAKA SDK dependency: {lib_path}") from exc

    def _invoke(self, method_name: str, *args: Any) -> Any:
        if self._robot is None:
            raise RuntimeError("JAKA SDK backend is not connected.")
        method = getattr(self._robot, method_name, None)
        if method is None:
            raise RuntimeError(f"Official JAKA SDK object does not provide {method_name}().")
        return method(*args)

    def _login(self) -> None:
        if self._robot is None:
            raise RuntimeError("JAKA SDK backend is not connected.")
        failures: list[str] = []
        for method_name, args in (
            ("login", ()),
            ("log_in", ()),
            ("login_in", ()),
            ("login_in", (self.config["ip"],)),
        ):
            method = getattr(self._robot, method_name, None)
            if method is None:
                continue
            try:
                result = method(*args)
            except TypeError:
                continue
            err = result[0] if isinstance(result, tuple) else result
            if err == 0:
                return
            failures.append(f"{method_name}{args} -> code {err}")
        details = "; ".join(failures) if failures else "no compatible method signature found"
        raise RuntimeError(f"Could not log in through JAKA SDK: {details}.")

    @staticmethod
    def _require_success(result: Any, method_name: str) -> None:
        err = result[0] if isinstance(result, tuple) else result
        if err != 0:
            raise RuntimeError(f"JAKA SDK call {method_name} failed with code {err}.")

    @staticmethod
    def _extract_joint_values(result: Any) -> list[float]:
        if isinstance(result, tuple):
            if len(result) >= 2 and hasattr(result[1], "jVal"):
                return [float(v) for v in result[1].jVal]
            if len(result) >= 2 and isinstance(result[1], (list, tuple)):
                return [float(v) for v in result[1]]
        if hasattr(result, "jVal"):
            return [float(v) for v in result.jVal]
        if isinstance(result, (list, tuple)) and len(result) == 6:
            return [float(v) for v in result]
        raise RuntimeError(f"Unrecognized JAKA joint state payload: {result!r}")

    def _encode_pose_for_sdk(self, pose: Pose) -> list[float]:
        # Inference from the local C++ SDK docs: linear_move commonly takes
        # [x_mm, y_mm, z_mm, rx_rad, ry_rad, rz_rad]. Confirm on the target machine.
        x_mm, y_mm, z_mm = [float(v) * 1000.0 for v in pose.position]
        qx, qy, qz, qw = [float(v) for v in pose.orientation_xyzw]
        rx, ry, rz = self._quat_to_rpy(qx, qy, qz, qw)
        return [x_mm, y_mm, z_mm, rx, ry, rz]

    @staticmethod
    def _quat_to_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    def _scaled_joint_speed(self) -> float:
        return max(0.01, 1.0 * self.speed_scale)

    def _scaled_joint_acc(self) -> float:
        return max(0.01, 1.0 * self.speed_scale)

    def _scaled_linear_speed_mm_s(self) -> float:
        return max(1.0, 100.0 * self.speed_scale)

    def _scaled_linear_acc_mm_s2(self) -> float:
        return max(1.0, 100.0 * self.speed_scale)
