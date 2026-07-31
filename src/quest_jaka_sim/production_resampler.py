"""ctypes bridge to the production native latest-destination/PWL resampler."""

from __future__ import annotations

import ctypes
from dataclasses import dataclass
from pathlib import Path
import sys
from typing import Iterable


SERVO_PERIOD_NS = 8_000_000


class _CResamplerPoint(ctypes.Structure):
    _fields_ = [
        ("position_rad", ctypes.c_double * 6),
        ("segment_velocity_rad_s", ctypes.c_double * 6),
        ("emitted_velocity_rad_s", ctypes.c_double * 6),
        ("emitted_acceleration_rad_s2", ctypes.c_double * 6),
        ("emitted_jerk_rad_s3", ctypes.c_double * 6),
        ("servo_time_ns", ctypes.c_uint64),
        ("from_sequence", ctypes.c_uint64),
        ("to_sequence", ctypes.c_uint64),
        ("from_accepted_ns", ctypes.c_uint64),
        ("to_accepted_ns", ctypes.c_uint64),
        ("alpha", ctypes.c_double),
        ("endpoint", ctypes.c_uint8),
        ("transition_limited", ctypes.c_uint8),
        ("recovered_from_transition", ctypes.c_uint8),
    ]


@dataclass(frozen=True, slots=True)
class ProductionResampledPoint:
    position_rad: tuple[float, ...]
    segment_velocity_rad_s: tuple[float, ...]
    emitted_velocity_rad_s: tuple[float, ...]
    emitted_acceleration_rad_s2: tuple[float, ...]
    emitted_jerk_rad_s3: tuple[float, ...]
    servo_time_ns: int
    from_sequence: int
    to_sequence: int
    from_accepted_ns: int
    to_accepted_ns: int
    alpha: float
    endpoint: bool
    transition_limited: bool
    recovered_from_transition: bool


def default_resampler_library(repository_root: Path | None = None) -> Path:
    root = Path(__file__).resolve().parents[2] if repository_root is None else repository_root
    suffix = (
        ".dylib"
        if sys.platform == "darwin"
        else ".dll"
        if sys.platform == "win32"
        else ".so"
    )
    return root / f"build/jaka_servo_worker/libjaka_servo_resampler{suffix}"


class ProductionJointServoResampler:
    """Owned instance of the exact resampler compiled into the native worker."""

    def __init__(self, library_path: Path | None = None) -> None:
        self.library_path = default_resampler_library() if library_path is None else Path(library_path)
        if not self.library_path.is_file():
            raise FileNotFoundError(
                f"production resampler library not built: {self.library_path}; "
                "run cmake -S native/jaka_servo_worker -B build/jaka_servo_worker "
                "&& cmake --build build/jaka_servo_worker -j"
            )
        self._library = ctypes.CDLL(str(self.library_path))
        self._configure()
        self._handle = self._library.jaka_resampler_create()
        if not self._handle:
            raise RuntimeError(self._error())

    def _configure(self) -> None:
        library = self._library
        pointer = ctypes.POINTER(ctypes.c_double)
        library.jaka_resampler_create.restype = ctypes.c_void_p
        library.jaka_resampler_destroy.argtypes = [ctypes.c_void_p]
        library.jaka_resampler_initialize.argtypes = [ctypes.c_void_p, pointer, ctypes.c_uint64]
        library.jaka_resampler_initialize.restype = ctypes.c_int
        library.jaka_resampler_hold.argtypes = [
            ctypes.c_void_p,
            pointer,
            ctypes.c_uint64,
            ctypes.c_uint64,
        ]
        library.jaka_resampler_hold.restype = ctypes.c_int
        library.jaka_resampler_configure_transition.argtypes = [
            ctypes.c_void_p,
            pointer,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
        ]
        library.jaka_resampler_configure_transition.restype = ctypes.c_int
        library.jaka_resampler_accept.argtypes = [
            ctypes.c_void_p,
            pointer,
            ctypes.c_uint64,
            ctypes.c_uint64,
        ]
        library.jaka_resampler_accept.restype = ctypes.c_int
        library.jaka_resampler_evaluate_selected.argtypes = [
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(_CResamplerPoint),
        ]
        library.jaka_resampler_evaluate_selected.restype = ctypes.c_int
        library.jaka_resampler_last_error.restype = ctypes.c_char_p

    @staticmethod
    def _six(values: Iterable[float]) -> ctypes.Array[ctypes.c_double]:
        result = tuple(float(value) for value in values)
        if len(result) != 6:
            raise ValueError("production resampler requires six joints")
        return (ctypes.c_double * 6)(*result)

    def _error(self) -> str:
        raw = self._library.jaka_resampler_last_error()
        return "production resampler operation failed" if not raw else raw.decode("utf-8")

    def _check(self, code: int) -> None:
        if code != 0:
            raise RuntimeError(self._error())

    def initialize(self, position_rad: Iterable[float], servo_time_ns: int) -> None:
        self._check(
            self._library.jaka_resampler_initialize(
                self._handle, self._six(position_rad), int(servo_time_ns)
            )
        )

    def configure_transition(
        self,
        *,
        maximum_velocity_rad_s: Iterable[float],
        recoverable_acceleration_rad_s2: float,
        hard_acceleration_rad_s2: float,
        maximum_jerk_rad_s3: float,
    ) -> None:
        self._check(
            self._library.jaka_resampler_configure_transition(
                self._handle,
                self._six(maximum_velocity_rad_s),
                float(recoverable_acceleration_rad_s2),
                float(hard_acceleration_rad_s2),
                float(maximum_jerk_rad_s3),
            )
        )

    def accept(self, position_rad: Iterable[float], accepted_ns: int, sequence: int) -> None:
        self._check(
            self._library.jaka_resampler_accept(
                self._handle,
                self._six(position_rad),
                int(accepted_ns),
                int(sequence),
            )
        )

    def hold(self, position_rad: Iterable[float], accepted_ns: int, sequence: int) -> None:
        self._check(
            self._library.jaka_resampler_hold(
                self._handle,
                self._six(position_rad),
                int(accepted_ns),
                int(sequence),
            )
        )

    def evaluate_and_commit(self, servo_time_ns: int) -> ProductionResampledPoint:
        point = _CResamplerPoint()
        self._check(
            self._library.jaka_resampler_evaluate_selected(
                self._handle, int(servo_time_ns), ctypes.byref(point)
            )
        )
        return ProductionResampledPoint(
            position_rad=tuple(point.position_rad),
            segment_velocity_rad_s=tuple(point.segment_velocity_rad_s),
            emitted_velocity_rad_s=tuple(point.emitted_velocity_rad_s),
            emitted_acceleration_rad_s2=tuple(point.emitted_acceleration_rad_s2),
            emitted_jerk_rad_s3=tuple(point.emitted_jerk_rad_s3),
            servo_time_ns=int(point.servo_time_ns),
            from_sequence=int(point.from_sequence),
            to_sequence=int(point.to_sequence),
            from_accepted_ns=int(point.from_accepted_ns),
            to_accepted_ns=int(point.to_accepted_ns),
            alpha=float(point.alpha),
            endpoint=bool(point.endpoint),
            transition_limited=bool(point.transition_limited),
            recovered_from_transition=bool(point.recovered_from_transition),
        )

    def close(self) -> None:
        handle = getattr(self, "_handle", None)
        if handle:
            self._library.jaka_resampler_destroy(handle)
            self._handle = None

    def __enter__(self) -> "ProductionJointServoResampler":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def __del__(self) -> None:
        self.close()
