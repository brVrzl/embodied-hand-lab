from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np

from embodiment_core.types import CameraIntrinsics


class CameraInterface(ABC):
    @abstractmethod
    def get_rgb(self) -> np.ndarray:
        raise NotImplementedError

    @abstractmethod
    def get_depth(self) -> np.ndarray:
        raise NotImplementedError

    @abstractmethod
    def get_intrinsics(self) -> CameraIntrinsics:
        raise NotImplementedError

    @abstractmethod
    def get_timestamp(self) -> float:
        raise NotImplementedError

