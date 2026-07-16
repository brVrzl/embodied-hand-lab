"""Interfaces for later gates; deliberately not wired to live robot motion."""

from __future__ import annotations

from typing import Protocol

from .contracts import ArmPoseSample, PoseTarget, RobotState, SafetyState


class MeasurementFilter(Protocol):
    def filter(self, sample: ArmPoseSample) -> ArmPoseSample: ...


class TargetShaper(Protocol):
    def shape(self, sample: ArmPoseSample, robot_state: RobotState) -> PoseTarget: ...


class SafetyLimiter(Protocol):
    def evaluate(self, target: PoseTarget, robot_state: RobotState) -> SafetyState: ...


class TrajectoryGenerator(Protocol):
    def update(self, target: PoseTarget, robot_state: RobotState, monotonic_ns: int) -> tuple[float, ...]: ...
