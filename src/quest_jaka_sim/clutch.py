"""Compatibility exports for the input-only dual-clutch contract.

The canonical implementation lives in :mod:`motion_input.clutch` so an input-
only transport gate can consume it without importing the simulation package or
MuJoCo.  Existing Quest/JAKA imports retain the same public names.
"""

from motion_input.clutch import (
    AnalogClutchSample,
    AnalogHoldToRun,
    ArmClutchMachine,
    ArmClutchState,
    ClutchAction,
    ClutchFault,
    ClutchTransition,
    HandClutchMachine,
    HandClutchState,
    HysteresisObservation,
)

__all__ = [
    "AnalogClutchSample",
    "AnalogHoldToRun",
    "ArmClutchMachine",
    "ArmClutchState",
    "ClutchAction",
    "ClutchFault",
    "ClutchTransition",
    "HandClutchMachine",
    "HandClutchState",
    "HysteresisObservation",
]
