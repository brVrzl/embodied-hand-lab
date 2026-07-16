"""Canonical JSON serialization for UMIP values."""

from __future__ import annotations

import json
from typing import Any, Mapping

from .errors import SerializationError
from .model import (
    DeviceDescriptor,
    GestureSample,
    HandArticulation,
    JointSample,
    MotionInputSample,
    MotionKind,
    Pose6D,
    Side,
    Timestamp,
    TrackingState,
    UMIP_VERSION,
)


def _timestamp_dict(value: Timestamp | None) -> dict[str, Any] | None:
    if value is None:
        return None
    return {
        "nanoseconds": value.nanoseconds,
        "clock_id": value.clock_id,
        "uncertainty_ns": value.uncertainty_ns,
    }


def _pose_dict(value: Pose6D | None) -> dict[str, Any] | None:
    if value is None:
        return None
    return {
        "position_m": list(value.position_m),
        "orientation_xyzw": list(value.orientation_xyzw),
    }


def device_to_dict(value: DeviceDescriptor) -> dict[str, Any]:
    return {
        "device_id": value.device_id,
        "device_type": value.device_type,
        "manufacturer": value.manufacturer,
        "model": value.model,
        "serial_number": value.serial_number,
        "firmware_version": value.firmware_version,
        "software_version": value.software_version,
        "metadata": dict(value.metadata),
    }


def device_from_dict(value: Mapping[str, Any]) -> DeviceDescriptor:
    try:
        return DeviceDescriptor(
            device_id=str(value["device_id"]),
            device_type=str(value["device_type"]),
            manufacturer=str(value["manufacturer"]),
            model=str(value["model"]),
            serial_number=_optional_str(value.get("serial_number")),
            firmware_version=_optional_str(value.get("firmware_version")),
            software_version=_optional_str(value.get("software_version")),
            metadata=_mapping(value.get("metadata", {}), "device.metadata"),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise SerializationError(f"invalid device descriptor: {exc}") from exc


def sample_to_dict(sample: MotionInputSample) -> dict[str, Any]:
    articulation = None
    if sample.articulation is not None:
        articulation = {
            "joints": [
                {
                    "name": joint.name,
                    "pose": _pose_dict(joint.pose),
                    "tracking_state": joint.tracking_state.value,
                    "confidence": joint.confidence,
                    "radius_m": joint.radius_m,
                }
                for joint in sample.articulation.joints
            ],
            "gestures": [
                {
                    "name": gesture.name,
                    "active": gesture.active,
                    "confidence": gesture.confidence,
                    "value": gesture.value,
                }
                for gesture in sample.articulation.gestures
            ],
            "pinch_strength": sample.articulation.pinch_strength,
            "grasp_strength": sample.articulation.grasp_strength,
            "confidence": sample.articulation.confidence,
        }
    return {
        "protocol_version": sample.protocol_version,
        "sample_id": sample.sample_id,
        "stream_id": sample.stream_id,
        "sequence_number": sample.sequence_number,
        "capture_timestamp": _timestamp_dict(sample.capture_timestamp),
        "receive_timestamp": _timestamp_dict(sample.receive_timestamp),
        "device_timestamp": _timestamp_dict(sample.device_timestamp),
        "processing_timestamp": _timestamp_dict(sample.processing_timestamp),
        "tracking_state": sample.tracking_state.value,
        "tracking_confidence": sample.tracking_confidence,
        "coordinate_frame": sample.coordinate_frame,
        "device": device_to_dict(sample.device),
        "side": sample.side.value,
        "wrist_pose": _pose_dict(sample.wrist_pose),
        "palm_pose": _pose_dict(sample.palm_pose),
        "motion_kind": sample.motion_kind.value,
        "articulation": articulation,
        "metadata": dict(sample.metadata),
        "extensions": dict(sample.extensions),
    }


def sample_from_dict(value: Mapping[str, Any]) -> MotionInputSample:
    try:
        version = str(value.get("protocol_version", UMIP_VERSION))
        if version.split(".", 1)[0] != UMIP_VERSION.split(".", 1)[0]:
            raise SerializationError(f"unsupported UMIP major version {version!r}")
        articulation_value = value.get("articulation")
        articulation = None
        if articulation_value is not None:
            articulation_map = _mapping(articulation_value, "articulation")
            articulation = HandArticulation(
                joints=tuple(_joint_from_dict(item) for item in articulation_map.get("joints", [])),
                gestures=tuple(
                    _gesture_from_dict(item) for item in articulation_map.get("gestures", [])
                ),
                pinch_strength=articulation_map.get("pinch_strength"),
                grasp_strength=articulation_map.get("grasp_strength"),
                confidence=articulation_map.get("confidence"),
            )
        return MotionInputSample(
            protocol_version=version,
            sample_id=str(value["sample_id"]),
            stream_id=str(value["stream_id"]),
            sequence_number=int(value["sequence_number"]),
            capture_timestamp=_timestamp_from_dict(value["capture_timestamp"], required=True),
            receive_timestamp=_timestamp_from_dict(value["receive_timestamp"], required=True),
            device_timestamp=_timestamp_from_dict(value.get("device_timestamp")),
            processing_timestamp=_timestamp_from_dict(value.get("processing_timestamp")),
            tracking_state=TrackingState(str(value["tracking_state"])),
            tracking_confidence=value.get("tracking_confidence"),
            coordinate_frame=str(value["coordinate_frame"]),
            device=device_from_dict(_mapping(value["device"], "device")),
            side=Side(str(value["side"])),
            wrist_pose=_pose_from_dict(value.get("wrist_pose")),
            palm_pose=_pose_from_dict(value.get("palm_pose")),
            motion_kind=MotionKind(str(value.get("motion_kind", MotionKind.ABSOLUTE.value))),
            articulation=articulation,
            metadata=_mapping(value.get("metadata", {}), "metadata"),
            extensions=_mapping(value.get("extensions", {}), "extensions"),
        )
    except SerializationError:
        raise
    except (KeyError, TypeError, ValueError) as exc:
        raise SerializationError(f"invalid UMIP sample: {exc}") from exc


def dumps_sample(sample: MotionInputSample, *, pretty: bool = False) -> str:
    return json.dumps(
        sample_to_dict(sample),
        ensure_ascii=False,
        allow_nan=False,
        sort_keys=True,
        indent=2 if pretty else None,
        separators=None if pretty else (",", ":"),
    )


def loads_sample(payload: str | bytes) -> MotionInputSample:
    try:
        value = json.loads(payload)
    except (json.JSONDecodeError, UnicodeDecodeError) as exc:
        raise SerializationError(f"invalid JSON: {exc}") from exc
    return sample_from_dict(_mapping(value, "sample"))


def _optional_str(value: Any) -> str | None:
    return None if value is None else str(value)


def _mapping(value: Any, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping) or not all(isinstance(key, str) for key in value):
        raise SerializationError(f"{name} must be an object")
    return value


def _timestamp_from_dict(value: Any, *, required: bool = False) -> Timestamp | None:
    if value is None:
        if required:
            raise SerializationError("required timestamp is null")
        return None
    item = _mapping(value, "timestamp")
    return Timestamp(
        nanoseconds=int(item["nanoseconds"]),
        clock_id=str(item["clock_id"]),
        uncertainty_ns=None if item.get("uncertainty_ns") is None else int(item["uncertainty_ns"]),
    )


def _pose_from_dict(value: Any) -> Pose6D | None:
    if value is None:
        return None
    item = _mapping(value, "pose")
    return Pose6D(
        position_m=tuple(float(number) for number in item["position_m"]),
        orientation_xyzw=tuple(float(number) for number in item["orientation_xyzw"]),
    )


def _joint_from_dict(value: Any) -> JointSample:
    item = _mapping(value, "joint")
    return JointSample(
        name=str(item["name"]),
        pose=_pose_from_dict(item.get("pose")),
        tracking_state=TrackingState(str(item["tracking_state"])),
        confidence=item.get("confidence"),
        radius_m=item.get("radius_m"),
    )


def _gesture_from_dict(value: Any) -> GestureSample:
    item = _mapping(value, "gesture")
    return GestureSample(
        name=str(item["name"]),
        active=bool(item["active"]),
        confidence=item.get("confidence"),
        value=item.get("value"),
    )
