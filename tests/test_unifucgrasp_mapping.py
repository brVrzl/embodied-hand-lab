from __future__ import annotations

import numpy as np

from rh56_driver.unifucgrasp_mapping import (
    UNIFUC_INSPIRE_JOINT_UPPER,
    inspire12_to_rh56_canonical_norm,
    mapping_metadata,
    normalize_unifuc_inspire_joints,
    parse_unifuc_inspire_target,
)


def test_unifuc_joint_normalization_clips_to_unit_range() -> None:
    values = UNIFUC_INSPIRE_JOINT_UPPER * 0.5
    normalized = normalize_unifuc_inspire_joints(values)

    assert np.allclose(normalized, [0.5] * 12)
    assert np.allclose(normalize_unifuc_inspire_joints(UNIFUC_INSPIRE_JOINT_UPPER * 2.0), [1.0] * 12)


def test_unifuc_inspire12_maps_to_project_rh56_canonical_order() -> None:
    joints = UNIFUC_INSPIRE_JOINT_UPPER * np.asarray(
        [
            0.6,  # thumb lateral
            0.2,
            0.4,
            0.6,  # thumb close avg = 0.4
            0.1,
            0.3,  # index avg = 0.2
            0.2,
            0.4,  # middle avg = 0.3
            0.3,
            0.5,  # ring avg = 0.4
            0.4,
            0.6,  # pinky avg = 0.5
        ],
        dtype=np.float32,
    )

    mapped = inspire12_to_rh56_canonical_norm(joints)

    assert np.allclose(mapped, [0.2, 0.3, 0.4, 0.5, 0.4, 0.6], atol=1e-6)


def test_parse_unifuc_inspire_target_returns_rh56_fields() -> None:
    target = np.zeros(19, dtype=np.float32)
    target[:3] = [1.0, 2.0, 3.0]
    target[3:7] = [1.0, 0.0, 0.0, 0.0]
    target[7:] = UNIFUC_INSPIRE_JOINT_UPPER

    pose = parse_unifuc_inspire_target(target)

    assert pose.position_m == [1.0, 2.0, 3.0]
    assert pose.quat_wxyz == [1.0, 0.0, 0.0, 0.0]
    assert np.allclose(pose.rh56_canonical_norm, [1.0] * 6)
    assert len(pose.rh56_raw_order) == 6


def test_mapping_metadata_is_serializable() -> None:
    metadata = mapping_metadata()

    assert metadata["target_canonical_order"] == ["index", "middle", "ring", "pinky", "thumb_close", "thumb_lateral"]
