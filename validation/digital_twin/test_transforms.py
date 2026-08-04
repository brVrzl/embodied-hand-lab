from __future__ import annotations

import numpy as np
import pytest

from digital_twin.registration.transforms import (
    apply_similarity,
    compose_transforms,
    invert_transform,
    matrix_to_quaternion_xyzw,
    quaternion_xyzw_to_matrix,
    ransac_similarity,
    umeyama_similarity,
)
from tools.digital_twin.register_reconstruction_to_robot import compose_to_B


def rigid(translation, quaternion):
    matrix = np.eye(4)
    matrix[:3, :3] = quaternion_xyzw_to_matrix(quaternion)
    matrix[:3, 3] = translation
    return matrix


def test_transform_composition_and_inverse() -> None:
    T_A_B = rigid([1, 2, 3], [0, 0, 0, 1])
    T_B_C = rigid([0.5, 0, 0], [0, 0, np.sin(np.pi / 4), np.cos(np.pi / 4)])
    T_A_C = compose_transforms(T_A_B, T_B_C)
    assert np.allclose(invert_transform(T_A_C) @ T_A_C, np.eye(4))
    assert np.allclose(T_A_C[:3, 3], [1.5, 2, 3])


def test_quaternion_round_trip() -> None:
    quaternion = np.asarray([0.2, -0.3, 0.1, 0.9])
    quaternion /= np.linalg.norm(quaternion)
    recovered = matrix_to_quaternion_xyzw(quaternion_xyzw_to_matrix(quaternion))
    assert np.allclose(recovered, quaternion) or np.allclose(recovered, -quaternion)


def test_umeyama_recovers_similarity() -> None:
    source = np.asarray([[0, 0, 0], [1, 0, 0], [0, 2, 0], [0, 0, 3], [1, 1, 1]], float)
    rotation = quaternion_xyzw_to_matrix([0, 0, np.sin(0.25), np.cos(0.25)])
    target = apply_similarity(source, 1.7, rotation, np.asarray([0.4, -0.2, 1.0]))
    result = umeyama_similarity(source, target)
    assert result.scale == pytest.approx(1.7)
    assert np.allclose(result.rotation, rotation)
    assert np.allclose(result.translation, [0.4, -0.2, 1.0])
    assert result.rms_error < 1e-12


def test_ransac_rejects_outlier_deterministically() -> None:
    rng = np.random.default_rng(9)
    source = rng.normal(size=(20, 3))
    target = apply_similarity(source, 0.8, np.eye(3), np.asarray([1, 2, 3]))
    target[4] += [4, -3, 2]
    result = ransac_similarity(source, target, threshold=1e-5, iterations=200, seed=7)
    assert not result.inliers[4]
    assert result.inliers.sum() == 19
    assert result.scale == pytest.approx(0.8)


def test_umeyama_rejects_collinear_input() -> None:
    source = np.asarray([[0, 0, 0], [1, 0, 0], [2, 0, 0]], float)
    with pytest.raises(ValueError, match="collinear"):
        umeyama_similarity(source, source)


def test_staged_P_registration_composes_to_B() -> None:
    registration = {
        "target_frame": "P",
        "scale": 0.1,
        "rotation_matrix": np.eye(3).tolist(),
        "translation_m": [1.0, 2.0, 3.0],
    }
    config = {"transforms": {"T_B_P": {
        "translation_m": [0.5, 0.0, 0.0],
        "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
    }}}
    result = compose_to_B(registration, config)
    assert result["transform"] == "T_B_R"
    assert result["scale"] == pytest.approx(0.1)
    assert result["translation_m"] == pytest.approx([1.5, 2.0, 3.0])


def test_staged_P_registration_preserves_unresolved_T_B_P() -> None:
    registration = {"target_frame": "P", "scale": 1.0, "rotation_matrix": np.eye(3).tolist(), "translation_m": [0, 0, 0]}
    config = {"transforms": {"T_B_P": {"translation_m": [None] * 3, "quaternion_xyzw": [None] * 4}}}
    assert compose_to_B(registration, config)["status"] == "unresolved"
