#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

## \namespace upkie.utils.rotations
## \brief Convert between various rotation representations.

"""!
Convert between various rotation representations.
"""

from typing import Tuple

import numpy as np
from scipy.spatial.transform import Rotation as ScipyRotation


def rotation_matrix_from_quaternion(
    quat: Tuple[float, float, float, float],
) -> np.ndarray:
    r"""!
    Convert a unit quaternion to the matrix representing the same rotation.

    \param quat Unit quaternion to convert, in `[w, x, y, z]` format.
    \return Rotation matrix corresponding to this quaternion.

    See `Conversion between quaternions and rotation matrices`_.

    .. _`Conversion between quaternions and rotation matrices`:
        https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Rotation_matrices
    """
    if abs(np.dot(quat, quat) - 1.0) > 1e-5:
        raise ValueError(f"Quaternion {quat} is not normalized")
    qw, qx, qy, qz = quat
    return np.array(
        [
            [
                1 - 2 * (qy**2 + qz**2),
                2 * (qx * qy - qz * qw),
                2 * (qw * qy + qx * qz),
            ],
            [
                2 * (qx * qy + qz * qw),
                1 - 2 * (qx**2 + qz**2),
                2 * (qy * qz - qx * qw),
            ],
            [
                2 * (qx * qz - qy * qw),
                2 * (qy * qz + qx * qw),
                1 - 2 * (qx**2 + qy**2),
            ],
        ]
    )


def quaternion_from_rotation_matrix(rotation_matrix: np.ndarray) -> np.ndarray:
    r"""!
    Convert a rotation matrix to a unit quaternion for the same rotation.

    \param rotation_matrix Rotation matrix to convert.
    \return Unit quaternion in `[w, x, y, z]` format.
    """
    if rotation_matrix.shape != (3, 3):
        raise ValueError(f"Expected 3x3 matrix, got {rotation_matrix.shape}")
    rotation = ScipyRotation.from_matrix(rotation_matrix)
    try:
        return rotation.as_quat(scalar_first=True)
    except TypeError:
        # Fallback for older scipy: as_quat() returns [x, y, z, w]
        quat_xyzw = rotation.as_quat()
        return np.array(
            [quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]]
        )
