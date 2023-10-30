#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron
# SPDX-License-Identifier: Apache-2.0

from typing import Tuple
from numpy.typing import NDArray

import numpy as np


def rotation_matrix_from_quaternion(
    quat: Tuple[float, float, float, float]
) -> NDArray[float]:
    """Convert a unit quaternion to the matrix representing the same rotation.

    Args:
        quat: Unit quaternion to convert, in ``[w, x, y, z]`` format.

    Returns:
        Rotation matrix corresponding to this quaternion.

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
                1 - 2 * (qy ** 2 + qz ** 2),
                2 * (qx * qy - qz * qw),
                2 * (qw * qy + qx * qz),
            ],
            [
                2 * (qx * qy + qz * qw),
                1 - 2 * (qx ** 2 + qz ** 2),
                2 * (qy * qz - qx * qw),
            ],
            [
                2 * (qx * qz - qy * qw),
                2 * (qy * qz + qx * qw),
                1 - 2 * (qx ** 2 + qy ** 2),
            ],
        ]
    )
