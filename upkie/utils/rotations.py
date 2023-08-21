#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from typing import Tuple

import numpy as np


def rotation_matrix_from_quaternion(
    quat: Tuple[float, float, float, float]
) -> np.ndarray:
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
