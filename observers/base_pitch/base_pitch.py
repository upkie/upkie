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

from typing import Optional, Tuple

import numpy as np

from upkie.utils.clamp import clamp
from upkie.utils.rotations import rotation_matrix_from_quaternion


def compute_pitch_frame_in_parent(
    orientation_frame_in_parent: np.ndarray,
) -> float:
    """
    Get pitch angle of a given frame relative to the parent vertical.

    Figure:
        parent z
          ^    frame z
          |     /
          |    /
          |   /
          |  /
          | /
          |/
         (x)-----------> heading vector
           \\  )
            \\+  positive pitch
             \\
              \\
               \\
              frame x

    Args:
        orientation_frame_in_parent: Rotation matrix from the target frame to
            the parent frame.

    Returns:
        Angle from the parent z-axis (gravity) to the frame z-axis.
        Equivalently, angle from the heading vector to the sagittal vector of
        the frame.

    Note:
        Angle is positive in the trigonometric sense (CCW positive, CW
        negative) in the heading-vertical plane directed by the lateral vector,
        which is ``(x)`` in the above schematic (pointing away) and not ``(.)``
        (poiting from screen to the reader).
    """
    sagittal = orientation_frame_in_parent[:, 0]
    sagittal /= np.linalg.norm(sagittal)  # needed for prec around cos_pitch=0.
    heading_in_parent = sagittal - sagittal[2] * np.array([0.0, 0.0, 1.0])
    heading_in_parent /= np.linalg.norm(heading_in_parent)
    if orientation_frame_in_parent[2, 2] < 0:
        heading_in_parent *= -1.0
    sign = +1 if sagittal[2] < 0 else -1
    cos_pitch = clamp(np.dot(sagittal, heading_in_parent), -1.0, 1.0)
    pitch: float = sign * np.arccos(cos_pitch)
    return pitch


def compute_base_orientation_from_imu(
    quat_imu_in_ars: Tuple[float, float, float, float],
    rotation_base_to_imu: Optional[np.ndarray] = None,
) -> np.ndarray:
    """Get the orientation of the base frame with respect to the world frame.

    Args:
        quat_imu_in_ars: Quaternion representing the rotation matrix from
            the IMU frame to the  attitude reference system (ARS) frame, in
            ``[w, x, y, z]`` format.
        rotation_base_to_imu: Rotation matrix from the base frame to the IMU
            frame. When not specified, the default Upkie mounting orientation
            is used.

    Returns:
        Rotation matrix from the base frame to the world frame.
    """
    if rotation_base_to_imu is None:
        # Default Upkie mounting orientation: USB connectors of the raspi
        # pointing to the left of the robot (XT-30 of the pi3hat to the right)
        rotation_base_to_imu = np.diag([-1.0, 1.0, -1.0])

    rotation_imu_to_ars = rotation_matrix_from_quaternion(quat_imu_in_ars)

    # The attitude reference system frame has +x forward, +y right and +z down,
    # whereas our world frame has +x forward, +y left and +z up:
    # https://github.com/mjbots/pi3hat/blob/ab632c82bd501b9fcb6f8200df0551989292b7a1/docs/reference.md#orientation
    rotation_ars_to_world = np.diag([1.0, -1.0, -1.0])

    rotation_base_to_world = (
        rotation_ars_to_world @ rotation_imu_to_ars @ rotation_base_to_imu
    )
    return rotation_base_to_world


def compute_base_pitch_from_imu(
    quat_imu_in_ars: Tuple[float, float, float, float],
    rotation_base_to_imu: Optional[np.ndarray] = None,
) -> float:
    """Get pitch angle of the base frame relative to the world frame.

    Args:
        quat_imu_in_ars: Quaternion representing the rotation matrix from
            the IMU frame to the  attitude reference system (ARS) frame, in
            ``[w, x, y, z]`` format.
        rotation_base_to_imu: Rotation matrix from the base frame to the IMU
            frame. When not specified, the default Upkie mounting orientation
            is used.

    Returns:
        Angle from the world z-axis (unit vector opposite to gravity) to the
        base z-axis. This angle is positive when the base leans forward.
    """
    rotation_base_to_world = compute_base_orientation_from_imu(
        quat_imu_in_ars, rotation_base_to_imu
    )
    pitch_base_in_world = compute_pitch_frame_in_parent(rotation_base_to_world)
    return pitch_base_in_world


def compute_base_angular_velocity_from_imu(
    angular_velocity_imu_in_imu: np.ndarray,
) -> np.ndarray:
    """
    Kron

        R_{WI}
        Rdot_{WI} = R_{WI} (I_omega_{WI} x)
        R_{WB} = R_WI R_IB
        Rdot_{WB} = Rdot_WI R_IB
                  = R_WI (I_omega_WI x) R_IB
                  = R_WI R_IB (B_omega_WIx)
                  = R_WB (B_omega_WIx) = R_WB (B_omega_WBx)
        thus:
           B_omega_WB = R_BI I_omega_WI
    """
    # TODO(scaron): move to config
    rotation_base_to_imu = np.diag([-1.0, 1.0, -1.0])
    rotation_imu_to_base = rotation_base_to_imu.T
    angular_velocity_base_in_base = (
        rotation_imu_to_base @ angular_velocity_imu_in_imu
    )
    return angular_velocity_base_in_base
