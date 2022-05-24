#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron

from typing import Tuple

import eigenpy
import numpy as np
from utils.clamp import clamp


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


def compute_base_pitch_from_imu(
    quat_imu_in_world: Tuple[float, float, float, float]
) -> float:
    """
    Get pitch angle of the *base* relative to the world vertical.

    Args:
        quat_imu_in_world: Quaternion representing the rotation matrix from
            the *IMU* frame to the world frame.

    Returns:
        Angle from the world z-axis (gravity) to the base z-axis.
        Equivalently, angle that the sagittal vector of the base frame makes
        with the heading vector.

    Note:
        The output pitch angle is for the base (x-axis pointing forward), but
        the input orientation is that of the IMU. Keep in mind that the IMU
        frame is turned 180 degrees around the yaw axis of the base frame.
    """
    quat = eigenpy.Quaternion(*quat_imu_in_world)
    orientation_imu_in_world = quat.toRotationMatrix()
    pitch_imu_in_world = compute_pitch_frame_in_parent(
        orientation_imu_in_world
    )
    return -pitch_imu_in_world  # 180 degrees yaw rotation
