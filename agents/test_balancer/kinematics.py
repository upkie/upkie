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

from utils.clamp import clamp_abs


def forward_kinematics(
    q_hip: float,
    q_knee: float,
    limb_length: float = 0.17,
) -> float:
    """
    Compute forward kinematics for a single leg.

    Args:
        q_hip: Angle of the hip joint, in radians.
        q_knee: Angle of the knee joint, in radians.
        limb_length: (Model) Length of both links of the leg.

    Returns:
        Crouch height (positive, zero for extended legs) in meters.

    The derivation of this function is documented in `Kinematics of a
    symmetric leg`_.

    .. _`Kinematics of a symmetric leg`:
        https://scaron.info/blog/kinematics-of-a-symmetric-leg.html
    """
    height = limb_length * (np.cos(q_hip) + np.cos(q_hip + q_knee))
    leg_length = 2.0 * limb_length
    crouch_height = leg_length - height
    return crouch_height


def inverse_kinematics(
    crouch_height: float,
    limb_length: float = 0.17,
) -> Tuple[float, float]:
    """
    Solve inverse kinematics for a single leg.

    Args:
        crouch_height: Target crouch height (positive, zero for extended legs)
            in meters.
        limb_length: (Model) Length of both links of the leg.

    Returns:
        ===========  =========================================================
        ``q_hip``     Angle of the hip joint in radians.
        ``q_knee``    Angle of the knee joint in radians.
        ===========  =========================================================

    The derivation of this function is documented in `Kinematics of a symmetric
    leg`_. We replaced the total height :math:`h` by the crouch height :math:`c
    = 2 \\ell - h`.

    .. _`Kinematics of a symmetric leg`:
        https://scaron.info/blog/kinematics-of-a-symmetric-leg.html
    """
    leg_length = 2.0 * limb_length
    assert 0.0 <= crouch_height < leg_length, (
        f"Invalid crouch height: {crouch_height} [m]; "
        "leg is under- or over-extended"
    )
    q_hip = clamp_abs(np.arccos(1.0 - crouch_height / leg_length), 0.5 * np.pi)
    q_knee = clamp_abs(-2.0 * q_hip, np.pi)
    return (q_hip, q_knee)


def velocity_limited_joint_control(
    target_position: float,
    current_position: float,
    dt: float,
    max_joint_velocity: float,
) -> float:
    """
    Joint velocity control with bounded output velocity.

    Args:
        target_position: Target joint angle in radians.
        current_position: Current joint angle in radians.
        dt: Duration in seconds until next cycle.
        max_joint_velocity: Maximum joint angular velocity in rad / s.

    Returns:
        Angular velocity for the joint.
    """
    return clamp_abs(
        (target_position - current_position) / dt,
        max_joint_velocity,
    )


def velocity_limited_inverse_kinematics(
    crouch_height: float,
    current_positions: Tuple[float, float],
    dt: float,
    max_joint_velocity: float,
) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    """
    Solve inverse kinematics for a single leg.

    Args:
        crouch_height: Target crouch height (positive, zero for extended legs)
            in meters.
        current_positions: Tuple of hip and joint angles, in radians.
        dt: Duration in seconds until next cycle.
        max_joint_velocity: Maximum joint angular velocity in rad / s.

    Returns:
        ===========  =========================================================
        ``q_hip``     Angle of the hip joint in radians.
        ``q_knee``    Angle of the knee joint in radians.
        ===========  =========================================================
    """
    target_hip, target_knee = inverse_kinematics(crouch_height)
    current_hip, current_knee = current_positions
    v_hip = velocity_limited_joint_control(
        target_hip, current_hip, dt, max_joint_velocity
    )
    v_knee = velocity_limited_joint_control(
        target_knee, current_knee, dt, max_joint_velocity
    )
    # On the manifold ``v_hip = -0.5 * v_knee``
    v_hip = clamp_abs(v_hip, 0.5 * abs(v_knee))
    q_hip = current_hip + v_hip * dt
    q_knee = current_knee + v_knee * dt
    return (q_hip, q_knee)
