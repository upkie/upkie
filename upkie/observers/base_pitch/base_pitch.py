#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

from typing import Optional

import numpy as np
from numpy.typing import NDArray


def compute_base_angular_velocity_from_imu(
    angular_velocity_imu_in_imu: NDArray[float],
    rotation_base_to_imu: Optional[NDArray] = None,
) -> NDArray[float]:
    r"""!
    Compute the body angular velocity of the base from IMU readings.

    @param angular_velocity_imu_in_imu Angular velocity from the IMU.
    @param rotation_base_to_imu Rotation matrix from base to IMU.
    @returns Body angular velocity of the base frame.

    Calculation checks:

    - \f$I\f$: IMU frame
    - \f$B\f$: base frame
    - \f$W\f$: world (inertial) frame

    Our input is \f${}_I \omega_{WI}\f$, we seek \f${}_B \omega_{WB}\f$.

    \f$
    \begin{align*}
        \dot{R}_{WI} & = R_{WI} ({}_I \omega_{WI} \times) \\
        R_{WB} & = R_{WI} R_{IB} \\
        \dot{R}_{WB} & = \dot{R}_{WI} R_{IB} \\
        & = R_{WI} ({}_I \omega_{WI} \times) R_{IB} \\
        & = R_{WI} R_{IB} ({}_B \omega_{WI} \times) \\
        & = R_{WB} ({}_B \omega_{WI} \times) = R_{WB} ({}_B \omega_{WB} \times)
    \end{align*}
    \f$

    Thus \f${}_B \omega_{WB} = R_{BI} {}_I \omega_{WI}\f$.
    """
    if rotation_base_to_imu is None:
        rotation_base_to_imu = np.diag([-1.0, 1.0, -1.0])
    rotation_imu_to_base = rotation_base_to_imu.T
    angular_velocity_base_in_base = (
        rotation_imu_to_base @ angular_velocity_imu_in_imu
    )
    return angular_velocity_base_in_base
