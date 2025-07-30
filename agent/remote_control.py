#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron
# Copyright 2023 Inria

import gin


@gin.configurable
class RemoteControl:
    """Remote control parameters.

    Attributes:
        max_linear_accel: Maximum acceleration for the ground position target,
            in [m] / [s]². Does not affect the commanded ground velocity.
        max_linear_velocity: Maximum velocity for the ground position target,
            in [m] / [s]. Indirectly affects the commanded ground velocity.
        max_yaw_accel: Maximum yaw angular acceleration in [rad] / [s]².
        max_yaw_velocity: Maximum yaw angular velocity in [rad] / [s].
    """

    max_linear_accel: float
    max_linear_velocity: float
    max_yaw_accel: float
    max_yaw_velocity: float

    def __init__(
        self,
        max_linear_velocity: float,
        max_linear_accel: float,
        max_yaw_velocity: float,
        max_yaw_accel: float,
    ) -> None:
        """Initialize remote-control parameters.

        Args:
            max_linear_accel: Maximum acceleration for the ground position
                target, in [m] / [s]². Does not affect the commanded ground
                velocity.
            max_linear_velocity: Maximum velocity for the ground position
                target, in [m] / [s]. Indirectly affects the commanded ground
                velocity.
            max_yaw_accel: Maximum yaw angular acceleration in [rad] / [s]².
            max_yaw_velocity: Maximum yaw angular velocity in [rad] / [s].
        """
        self.max_linear_accel = max_linear_accel
        self.max_linear_velocity = max_linear_velocity
        self.max_yaw_accel = max_yaw_accel
        self.max_yaw_velocity = max_yaw_velocity
