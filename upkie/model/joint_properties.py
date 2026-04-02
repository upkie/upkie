#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0


class JointProperties:
    r"""!
    Per-joint simulation properties for the PyBullet backend.
    """

    ## \var friction
    ## Kinetic friction torque in N·m applied opposite to the direction of
    ## motion when the joint velocity exceeds the stiction threshold.
    friction: float

    ## \var torque_control_noise
    ## Standard deviation of Gaussian white noise in N·m added to the torque
    ## actually applied to the joint during simulation.
    torque_control_noise: float

    ## \var torque_measurement_noise
    ## Standard deviation of Gaussian white noise in N·m added to the torque
    ## reported in observations.
    torque_measurement_noise: float

    def __init__(
        self,
        friction: float = 0.0,
        torque_control_noise: float = 0.0,
        torque_measurement_noise: float = 0.0,
    ):
        r"""!
        Initialize joint properties.

        \param friction Kinetic friction torque in N·m.
        \param torque_control_noise Control noise standard deviation in N·m.
        \param torque_measurement_noise Measurement noise standard deviation
            in N·m.
        """
        self.friction = friction
        self.torque_control_noise = torque_control_noise
        self.torque_measurement_noise = torque_measurement_noise
