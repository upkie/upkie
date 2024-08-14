#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria


class JointLimit:
    """!
    Joint limit in configuration and tangent spaces.

    Attributes of this class match the URDF specification.
    """

    ## @var lower
    ## Lower configuration limit of the joint, in radians.
    lower: float

    ## @var upper
    ## Upper configuration limit of the joint, in radians.
    upper: float

    ## @var velocity
    ## Velocity limit of the joint, in [rad] / [s].
    velocity: float

    ## @var effort
    ## Torque limit of the joint, in [N m].
    effort: float

    def __init__(
        self,
        lower: float,
        upper: float,
        velocity: float,
        effort: float,
    ):
        """!
        Manual constructor for joint limits.

        (We don't use a ``dataclass`` because Doxygen does not detect
        attribute-only declarations in Python as of version 1.9.1.)
        """
        self.lower = lower
        self.upper = upper
        self.velocity = velocity
        self.effort = effort
