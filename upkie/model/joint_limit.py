#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

from dataclasses import dataclass


@dataclass
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
    ## Velocity limit of the joint, in rad/s.
    velocity: float

    ## @var effort
    ## Torque limit of the joint, in N.m.
    effort: float
