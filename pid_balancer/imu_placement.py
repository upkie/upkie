#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

from typing import List, Optional

import gin


@gin.configurable
class IMUPlacement:
    """!
    Optional IMU placement configuration.
    """

    ## @var rotation_base_to_imu
    ## Rotation matrix from base frame to IMU frame.
    rotation_base_to_imu: Optional[List[float]] = None

    def __init__(self, rotation_base_to_imu: Optional[List[float]] = None):
        r"""!
        Initialize IMU placement.

        \param rotation_base_to_imu Rotation matrix from base frame to IMU
            frame.
        """
        self.rotation_base_to_imu = rotation_base_to_imu
