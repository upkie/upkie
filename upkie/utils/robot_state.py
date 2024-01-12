#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

"""!
Robot state with optional randomization.
"""

from typing import Optional

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation as ScipyRotation

from upkie.utils.robot_state_randomization import RobotStateRandomization


class RobotState:

    """!
    Robot state (configuration and velocity) with optional randomization.
    """

    angular_velocity_base_in_base: NDArray[float]
    joint_configuration: NDArray[float]
    joint_velocity: NDArray[float]
    linear_velocity_base_to_world_in_world: NDArray[float]
    orientation_base_in_world: ScipyRotation
    position_base_in_world: NDArray[float]
    randomization: RobotStateRandomization

    def __init__(
        self,
        angular_velocity_base_in_base: Optional[NDArray[float]] = None,
        joint_configuration: Optional[NDArray[float]] = None,
        joint_velocity: Optional[NDArray[float]] = None,
        linear_velocity_base_to_world_in_world: Optional[ NDArray[float] ] = None,
        orientation_base_in_world: Optional[ScipyRotation] = None,
        position_base_in_world: Optional[NDArray[float]] = None,
        randomization: Optional[RobotStateRandomization] = None,
    ):
        self.orientation_base_in_world = (
            orientation_base_in_world
            if orientation_base_in_world is not None
            else ScipyRotation.identity()
        )
        self.position_base_in_world = (
            position_base_in_world
            if position_base_in_world is not None
            else np.zeros(3)
        )
        self.linear_velocity_base_to_world_in_world = (
            linear_velocity_base_to_world_in_world
            if linear_velocity_base_to_world_in_world is not None
            else np.zeros(3)
        )
        self.angular_velocity_base_in_base = (
            angular_velocity_base_in_base
            if angular_velocity_base_in_base is not None
            else np.zeros(3)
        )
        self.randomization = (
            randomization
            if randomization is not None
            else RobotStateRandomization()
        )

    def sample_orientation(self, np_random) -> ScipyRotation:
        rotation_rand_to_base = self.randomization.sample_orientation(
            np_random
        )
        rotation_base_to_world = self.orientation_base_in_world
        return rotation_base_to_world * rotation_rand_to_base

    def sample_position(self, np_random) -> NDArray[float]:
        position_rand_in_world = self.randomization.sample_position(np_random)
        return self.position_base_in_world + position_rand_in_world

    def sample_linear_velocity(self, np_random) -> NDArray[float]:
        linear_velocity_rand_to_world_in_world = (
            self.randomization.sample_linear_velocity(np_random)
        )
        return (
            self.linear_velocity_base_to_world_in_world
            + linear_velocity_rand_to_world_in_world
        )

    def sample_angular_velocity(self, np_random) -> NDArray[float]:
        angular_velocity_rand_to_base_in_base = (
            self.randomization.sample_angular_velocity(np_random)
        )
        angular_velocity_base_to_world_in_base = (
            self.angular_velocity_base_in_base
        )
        return (
            angular_velocity_base_to_world_in_base
            + angular_velocity_rand_to_base_in_base
        )
