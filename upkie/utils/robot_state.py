#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

## \namespace upkie.utils.robot_state
## \brief Robot state with optional randomization.

"""!
Robot state with optional randomization.
"""

from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation as ScipyRotation

from upkie.utils.robot_state_randomization import RobotStateRandomization


class RobotState:
    """!
    Robot state (configuration and velocity) with optional randomization.
    """

    ## \var angular_velocity_base_in_base
    ## Angular velocity of the base in body coordinates, in rad/s.
    angular_velocity_base_in_base: np.ndarray

    ## \var joint_configuration
    ## Vector of joint angles in radians.
    joint_configuration: np.ndarray

    ## \var joint_velocity
    ## Vector of joint velocities, in rad/s.
    joint_velocity: np.ndarray

    ## \var linear_velocity_base_to_world_in_world
    ## Linear velocity of the base in world coordinates, in m/s.
    linear_velocity_base_to_world_in_world: np.ndarray

    ## \var orientation_base_in_world
    ## Orientation of the base frame with respect to the world frame.
    orientation_base_in_world: ScipyRotation

    ## \var position_base_in_world
    ## Position of the base frame in the world frame.
    position_base_in_world: np.ndarray

    ## \var randomization
    ## Displacements and velocities added to the state when calling one of the
    ## sampling functions.
    randomization: RobotStateRandomization

    def __init__(
        self,
        angular_velocity_base_in_base: Optional[np.ndarray] = None,
        joint_configuration: Optional[np.ndarray] = None,
        joint_velocity: Optional[np.ndarray] = None,
        linear_velocity_base_to_world_in_world: Optional[np.ndarray] = None,
        orientation_base_in_world: Optional[ScipyRotation] = None,
        position_base_in_world: Optional[np.ndarray] = None,
        randomization: Optional[RobotStateRandomization] = None,
    ):
        r"""!
        Initialize a new state.

        \param[in] angular_velocity_base_in_base Body angular velocity of the
            floating-base link.
        \param[in] joint_configuration Joint configuration vector (not
            including the floating-base joint).
        \param[in] joint_velocity Joint velocity vector (not including the
            floating-base joint).
        \param[in] linear_velocity_base_to_world_in_world Linear velocity of
            the floating-base link in the world frame.
        \param[in] orientation_base_in_world Rotation from the floating-base
            frame to the world frame.
        \param[in] position_base_in_world Position of the floating-base frame
            in the world frame.
        \param[in] randomization Optional state randomization distribution.
        """
        self.angular_velocity_base_in_base = (
            angular_velocity_base_in_base
            if angular_velocity_base_in_base is not None
            else np.zeros(3)
        )
        self.joint_configuration = (
            joint_configuration
            if joint_configuration is not None
            else np.zeros(6)
        )
        self.joint_velocity = (
            joint_velocity if joint_velocity is not None else np.zeros(6)
        )
        self.linear_velocity_base_to_world_in_world = (
            linear_velocity_base_to_world_in_world
            if linear_velocity_base_to_world_in_world is not None
            else np.zeros(3)
        )
        self.orientation_base_in_world = (
            orientation_base_in_world
            if orientation_base_in_world is not None
            else ScipyRotation.identity()
        )
        self.position_base_in_world = (
            position_base_in_world
            if position_base_in_world is not None
            else np.array([0.0, 0.0, 0.6])  # Upkie above horizontal plane
        )
        self.randomization = (
            randomization
            if randomization is not None
            else RobotStateRandomization()
        )

    def sample_angular_velocity(
        self, np_random: np.random.Generator
    ) -> np.ndarray:
        r"""!
        Sample an angular velocity around the one in this state.

        \param[in] np_random NumPy random number generator.
        \return Sampled angular velocity.
        """
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

    def sample_linear_velocity(
        self, np_random: np.random.Generator
    ) -> np.ndarray:
        r"""!
        Sample a linear velocity around the one in this state.

        \param[in] np_random NumPy random number generator.
        \return Sampled linear velocity.
        """
        linear_velocity_rand_to_world_in_world = (
            self.randomization.sample_linear_velocity(np_random)
        )
        return (
            self.linear_velocity_base_to_world_in_world
            + linear_velocity_rand_to_world_in_world
        )

    def sample_orientation(
        self, np_random: np.random.Generator
    ) -> ScipyRotation:
        r"""!
        Sample an orientation around the one in this state.

        \param[in] np_random NumPy random number generator.
        \return Sampled orientation.
        """
        rotation_rand_to_base = self.randomization.sample_orientation(
            np_random
        )
        rotation_base_to_world = self.orientation_base_in_world
        return rotation_base_to_world * rotation_rand_to_base

    def sample_position(self, np_random: np.random.Generator) -> np.ndarray:
        r"""!
        Sample a position around the one in this state.

        \param[in] np_random NumPy random number generator.
        \return Randomized position.
        """
        position_rand_in_world = self.randomization.sample_position(np_random)
        return self.position_base_in_world + position_rand_in_world

    def sample_state(self, np_random: np.random.Generator) -> "RobotState":
        r"""!
        Sample a randomized robot state around this state.

        \param[in] np_random NumPy random number generator.
        \return Sampled robot state with randomization applied.
        """
        sampled_angular_velocity = self.sample_angular_velocity(np_random)
        sampled_joint_configuration = self.joint_configuration
        sampled_joint_velocity = self.joint_velocity
        sampled_linear_velocity = self.sample_linear_velocity(np_random)
        sampled_orientation = self.sample_orientation(np_random)
        sampled_position = self.sample_position(np_random)
        return RobotState(
            angular_velocity_base_in_base=sampled_angular_velocity,
            joint_configuration=sampled_joint_configuration,
            joint_velocity=sampled_joint_velocity,
            linear_velocity_base_to_world_in_world=sampled_linear_velocity,
            orientation_base_in_world=sampled_orientation,
            position_base_in_world=sampled_position,
            randomization=self.randomization,
        )
