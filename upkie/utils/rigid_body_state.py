#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria
# SPDX-License-Identifier: Apache-2.0

"""!
Domain randomization of Upkie environments.
"""

from dataclasses import dataclass
from typing import Optional

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation as ScipyRotation


@dataclass
class BaseStateRandomization:

    """!
    Domain randomization parameters for Upkie's floating-base state.
    """

    roll: float = 0.0
    pitch: float = 0.0
    x: float = 0.0
    z: float = 0.0
    omega_x: float = 0.0
    omega_y: float = 0.0
    v_x: float = 0.0
    v_z: float = 0.0

    def update(
        self,
        roll: Optional[float] = None,
        pitch: Optional[float] = None,
        x: Optional[float] = None,
        z: Optional[float] = None,
        omega_x: Optional[float] = None,
        omega_y: Optional[float] = None,
        v_x: Optional[float] = None,
        v_z: Optional[float] = None,
    ) -> None:
        """!
        Update some fields.

        @param roll Roll angle for the rotation from the randomized frame to
            the parent frame (Euler Z-Y-X convention).
        @param pitch Pitch angle for the rotation from the randomized frame to
            the parent frame (Euler Z-Y-X convention).
        @param x Translation of the randomized frame along the x-axis of the
            parent frame.
        @param z Translation of the randomized frame along the z-axis of the
            parent frame.
        @param omega_x Angular velocity from the randomized frame to the parent
            frame, expressed in the randomized frame, along the x-axis.
        @param omega_z Angular velocity from the randomized frame to the parent
            frame, expressed in the randomized frame, along the z-axis.
        @param v_x Linear velocity from the randomized frame to the parent
            frame, expressed in the parent frame and along the x-axis.
        @param v_z Linear velocity from the randomized frame to the parent
            frame, expressed in the parent frame and along the z-axis.
        """
        if roll is not None:
            self.roll = roll
        if pitch is not None:
            self.pitch = pitch
        if x is not None:
            self.x = x
        if z is not None:
            self.z = z
        if v_x is not None:
            self.v_x = v_x
        if v_z is not None:
            self.v_z = v_z
        if omega_x is not None:
            self.omega_x = omega_x
        if omega_y is not None:
            self.omega_y = omega_y

    def sample_orientation(self, np_random) -> ScipyRotation:
        yaw_pitch_roll_bounds = np.array([0.0, self.pitch, self.roll])
        yaw_pitch_roll = np_random.uniform(
            low=-yaw_pitch_roll_bounds,
            high=+yaw_pitch_roll_bounds,
            size=3,
        )
        return ScipyRotation.from_euler("ZYX", yaw_pitch_roll)

    def sample_position(self, np_random) -> NDArray[float]:
        return np_random.uniform(
            low=np.array([-self.x, 0.0, 0.0]),
            high=np.array([+self.x, 0.0, self.z]),
            size=3,
        )

    def sample_angular_velocity(self, np_random) -> NDArray[float]:
        return np_random.uniform(
            low=np.array([-self.omega_x, -self.omega_y, 0.0]),
            high=np.array([+self.omega_x, +self.omega_y, 0.0]),
            size=3,
        )

    def sample_linear_velocity(self, np_random) -> NDArray[float]:
        return np_random.uniform(
            low=np.array([-self.v_x, 0.0, -self.v_z]),
            high=np.array([+self.v_x, 0.0, +self.v_z]),
            size=3,
        )


class BaseState:

    """!
    Floating-base state for Upkie with optional randomization.
    """

    orientation_base_in_world: ScipyRotation
    position_base_in_world: NDArray[float]
    angular_velocity_base_in_base: NDArray[float]
    linear_velocity_base_to_world_in_world: NDArray[float]
    randomization: BaseStateRandomization

    def __init__(
        self,
        orientation_base_in_world: Optional[ScipyRotation] = None,
        position_base_in_world: Optional[NDArray[float]] = None,
        linear_velocity_base_to_world_in_world: Optional[
            NDArray[float]
        ] = None,
        angular_velocity_base_in_base: Optional[NDArray[float]] = None,
        randomization: Optional[BaseStateRandomization] = None,
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
            else BaseStateRandomization()
        )

    def sample_orientation(self, np_random) -> ScipyRotation:
        rotation_rand_to_base = self.randomization.sample_orientation()
        rotation_base_to_world = self.orientation
        return rotation_base_to_world * rotation_rand_to_base

    def sample_position(self, np_random) -> NDArray[float]:
        position_rand_in_world = self.randomization.sample_position()
        return self.position_base_in_world + position_rand_in_world

    def sample_linear_velocity(self, np_random) -> NDArray[float]:
        linear_velocity_rand_to_world_in_world = (
            self.randomization.sample_linear_velocity()
        )
        return (
            self.linear_velocity_base_to_world_in_world
            + linear_velocity_rand_to_world_in_world
        )

    def sample_angular_velocity(self, np_random) -> NDArray[float]:
        angular_velocity_rand_to_base_in_base = (
            self.randomization.sample_angular_velocity()
        )
        angular_velocity_base_to_world_in_base = (
            self.angular_velocity_base_in_base
        )
        return (
            angular_velocity_base_to_world_in_base
            + angular_velocity_rand_to_base_in_base
        )
