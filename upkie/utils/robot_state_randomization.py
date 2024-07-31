#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

"""!
Domain randomization of robot states.
"""

from typing import Optional

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation as ScipyRotation


class RobotStateRandomization:
    r"""!
    Domain randomization parameters for Upkie's state.

    \note The API for this class is likely to change. Initially all its
    attributes where floating-point numbers due to a quick-and-dirty design
    decision.
    """

    ## @var roll
    ## Amount of roll angle randomization, in radians.
    roll: float

    ## @var pitch
    ## Amount of pitch angle randomization, in radians.
    pitch: float

    ## @var x
    ## Amount of x-axis position randomization, in meters.
    x: float

    ## @var z
    ## Amount of z-axis position randomization, in meters.
    z: float

    ## @var omega_x
    ## Amount of x-axis randomization on the angular-velocity vector, in rad/s.
    omega_x: float

    ## @var omega_y
    ## Amount of y-axis randomization on the angular-velocity vector, in rad/s.
    omega_y: float

    ## @var linear_velocity
    ## Magnitude of linear velocity randomization, as a 3D vector in m/s.
    linear_velocity: NDArray[float]

    def __init__(
        self,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        omega_x: float = 0.0,
        omega_y: float = 0.0,
        omega_z: float = 0.0,
        linear_velocity: Optional[NDArray[float]] = None,
    ):
        r"""!
        Initialize sampler.
        \param yaw Amount of yaw angle randomization, in radians.
        \param roll Amount of roll angle randomization, in radians.
        \param pitch Amount of pitch angle randomization, in radians.
        \param x Amount of x-axis position randomization, in meters.
        \param y Amount of y-axis position randomization, in meters.
        \param z Amount of z-axis position randomization, in meters.
        \param omega_x Amount of x-axis randomization on the angular-velocity
            vector, in rad/s.
        \param omega_y Amount of y-axis randomization on the angular-velocity
            vector, in rad/s.
        \param omega_z Amount of z-axis randomization on the angular-velocity
        \param linear_velocity Magnitude of linear velocity randomization, as a
            3D vector in m/s.
        """
        self.yaw = yaw
        self.roll = roll
        self.pitch = pitch
        self.x = x
        self.y = y
        self.z = z
        self.omega_x = omega_x
        self.omega_y = omega_y
        self.omega_z = omega_z
        self.linear_velocity = (
            linear_velocity if linear_velocity is not None else np.zeros(3)
        )

    def update(
        self,
        yaw: Optional[float] = None,
        roll: Optional[float] = None,
        pitch: Optional[float] = None,
        x: Optional[float] = None,
        y: Optional[float] = None,
        z: Optional[float] = None,
        omega_x: Optional[float] = None,
        omega_y: Optional[float] = None,
        omega_z: Optional[float] = None,
        v_x: Optional[float] = None,
        v_y: Optional[float] = None,
        v_z: Optional[float] = None,
    ) -> None:
        r"""!
        Update some fields.
        \param yaw Yaw angle for the rotation from the randomized frame to the
            parent frame (Euler Z-Y-X convention).
        \param roll Roll angle for the rotation from the randomized frame to
            the parent frame (Euler Z-Y-X convention).
        \param pitch Pitch angle for the rotation from the randomized frame to
            the parent frame (Euler Z-Y-X convention).
        \param x Translation of the randomized frame along the x-axis of the
            parent frame.
        \param y Translation of the randomized frame along the y-axis of the
            parent frame.
        \param z Translation of the randomized frame along the z-axis of the
            parent frame.
        \param omega_x Angular velocity from the randomized frame to the parent
            frame, expressed in the randomized frame, along the x-axis.
        \param omega_y Angular velocity from the randomized frame to the parent
            frame, expressed in the randomized frame, along the y-axis.
        \param omega_z Angular velocity from the randomized frame to the parent
            frame, expressed in the randomized frame, along the z-axis.
        \param v_x Linear velocity from the randomized frame to the parent
            frame, expressed in the parent frame and along the x-axis.
        \param v_z Linear velocity from the randomized frame to the parent
            frame, expressed in the parent frame and along the z-axis.
        """
        if yaw is not None:
            self.yaw = yaw
        if roll is not None:
            self.roll = roll
        if pitch is not None:
            self.pitch = pitch
        if x is not None:
            self.x = x
        if y is not None:
            self.y = y
        if z is not None:
            self.z = z
        if omega_x is not None:
            self.omega_x = omega_x
        if omega_y is not None:
            self.omega_y = omega_y
        if omega_z is not None:
            self.omega_z = omega_z
        if v_x is not None:
            self.linear_velocity[0] = v_x
        if v_y is not None:
            self.linear_velocity[0] = v_y
        if v_z is not None:
            self.linear_velocity[2] = v_z

    def sample_orientation(
        self, np_random: np.random.Generator
    ) -> ScipyRotation:
        r"""!
        Sample an orientation within the given bounds.

        \param[in] np_random NumPy random number generator.
        \return Sampled rotation matrix.
        """
        yaw_pitch_roll_bounds = np.array([self.yaw, self.pitch, self.roll])
        yaw_pitch_roll = np_random.uniform(
            low=-yaw_pitch_roll_bounds,
            high=+yaw_pitch_roll_bounds,
            size=3,
        )
        return ScipyRotation.from_euler("ZYX", yaw_pitch_roll)

    def sample_position(
        self, np_random: np.random.Generator
    ) -> NDArray[float]:
        r"""!
        Sample a position within the given bounds.

        \param[in] np_random NumPy random number generator.
        \return Sampled position vector.
        """
        return np_random.uniform(
            low=np.array([-self.x, -self.y, 0.0]),
            high=np.array([+self.x, self.y, self.z]),
            size=3,
        )

    def sample_angular_velocity(
        self, np_random: np.random.Generator
    ) -> NDArray[float]:
        r"""!
        Sample an angular velocity within the given bounds.

        \param[in] np_random NumPy random number generator.
        \return Sampled angular-velocity vector.
        """
        return np_random.uniform(
            low=np.array([-self.omega_x, -self.omega_y, -self.omega_z]),
            high=np.array([+self.omega_x, +self.omega_y, -self.omega_z]),
            size=3,
        )

    def sample_linear_velocity(
        self, np_random: np.random.Generator
    ) -> NDArray[float]:
        r"""!
        Sample a linear velocity within the given bounds.

        \param[in] np_random NumPy random number generator.
        \return Sampled linear-velocity vector.
        """
        return np_random.uniform(
            low=-self.linear_velocity,
            high=self.linear_velocity,
            size=3,
        )
