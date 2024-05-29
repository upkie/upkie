#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

"""!
Domain randomization of robot states.
"""

from dataclasses import dataclass
from typing import Optional

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation as ScipyRotation


@dataclass
class RobotStateRandomization:
    """!
    Domain randomization parameters for Upkie's state.
    """

    ## @var roll
    ## Amount of roll angle randomization, in radians.
    roll: float = 0.0

    ## @var pitch
    ## Amount of pitch angle randomization, in radians.
    pitch: float = 0.0

    ## @var x
    ## Amount of x-axis position randomization, in meters.
    x: float = 0.0

    ## @var z
    ## Amount of z-axis position randomization, in meters.
    z: float = 0.0

    ## @var omega_x
    ## Amount of x-axis randomization on the angular-velocity vector, in rad/s.
    omega_x: float = 0.0

    ## @var omega_y
    ## Amount of y-axis randomization on the angular-velocity vector, in rad/s.
    omega_y: float = 0.0

    ## @var v_x
    ## Amount of x-axis randomization on the linear-velocity vector, in m/s.
    v_x: float = 0.0

    ## @var v_z
    ## Amount of z-axis randomization on the linear-velocity vector, in m/s.
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
        @param omega_y Angular velocity from the randomized frame to the parent
            frame, expressed in the randomized frame, along the y-axis.
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
