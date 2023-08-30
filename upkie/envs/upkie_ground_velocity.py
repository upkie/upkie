#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron
# Copyright 2023 Inria
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math

import numpy as np
from gymnasium import spaces

from upkie.utils.filters import abs_bounded_derivative_filter

from .upkie_wheeled_pendulum import UpkieWheeledPendulum


class UpkieGroundVelocity(UpkieWheeledPendulum):

    """!
    Environment where Upkie balances by ground velocity control.

    The environment id is ``UpkieGroundVelocity-v1``.

    ### Action space

    Vectorized actions have the following structure:

    <table>
        <tr>
            <td><strong>Index</strong></td>
            <td><strong>Description</strong></td>
            </tr>
        <tr>
            <td>``0``</td>
            <td>Ground velocity in [m] / [s].</td>
        </tr>
    </table>

    ### Observation space

    Vectorized observations have the following structure:

    <table>
        <tr>
            <td><strong>Index</strong></td>
            <td><strong>Description</strong></td>
        </tr>
        <tr>
            <td>0</td>
            <td>Pitch angle of the base with respect to the world vertical, in
            radians. This angle is positive when the robot leans forward.</td>
        </tr>
        <tr>
            <td>1</td>
            <td>Position of the average wheel contact point, in meters.</td>
        </tr>
        <tr>
            <td>2</td>
            <td>Body angular velocity of the base frame along its lateral axis,
            in radians per seconds.</td>
        </tr>
        <tr>
            <td>3</td>
            <td>Velocity of the average wheel contact point, in meters per
            seconds.</td>
        </tr>
    </table>

    ### Attributes

    The environment class defines the following attributes:

    - ``max_ground_accel``: Maximum commanded ground acceleration in m/s².
    - ``max_ground_velocity``: Maximum commanded ground velocity in m/s.
    - ``version``: Environment version number.
    - ``wheel_radius``: Wheel radius in [m].

    """

    _ground_velocity: float
    max_ground_accel: float
    max_ground_velocity: float
    version: int = 1
    wheel_radius: float

    LEG_JOINTS = [
        f"{side}_{joint}"
        for side in ("left", "right")
        for joint in ("hip", "knee")
    ]

    def __init__(
        self,
        max_ground_accel: float = 10.0,
        max_ground_velocity: float = 1.0,
        wheel_radius: float = 0.06,
        **kwargs,
    ):
        """!
        Initialize environment.

        @param max_ground_accel Maximum commanded ground acceleration in m/s^2.
        @param max_ground_velocity Maximum commanded ground velocity in m/s.
        @param wheel_radius Wheel radius in [m].

        Other keyword arguments are forwarded as-is to parent class
        constructors. Follow the chain up from @ref
        envs.upkie_wheeled_pendulum.UpkieWheeledPendulum "UpkieWheeledPendulum"
        for their documentation.
        """
        super().__init__(**kwargs)

        # gymnasium.Env: action_space
        self.action_space = spaces.Box(
            -np.float32(max_ground_velocity),
            +np.float32(max_ground_velocity),
            shape=(1,),
            dtype=np.float32,
        )

        self._ground_velocity = 0.0
        self.max_ground_accel = max_ground_accel
        self.max_ground_velocity = max_ground_velocity
        self.wheel_radius = wheel_radius

    def dictionarize_action(self, action: np.ndarray) -> dict:
        """!
        Convert action vector into a spine action dictionary.

        @param action Action vector.
        @returns Action dictionary.
        """
        self._ground_velocity = abs_bounded_derivative_filter(
            self._ground_velocity,
            action[0],
            self.dt,
            self.max_ground_velocity,
            self.max_ground_accel,
        )
        wheel_velocity = self._ground_velocity / self.wheel_radius
        action_dict = {
            "servo": {
                joint: {
                    "position": self.init_position[joint],
                    "velocity": 0.0,
                }
                for joint in self.LEG_JOINTS
            }
        }
        action_dict["servo"].update(
            {
                "left_wheel": {
                    "position": math.nan,
                    "velocity": +wheel_velocity,
                },
                "right_wheel": {
                    "position": math.nan,
                    "velocity": -wheel_velocity,
                },
            }
        )
        return action_dict
