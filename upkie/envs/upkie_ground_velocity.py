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
from typing import Dict, Optional, Tuple

import numpy as np
from gymnasium import spaces

from upkie.utils.exceptions import UpkieException
from upkie.utils.filters import abs_bounded_derivative_filter, low_pass_filter

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

    Note that, while this action is not normalized, [-1, 1] m/s is a reasonable
    range for ground velocities.

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

    _filtered_action: float
    _ground_velocity: float
    max_ground_accel: float
    max_ground_velocity: float
    version: int = 1
    velocity_filter: Optional[float]
    wheel_radius: float

    def __init__(
        self,
        max_ground_accel: float = 10.0,
        max_ground_velocity: float = 1.0,
        velocity_filter: Optional[float] = None,
        velocity_filter_rand: Optional[Tuple[float, float]] = None,
        wheel_radius: float = 0.06,
        **kwargs,
    ):
        """!
        Initialize environment.

        @param max_ground_accel Maximum commanded ground acceleration in m/s^2.
        @param max_ground_velocity Maximum commanded ground velocity in m/s.
        @param velocity_filter If set, cutoff period in seconds of a low-pass
            filter applied to commanded velocities.
        @param velocity_filter_rand If set, couple of lower and upper bounds
            for the @ref velocity_filter parameter. At new period is sampled
            uniformly at random between these bounds at every reset of the
            environment.
        @param wheel_radius Wheel radius in [m].
        @param kwargs Other keyword arguments are forwarded as-is to parent
            class constructors. Follow the chain up from @ref
            envs.upkie_wheeled_pendulum.UpkieWheeledPendulum
            "UpkieWheeledPendulum" for their documentation.
        """
        super().__init__(**kwargs)

        if self.dt is None:
            raise UpkieException("This environment needs a loop frequency")

        # gymnasium.Env: action_space
        self.action_space = spaces.Box(
            -np.float32(max_ground_velocity),
            +np.float32(max_ground_velocity),
            shape=(1,),
            dtype=np.float32,
        )

        self._filtered_action = 0.0
        self._ground_velocity = 0.0
        self.max_ground_accel = max_ground_accel
        self.max_ground_velocity = max_ground_velocity
        self.velocity_filter = velocity_filter
        self.velocity_filter_rand = velocity_filter_rand
        self.wheel_radius = wheel_radius

    def reset(
        self,
        *,
        seed: Optional[int] = None,
        options: Optional[dict] = None,
    ) -> Tuple[np.ndarray, Dict]:
        """!
        Resets the environment and get an initial observation.

        @param seed Number used to initialize the environment’s internal random
            number generator.
        @param options Currently unused.
        @returns
            - ``observation``: Initial vectorized observation, i.e. an element
              of the environment's ``observation_space``.
            - ``info``: Dictionary with auxiliary diagnostic information. For
              Upkie this is the full observation dictionary sent by the spine.
        """
        observation, info = super().reset(seed=seed)

        if self.velocity_filter_rand is not None:
            low, high = self.velocity_filter_rand
            self.velocity_filter = self.np_random.uniform(low=low, high=high)

        return observation, info

    def dictionarize_action(self, action: np.ndarray) -> dict:
        """!
        Convert action vector into a spine action dictionary.

        @param action Action vector.
        @returns Action dictionary.
        """
        if self.velocity_filter is not None:
            self._filtered_action = low_pass_filter(
                self._filtered_action,
                self.velocity_filter,
                action[0],
                self.dt,
            )
        else:  # self.velocity_filter is None
            self._filtered_action = action[0]

        self._ground_velocity = abs_bounded_derivative_filter(
            self._ground_velocity,
            self._filtered_action,
            self.dt,
            self.max_ground_velocity,
            self.max_ground_accel,
        )

        wheel_velocity = self._ground_velocity / self.wheel_radius
        servo_dict = self.get_leg_servo_action()
        servo_dict.update(
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
        action_dict = {"servo": servo_dict}
        return action_dict
