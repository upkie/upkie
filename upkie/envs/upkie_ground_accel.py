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

from upkie.utils.clamp import clamp_and_warn

from .upkie_pendulum_env import UpkiePendulum


class UpkieGroundAccel(UpkiePendulum):

    """!
    Environment where Upkie balances by ground velocity control.

    ### Action space

    Vectorized actions have the following structure:

    <table>
        <tr>
            <td><strong>Index</strong></td>
            <td><strong>Description</strong></td>
            </tr>
        <tr>
            <td>``0``</td>
            <td>Ground acceleration in [m] / [s]^2.</td>
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

    - ``fall_pitch``: Fall pitch angle, in radians.
    - ``max_ground_accel``: Maximum commanded ground acceleration in m/s^2.
    - ``max_ground_velocity``: Maximum commanded ground velocity in m/s.
    - ``wheel_radius``: Wheel radius in [m].

    """

    fall_pitch: float
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

        @param reward Reward function.
        @param fall_pitch Fall pitch angle, in radians.
        @param frequency Regulated frequency of the control loop, in Hz.
        @param max_ground_accel Maximum commanded ground acceleration in m/s^2.
        @param max_ground_velocity Maximum commanded ground velocity in m/s.
        @param shm_name Name of shared-memory file.
        @param spine_config Additional spine configuration overriding the
            defaults from ``//config:spine.yaml``. The combined configuration
            dictionary is sent to the spine at every :func:`reset`.
        @param wheel_radius Wheel radius in [m].
        """
        super().__init__(
            **kwargs,
        )

        # gymnasium.Env: action_space
        self.action_space = spaces.Box(
            -np.float32(max_ground_accel),
            +np.float32(max_ground_accel),
            shape=(1,),
            dtype=np.float32,
        )

        if self.dt is None:
            raise ValueError(
                "this environment needs to know the control loop frequency"
            )

        self._commanded_velocity = 0.0
        self.max_ground_accel = max_ground_accel
        self.max_ground_velocity = max_ground_velocity
        self.wheel_radius = wheel_radius

    def dictionarize_action(self, action: np.ndarray) -> dict:
        """!
        Convert action vector into a spine action dictionary.

        @param action Action vector.
        @returns Action dictionary.
        """
        commanded_accel: float = clamp_and_warn(
            action[0],
            lower=-self.max_ground_accel,
            upper=+self.max_ground_accel,
            label="commanded_accel",
        )
        self._commanded_velocity = clamp_and_warn(
            self._commanded_velocity + commanded_accel * self.dt / 2.0,
            lower=-1.0,
            upper=+1.0,
            label="commanded_velocity",
        )
        wheel_velocity = self._commanded_velocity / self.wheel_radius
        action_dict = {
            "servo": (
                {
                    joint: {
                        "position": self.init_position[joint],
                        "velocity": 0.0,
                    }
                    for joint in self.LEG_JOINTS
                }
                | {
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
        }
        return action_dict
