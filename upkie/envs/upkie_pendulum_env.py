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

from typing import Optional

import numpy as np
from gymnasium import spaces

from upkie.observers.base_pitch import (
    compute_base_angular_velocity_from_imu,
    compute_base_pitch_from_imu,
)

from .reward import Reward
from .survival_reward import SurvivalReward
from .upkie_base_env import UpkieBaseEnv


class UpkiePendulumEnv(UpkieBaseEnv):

    """!
    Environment where Upkie is used in wheeled inverted pendulum mode.

    The environment id is ``UpkiePendulumEnv-v4``.

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

    - ``fall_pitch``: Fall pitch angle, in radians.
    - ``version``: Environment version number.

    """

    fall_pitch: float
    version: int = 1

    LEG_JOINTS = [
        f"{side}_{joint}"
        for side in ("left", "right")
        for joint in ("hip", "knee")
    ]

    def __init__(
        self,
        reward: Optional[Reward] = None,
        fall_pitch: float = 1.0,
        frequency: float = 200.0,
        shm_name: str = "/vulp",
        spine_config: Optional[dict] = None,
    ):
        """!
        Initialize environment.

        @param reward Reward function.
        @param fall_pitch Fall pitch angle, in radians.
        @param frequency Regulated frequency of the control loop, in Hz.
        @param shm_name Name of shared-memory file.
        @param spine_config Additional spine configuration overriding the
            defaults from ``//config:spine.yaml``. The combined configuration
            dictionary is sent to the spine at every :func:`reset`.
        """
        super().__init__(
            reward=reward if reward is not None else SurvivalReward(),
            fall_pitch=fall_pitch,
            frequency=frequency,
            shm_name=shm_name,
            spine_config=spine_config,
        )

        MAX_BASE_PITCH: float = np.pi
        MAX_GROUND_POSITION: float = float("inf")
        MAX_GROUND_VELOCITY: float = 2.0  # m/s
        MAX_BASE_ANGULAR_VELOCITY: float = 1000.0  # rad/s
        observation_limit = np.array(
            [
                MAX_BASE_PITCH,
                MAX_GROUND_POSITION,
                MAX_BASE_ANGULAR_VELOCITY,
                MAX_GROUND_VELOCITY,
            ],
            dtype=np.float32,
        )

        # gymnasium.Env: observation_space
        self.observation_space = spaces.Box(
            -observation_limit,
            +observation_limit,
            shape=(4,),
            dtype=np.float32,
        )

        # Class members
        self.fall_pitch = fall_pitch
        self.init_position = {}

    def parse_first_observation(self, observation_dict: dict) -> None:
        """!
        Parse first observation after the spine interface is initialize.

        @param observation_dict First observation.
        """
        self.init_position = {
            joint: observation_dict["servo"][joint]["position"]
            for joint in self.LEG_JOINTS
        }

    def vectorize_observation(self, observation_dict: dict) -> np.ndarray:
        """!
        Extract observation vector from a full observation dictionary.

        @param observation_dict Full observation dictionary from the spine.
        @returns Observation vector.
        """
        imu = observation_dict["imu"]
        pitch_base_in_world = compute_base_pitch_from_imu(imu["orientation"])
        angular_velocity_base_in_base = compute_base_angular_velocity_from_imu(
            observation_dict["imu"]["angular_velocity"]
        )
        ground_position = observation_dict["wheel_odometry"]["position"]
        ground_velocity = observation_dict["wheel_odometry"]["velocity"]

        obs = np.empty(4, dtype=np.float32)
        obs[0] = pitch_base_in_world
        obs[1] = ground_position
        obs[2] = angular_velocity_base_in_base[1]
        obs[3] = ground_velocity
        return obs
