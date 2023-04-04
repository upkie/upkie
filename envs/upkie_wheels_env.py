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
from typing import Optional

import numpy as np
from gym import spaces

from upkie_locomotion.observers.base_pitch import compute_base_pitch_from_imu

from .standing_reward import StandingReward
from .upkie_base_env import UpkieBaseEnv

MAX_BASE_PITCH: float = np.pi
MAX_GROUND_POSITION: float = float("inf")
MAX_IMU_ANGULAR_VELOCITY: float = 1000.0  # rad/s


class UpkieWheelsEnv(UpkieBaseEnv):

    """!
    Upkie with full observation but only wheel velocity actions.

    The environment has the following attributes:

    - ``fall_pitch``: Fall pitch angle, in radians.
    - ``max_ground_velocity``: Maximum commanded ground velocity in m/s.
    - ``version``: Environment version number.
    - ``wheel_radius``: Wheel radius in [m].

    Vectorized observations have the following structure:

    <table>
        <tr>
            <td><strong>Index</strong></td>
            <td><strong>Description</strong></td>
            </tr>
        <tr>
            <td>0</td>
            <td>Base pitch in rad.</td>
        </tr>
        <tr>
            <td>1</td>
            <td>Position of the average wheel contact point, in m.</td>
        </tr>
        <tr>
            <td>2</td>
            <td>Velocity of the average wheel contact point, in m/s.</td>
        </tr>
        <tr>
            <td>3</td>
            <td>Body angular velocity of the IMU frame along its y-axis, in
            rad/s.</td>
        </tr>
    </table>

    The reward function is defined in @ref
    envs.standing_reward.StandingReward "StandingReward".

    See also @ref envs.upkie_base_env.UpkieBaseEnv "UpkieBaseEnv" for notes on
    using this environment.
    """

    fall_pitch: float
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
        config: Optional[dict] = None,
        fall_pitch: float = 1.0,
        max_ground_velocity: float = 1.0,
        shm_name: str = "/vulp",
        wheel_radius: float = 0.06,
    ):
        """!
        Initialize environment.

        @param config Configuration dictionary, also sent to the spine.
        @param fall_pitch Fall pitch angle, in radians.
        @param max_ground_velocity Maximum commanded ground velocity in m/s.
        @param shm_name Name of shared-memory file.
        @param wheel_radius Wheel radius in [m].
        """
        super().__init__(config, fall_pitch, shm_name)

        observation_limit = np.array(
            [
                MAX_BASE_PITCH,
                MAX_GROUND_POSITION,
                max_ground_velocity,
                MAX_IMU_ANGULAR_VELOCITY,
            ]
        )

        # gym.Env: action_space
        self.action_space = spaces.Box(
            -max_ground_velocity,
            +max_ground_velocity,
            shape=(1,),
            dtype=np.float32,
        )

        # gym.Env: observation_space
        self.observation_space = spaces.Box(
            -observation_limit,
            +observation_limit,
            shape=(4,),
            dtype=np.float32,
        )

        # gym.Env: reward_range
        self.reward_range = StandingReward.get_range()

        # Class members
        self.fall_pitch = fall_pitch
        self.init_position = {}
        self.max_ground_velocity = max_ground_velocity
        self.reward = StandingReward()
        self.wheel_radius = wheel_radius

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
        obs = np.empty(4)
        obs[0] = compute_base_pitch_from_imu(imu["orientation"])
        obs[1] = observation_dict["wheel_odometry"]["position"]
        obs[2] = observation_dict["wheel_odometry"]["velocity"]
        obs[3] = observation_dict["imu"]["angular_velocity"][1]
        return obs

    def dictionarize_action(self, action: np.ndarray) -> dict:
        """!
        Convert action vector into a spine action dictionary.

        @param action Action vector.
        @returns Action dictionary.
        """
        commanded_velocity: float = action[0]
        action_dict = {
            "servo": {
                joint: {
                    "position": self.init_position[joint],
                    "velocity": 0.0,
                }
                for joint in self.LEG_JOINTS
            }
        }
        action_dict["servo"]["left_wheel"] = {
            "position": math.nan,
            "velocity": +commanded_velocity / self.wheel_radius,
        }
        action_dict["servo"]["right_wheel"] = {
            "position": math.nan,
            "velocity": -commanded_velocity / self.wheel_radius,
        }
        return action_dict
