#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
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
from typing import Dict, Optional

import numpy as np
from gym import spaces
from upkie.observers.base_pitch import compute_base_pitch_from_imu

from .standing_reward import StandingReward
from .upkie_base_env import UpkieBaseEnv

MAX_BASE_PITCH: float = np.pi  # rad
MAX_GROUND_POSITION: float = float("inf")
MAX_IMU_ANGULAR_VELOCITY: float = 1000.0  # rad/s
MAX_GROUND_VELOCITY: float = 1.0  # m/s


class UpkieCartPoleEnv(UpkieBaseEnv):

    """!
    Upkie with cart-pole observation and action.

    The environment has the following attributes:

    - ``fall_pitch``: Fall pitch angle, in radians.
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

    Vectorized actions have the following structure:

    <table>
        <tr>
            <td><strong>Index</strong></td>
            <td><strong>Description</strong></td>
            </tr>
        <tr>
            <td>``0``</td>
            <td>Linear ground force in [N].</td>
        </tr>
    </table>

    The reward function is defined in @ref
    envs.standing_reward.StandingReward "StandingReward".

    See also @ref envs.upkie_base_env.UpkieBaseEnv "UpkieBaseEnv" for notes on
    using this environment.
    """

    init_position: Dict[str, float]
    fall_pitch: float
    max_ground_force: float
    version: int = 1
    wheel_radius: float

    # TODO(scaron): move to Python Upkie layout
    LEG_JOINTS = [
        f"{side}_{joint}"
        for side in ("left", "right")
        for joint in ("hip", "knee")
    ]

    def __init__(
        self,
        config: Optional[dict] = None,
        fall_pitch: float = 1.0,
        frequency: float = 200.0,
        max_ground_force: float = 10.0,
        shm_name: str = "/vulp",
        wheel_radius: float = 0.06,
    ):
        """!
        Initialize environment.

        @param config Configuration dictionary, also sent to the spine.
        @param fall_pitch Fall pitch angle, in radians.
        @param frequency Regulated frequency of the control loop, in Hz.
        @param max_ground_force Maximum commanded ground force in N.
        @param shm_name Name of shared-memory file.
        @param wheel_radius Wheel radius in [m].
        """
        super().__init__(
            config=config,
            fall_pitch=fall_pitch,
            frequency=frequency,
            shm_name=shm_name,
        )

        observation_limit = np.array(
            [
                MAX_BASE_PITCH,
                MAX_GROUND_POSITION,
                MAX_GROUND_VELOCITY,
                MAX_IMU_ANGULAR_VELOCITY,
            ],
            dtype=np.float32,
        )

        # gym.Env: action_space
        self.action_space = spaces.Box(
            -np.float32(max_ground_force),
            +np.float32(max_ground_force),
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
        self.init_position = {}
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
        commanded_force: float = action[0]
        commanded_torque = commanded_force * self.wheel_radius
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
            "velocity": 0.0,
            "kp_scale": 0.0,
            "kd_scale": 0.0,
            "torque": commanded_torque,
        }
        action_dict["servo"]["right_wheel"] = {
            "position": math.nan,
            "velocity": 0.0,
            "kp_scale": 0.0,
            "kd_scale": 0.0,
            "torque": -commanded_torque,
        }
        return action_dict
