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
from typing import Dict, Optional, Tuple, Union

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

    - ``action_dim``: Dimension of action space.
    - ``fall_pitch``: Fall pitch angle, in radians.
    - ``max_ground_velocity``: Maximum commanded ground velocity in m/s.
    - ``observation_dim``: Dimension of observation space.
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

    @note This environment is made to run on a single CPU thread rather than on
    GPU/TPU. The downside for reinforcement learning is that computations are
    not massively parallel. The upside is that it simplifies deployment to the
    real robot, as it relies on the same spine interface that runs on Upkie.
    """

    action_dim: int
    fall_pitch: float
    max_ground_velocity: float
    observation_dim: int
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
        super().__init__(config, shm_name)

        action_dim = 1
        observation_dim = 4
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
            shape=(action_dim,),
            dtype=np.float32,
        )

        # gym.Env: observation_space
        self.observation_space = spaces.Box(
            -observation_limit,
            +observation_limit,
            shape=(observation_dim,),
            dtype=np.float32,
        )

        # gym.Env: reward_range
        self.reward_range = StandingReward.get_range()

        # Class members
        self.action_dim = action_dim
        self.fall_pitch = fall_pitch
        self.init_position = {}
        self.max_ground_velocity = max_ground_velocity
        self.observation_dim = observation_dim
        self.reward = StandingReward()
        self.wheel_radius = wheel_radius

    def vectorize_observation(self, observation_dict: dict) -> np.ndarray:
        """!
        Extract observation vector from a full observation dictionary.

        @param observation_dict Full observation dictionary from the spine.
        @returns Observation vector.
        """
        imu = observation_dict["imu"]
        obs = np.empty(self.observation_dim)
        obs[0] = compute_base_pitch_from_imu(imu["orientation"])
        obs[1] = observation_dict["wheel_odometry"]["position"]
        obs[2] = observation_dict["wheel_odometry"]["velocity"]
        obs[3] = observation_dict["imu"]["angular_velocity"][1]
        return obs

    def reset(
        self,
        *,
        seed: Optional[int] = None,
        return_info: bool = False,
        options: Optional[dict] = None,
    ) -> Union[np.ndarray, Tuple[np.ndarray, Dict]]:
        """!
        Resets the spine and get an initial observation.

        @param seed Will be used once we upgrade to gym >= 0.21.0.
        @param return_info If true, return an extra info dictionary.
        @param options Currently unused.
        @returns
            - ``observation``: the initial vectorized observation.
            - ``info``: an optional dictionary containing extra information.
                It is only returned if ``return_info`` is set to true.
        """
        observation_dict = self.dict_reset()
        self.init_position = {
            joint: observation_dict["servo"][joint]["position"]
            for joint in self.LEG_JOINTS
        }
        observation = self.vectorize_observation(observation_dict)
        if not return_info:
            return observation
        else:  # return_info
            return observation, {}

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, dict]:
        """!
        Run one timestep of the environment's dynamics. When end of episode is
        reached, you are responsible for calling `reset()` to reset the
        environment's state.

        @param action Action from the agent.
        @returns
            - ``observation``: Agent's observation of the environment.
            - ``reward``: Amount of reward returned after previous action.
            - ``done``: Whether the agent reaches the terminal state, which can
              be a good or a bad thing. If true, the user needs to call
              :func:`reset()`.
            - ``info``: Contains auxiliary diagnostic information (helpful for
              debugging, logging, and sometimes learning).
        """
        # Send action
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
        self.dict_act(action_dict)

        # Read observation
        observation_dict = self.dict_observe()
        observation = self.vectorize_observation(observation_dict)

        # Compute reward
        reward = self.reward.get(observation)

        # Check termination
        done = self.detect_fall(pitch=observation[0])
        info = {
            "action": action_dict,
            "observation": observation_dict,
        }
        return observation, reward, done, info

    def detect_fall(self, pitch: float) -> bool:
        """!
        Detect a fall based on the body-to-world pitch angle.

        @param pitch Current pitch angle in [rad].
        @returns True if and only if a fall is detected.
        """
        return abs(pitch) > self.fall_pitch
