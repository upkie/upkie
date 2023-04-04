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

from typing import Optional

import numpy as np
import pinocchio as pin
import upkie_description
from gym import spaces

from .standing_reward import StandingReward
from .upkie_base_env import UpkieBaseEnv

MAX_BASE_PITCH: float = np.pi
MAX_GROUND_POSITION: float = float("inf")
MAX_GROUND_VELOCITY: float = 10.0  # m/s
MAX_IMU_ANGULAR_VELOCITY: float = 1000.0  # rad/s


class UpkieServosEnv(UpkieBaseEnv):

    """!
    Upkie with full observation and joint position-velocity actions.

    The environment has the following attributes:

    - ``reward``: Reward function.
    - ``robot``: Pinocchio robot wrapper.
    - ``version``: Environment version number.

    Vectorized observations have the following structure:

    <table>
        <tr>
            <td><strong>Index</strong></td>
            <td><strong>Description</strong></td>
            </tr>
        <tr>
            <td>[0:nq]</td>
            <td>Joint positions in [rad].</td>
        </tr>
        <tr>
            <td>[nq:nq + nv]</td>
            <td>Joint velocities in [rad] / [s].</td>
        </tr>
    </table>

    Vectorized actions have the following structure:

    <table>
        <tr>
            <td><strong>Index</strong></td>
            <td><strong>Description</strong></td>
            </tr>
        <tr>
            <td>[0:nq]</td>
            <td>Joint position commands in [rad].</td>
        </tr>
        <tr>
            <td>[nq:nq + nv]</td>
            <td>Joint velocity commands in [rad] / [s].</td>
        </tr>
    </table>

    The reward function is defined in @ref
    envs.standing_reward.StandingReward "StandingReward".

    See also @ref envs.upkie_base_env.UpkieBaseEnv "UpkieBaseEnv" for notes on
    using this environment.
    """

    reward: StandingReward
    robot: pin.RobotWrapper
    version: int = 1

    def __init__(
        self,
        config: Optional[dict] = None,
        fall_pitch: float = 1.0,
        shm_name: str = "/vulp",
    ):
        """!
        Initialize environment.

        @param config Configuration dictionary, also sent to the spine.
        @param fall_pitch Fall pitch angle, in radians.
        @param shm_name Name of shared-memory file.
        """
        super().__init__(config, fall_pitch, shm_name)

        robot = upkie_description.load_in_pinocchio(root_joint=None)
        model = robot.model
        x_min = np.hstack([model.lowerPositionLimit, -model.velocityLimit])
        x_max = np.hstack([model.upperPositionLimit, +model.velocityLimit])
        nx = model.nq + model.nv

        # gym.Env: action_space
        self.action_space = spaces.Box(
            x_min,
            x_max,
            shape=(nx,),
            dtype=np.float32,
        )

        # gym.Env: observation_space
        self.observation_space = spaces.Box(
            x_min,
            x_max,
            shape=(nx,),
            dtype=np.float32,
        )

        # gym.Env: reward_range
        self.reward_range = StandingReward.get_range()

        # Class members
        self.__joints = list(robot.model.names)[1:]
        self.__last_positions = {}
        self.reward = StandingReward()
        self.robot = robot

    def parse_first_observation(self, observation_dict: dict) -> None:
        """!
        Parse first observation after the spine interface is initialize.

        @param observation_dict First observation.
        """
        self.__last_positions = {
            joint: observation_dict["servo"][joint]["position"]
            for joint in self.__joints
        }

    def vectorize_observation(self, observation_dict: dict) -> np.ndarray:
        """!
        Extract observation vector from a full observation dictionary.

        @param observation_dict Full observation dictionary from the spine.
        @returns Observation vector.
        """
        nq, nv = self.robot.model.nq, self.robot.model.nv
        model = self.robot.model
        obs = np.empty(nq + nv)
        for joint in self.__joints:
            i = model.getJointId(joint) - 1
            obs[i] = observation_dict["servo"][joint]["position"]
            obs[nq + i] = observation_dict["servo"][joint]["velocity"]
        return obs

    def dictionarize_action(self, action: np.ndarray) -> dict:
        """!
        Convert action vector into a spine action dictionary.

        @param action Action vector.
        @returns Action dictionary.
        """
        nq = self.robot.model.nq
        model = self.robot.model
        servo_action = {
            joint: {
                "position": self.__last_positions[joint],
                "velocity": 0.0,
            }
            for joint in self.__joints
        }

        nq = model.nq
        nv = model.nv
        q = action[:nq]
        v = action[nq : nq + nv]
        # TODO(scaron): clamp q + test
        # TODO(scaron): clamp v + test
        for joint in self.__joints:
            i = model.getJointId(joint) - 1
            servo_action[joint]["position"] = q[i]
            servo_action[joint]["velocity"] = v[i]
            self.__last_positions[joint] = q[i]
        return {"servo": servo_action}
