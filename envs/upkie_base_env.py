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

import abc
import asyncio
from typing import Dict, Optional, Tuple, Union

import gymnasium as gym
import numpy as np
from loop_rate_limiters import AsyncRateLimiter, RateLimiter
from vulp.spine import SpineInterface

from upkie.observers.base_pitch import compute_base_pitch_from_imu

DEFAULT_CONFIG = {
    "bullet": {
        "control_mode": "torque",
        "follower_camera": False,
        "gui": True,
        "position_init_base_in_world": [0.0, 0.0, 0.6],
        "torque_control": {
            "kp": 20.0,
            "kd": 1.0,
        },
    },
    "floor_contact": {
        "upper_leg_torque_threshold": 10.0,
    },
    "wheel_contact": {
        "cutoff_period": 0.2,
        "liftoff_inertia": 0.001,
        "min_touchdown_acceleration": 2.0,
        "min_touchdown_torque": 0.015,
        "touchdown_inertia": 0.004,
    },
}


class UpkieBaseEnv(abc.ABC, gym.Env):

    """!
    Base class for Upkie environments.

    This class has the following attributes:

    - ``config``: Configuration dictionary, also sent to the spine.
    - ``fall_pitch``: Fall pitch angle, in radians.

    @note This environment is made to run on a single CPU thread rather than on
    GPU/TPU. The downside for reinforcement learning is that computations are
    not massively parallel. The upside is that it simplifies deployment to the
    real robot, as it relies on the same spine interface that runs on Upkie.
    """

    __frequency: Optional[float]
    __async_rate: Optional[AsyncRateLimiter]
    __rate: Optional[RateLimiter]
    _spine: SpineInterface
    config: dict
    fall_pitch: float

    def __init__(
        self,
        config: Optional[dict],
        fall_pitch: float,
        frequency: Optional[float],
        shm_name: str,
    ) -> None:
        """!
        Initialize environment.

        @param config Configuration dictionary, also sent to the spine.
        @param fall_pitch Fall pitch angle, in radians.
        @param frequency Regulated frequency of the control loop, in Hz. Set to
            ``None`` to disable loop frequency regulation.
        @param shm_name Name of shared-memory file.
        """
        if config is None:
            config = DEFAULT_CONFIG
        self.__frequency = frequency
        self._spine = SpineInterface(shm_name)
        self.config = config
        self.fall_pitch = fall_pitch

    @property
    def frequency(self) -> Optional[float]:
        return self.__frequency

    def close(self) -> None:
        """!
        Stop the spine properly.
        """
        self._spine.stop()

    def reset(
        self,
        *,
        seed: Optional[int] = None,
        return_info: bool = True,
        options: Optional[dict] = None,
    ) -> Union[np.ndarray, Tuple[np.ndarray, Dict]]:
        """!
        Resets the spine and get an initial observation.

        @param seed is not used yet but should be (TODO).
        @param return_info If true, return the dictionary observation as an
            extra info dictionary.
        @param options Currently unused.
        @returns
            - ``observation``: the initial vectorized observation.
            - ``info``: an optional dictionary containing extra information.
                It is only returned if ``return_info`` is set to true.
        """
        # super().reset(seed=seed) 
        self.__reset_rates()
        self._spine.stop()
        self._spine.start(self.config)
        self._spine.get_observation()  # might be a pre-reset observation
        observation_dict = self._spine.get_observation()
        self.parse_first_observation(observation_dict)
        observation = self.vectorize_observation(observation_dict)
        if not return_info:
            return observation
        else:  # return_info
            return observation, observation_dict

    def __reset_rates(self):
        self.__async_rate = None
        self.__rate = None
        if self.__frequency is None:  # no rate
            return
        try:
            asyncio.get_running_loop()
            self.__async_rate = AsyncRateLimiter(self.__frequency)
        except RuntimeError:  # not asyncio
            self.__rate = RateLimiter(self.__frequency)

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, dict]:
        """!
        Run one timestep of the environment's dynamics. When the end of the
        episode is reached, you are responsible for calling `reset()` to reset
        the environment's state.

        @param action Action from the agent.
        @returns
            - ``observation``: Agent's observation of the environment.
            - ``reward``: Amount of reward returned after previous action.
            - ``terminated``: Whether the agent reaches the terminal state, which can
              be a good or a bad thing. If true, the user needs to call
              :func:`reset()`.
            - ``truncated'': Whether the episode is truncated (reaching max number of steps).
            - ``info``: Contains auxiliary diagnostic information (helpful for
              debugging, logging, and sometimes learning).
        """
        action_dict = self.dictionarize_action(action)
        if self.__rate is not None:
            self.__rate.sleep()  # wait until clock tick to send the action
        return self.__step(action_dict)

    async def async_step(
        self, action: np.ndarray
    ) -> Tuple[np.ndarray, float, bool, bool, dict]:
        """!
        Run one timestep of the environment's dynamics using asynchronous I/O.
        When the end of the episode is reached, you are responsible for calling
        `reset()` to reset the environment's state.

        @param action Action from the agent.
        @returns
            - ``observation``: Agent's observation of the environment.
            - ``reward``: Amount of reward returned after previous action.
            - ``terminated``: Whether the agent reaches the terminal state, which can
              be a good or a bad thing. If true, the user needs to call
              :func:`reset()`.
            - ``truncated'': Whether the episode is truncated (reaching max number of steps).
            - ``info``: Contains auxiliary diagnostic information (helpful for
              debugging, logging, and sometimes learning).
        """
        action_dict = self.dictionarize_action(action)
        if self.__async_rate is not None:
            await self.__async_rate.sleep()  # send action at next clock tick
        return self.__step(action_dict)

    def __step(
        self, action_dict: dict
    ) -> Tuple[np.ndarray, float, bool, dict]:
        self._spine.set_action(action_dict)
        observation_dict = self._spine.get_observation()
        observation = self.vectorize_observation(observation_dict)
        reward = self.reward.get(observation)
        terminated = self.detect_fall(observation_dict)
        truncated = False
        info = {
            "action": action_dict,
            "observation": observation_dict,
        }
        return observation, reward, terminated, truncated, info

    def detect_fall(self, observation_dict: dict) -> bool:
        """!
        Detect a fall based on the body-to-world pitch angle.

        @param observation_dict Observation dictionary with an "imu" key.
        @returns True if and only if a fall is detected.
        """
        imu = observation_dict["imu"]
        pitch = compute_base_pitch_from_imu(imu["orientation"])
        return abs(pitch) > self.fall_pitch

    @abc.abstractmethod
    def parse_first_observation(self, observation_dict: dict) -> None:
        """!
        Parse first observation after the spine interface is initialize.

        @param observation_dict First observation.
        """

    @abc.abstractmethod
    def vectorize_observation(self, observation_dict: dict) -> np.ndarray:
        """!
        Extract observation vector from a full observation dictionary.

        @param observation_dict Full observation dictionary from the spine.
        @returns Observation vector.
        """

    @abc.abstractmethod
    def dictionarize_action(self, action: np.ndarray) -> dict:
        """!
        Convert action vector into a spine action dictionary.

        @param action Action vector.
        @returns Action dictionary.
        """
