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
from typing import Dict, Optional, Tuple

import gymnasium
import numpy as np
from loop_rate_limiters import AsyncRateLimiter, RateLimiter
from vulp.spine import SpineInterface

import upkie.config
from upkie.observers.base_pitch import compute_base_pitch_from_imu


class UpkieBaseEnv(abc.ABC, gymnasium.Env):

    """!
    Base class for Upkie environments.

    This class has the following attributes:

    - ``config``: Configuration dictionary sent to the spine.
    - ``fall_pitch``: Fall pitch angle, in radians.

    @note This environment is made to run on a single CPU thread rather than on
    GPU/TPU. The downside for reinforcement learning is that computations are
    not massively parallel. The upside is that it simplifies deployment to the
    real robot, as it relies on the same spine interface that runs on Upkie.
    """

    __async_rate: Optional[AsyncRateLimiter]
    __frequency: Optional[float]
    __rate: Optional[RateLimiter]
    _spine: SpineInterface
    fall_pitch: float
    spine_config: dict

    def __init__(
        self,
        fall_pitch: float,
        frequency: Optional[float],
        shm_name: str,
        spine_config: Optional[dict],
    ) -> None:
        """!
        Initialize environment.

        @param fall_pitch Fall pitch angle, in radians.
        @param frequency Regulated frequency of the control loop, in Hz. Set to
            ``None`` to disable loop frequency regulation.
        @param shm_name Name of shared-memory file.
        @param spine_config Additional spine configuration overriding the
            defaults from ``//config:spine.yaml``. The combined configuration
            dictionary is sent to the spine at every :func:`reset`.
        """
        merged_spine_config = upkie.config.SPINE_CONFIG.copy()
        if spine_config is not None:
            merged_spine_config.update(spine_config)
        self.__frequency = frequency
        self._spine = SpineInterface(shm_name)
        self.fall_pitch = fall_pitch
        self.spine_config = merged_spine_config

    @property
    def dt(self) -> Optional[float]:
        """!
        Regulated period of the control loop in seconds, or ``None`` if there
        is no loop frequency regulation.
        """
        return 1.0 / self.__frequency if self.__frequency is not None else None

    @property
    def frequency(self) -> Optional[float]:
        """!
        Regulated frequency of the control loop in Hz, or ``None`` if there is
        no loop frequency regulation.
        """
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
        options: Optional[dict] = None,
    ) -> Tuple[np.ndarray, Dict]:
        """!
        Resets the spine and get an initial observation.

        @param seed It is not used yet but should be. TODO(perrin-isir)
        @param options Currently unused.
        @returns
            - ``observation``: Initial vectorized observation, i.e. an element
              of the environment's ``observation_space``.
            - ``info``: Dictionary with auxiliary diagnostic information. For
              us this will be the full observation dictionary coming sent by
              the spine.
        """
        super().reset(seed=seed)
        self.__reset_rates()
        self._spine.stop()
        self._spine.start(self.spine_config)
        self._spine.get_observation()  # might be a pre-reset observation
        observation_dict = self._spine.get_observation()
        self.parse_first_observation(observation_dict)
        observation = self.vectorize_observation(observation_dict)
        info = {
            "action": None,
            "observation": observation_dict,
        }
        return observation, info

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

    def step(
        self,
        action: np.ndarray,
    ) -> Tuple[np.ndarray, float, bool, bool, dict]:
        """!
        Run one timestep of the environment's dynamics. When the end of the
        episode is reached, you are responsible for calling `reset()` to reset
        the environment's state.

        @param action Action from the agent.
        @returns
            - ``observation``: Observation of the environment, i.e. an element
              of its ``observation_space``.
            - ``reward``: Reward returned after taking the action.
            - ``terminated``: Whether the agent reached a terminal state,
              which can be a good or a bad thing. When true, the user needs to
              call :func:`reset()`.
            - ``truncated'': Whether the episode is reaching max number of
              steps. This boolean can signal a premature end of the episode,
              i.e. before a terminal state is reached. When true, the user
              needs to call :func:`reset()`.
            - ``info``: Dictionary with auxiliary diagnostic information. For
              us this will be the full observation dictionary coming sent by
              the spine.
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
            - ``observation``: Observation of the environment, i.e. an element
              of its ``observation_space``.
            - ``reward``: Reward returned after taking the action.
            - ``terminated``: Whether the agent reached a terminal state,
              which can be a good or a bad thing. When true, the user needs to
              call :func:`reset()`.
            - ``truncated'': Whether the episode is reaching max number of
              steps. This boolean can signal a premature end of the episode,
              i.e. before a terminal state is reached. When true, the user
              needs to call :func:`reset()`.
            - ``info``: Dictionary with auxiliary diagnostic information. For
              us this will be the full observation dictionary coming sent by
              the spine.
        """
        action_dict = self.dictionarize_action(action)
        if self.__async_rate is not None:
            await self.__async_rate.sleep()  # send action at next clock tick
        return self.__step(action_dict)

    def __step(
        self, action_dict: dict
    ) -> Tuple[np.ndarray, float, bool, bool, dict]:
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
