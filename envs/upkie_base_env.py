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
from typing import Dict, Optional, Tuple, Union

import gym
import numpy as np
from vulp.spine import SpineInterface

from upkie_locomotion.observers.base_pitch import compute_base_pitch_from_imu

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
    }
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

    _spine: SpineInterface
    config: dict
    fall_pitch: float

    def __init__(
        self,
        config: dict,
        fall_pitch: float,
        shm_name: str,
    ) -> None:
        """!
        Initialize environment.

        @param config Configuration dictionary, also sent to the spine.
        @param fall_pitch Fall pitch angle, in radians.
        @param shm_name Name of shared-memory file.
        """
        if config is None:
            config = DEFAULT_CONFIG
        self._spine = SpineInterface(shm_name)
        self.config = config
        self.fall_pitch = fall_pitch

    def close(self) -> None:
        """!
        Stop the spine properly.
        """
        self._spine.stop()

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
        @param return_info If true, return the dictionary observation as an
            extra info dictionary.
        @param options Currently unused.
        @returns
            - ``observation``: the initial vectorized observation.
            - ``info``: an optional dictionary containing extra information.
                It is only returned if ``return_info`` is set to true.
        """
        # super().reset(seed=seed)  # we are pinned at gym==0.21.0
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
        action_dict = self.dictionarize_action(action)
        self._spine.set_action(action_dict)
        observation_dict = self._spine.get_observation()
        imu = observation_dict["imu"]
        # TODO(scaron): use tilt (angle to the vertical) rather than pitch
        pitch = compute_base_pitch_from_imu(imu["orientation"])
        observation = self.vectorize_observation(observation_dict)
        reward = self.reward.get(observation)
        done = self.detect_fall(pitch)
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
