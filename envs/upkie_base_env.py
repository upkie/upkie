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
from os import path
from typing import Optional

import gym
import yaml
from vulp.spine import SpineInterface


class UpkieBaseEnv(abc.ABC, gym.Env):

    """!
    Base class for Upkie environments.

    This class implements "dict_" variants of the standard act, observe and
    reset functions from the Gym API. Child classes are responsible for
    implementing their own act, observe and reset functions.

    The base environment has the following attributes:

    - ``_config``: Configuration dictionary, also sent to the spine.
    - ``_last_action``: Last action dictionary sent to the spine.
    - ``_last_observation``: Last observation dictionary read from the spine.
    - ``_spine``: Internal spine interface.

    @note This environment is made to run on a single CPU thread rather than on
    GPU/TPU. The downside for reinforcement learning is that computations are
    not massively parallel. The upside is that it simplifies deployment to the
    real robot, as it relies on the same spine interface that runs on Upkie.
    """

    _config: dict
    _last_action: dict
    _last_observation: dict
    _spine: SpineInterface

    def __init__(
        self,
        config: Optional[dict] = None,
        shm_name: str = "/vulp",
    ) -> None:
        """!
        Initialize environment.

        @param config Configuration dictionary, also sent to the spine.
        @param fall_pitch Fall pitch angle, in radians.
        @param max_ground_velocity Maximum commanded ground velocity in m/s.
        @param shm_name Name of shared-memory file.
        @param wheel_radius Wheel radius in [m].
        """
        if config is None:
            envs_dir = path.dirname(__file__)
            with open(f"{envs_dir}/spine.yaml", "r") as fh:
                config = yaml.safe_load(fh)
        self._config = config
        self._last_action = {}
        self._last_observation = {}
        self._spine = SpineInterface(shm_name)

    def close(self) -> None:
        """!
        Stop the spine properly.
        """
        self._spine.stop()

    def dict_act(self, action_dict) -> None:
        """!
        Internal function to send an action dictionary to the spine.

        @param action_dict Action dictionary.
        """
        self._spine.set_action(action_dict)
        self._last_action = action_dict

    def dict_observe(self) -> dict:
        """!
        Internal function to get an observation dictionary from the spine.

        @returns Observation dictionary.
        """
        observation_dict = self._spine.get_observation()
        self._last_observation = observation_dict
        return observation_dict

    def dict_reset(self) -> dict:
        """!
        Resets the spine and get an initial observation.

        @returns Initial dictionary observation.
        """
        # super().reset(seed=seed)  # we are pinned at gym==0.21.0
        self._spine.stop()
        self._spine.start(self._config)
        self._spine.get_observation()  # might be a pre-reset observation
        observation_dict = self.dict_observe()
        return observation_dict
