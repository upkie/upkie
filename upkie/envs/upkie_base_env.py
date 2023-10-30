#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria
# SPDX-License-Identifier: Apache-2.0

import abc
from typing import Dict, Optional, Tuple, Union

import gymnasium
import numpy as np
from loop_rate_limiters import RateLimiter
from numpy.typing import NDArray
from vulp.spine import SpineInterface

import upkie.config
from upkie.observers.base_pitch import compute_base_pitch_from_imu
from upkie.utils.exceptions import UpkieException
from upkie.utils.nested_update import nested_update

from .init_randomization import InitRandomization


class UpkieBaseEnv(abc.ABC, gymnasium.Env):

    """!
    Base class for Upkie environments.

    ### Attributes

    This base environment has the following attributes:

    - ``fall_pitch``: Fall pitch angle, in radians.
    - ``init_rand``: Initial state randomization ranges.

    @note This environment is made to run on a single CPU thread rather than on
    GPU/TPU. The downside for reinforcement learning is that computations are
    not massively parallel. The upside is that it simplifies deployment to the
    real robot, as it relies on the same spine interface that runs on Upkie.
    """

    __frequency: Optional[float]
    __log: Dict[str, Union[float, NDArray[float]]]
    __rate: Optional[RateLimiter]
    __regulate_frequency: bool
    _spine: SpineInterface
    _spine_config: Dict
    fall_pitch: float
    init_rand: InitRandomization

    def __init__(
        self,
        fall_pitch: float = 1.0,
        frequency: Optional[float] = 200.0,
        regulate_frequency: bool = True,
        init_rand: Optional[InitRandomization] = None,
        shm_name: str = "/vulp",
        spine_config: Optional[dict] = None,
        spine_retries: int = 10,
    ) -> None:
        """!
        Initialize environment.

        @param fall_pitch Fall detection pitch angle, in radians.
        @param frequency Regulated frequency of the control loop, in Hz. Can be
            set even when `regulate_frequency` is false, as some environments
            make use of e.g. `self.dt` internally.
        @param regulate_frequency Enables loop frequency regulation.
        @param init_rand Magnitude of the random disturbance added to the
            default initial state when the environment is reset.
        @param shm_name Name of shared-memory file to exchange with the spine.
        @param spine_config Additional spine configuration overriding the
            defaults from ``//config:spine.yaml``. The combined configuration
            dictionary is sent to the spine at every :func:`reset`.
        @param spine_retries Number of times to try opening the shared-memory
            file to communicate with the spine.
        """
        merged_spine_config = upkie.config.SPINE_CONFIG.copy()
        if spine_config is not None:
            nested_update(merged_spine_config, spine_config)
        if regulate_frequency and frequency is None:
            raise UpkieException(f"{regulate_frequency=} but {frequency=}")
        if init_rand is None:
            init_rand = InitRandomization()

        self.__frequency = frequency
        self.__log = {}
        self.__rate = None
        self.__regulate_frequency = regulate_frequency
        self._spine = SpineInterface(shm_name, retries=spine_retries)
        self._spine_config = merged_spine_config
        self.fall_pitch = fall_pitch
        self.init_rand = init_rand

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

    def update_init_rand(self, **kwargs) -> None:
        """!
        Update initial-state randomization.

        Keyword arguments are forwarded as is to @ref
        upkie.envs.init_randomization.InitRandomization.update.
        """
        self.init_rand.update(**kwargs)

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
    ) -> Tuple[NDArray[float], Dict]:
        """!
        Resets the spine and get an initial observation.

        @param seed Number used to initialize the environment’s internal random
            number generator.
        @param options Currently unused.
        @returns
            - ``observation``: Initial vectorized observation, i.e. an element
              of the environment's ``observation_space``.
            - ``info``: Dictionary with auxiliary diagnostic information. For
              Upkie this is the full observation dictionary sent by the spine.
        """
        super().reset(seed=seed)
        self._spine.stop()
        self.__reset_rate()
        self.__reset_initial_robot_state()
        self._spine.start(self._spine_config)
        self._spine.get_observation()  # might be a pre-reset observation
        observation_dict = self._spine.get_observation()
        self.parse_first_observation(observation_dict)
        observation = self.vectorize_observation(observation_dict)
        info = {"observation": observation_dict}
        self.__log["observation"] = observation
        return observation, info

    def __reset_rate(self):
        if self.__regulate_frequency:
            rate_name = f"{self.__class__.__name__} rate limiter"
            self.__rate = RateLimiter(self.__frequency, name=rate_name)

    def __reset_initial_robot_state(self):
        orientation_matrix = self.init_rand.sample_orientation(self.np_random)
        qx, qy, qz, qw = orientation_matrix.as_quat()
        orientation_quat = np.array([qw, qx, qy, qz])
        position = self.init_rand.sample_position(self.np_random)
        linear_velocity = self.init_rand.sample_linear_velocity(self.np_random)
        omega = self.init_rand.sample_angular_velocity(self.np_random)

        bullet_config = self._spine_config["bullet"]
        reset = bullet_config["reset"]
        reset["orientation_base_in_world"] = orientation_quat
        reset["position_base_in_world"] = position
        reset["linear_velocity_base_to_world_in_world"] = linear_velocity
        reset["angular_velocity_base_in_base"] = omega

    def step(
        self,
        action: NDArray[float],
    ) -> Tuple[NDArray[float], float, bool, bool, dict]:
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
              us this is the full observation dictionary coming from the spine.
        """
        if self.__regulate_frequency:
            self.__rate.sleep()  # wait until clock tick to send the action

        # Act
        self.__log["action"] = action  # vector, not dictionary
        action_dict = self.dictionarize_action(action)
        action_dict["env"] = self.__log
        self._spine.set_action(action_dict)

        # Observe
        observation_dict = self._spine.get_observation()
        observation = self.vectorize_observation(observation_dict)
        reward = self.get_reward(observation, action)
        terminated = self.detect_fall(observation_dict)
        truncated = False
        info = {"observation": observation_dict}
        self.__log["observation"] = observation
        self.__log["reward"] = reward
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
        Parse first observation after the spine interface is initialized.

        @param observation_dict First observation.
        """

    @abc.abstractmethod
    def vectorize_observation(self, observation_dict: dict) -> NDArray[float]:
        """!
        Extract observation vector from a full observation dictionary.

        @param observation_dict Full observation dictionary from the spine.
        @returns Observation vector.
        """

    @abc.abstractmethod
    def dictionarize_action(self, action: NDArray[float]) -> dict:
        """!
        Convert action vector into a spine action dictionary.

        @param action Action vector.
        @returns Action dictionary.
        """

    @abc.abstractmethod
    def get_reward(self, observation: NDArray[float], action: NDArray[float]) -> float:
        """!
        Get reward from observation and action.

        @param observation Observation vector.
        @param action Action vector.
        @returns Reward.
        """
