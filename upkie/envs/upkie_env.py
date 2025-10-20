#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

## \namespace upkie.envs.upkie_env
## \brief Base class with features shared by all Upkie environments.

from abc import ABC, abstractmethod
from typing import Any, Dict, Optional, Tuple

import gymnasium as gym
import numpy as np
from loop_rate_limiters import RateLimiter

from upkie.envs.backends import Backend
from upkie.exceptions import UpkieException
from upkie.model import Model
from upkie.utils.robot_state import RobotState


class UpkieEnv(gym.Env, ABC):
    r"""!
    Base class with features shared by all Upkie environments.

    Upkie environments do a number of things under the hood:

    - Communication with backends (simulators or spines).
    - Initial state randomization (e.g. when training a policy).
    - Loop frequency regulation (optional).
    """

    __frequency: Optional[float]
    __log: Dict[str, Any]
    __rate: Optional[RateLimiter]
    __regulate_frequency: bool

    ## \var action_space
    ## Action space of the environment.
    action_space: gym.Space

    ## \var backend
    ## Spine or simulation backend.
    backend: Backend

    ## \var init_state
    ## Initial state for the floating base of the robot, which may be
    ## randomized upon resets.
    init_state: RobotState

    ## \var model
    ## Upkie robot model.
    model: Model

    ## \var observation_space
    ## Observation space of the environment.
    observation_space: gym.Space

    def __init__(
        self,
        backend: Backend,
        frequency: Optional[float],
        frequency_checks: bool,
        init_state: Optional[RobotState],
        regulate_frequency: bool,
    ) -> None:
        r"""!
        Initialize environment.

        \param backend Backend for interfacing with a simulator or a spine.
        \param frequency Regulated frequency of the control loop, in Hz. Can be
            prescribed even when `regulate_frequency` is unset, in which case
            `self.dt` will be defined but the loop frequency will not be
            regulated.
        \param frequency_checks If `regulate_frequency` is set and this
            parameter is `True`, a warning will be issued every time the
            control loop runs slower than the desired `frequency`.
        \param init_state Initial state of the robot, only used in simulation.
        \param regulate_frequency If set (default), the environment will
            regulate the control loop frequency to the value prescribed in
            `frequency`.

        \throw UpkieException If the configuration is invalid.
        """
        if regulate_frequency and frequency is None:
            raise UpkieException(f"{regulate_frequency=} but {frequency=}")
        if init_state is None:
            init_state = RobotState(
                position_base_in_world=np.array([0.0, 0.0, 0.6])
            )

        # Instance attributes
        self.__frequency = frequency
        self.__frequency_checks = frequency_checks
        self.__log = {}
        self.__rate = None
        self.__regulate_frequency = regulate_frequency
        self.action_space = None  # subclasses should set this
        self.backend = backend
        self.init_state = init_state
        self.model = Model()
        self.observation_space = None  # subclasses should set this

    def close(self) -> None:
        r"""!
        Close the environment and clean up resources.

        This function is part of the Gymnasium API and ensures proper cleanup
        of backend resources when the environment is closed.
        """
        self.backend.close()

    @property
    def dt(self) -> Optional[float]:
        r"""!
        Regulated period of the control loop in seconds, or `None` if there
        is no loop frequency regulation.
        """
        return 1.0 / self.__frequency if self.__frequency is not None else None

    @property
    def frequency(self) -> Optional[float]:
        r"""!
        Regulated frequency of the control loop in Hz, or `None` if there is
        no loop frequency regulation.
        """
        return self.__frequency

    @abstractmethod
    def get_env_observation(self, spine_observation: dict):
        r"""!
        Get Gym environment observation from spine observation.

        \param[in] spine_observation Spine observation dictionary.
        \return Gym environment observation.
        """

    @abstractmethod
    def get_neutral_action(self):
        r"""!
        Get neutral action for this environment.
        """

    @abstractmethod
    def get_spine_action(self, env_action) -> dict:
        r"""!
        Get spine action from Gym environment action.

        \param[in] env_action Gym environment action.
        \return Spine action dictionary.
        """

    def log(self, name: str, entry: Any) -> None:
        r"""!
        Log a new entry to the "env" key of the action dictionary.

        \param name Name of the entry.
        \param entry Dictionary to log along with the actual action.
        """
        self.__log[name] = entry.copy() if isinstance(entry, dict) else entry

    def reset(
        self,
        seed: Optional[int] = None,
        options: Optional[dict] = None,
    ) -> Tuple[dict, dict]:
        r"""!
        Resets the backend and get an initial observation.

        \param seed Number used to initialize the environment's internal random
            number generator.
        \param options Currently unused.
        \return
            - `observation`: Initial vectorized observation, i.e. an element
              of the environment's `observation_space`.
            - `info`: Dictionary with auxiliary diagnostic information. For
              Upkie this is the full observation dictionary sent by the
              backend.
        """
        super().reset(seed=seed)
        if self.__regulate_frequency:
            rate_name = f"{self.__class__.__name__} rate limiter"
            self.__rate = RateLimiter(
                self.__frequency,
                name=rate_name,
                warn=self.__frequency_checks,
            )

        # Sample initial state and reset backend
        sampled_init_state = self.init_state.sample_state(self.np_random)
        spine_observation = self.backend.reset(sampled_init_state)
        observation = self.get_env_observation(spine_observation)
        info = {"spine_observation": spine_observation}
        return observation, info

    def step(self, action: dict) -> Tuple[dict, float, bool, bool, dict]:
        r"""!
        Run one timestep of the environment's dynamics.

        When the end of the episode is reached, you are responsible for calling
        `reset()` to reset the environment's state.

        \param action Action from the agent.
        \return
            - `observation`: Observation of the environment, i.e. an element
              of its `observation_space`.
            - `reward`: Reward returned after taking the action.
            - `terminated`: Whether the agent reached a terminal state,
              which may be a good or a bad thing. When true, the user needs to
              call `reset()`.
            - `truncated`: Whether the episode is reaching max number of
              steps. This boolean can signal a premature end of the episode,
              i.e. before a terminal state is reached. When true, the user
              needs to call `reset()`.
            - `info`: Dictionary with additional information, reporting in
              particular the full observation dictionary coming from the
              backend.
        """
        # Regulate loop frequency, if applicable
        if self.__regulate_frequency:
            self.__rate.sleep()  # wait until clock tick to send the action

        # Convert environment action to spine action and apply it
        spine_action = self.get_spine_action(action)
        spine_action["env"] = self.__log
        spine_observation = self.backend.step(spine_action)

        # Get observation
        observation = self.get_env_observation(spine_observation)
        reward = 1.0  # reward can be decided by a wrapper
        terminated = False
        truncated = False  # will be handled by e.g. a TimeLimit wrapper
        info = {"spine_observation": spine_observation}

        return observation, reward, terminated, truncated, info

    def update_init_rand(self, **kwargs) -> None:
        r"""!
        Update initial-state randomization.

        Keyword arguments are forwarded as is to \ref
        upkie.utils.robot_state_randomization.RobotStateRandomization.update.
        """
        self.init_state.randomization.update(**kwargs)
