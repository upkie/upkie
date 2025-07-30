#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

from typing import Optional, Tuple

import gymnasium as gym
import numpy as np
from loop_rate_limiters import RateLimiter

from upkie.envs.pipelines import Pipeline, ServoPipeline
from upkie.exceptions import UpkieException
from upkie.model import Model
from upkie.utils.robot_state import RobotState


class UpkieEnv(gym.Env):
    r"""!
    Base class with features shared by all Upkie environments.

    Upkie environments do a number of things under the hood:

    - Communication with the spine process.
    - Initial state randomization (e.g. when training a policy).
    - Loop frequency regulation (optional).

    Note that Upkie environments are made to run on a single CPU thread. The
    downside for reinforcement learning is that computations are not massively
    parallel. The upside is that it simplifies deployment to the real robot, as
    it relies on the same spine interface that runs on real robots.
    """

    __frequency: Optional[float]
    __rate: Optional[RateLimiter]
    __regulate_frequency: bool

    ## \var init_state
    ## Initial state for the floating base of the robot, which may be
    ## randomized upon resets.
    init_state: RobotState

    def __init__(
        self,
        frequency: Optional[float] = 200.0,
        frequency_checks: bool = True,
        init_state: Optional[RobotState] = None,
        pipeline: Optional[Pipeline] = None,
        regulate_frequency: bool = True,
    ) -> None:
        r"""!
        Initialize environment.

        \param frequency Regulated frequency of the control loop, in Hz. Can be
            prescribed even when `regulate_frequency` is unset, in which case
            `self.dt` will be defined but the loop frequency will not be
            regulated.
        \param frequency_checks If `regulate_frequency` is set and this
            parameter is true (default), a warning is issued every time the
            control loop runs slower than the desired `frequency`. Set this
            parameter to false to disable these warnings.
        \param init_state Initial state of the robot, only used in simulation.
        \param pipeline Spine dictionary interface selected via the --pipeline
            command-line argument of the spine binary, if any.
        \param regulate_frequency If set (default), the environment will
            regulate the control loop frequency to the value prescribed in
            `frequency`.

        \throw SpineError If the spine did not respond after the prescribed
            number of trials.
        """
        if regulate_frequency and frequency is None:
            raise UpkieException(f"{regulate_frequency=} but {frequency=}")
        if init_state is None:
            init_state = RobotState(
                position_base_in_world=np.array([0.0, 0.0, 0.6])
            )
        if pipeline is None:
            pipeline = ServoPipeline()

        # Class attributes
        self.__frequency = frequency
        self.__frequency_checks = frequency_checks
        self.__pipeline = pipeline
        self.__rate = None
        self.__regulate_frequency = regulate_frequency
        self.action_space = pipeline.action_space
        self.init_state = init_state
        self.observation_space = pipeline.observation_space

    @property
    def dt(self) -> Optional[float]:
        """!
        Regulated period of the control loop in seconds, or `None` if there
        is no loop frequency regulation.
        """
        return 1.0 / self.__frequency if self.__frequency is not None else None

    @property
    def frequency(self) -> Optional[float]:
        """!
        Regulated frequency of the control loop in Hz, or `None` if there is
        no loop frequency regulation.
        """
        return self.__frequency

    def get_env_observation(self, spine_observation):
        return self.__pipeline.get_env_observation(spine_observation)

    def get_neutral_action(self):
        return self.__pipeline.get_neutral_action()

    def get_spine_action(self, env_action):
        return self.__pipeline.get_spine_action(env_action)

    @property
    def model(self) -> Model:
        return self.__pipeline.model

    def reset(
        self,
        seed: Optional[int] = None,
        options: Optional[dict] = None,
    ) -> Tuple[dict, dict]:
        r"""!
        Resets the spine and get an initial observation.

        \param seed Number used to initialize the environment’s internal random
            number generator.
        \param options Currently unused.
        \return
            - `observation`: Initial vectorized observation, i.e. an element
              of the environment's `observation_space`.
            - `info`: Dictionary with auxiliary diagnostic information. For
              Upkie this is the full observation dictionary sent by the spine.
        """
        super().reset(seed=seed)
        if self.__regulate_frequency:
            rate_name = f"{self.__class__.__name__} rate limiter"
            self.__rate = RateLimiter(
                self.__frequency,
                name=rate_name,
                warn=self.__frequency_checks,
            )
        return {}, {}

    def step(self) -> None:
        r"""!
        Regulate the control loop frequency, if applicable.
        """
        if self.__regulate_frequency:
            self.__rate.sleep()  # wait until clock tick to send the action

    def update_init_rand(self, **kwargs) -> None:
        r"""!
        Update initial-state randomization.

        Keyword arguments are forwarded as is to \ref
        upkie.utils.robot_state_randomization.RobotStateRandomization.update.
        """
        self.init_state.randomization.update(**kwargs)
