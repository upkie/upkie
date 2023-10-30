#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria
# SPDX-License-Identifier: Apache-2.0

from typing import Union

import gymnasium
import numpy as np
from gymnasium.spaces import Box
from numpy.typing import NDArray

from upkie.utils.exceptions import UpkieException
from upkie.utils.filters import low_pass_filter


class ActionLowPassFilter(gymnasium.Wrapper):
    filtered_action: NDArray[float]
    time_constant: float
    time_constant_box: Box

    def __init__(self, env, time_constant: Union[float, Box]):
        """!
        Initialize wrapper.

        @param env Environment to wrap.
        @param time_constant Cutoff period in seconds of a low-pass filter
            applied to the action. If a Box is provided, couple of lower and
            upper bounds for the action: a new time constant is sampled
            uniformly at random between these bounds at every reset of the
            environment.
        """
        super().__init__(env)
        time_constant_box = (
            time_constant
            if isinstance(time_constant, Box)
            else Box(low=time_constant - 1e-10, high=time_constant + 1e-10)
        )
        self.filtered_action = np.zeros(env.action_space.shape)
        self.time_constant = time_constant_box.sample()
        self.time_constant_box = time_constant_box

    def reset(self, **kwargs):
        self.filtered_action = np.zeros(self.env.action_space.shape)
        self.time_constant = self.time_constant_box.sample()
        return self.env.reset(**kwargs)

    def step(self, action):
        if self.time_constant <= 2.0 * self.env.dt:
            # Nyquist–Shannon sampling theorem
            return self.env.step(action)

        self.filtered_action = low_pass_filter(
            self.filtered_action,
            self.time_constant,
            action,
            self.env.dt,
        )
        return self.env.step(self.filtered_action)


class ActionNoiser(gymnasium.ActionWrapper):
    def __init__(self, env, noise: np.ndarray):
        super().__init__(env)
        if noise.shape != env.action_space.shape:
            raise UpkieException(
                f"Action {noise.shape=} does not "
                f"match {env.action_space.shape=}"
            )
        self.high = +np.abs(noise)
        self.low = -np.abs(noise)

    def action(self, action):
        noise = self.np_random.uniform(low=self.low, high=self.high)
        noisy_action = np.clip(
            action + noise,
            self.env.action_space.low,
            self.env.action_space.high,
        )
        return noisy_action


class ObservationNoiser(gymnasium.ObservationWrapper):
    def __init__(self, env, noise: np.ndarray):
        super().__init__(env)
        if noise.shape != env.observation_space.shape:
            raise UpkieException(
                f"Observation {noise.shape=} does not "
                f"match {env.observation_space.shape=}"
            )
        self.high = +np.abs(noise)
        self.low = -np.abs(noise)

    def observation(self, observation):
        noise = self.np_random.uniform(low=self.low, high=self.high)
        noisy_observation = np.clip(
            observation + noise,
            self.env.observation_space.low,
            self.env.observation_space.high,
        )
        return noisy_observation
