#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria
# SPDX-License-Identifier: Apache-2.0

import gymnasium
import numpy as np
from gymnasium import spaces


class ActionNoiser(gymnasium.ActionWrapper):
    def __init__(self, env, noise: np.ndarray):
        super(ActionNoiser, self).__init__(env)
        self.high = +np.abs(noise)
        self.low = -np.abs(noise)
        self.action_space = spaces.Box(
            low=env.action_space.low + self.low,
            high=env.action_space.high + self.high,
            shape=env.action_space.shape,
            dtype=env.action_space.dtype,
        )

    def action(self, action):
        noise = self.np_random.uniform(low=self.low, high=self.high)
        return action + noise


class ObservationNoiser(gymnasium.Wrapper):
    def __init__(self, env, noise: np.ndarray):
        self.high = +np.abs(noise)
        self.low = -np.abs(noise)
        super(ObservationNoiser, self).__init__(env)

    def reset(self, **kwargs):
        return self.env.reset(**kwargs)

    def step(self, action):
        obs, reward, terminated, truncated, info = self.env.step(action)
        noise = self.np_random.uniform(low=self.low, high=self.high)
        return (obs + noise), reward, terminated, truncated, info
