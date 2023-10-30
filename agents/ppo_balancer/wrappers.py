#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria
# SPDX-License-Identifier: Apache-2.0

import gymnasium
import numpy as np


class ActionNoiser(gymnasium.Wrapper):
    def __init__(self, env, noise: np.ndarray):
        self.high = +np.abs(noise)
        self.low = -np.abs(noise)
        super(ActionNoiser, self).__init__(env)

    def reset(self, **kwargs):
        return self.env.reset(**kwargs)

    def step(self, action):
        noise = self.np_random.uniform(low=self.low, high=self.high)
        return self.env.step(action + noise)


class ActionNormalizer(gymnasium.Wrapper):
    def __init__(self, env):
        assert isinstance(env.action_space, gymnasium.spaces.Box)
        self.rescaled_low = env.action_space.low
        self.rescaled_high = env.action_space.high
        env.action_space = gymnasium.spaces.Box(
            low=-1.0,
            high=1.0,
            shape=env.action_space.shape,
            dtype=np.float32,
        )
        super(ActionNormalizer, self).__init__(env)

    def reset(self, **kwargs):
        return self.env.reset(**kwargs)

    def step(self, action):
        rescaled_action = self.rescaled_low + 0.5 * (1.0 + action) * (
            self.rescaled_high - self.rescaled_low
        )
        observation, reward, terminated, truncated, info = self.env.step(
            rescaled_action
        )
        return observation, reward, terminated, truncated, info
