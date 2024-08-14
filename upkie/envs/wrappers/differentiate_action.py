#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

from typing import Tuple

import gymnasium
import numpy as np
from gymnasium import spaces


class DifferentiateAction(gymnasium.Wrapper):
    def __init__(
        self,
        env,
        min_derivative: np.ndarray,
        max_derivative: np.ndarray,
        action_penalty: float = 0.0,
    ):
        r"""!
        Act on the derivative of the action.

        \param env Environment to wrap.
        \param min_derivative Lower bound on the derivative of the original
            action.
        \param max_derivative Upper bound on the derivative of the original
            action.
        \param action_penalty Weight for an additional penalty on the
            differential action added to the reward.

        \note We assume the original action lives in a vector space.
        """
        super().__init__(env)
        self.action_space = spaces.Box(
            np.float32(min_derivative),
            np.float32(max_derivative),
            shape=env.action_space.shape,
            dtype=np.float32,
        )
        self._integral = np.zeros(env.action_space.shape)
        self.action_penalty = action_penalty

    def reset(self, **kwargs):
        self._integral = np.zeros(self.action_space.shape)
        return self.env.reset(**kwargs)

    def step(
        self,
        action: np.ndarray,
    ) -> Tuple[np.ndarray, float, bool, bool, dict]:
        self._integral = np.clip(
            self._integral + action * self.env.unwrapped.dt,
            self.env.action_space.low,
            self.env.action_space.high,
        )
        observation, reward, terminated, truncated, info = self.env.step(
            self._integral
        )
        wrapped_reward = reward - self.action_penalty * action.dot(action)
        return observation, wrapped_reward, terminated, truncated, info
