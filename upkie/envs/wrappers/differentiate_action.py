#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria
# SPDX-License-Identifier: Apache-2.0

import gymnasium
import numpy as np
from gymnasium import spaces
from numpy.typing import NDArray


class DifferentiateAction(gymnasium.Wrapper):
    def __init__(
        self,
        env,
        min_derivative: NDArray[float],
        max_derivative: NDArray[float],
    ):
        """!
        Act on the derivative of the action.

        @param env Environment to wrap.
        @param min_derivative Lower bound on the derivative of the original
            action.
        @param max_derivative Upper bound on the derivative of the original
            action.

        @note We assume the original action lives in a vector space.
        """
        super().__init__(env)
        self.action_space = spaces.Box(
            np.float32(min_derivative),
            np.float32(max_derivative),
            shape=env.action_space.shape,
            dtype=np.float32,
        )
        self._integral = np.zeros(env.action_space.shape)

    def reset(self, **kwargs):
        self._integral = np.zeros(self.action_space.shape)
        return self.env.reset(**kwargs)

    def step(self, action):
        self._integral = np.clip(
            self._integral + action * self.env.dt,
            self.env.action_space.low,
            self.env.action_space.high,
        )
        return self.env.step(self._integral)
