#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria
# SPDX-License-Identifier: Apache-2.0

import gymnasium
import numpy as np

from upkie.utils.exceptions import UpkieException


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
