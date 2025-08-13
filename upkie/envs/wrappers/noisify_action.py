#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

## \namespace upkie.envs.wrappers.noisify_action
## \brief Add noise to the action of an environment.

"""Add noise to the action of an environment."""

import gymnasium as gym
import numpy as np

from upkie.exceptions import UpkieException


class NoisifyAction(gym.ActionWrapper):
    """!
    Add noise to the action of an environment.
    """

    ## \var high
    ## Upper bound on action noise.
    high: np.ndarray

    ## \var low
    ## Lower bound on action noise.
    low: np.ndarray

    def __init__(self, env, noise: np.ndarray):
        r"""!
        Create wrapper.

        \param env Environment to wrap.
        \param noise Noise level.
        """
        super().__init__(env)
        if noise.shape != env.action_space.shape:
            raise UpkieException(
                f"Action {noise.shape=} does not "
                f"match {env.action_space.shape=}"
            )
        self.high = +np.abs(noise)
        self.low = -np.abs(noise)

    def action(self, action: np.ndarray) -> np.ndarray:
        r"""!
        Get the noisy action.

        \param action Original action.
        \return Noisy action.
        """
        noise = self.np_random.uniform(low=self.low, high=self.high)
        noisy_action = np.clip(
            action + noise,
            self.env.action_space.low,
            self.env.action_space.high,
        )
        return noisy_action
