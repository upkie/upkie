#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

import gymnasium
import numpy as np
from numpy.typing import NDArray

from upkie.exceptions import UpkieException


class NoisifyObservation(gymnasium.ObservationWrapper):

    """!
    Add noise to the observation of an environment.
    """

    def __init__(self, env, noise: NDArray[float]):
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
        ).astype(self.env.observation_space.dtype)
        return noisy_observation
