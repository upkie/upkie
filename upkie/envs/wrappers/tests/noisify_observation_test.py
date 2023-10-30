#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria
# SPDX-License-Identifier: Apache-2.0

"""Test UpkieBaseEnv."""

import unittest

import gymnasium
import numpy as np
from gymnasium import spaces

from upkie.envs.wrappers.noisify_observation import NoisifyObservation


class ConstantObservationEnv(gymnasium.Env):
    def __init__(self, constant: float):
        self.observation_space = spaces.Box(0.0, 2.0, shape=(1,))
        self.constant = constant

    def step(self, action):
        observation = np.array([self.constant])
        return observation, 0.0, False, False, {}


class NoisifyObservationTestCase(unittest.TestCase):
    def test_noise(self):
        env = ConstantObservationEnv(1.0)
        observation, _, _, _, _ = env.step(None)
        self.assertLess(abs(observation[0] - 1.0), 1e-10)

        noisy_env = NoisifyObservation(env, noise=np.array([0.42]))
        observation, _, _, _, _ = noisy_env.step(None)
        self.assertGreater(abs(observation[0] - 1.0), 1e-10)


if __name__ == "__main__":
    unittest.main()  # necessary for `bazel test`
