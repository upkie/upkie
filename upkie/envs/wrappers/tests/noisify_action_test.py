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

from upkie.envs.wrappers.noisify_action import NoisifyAction


class ActionObserverEnv(gymnasium.Env):
    def __init__(self):
        self.action_space = spaces.Box(0.0, 2.0, shape=(1,))

    def step(self, action):
        observation = action
        return observation, 0.0, False, False, {}


class NoisifyActionTestCase(unittest.TestCase):
    def test_noise(self):
        noisy_env = NoisifyAction(ActionObserverEnv(), noise=np.array([0.42]))
        action = np.array([1.0])
        inner_action, _, _, _, _ = noisy_env.step(action)
        self.assertGreater(np.abs(inner_action - action), 1e-10)


if __name__ == "__main__":
    unittest.main()  # necessary for `bazel test`
