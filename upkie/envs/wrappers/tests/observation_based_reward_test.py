#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

"""Test ObservationBasedReward wrapper."""

import unittest

import gymnasium as gym
import numpy as np

from upkie.envs.wrappers.observation_based_reward import ObservationBasedReward
from upkie.envs.wrappers.tests.envs import ActionObserverEnv


class TestReward(ObservationBasedReward):
    def reward(self, observation, info):
        return 42.0


class ObservationBasedRewardTestCase(unittest.TestCase):
    def test_reward(self):
        wrapped_env = TestReward(ActionObserverEnv())
        wrapped_env.reset()
        _, reward, _, _, _ = wrapped_env.step(np.array([1.0]))
        self.assertAlmostEqual(reward, 42.0)

    def test_check_env(self):
        try:
            from stable_baselines3.common.env_checker import check_env

            env = gym.make("Pendulum-v1")
            wrapped_env = TestReward(env)
            check_env(wrapped_env)
        except ImportError:
            pass


if __name__ == "__main__":
    unittest.main()  # necessary for `bazel test`
