#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

"""Test RandomPush wrapper."""

import unittest

import gymnasium
import numpy as np

from upkie.envs.wrappers.random_push import (
    RandomPush,
)
from upkie.envs.wrappers.tests.envs import ConstantObservationEnv


class RandomPushTestCase(unittest.TestCase):
    def test_wrapper(self):
        env = ConstantObservationEnv(42.0)
        wrapped_env = RandomPush(env)
        action = np.array([1.0])
        observation, _, _, _, _ = wrapped_env.step(action)
        self.assertAlmostEqual(observation[0], 42.0)
        self.assertAlmostEqual(observation[1], 1.0)

    def test_check_env(self):
        try:
            from stable_baselines3.common.env_checker import check_env

            env = gymnasium.make("Pendulum-v1")
            wrapped_env = AddActionToObservation(env)
            check_env(wrapped_env)
        except ImportError:
            pass


if __name__ == "__main__":
    unittest.main()  # necessary for `bazel test`
