#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

"""Test DifferentiateAction wrapper."""

import unittest

import gymnasium as gym
import numpy as np

from upkie.envs.wrappers.differentiate_action import DifferentiateAction
from upkie.envs.wrappers.tests.envs import ActionObserverEnv


class DifferentiateActionTestCase(unittest.TestCase):
    def test_diff(self):
        env = ActionObserverEnv()
        diff_env = DifferentiateAction(
            env,
            min_derivative=-2.0,
            max_derivative=+2.0,
        )
        action = np.array([1.0])
        inner_action, _, _, _, _ = diff_env.step(action)
        self.assertTrue(np.allclose(action * env.unwrapped.dt, inner_action))

    def test_check_env(self):
        try:
            from stable_baselines3.common.env_checker import check_env

            env = gym.make("Pendulum-v1")
            diff_env = DifferentiateAction(
                env,
                min_derivative=-0.1,
                max_derivative=0.1,
            )
            check_env(diff_env)
        except ImportError:
            pass


if __name__ == "__main__":
    unittest.main()  # necessary for `bazel test`
