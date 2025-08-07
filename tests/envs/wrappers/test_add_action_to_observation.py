#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

"""Test AddActionToObservation wrapper."""

import unittest

import gymnasium as gym
import numpy as np

from upkie.envs.wrappers.add_action_to_observation import (
    AddActionToObservation,
)
from upkie.envs.wrappers.tests.envs import ConstantObservationEnv


class AddActionToObservationTestCase(unittest.TestCase):
    def test_wrapper(self):
        env = ConstantObservationEnv(42.0)
        wrapped_env = AddActionToObservation(env)
        action = np.array([1.0])
        observation, _, _, _, _ = wrapped_env.step(action)
        self.assertAlmostEqual(observation[0], 42.0)
        self.assertAlmostEqual(observation[1], 1.0)

    def test_check_env(self):
        try:
            from stable_baselines3.common.env_checker import check_env

            env = gym.make("Pendulum-v1")
            env = gym.wrappers.RescaleAction(env, min_action=-1, max_action=1)
            wrapped_env = AddActionToObservation(env)
            check_env(wrapped_env)
        except ImportError:
            pass


if __name__ == "__main__":
    unittest.main()  # necessary for `bazel test`
