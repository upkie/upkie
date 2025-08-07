#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

"""Test NoisifyAction wrapper."""

import unittest

import gymnasium as gym
import numpy as np

from upkie.envs.wrappers.add_lag_to_action import AddLagToAction
from upkie.envs.wrappers.tests.envs import ActionObserverEnv


class AddLagToActionTestCase(unittest.TestCase):
    def test_lpf(self):
        env = ActionObserverEnv()
        lpf_env = AddLagToAction(env, time_constant=1.0)
        action = np.array([1.0])
        inner_action, _, _, _, _ = lpf_env.step(action)
        self.assertTrue(np.allclose(action * env.unwrapped.dt, inner_action))

    def test_check_env(self):
        try:
            from stable_baselines3.common.env_checker import check_env

            env = gym.make("Pendulum-v1")
            env = gym.wrappers.RescaleAction(env, min_action=-1, max_action=1)
            lpf_env = AddLagToAction(env, time_constant=1.0)
            check_env(lpf_env)
        except ImportError:
            pass


if __name__ == "__main__":
    unittest.main()  # necessary for `bazel test`
