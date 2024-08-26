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
        env = ConstantObservationEnv(1.0)
        env.get_spine_action = lambda action: {}
        wrapped_env = RandomPush(
            env,
            push_prob=1,
            push_generator=lambda: np.array([42, 42, 42])
            )
        action = np.array([1.0])
        spine_action = wrapped_env.get_spine_action(action)
        self.assertTrue("bullet" in spine_action)
        assert "external_forces" in spine_action["bullet"]
        assert "torso" in spine_action["bullet"]["external_forces"]
        assert "force" in spine_action["bullet"]["external_forces"]["torso"]
        assert np.allclose(
            spine_action["bullet"]["external_forces"]["torso"]["force"],
            np.array([42, 42, 42])
            )

    def test_check_env(self):
        try:
            from stable_baselines3.common.env_checker import check_env

            env = gymnasium.make("Acrobot-v1")
            env.get_spine_action = lambda action: {}
            noisy_env = RandomPush(env, noise=np.full(6, 0.42))
            check_env(noisy_env)
        except ImportError:
            pass


if __name__ == "__main__":
    unittest.main()  # necessary for `bazel test`
