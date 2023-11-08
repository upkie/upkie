#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria
# SPDX-License-Identifier: Apache-2.0

"""Test AddActionToObservation wrapper."""

import unittest

import gymnasium
import numpy as np
from stable_baselines3.common.env_checker import check_env

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
        env = gymnasium.make("Pendulum-v1")
        wrapped_env = AddActionToObservation(env)
        check_env(wrapped_env)


if __name__ == "__main__":
    unittest.main()  # necessary for `bazel test`
