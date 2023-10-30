#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria
# SPDX-License-Identifier: Apache-2.0

"""Test NoisifyAction wrapper."""

import unittest

import numpy as np

from upkie.envs.wrappers.differentiate_action import DifferentiateAction
from upkie.envs.wrappers.tests.envs import ActionObserverEnv


class DifferentiateActionTestCase(unittest.TestCase):
    def test_noise(self):
        env = ActionObserverEnv()
        diff_env = DifferentiateAction(
            env,
            min_derivative=-2.0,
            max_derivative=+2.0,
        )
        action = np.array([1.0])
        inner_action, _, _, _, _ = diff_env.step(action)
        self.assertTrue(np.allclose(action * env.dt, inner_action))


if __name__ == "__main__":
    unittest.main()  # necessary for `bazel test`
