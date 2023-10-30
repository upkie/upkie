#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria
# SPDX-License-Identifier: Apache-2.0

"""Test NoisifyAction wrapper."""

import unittest

import numpy as np

from upkie.envs.wrappers.noisify_action import NoisifyAction
from upkie.envs.wrappers.tests.envs import ActionObserverEnv


class NoisifyActionTestCase(unittest.TestCase):
    def test_noise(self):
        noisy_env = NoisifyAction(ActionObserverEnv(), noise=np.array([0.42]))
        action = np.array([1.0])
        inner_action, _, _, _, _ = noisy_env.step(action)
        self.assertGreater(np.abs(inner_action - action), 1e-10)


if __name__ == "__main__":
    unittest.main()  # necessary for `bazel test`
