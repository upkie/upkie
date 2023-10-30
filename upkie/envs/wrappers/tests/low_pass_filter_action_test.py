#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria
# SPDX-License-Identifier: Apache-2.0

"""Test NoisifyAction wrapper."""

import unittest

import numpy as np

from upkie.envs.wrappers.low_pass_filter_action import LowPassFilterAction
from upkie.envs.wrappers.tests.envs import ActionObserverEnv


class LowPassFilterActionTestCase(unittest.TestCase):
    def test_lpf(self):
        env = ActionObserverEnv()
        lpf_env = LowPassFilterAction(env, time_constant=1.0)
        action = np.array([1.0])
        inner_action, _, _, _, _ = lpf_env.step(action)
        self.assertTrue(np.allclose(action * env.dt, inner_action))


if __name__ == "__main__":
    unittest.main()  # necessary for `bazel test`
