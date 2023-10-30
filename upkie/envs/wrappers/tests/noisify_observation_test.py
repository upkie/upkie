#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria
# SPDX-License-Identifier: Apache-2.0

"""Test NoisifyObservation wrapper."""

import unittest

import numpy as np

from upkie.envs.wrappers.noisify_observation import NoisifyObservation
from upkie.envs.wrappers.tests.envs import ConstantObservationEnv


class NoisifyObservationTestCase(unittest.TestCase):
    def test_noise(self):
        env = ConstantObservationEnv(1.0)
        observation, _, _, _, _ = env.step(None)
        self.assertLess(abs(observation[0] - 1.0), 1e-10)

        noisy_env = NoisifyObservation(env, noise=np.array([0.42]))
        observation, _, _, _, _ = noisy_env.step(None)
        self.assertGreater(abs(observation[0] - 1.0), 1e-10)


if __name__ == "__main__":
    unittest.main()  # necessary for `bazel test`
