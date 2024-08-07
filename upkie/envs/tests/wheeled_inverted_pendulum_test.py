#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

"""Test UpkieGroundVelocity."""

import unittest

import numpy as np

from upkie.envs import WheeledInvertedPendulum


class TestWheeledInvertedPendulum(unittest.TestCase):
    def setUp(self):
        self.env = WheeledInvertedPendulum(
            fall_pitch=1.0,
            frequency=100.0,
            max_ground_velocity=1.0,
        )

    def test_reset(self):
        observation, _ = self.env.reset()
        self.assertEqual(observation.size, 4)
        self.assertAlmostEqual(observation.dot(observation), 0.0)

    def test_reset_after_steps(self):
        observation, _ = self.env.reset()
        action = np.ones(self.env.action_space.shape)
        for step in range(10):
            observation, _, _, _, _ = self.env.step(action)
        self.assertGreater(observation.dot(observation), 1e-2)
        observation, _ = self.env.reset()
        self.assertAlmostEqual(observation.dot(observation), 0.0)

    def test_reward(self):
        observation, info = self.env.reset()
        action = np.zeros(self.env.action_space.shape)
        observation, reward, terminated, truncated, _ = self.env.step(action)
        self.assertNotEqual(reward, 0.0)  # non-zero base velocity

    def test_check_env(self):
        try:
            from stable_baselines3.common.env_checker import check_env

            check_env(self.env)
        except ImportError:
            pass


if __name__ == "__main__":
    unittest.main()
