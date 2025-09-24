#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Upkie Team

"""Test UpkieNavigation wrapper."""

import unittest

import numpy as np

from upkie.envs.backends import MockBackend
from upkie.envs.upkie_navigation import UpkieNavigation
from upkie.envs.upkie_pendulum import UpkiePendulum
from upkie.envs.upkie_servos import UpkieServos


class NavigationTestCase(unittest.TestCase):
    def setUp(self):
        """Set up test environment with mock backend."""
        self.backend = MockBackend()
        servos_env = UpkieServos(
            backend=self.backend,
            frequency=100.0,
        )
        pendulum_env = UpkiePendulum(
            servos_env,
            fall_pitch=1.0,
            max_ground_velocity=1.0,
        )
        navigation_env = UpkieNavigation(
            pendulum_env,
            max_linear_velocity=1.0,
            max_angular_velocity=2.0,
        )
        self.servos_env = servos_env
        self.pendulum_env = pendulum_env
        self.env = navigation_env

    def test_action_space_dtype(self):
        """Test that action space has correct dtype np.float32."""
        self.assertEqual(self.env.action_space.dtype, np.float32)
        self.assertEqual(self.env.action_space.shape, (2,))
        # Test limits
        np.testing.assert_array_equal(
            self.env.action_space.low, np.array([-1.0, -2.0], dtype=np.float32)
        )
        np.testing.assert_array_equal(
            self.env.action_space.high, np.array([1.0, 2.0], dtype=np.float32)
        )

    def test_observation_space_dtype(self):
        """Test that observation space has correct dtype."""
        # UpkieNavigation observation space should be float (not float32)
        self.assertEqual(self.env.observation_space.dtype, float)
        self.assertEqual(self.env.observation_space.shape, (3,))
        # Check SE(2) limits: [x, y, theta]
        expected_low = np.array([-np.inf, -np.inf, -np.pi], dtype=float)
        expected_high = np.array([np.inf, np.inf, np.pi], dtype=float)
        np.testing.assert_array_equal(
            self.env.observation_space.low, expected_low
        )
        np.testing.assert_array_equal(
            self.env.observation_space.high, expected_high
        )

if __name__ == "__main__":
    unittest.main()
