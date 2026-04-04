#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

"""Test UpkieBaseVelocity wrapper."""

import math
import unittest
from multiprocessing.shared_memory import SharedMemory

import numpy as np

from upkie.envs.backends import SpineBackend
from upkie.envs.testing import MockSpine
from upkie.envs.upkie_base_velocity import UpkieBaseVelocity
from upkie.envs.upkie_servos import UpkieServos


class BaseVelocityTestCase(unittest.TestCase):
    def setUp(self):
        shared_memory = SharedMemory(name=None, size=42, create=True)
        self.backend = SpineBackend(shm_name=shared_memory._name)
        servos_env = UpkieServos(
            backend=self.backend,
            frequency=100.0,
        )
        self.backend._spine = MockSpine()
        self.backend._spine.observation["floor_contact"] = {"contact": True}
        base_velocity = UpkieBaseVelocity(
            servos_env,
            fall_pitch=1.0,
            max_ground_velocity=1.0,
            max_yaw_velocity=1.0,
        )
        shared_memory.close()
        self.servos_env = servos_env
        self.env = base_velocity

    def test_observation_shape(self):
        observation, _ = self.env.reset()
        self.assertEqual(observation.shape, (3,))

    def test_action_shape(self):
        self.assertEqual(self.env.action_space.shape, (2,))

    def test_reset_returns_zeros(self):
        observation, _ = self.env.reset()
        np.testing.assert_array_almost_equal(observation, [0.0, 0.0, 0.0])

    def test_reset_clears_position(self):
        self.env.reset()
        action = np.array([0.5, 0.0], dtype=np.float32)
        self.env.step(action)
        observation, _ = self.env.reset()
        np.testing.assert_array_almost_equal(observation, [0.0, 0.0, 0.0])

    def test_yaw_integration(self):
        self.env.reset()
        yaw_velocity = 0.5  # rad/s
        action = np.array([0.0, yaw_velocity], dtype=np.float32)
        observation, _, _, _, _ = self.env.step(action)
        dt = self.env.unwrapped.dt
        expected_yaw = yaw_velocity * dt
        self.assertAlmostEqual(observation[2], expected_yaw, places=5)

    def test_xy_integration(self):
        self.env.reset()
        linear_velocity = 0.5  # m/s
        action = np.array([linear_velocity, 0.0], dtype=np.float32)
        observation, _, _, _, _ = self.env.step(action)
        # At yaw ~0, x increases, y stays ~0
        self.assertGreater(abs(observation[0]), 0.0)
        self.assertAlmostEqual(observation[1], 0.0, places=4)

    def test_xy_integration_with_yaw(self):
        self.env.reset()
        dt = self.env.unwrapped.dt
        # First step: turn to establish a heading
        yaw_velocity = math.pi / 2 / dt  # turn ~pi/2 in one step
        action_turn = np.array([0.0, yaw_velocity], dtype=np.float32)
        obs_after_turn, _, _, _, _ = self.env.step(action_turn)
        yaw = obs_after_turn[2]
        # Second step: move forward at this heading
        linear_velocity = 1.0
        action_forward = np.array([linear_velocity, 0.0], dtype=np.float32)
        obs, _, _, _, _ = self.env.step(action_forward)
        # x should change by linear_velocity * cos(yaw) * dt
        expected_x = obs_after_turn[0] + linear_velocity * math.cos(yaw) * dt
        expected_y = obs_after_turn[1] + linear_velocity * math.sin(yaw) * dt
        self.assertAlmostEqual(obs[0], expected_x, places=4)
        self.assertAlmostEqual(obs[1], expected_y, places=4)

    def test_dtype_consistency(self):
        self.assertEqual(self.env.action_space.dtype, np.float32)
        self.assertEqual(self.env.observation_space.dtype, np.float32)

        observation, _ = self.env.reset()
        self.assertEqual(observation.dtype, np.float32)

        action = np.zeros(self.env.action_space.shape, dtype=np.float32)
        observation, _, _, _, _ = self.env.step(action)
        self.assertEqual(observation.dtype, np.float32)

    def test_check_env(self):
        try:
            from stable_baselines3.common.env_checker import check_env

            check_env(self.env)
        except ImportError:
            pass

    def test_mpc_balancer_resets(self):
        self.env.reset()
        action = np.array([0.5, 0.0], dtype=np.float32)
        self.env.step(action)
        self.env.reset()
        self.assertAlmostEqual(self.env.mpc_balancer.commanded_velocity, 0.0)


if __name__ == "__main__":
    unittest.main()
