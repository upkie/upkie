#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

"""Test UpkieGroundVelocity."""

import unittest
from multiprocessing.shared_memory import SharedMemory

import numpy as np

from upkie.envs import UpkieGroundVelocity
from upkie.envs.tests.mock_spine import MockSpine


class TestUpkieGroundVelocity(unittest.TestCase):
    def setUp(self):
        shared_memory = SharedMemory(name=None, size=42, create=True)
        self.env = UpkieGroundVelocity(
            fall_pitch=1.0,
            frequency=100.0,
            max_ground_velocity=1.0,
            shm_name=shared_memory._name,
        )
        shared_memory.close()
        self.env._spine = MockSpine()

    def test_reset(self):
        observation, info = self.env.reset()
        spine_observation = info["spine_observation"]
        self.assertAlmostEqual(
            observation[1], spine_observation["wheel_odometry"]["position"]
        )
        self.assertGreaterEqual(spine_observation["number"], 1)

    def test_reward(self):
        observation, info = self.env.reset()
        action = np.zeros(self.env.action_space.shape)
        observation, reward, terminated, truncated, _ = self.env.step(action)
        self.assertNotEqual(reward, 0.0)  # non-zero base velocity

        base_orientation = self.env._spine.observation["base_orientation"]
        base_orientation["pitch"] = 0.0
        base_orientation["angular_velocity"] = [0.0, 0.0, 0.0]
        base_orientation["linear_velocity"] = [0.0, 0.0, 0.0]
        observation, reward, terminated, truncated, _ = self.env.step(action)
        self.assertAlmostEqual(reward, 1.0)  # ideal base state

    def test_check_env(self):
        try:
            from stable_baselines3.common.env_checker import check_env

            check_env(self.env)
        except ImportError:
            pass


if __name__ == "__main__":
    unittest.main()
