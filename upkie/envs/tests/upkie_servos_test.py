#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria
# SPDX-License-Identifier: Apache-2.0

"""Tests for UpkieServos environment."""

import unittest

import numpy as np
import posix_ipc

from upkie.envs import UpkieServos
from upkie.envs.tests.mock_spine import MockSpine


class TestUpkieServos(unittest.TestCase):
    def setUp(self):
        shm_name = "/vroum"
        shared_memory = posix_ipc.SharedMemory(
            shm_name, posix_ipc.O_RDWR | posix_ipc.O_CREAT, size=42
        )
        self.env = UpkieServos(
            fall_pitch=1.0,
            frequency=100.0,
            shm_name=shm_name,
        )
        shared_memory.close_fd()
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
        self.assertAlmostEqual(reward, 1.0)  # survival reward


if __name__ == "__main__":
    unittest.main()
