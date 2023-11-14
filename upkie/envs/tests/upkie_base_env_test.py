#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria
# SPDX-License-Identifier: Apache-2.0

"""Test UpkieBaseEnv."""

import unittest

import numpy as np
import posix_ipc
from numpy.typing import NDArray

from upkie.envs import Reward, UpkieBaseEnv
from upkie.envs.tests.mock_spine import MockSpine


class UpkieBaseChild(UpkieBaseEnv):
    def parse_first_observation(self, observation_dict: dict) -> None:
        pass

    def vectorize_observation(self, observation_dict: dict) -> NDArray[float]:
        return np.empty(42)

    def dictionarize_action(self, action: NDArray[float]) -> dict:
        return {}


class TestUpkieBaseEnv(unittest.TestCase):
    def setUp(self):
        shm_name = "/vroum"
        shared_memory = posix_ipc.SharedMemory(
            shm_name, posix_ipc.O_RDWR | posix_ipc.O_CREAT, size=42
        )
        self.env = UpkieBaseChild(
            reward=Reward(),
            fall_pitch=1.0,
            frequency=100.0,
            shm_name=shm_name,
            spine_config=None,
        )
        shared_memory.close_fd()
        self.env._spine = MockSpine()

    def test_reset(self):
        _, info = self.env.reset()
        observation_dict = info["observation"]
        self.assertGreaterEqual(observation_dict["number"], 1)

    def test_spine_config(self):
        """Check that runtime and default configs are merged properly."""
        shm_name = "/vroum"
        shared_memory = posix_ipc.SharedMemory(
            shm_name, posix_ipc.O_RDWR | posix_ipc.O_CREAT, size=42
        )
        env = UpkieBaseChild(
            reward=Reward(),
            fall_pitch=1.0,
            frequency=100.0,
            shm_name=shm_name,
            spine_config={"some_value": 12, "bullet": {"gui": False}},
        )
        shared_memory.close_fd()
        self.assertEqual(env._spine_config["some_value"], 12)
        self.assertEqual(env._spine_config["some_value"], 12)
        self.assertEqual(env._spine_config["bullet"]["gui"], False)


if __name__ == "__main__":
    unittest.main()
