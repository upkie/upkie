#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria
# SPDX-License-Identifier: Apache-2.0

"""Test UpkieBaseEnv."""

import unittest

import numpy as np
import posix_ipc
from gymnasium import spaces
from numpy.typing import NDArray
from stable_baselines3.common.env_checker import check_env

from upkie.envs import UpkieBaseEnv
from upkie.envs.tests.mock_spine import MockSpine


class UpkieTestEnv(UpkieBaseEnv):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.observation_space = spaces.Box(
            -1.0,
            +1.0,
            shape=(1,),
            dtype=np.float32,
        )
        self.action_space = spaces.Box(
            -1.0,
            1.0,
            shape=(1,),
            dtype=np.float32,
        )

    def parse_first_observation(self, observation_dict: dict) -> None:
        pass

    def vectorize_observation(self, observation_dict: dict) -> NDArray[float]:
        return np.full((1,), 0.5, dtype=self.observation_space.dtype)

    def dictionarize_action(self, action: NDArray[float]) -> dict:
        return {"test": action}

    def get_reward(
        self, observation: NDArray[float], action: NDArray[float]
    ) -> float:
        return 1.0


class TestUpkieBaseEnv(unittest.TestCase):
    def setUp(self):
        shm_name = "/vroum"
        shared_memory = posix_ipc.SharedMemory(
            shm_name, posix_ipc.O_RDWR | posix_ipc.O_CREAT, size=42
        )
        self.env = UpkieTestEnv(
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
        env = UpkieTestEnv(
            fall_pitch=1.0,
            frequency=100.0,
            shm_name=shm_name,
            spine_config={"some_value": 12, "bullet": {"gui": False}},
        )
        shared_memory.close_fd()
        self.assertEqual(env._spine_config["some_value"], 12)
        self.assertEqual(env._spine_config["some_value"], 12)
        self.assertEqual(env._spine_config["bullet"]["gui"], False)

    def test_check_env(self):
        check_env(self.env)


if __name__ == "__main__":
    unittest.main()
