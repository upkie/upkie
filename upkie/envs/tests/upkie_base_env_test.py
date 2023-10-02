#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Test UpkieBaseEnv."""

import unittest

import numpy as np
import posix_ipc

from upkie.envs import Reward, UpkieBaseEnv
from upkie.envs.tests.mock_spine import MockSpine


class UpkieBaseChild(UpkieBaseEnv):
    def parse_first_observation(self, observation_dict: dict) -> None:
        pass

    def vectorize_observation(self, observation_dict: dict) -> np.ndarray:
        return np.empty(42)

    def dictionarize_action(self, action: np.ndarray) -> dict:
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
        self.assertEqual(env.spine_config["some_value"], 12)
        self.assertEqual(env.spine_config["some_value"], 12)
        self.assertEqual(env.spine_config["bullet"]["control_mode"], "torque")
        self.assertEqual(env.spine_config["bullet"]["gui"], False)


if __name__ == "__main__":
    unittest.main()
