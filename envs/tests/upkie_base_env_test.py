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

from upkie_locomotion.envs import UpkieBaseEnv


class MockSpine:
    def __init__(self):
        self.observation = {
            "number": 0,
            "servo": {
                f"{side}_{joint}": {"position": 0.0}
                for side in ("left", "right")
                for joint in ("hip", "knee", "wheel")
            },
            "imu": {
                "orientation": np.array([1.0, 0.0, 0.0, 0.0]),
                "angular_velocity": np.zeros(3),
            },
            "wheel_odometry": {
                "position": 0.0,
                "velocity": 0.0,
            },
        }

    def start(self, config: dict) -> None:
        pass

    def stop(self) -> None:
        pass

    def set_action(self, action) -> None:
        self.action = action

    def get_observation(self) -> dict:
        self.observation["number"] += 1
        return self.observation


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
            config=None, fall_pitch=1.0, shm_name=shm_name
        )
        shared_memory.close_fd()
        self.env._spine = MockSpine()

    def test_reset(self):
        _, observation_dict = self.env.reset(return_info=True)
        self.assertGreaterEqual(observation_dict["number"], 1)


if __name__ == "__main__":
    unittest.main()
