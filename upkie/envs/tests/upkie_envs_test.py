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

"""Test upkie.envs submodule."""

import unittest

import gymnasium as gym
import posix_ipc

from upkie.envs import register


class TestUpkieEnvs(unittest.TestCase):
    def test_register(self, env_name: str = "UpkieServos"):
        with self.assertRaises(gym.error.NameNotFound):
            # runs in the same test so that we make sure this is executed
            # before the call to ``register()``
            gym.make(env_name)
        shm_name = "/foobar"
        shared_memory = posix_ipc.SharedMemory(
            shm_name, posix_ipc.O_RDWR | posix_ipc.O_CREAT, size=42
        )
        register()
        gym.make(env_name, shm_name=shm_name)
        shared_memory.close_fd()


if __name__ == "__main__":
    unittest.main()
