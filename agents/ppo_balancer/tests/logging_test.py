#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron
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

import unittest

import posix_ipc

from agents.ppo_balancer.logging import SummaryWriterCallback
from upkie_locomotion.envs import UpkieWheelsEnv


class TestSummaryWriterCallback(unittest.TestCase):
    def test_init(self):
        shm_name = "/vroum"
        shared_memory = posix_ipc.SharedMemory(
            shm_name, posix_ipc.O_RDWR | posix_ipc.O_CREAT, size=42
        )
        env = UpkieWheelsEnv(shm_name=shm_name)
        shared_memory.close_fd()
        writer = SummaryWriterCallback(env)
        self.assertIsNotNone(writer.env)


if __name__ == "__main__":
    unittest.main()
