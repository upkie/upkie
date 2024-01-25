#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

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
