#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

"""Test upkie.envs submodule."""

import unittest
from multiprocessing.shared_memory import SharedMemory

import gymnasium as gym

from upkie.envs import register


class TestUpkieEnvs(unittest.TestCase):
    def test_register(self, env_name: str = "UpkieServos"):
        with self.assertRaises(gym.error.NameNotFound):
            # runs in the same test so that we make sure this is executed
            # before the call to ``register()``
            gym.make(env_name)
        shared_memory = SharedMemory(name=None, size=42, create=True)
        register()
        gym.make(env_name, shm_name=shared_memory._name)
        shared_memory.close()


if __name__ == "__main__":
    unittest.main()
