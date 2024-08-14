#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

"""Test upkie.envs submodule."""

import unittest
from multiprocessing.shared_memory import SharedMemory

import gymnasium as gym

import upkie.envs


class TestUpkieEnvs(unittest.TestCase):
    def test_registration(self):
        with self.assertRaises(gym.error.NameNotFound):
            # runs in the same test so that we make sure this is executed
            # before the call to ``register()``
            gym.make("UpkieServos")
        shared_memory = SharedMemory(name=None, size=42, create=True)
        upkie.envs.register()
        for env_name in upkie.envs.__all__:
            if env_name in ("register", "UpkieBaseEnv"):
                continue
            gym.make(env_name, shm_name=shared_memory._name)
        shared_memory.close()


if __name__ == "__main__":
    unittest.main()
