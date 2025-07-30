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


class UpkieEnvsTestCase(unittest.TestCase):
    def test_registration(self):
        with self.assertRaises(gym.error.NameNotFound):
            # runs in the same test so that we make sure this is executed
            # before the call to `register()`
            gym.make("Upkie-Servos-NotFound")
        shared_memory = SharedMemory(name=None, size=42, create=True)
        upkie.envs.register()
        upkie_envs = [
            env_id
            for env_id in gym.envs.registry.keys()
            if env_id.startswith("Upkie")
        ]
        for env_id in upkie_envs:
            if env_id in ("register", "UpkieEnv"):
                continue
            kwargs = {}
            if env_id.startswith("Upkie") and "Spine" in env_id:
                kwargs["shm_name"] = shared_memory._name
            gym.make(env_id, **kwargs)
        shared_memory.close()


if __name__ == "__main__":
    unittest.main()
