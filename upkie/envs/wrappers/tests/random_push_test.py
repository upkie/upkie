#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

"""Test RandomPush wrapper."""

import unittest
import numpy as np
from multiprocessing.shared_memory import SharedMemory
from upkie.envs import UpkieGroundVelocity
from upkie.envs.tests.mock_spine import MockSpine

from upkie.envs.wrappers.random_push import (
    RandomPush,
)
from upkie.envs.wrappers.tests.envs import ConstantObservationEnv


class RandomPushTestCase(unittest.TestCase):
    def setUp(self):
        shared_memory = SharedMemory(name=None, size=42, create=True)
        self.env = UpkieGroundVelocity(
            fall_pitch=1.0,
            frequency=100.0,
            max_ground_velocity=1.0,
            shm_name=shared_memory._name,
        )
        shared_memory.close()
        self.env._spine = MockSpine()

    def test_wrapper(self):
        env = ConstantObservationEnv(1.0)
        env.get_spine_action = lambda action: {}
        wrapped_env = RandomPush(
            env,
            push_prob=1,
            push_generator=lambda: np.array([42, 42, 42])
            )
        action = np.array([1.0])
        spine_action = wrapped_env.get_spine_action(action)
        self.assertTrue("bullet" in spine_action)
        self.assertTrue("external_forces" in spine_action["bullet"])
        self.assertTrue("torso" in spine_action["bullet"]["external_forces"])
        self.assertTrue("force" in spine_action["bullet"]["external_forces"]["torso"])
        self.assertTrue(np.allclose(
            spine_action["bullet"]["external_forces"]["torso"]["force"],
            np.array([42, 42, 42])
            ))

    def test_check_env(self):
        try:
            from stable_baselines3.common.env_checker import check_env

            self.setUp()
            self.env = RandomPush(self.env, noise=np.full(6, 0.42))
            check_env(self.env)
        except ImportError:
            pass


if __name__ == "__main__":
    unittest.main()  # necessary for `bazel test`
