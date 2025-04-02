#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

"""Test RandomPush wrapper."""

import unittest
from multiprocessing.shared_memory import SharedMemory

import numpy as np

from upkie.envs import UpkieGroundVelocity, UpkieServos
from upkie.envs.tests.mock_spine import MockSpine
from upkie.envs.wrappers.random_push import RandomPush
from upkie.envs.wrappers.tests.envs import ConstantObservationEnv


class RandomPushTestCase(unittest.TestCase):
    def setUp(self):
        shared_memory = SharedMemory(name=None, size=42, create=True)
        servos_env = UpkieServos(
            frequency=100.0,
            shm_name=shared_memory._name,
        )
        servos_env._spine = MockSpine()
        ground_velocity_env = UpkieGroundVelocity(
            servos_env,
            fall_pitch=1.0,
            max_ground_velocity=1.0,
        )
        shared_memory.close()
        self.env = ground_velocity_env

    def test_wrapper(self):
        env = ConstantObservationEnv(1.0)
        env.get_spine_action = lambda action: {}
        wrapped_env = RandomPush(
            env, push_prob=1, push_generator=lambda: np.array([42, 42, 42])
        )
        action = np.array([1.0])
        wrapped_env.step(action)
        bullet_action = wrapped_env.unwrapped.get_bullet_action()
        self.assertTrue("external_forces" in bullet_action)
        self.assertTrue("torso" in bullet_action["external_forces"])
        self.assertTrue("force" in bullet_action["external_forces"]["torso"])
        self.assertTrue(
            np.allclose(
                bullet_action["external_forces"]["torso"]["force"],
                np.array([42, 42, 42]),
            )
        )

    def test_check_env(self):
        try:
            from stable_baselines3.common.env_checker import check_env

            wrapped_env = RandomPush(self.env)
            check_env(wrapped_env)
        except ImportError:
            pass


if __name__ == "__main__":
    unittest.main()  # necessary for `bazel test`
