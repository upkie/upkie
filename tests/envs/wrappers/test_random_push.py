#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

"""Test RandomPush wrapper."""

import unittest
from multiprocessing.shared_memory import SharedMemory

import numpy as np

from upkie.envs.backends import SpineBackend
from upkie.envs.testing import ConstantObservationEnv, MockSpine
from upkie.envs.upkie_pendulum import UpkiePendulum
from upkie.envs.upkie_servos import UpkieServos
from upkie.envs.wrappers.random_push import RandomPush


class RandomPushTestCase(unittest.TestCase):
    def setUp(self):
        shared_memory = SharedMemory(name=None, size=42, create=True)
        backend = SpineBackend(shm_name=shared_memory._name)
        servos_env = UpkieServos(
            backend=backend,
            frequency=100.0,
        )
        backend._spine = MockSpine()
        ground_velocity_env = UpkiePendulum(
            servos_env,
            fall_pitch=1.0,
            max_ground_velocity=1.0,
        )
        shared_memory.close()
        self.env = ground_velocity_env

    def test_wrapper(self):
        env = ConstantObservationEnv(1.0)
        env.get_spine_action = lambda action: {}

        # Mock backend with set_external_forces method
        class MockBackend:
            def __init__(self):
                self.external_forces = {}

            def set_external_forces(self, forces):
                self.external_forces = forces

        env.backend = MockBackend()

        wrapped_env = RandomPush(
            env, push_prob=1, push_generator=lambda: np.array([42, 42, 42])
        )
        action = np.array([1.0])
        wrapped_env.step(action)

        # Check that external forces were applied to the backend
        self.assertTrue("torso" in env.backend.external_forces)
        self.assertTrue("force" in env.backend.external_forces["torso"])
        self.assertTrue(
            np.allclose(
                env.backend.external_forces["torso"]["force"],
                np.array([42, 42, 42]),
            )
        )

    def test_check_env(self):
        try:
            from stable_baselines3.common.env_checker import check_env

            # Add mock backend with set_external_forces method
            class MockBackend:
                def set_external_forces(self, forces):
                    pass

            self.env.backend = MockBackend()
            wrapped_env = RandomPush(self.env)
            check_env(wrapped_env)
        except ImportError:
            pass


if __name__ == "__main__":
    unittest.main()  # necessary for `bazel test`
