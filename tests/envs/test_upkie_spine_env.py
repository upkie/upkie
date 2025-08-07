#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

"""Tests for UpkieServos environment."""

import unittest
from multiprocessing.shared_memory import SharedMemory

import gymnasium as gym
import numpy as np

from upkie.envs.tests.mock_spine import MockSpine
from upkie.envs.upkie_spine_env import UpkieSpineEnv
from upkie.exceptions import UpkieTimeoutError


class UpkieSpineEnvTestCase(unittest.TestCase):
    def setUp(self):
        shared_memory = SharedMemory(name=None, size=42, create=True)
        self.env = UpkieSpineEnv(
            frequency=100.0,
            shm_name=shared_memory._name,
        )
        shared_memory.close()
        self.env._spine = MockSpine()

    def test_reset(self):
        observation, info = self.env.reset()
        spine_observation = info["spine_observation"]
        self.assertAlmostEqual(
            observation["left_wheel"]["position"],
            spine_observation["servo"]["left_wheel"]["position"],
        )
        self.assertGreaterEqual(spine_observation["number"], 1)

        right_knee = observation["right_knee"]
        self.assertIsInstance(right_knee["position"], np.ndarray)
        self.assertIsInstance(right_knee["velocity"], np.ndarray)
        self.assertIsInstance(right_knee["torque"], np.ndarray)
        self.assertIsInstance(right_knee["temperature"], np.ndarray)
        self.assertIsInstance(right_knee["voltage"], np.ndarray)
        self.assertEqual(right_knee["position"].shape, (1,))
        self.assertEqual(right_knee["velocity"].shape, (1,))
        self.assertEqual(right_knee["torque"].shape, (1,))
        self.assertEqual(right_knee["temperature"].shape, (1,))
        self.assertEqual(right_knee["voltage"].shape, (1,))

    def test_reward(self):
        self.env.reset()
        action = {
            joint.name: {
                "position": np.nan,
                "velocity": 0.0,
            }
            for joint in self.env.model.joints
        }
        _, reward, _, _, _ = self.env.step(action)
        self.assertAlmostEqual(reward, 1.0)  # survival reward

    def test_action_clamping(self):
        action = {
            joint.name: {
                "position": np.nan,
                "velocity": 0.0,
                "feedforward_torque": 0.0,
            }
            for joint in self.env.model.joints
        }
        not_wheel = "left_hip"  # wheels don't have position limits
        self.env.reset()

        action[not_wheel]["position"] = np.nan
        self.env.step(action)
        self.assertTrue(
            np.isnan(self.env._spine.action["servo"][not_wheel]["position"])
        )

        action[not_wheel]["position"] = 0.5
        self.env.step(action)
        self.assertAlmostEqual(
            self.env._spine.action["servo"][not_wheel]["position"],
            0.5,
            places=5,
        )

        action[not_wheel]["position"] = 5e5
        self.env.step(action)
        self.assertAlmostEqual(
            self.env._spine.action["servo"][not_wheel]["position"],
            self.env.action_space[not_wheel]["position"].high[0],
            places=5,
        )

        action[not_wheel]["feedforward_torque"] = 1e20
        self.env.step(action)
        self.assertAlmostEqual(
            self.env._spine.action["servo"][not_wheel]["feedforward_torque"],
            self.env.action_space[not_wheel]["feedforward_torque"].high[0],
            places=5,
        )

    def test_registration(self):
        shm = SharedMemory(name=None, size=42, create=True)
        env = gym.make("Upkie-Spine-Servos", shm_name=shm._name)
        self.assertIsNotNone(env)
        try:
            del env  # we delete it explicitly
        except UpkieTimeoutError:  # to catch this exception
            pass  # which is ok: there is no spine, thus no response
        shm.close()


if __name__ == "__main__":
    unittest.main()
