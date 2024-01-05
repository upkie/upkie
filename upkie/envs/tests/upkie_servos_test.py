#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria
# SPDX-License-Identifier: Apache-2.0

"""Tests for UpkieServos environment."""

import unittest

import numpy as np
import posix_ipc

from upkie.envs import UpkieServos
from upkie.envs.tests.mock_spine import MockSpine
from upkie.utils.exceptions import ActionError


class TestUpkieServos(unittest.TestCase):
    def setUp(self):
        shm_name = "/vroum"
        shared_memory = posix_ipc.SharedMemory(
            shm_name, posix_ipc.O_RDWR | posix_ipc.O_CREAT, size=42
        )
        self.env = UpkieServos(
            fall_pitch=1.0,
            frequency=100.0,
            shm_name=shm_name,
        )
        shared_memory.close_fd()
        self.env._spine = MockSpine()

    def test_reset(self):
        observation, info = self.env.reset()
        spine_observation = info["spine_observation"]
        self.assertAlmostEqual(
            observation["wheel_odometry"]["position"],
            spine_observation["wheel_odometry"]["position"],
        )
        self.assertGreaterEqual(spine_observation["number"], 1)

        imu = spine_observation["imu"]
        self.assertIsInstance(imu["angular_velocity"], np.ndarray)
        self.assertIsInstance(imu["linear_acceleration"], np.ndarray)
        self.assertIsInstance(imu["orientation"], np.ndarray)
        self.assertEqual(imu["angular_velocity"].shape, (3,))
        self.assertEqual(imu["linear_acceleration"].shape, (3,))
        self.assertEqual(imu["orientation"].shape, (4,))

    def test_reward(self):
        self.env.reset()
        action = {
            servo: {
                "position": np.nan,
                "velocity": 0.0,
            }
            for servo in self.env.JOINT_NAMES
        }
        _, reward, _, _, _ = self.env.step(action)
        self.assertAlmostEqual(reward, 1.0)  # survival reward

    def test_action_needs_position(self):
        observation, info = self.env.reset()
        action = {servo: {"velocity": 0.0} for servo in self.env.JOINT_NAMES}
        with self.assertRaises(ActionError):
            self.env.step(action)

    def test_action_clamping(self):
        action = {
            servo: {
                "position": np.nan,
                "velocity": 0.0,
                "feedforward_torque": 0.0,
            }
            for servo in self.env.JOINT_NAMES
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
            float(self.env.action_space[not_wheel]["position"].high),
            places=5,
        )


if __name__ == "__main__":
    unittest.main()
