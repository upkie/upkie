#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

"""Tests for UpkieServoTorques environment."""

import unittest
from multiprocessing.shared_memory import SharedMemory

import numpy as np

from upkie.envs import UpkieServoTorques
from upkie.envs.tests.mock_spine import MockSpine


class TestUpkieServoTorques(unittest.TestCase):
    def setUp(self):
        shared_memory = SharedMemory(name=None, size=42, create=True)
        self.env = UpkieServoTorques(
            frequency=100.0,
            shm_name=shared_memory._name,
        )
        shared_memory.close()
        self.env._spine = MockSpine()

    def test_action_clamping(self):
        self.env.reset()
        action = {
            joint.name: {
                "feedforward_torque": 1e20,
            }
            for joint in self.env.model.joints
        }
        self.env.step(action)
        self.assertAlmostEqual(
            self.env._spine.action["servo"]["left_hip"]["feedforward_torque"],
            self.env.action_space["left_hip"]["feedforward_torque"].high[0],
            places=5,
        )

    def test_action_masking(self):
        self.env.reset()
        action = {
            joint.name: {
                "position": 1.0,
                "velocity": 1e5,
                "feedforward_torque": 1.2,
            }
            for joint in self.env.model.joints
        }
        self.env.step(action)
        # Commanded value in feedforward torque
        self.assertAlmostEqual(
            self.env._spine.action["servo"]["left_hip"]["feedforward_torque"],
            1.2,
            places=5,
        )
        # Neutral action in all other fields
        self.assertTrue(
            np.isnan(self.env._spine.action["servo"]["left_hip"]["position"]),
        )
        self.assertAlmostEqual(
            self.env._spine.action["servo"]["left_hip"]["velocity"],
            0.0,
            places=5,
        )


if __name__ == "__main__":
    unittest.main()
